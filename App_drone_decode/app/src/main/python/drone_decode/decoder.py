"""Fixed-slot receiver state machine."""

from collections import deque
from dataclasses import replace
from typing import Any

from .constants import DATA_ACTION_COUNT, SYNC_ACTIONS
from .frame import actions_to_symbols, decode_data_actions, normalize_action
from .models import DecoderState, FrameDecodeResult
from .sync import sync_matches, sync_score


class ProtocolDecoder:
    def __init__(
        self,
        sync_max_distance: int = 0,
        sync_actions: tuple[str, ...] = SYNC_ACTIONS,
        action_mapping: dict[str, str] | None = None,
    ):
        self.sync_max_distance = sync_max_distance
        self.sync_actions = tuple(normalize_action(action) for action in sync_actions)
        self.action_mapping = action_mapping
        self.state = DecoderState.IDLE
        self._sync_history: deque[str] = deque(maxlen=len(self.sync_actions))
        self._data_actions: list[str] = []
        self._data_confidences: list[float] = []
        self._last_sequence: int | None = None
        self.last_result: FrameDecodeResult | None = None

    def start(self) -> None:
        if self.state == DecoderState.IDLE:
            self.state = DecoderState.SEARCH_SYNC

    def pause(self) -> None:
        self.state = DecoderState.IDLE

    def reset(self) -> None:
        self.state = DecoderState.SEARCH_SYNC
        self._sync_history.clear()
        self._data_actions.clear()
        self._data_confidences.clear()
        self._last_sequence = None
        self.last_result = None

    def process_slot(self, action: str, confidence: float = 0.0, slot_index: int | None = None) -> dict[str, Any]:
        normalized = normalize_action(action)
        confidence = max(0.0, min(1.0, float(confidence)))
        if self.state == DecoderState.IDLE:
            return self._event(slot_index, normalized, DecoderState.IDLE)

        if self.state == DecoderState.SEARCH_SYNC:
            self._sync_history.append(normalized)
            score = sync_score(tuple(self._sync_history), self.sync_actions) if len(self._sync_history) == len(self.sync_actions) else 0.0
            if len(self._sync_history) == len(self.sync_actions) and sync_matches(
                tuple(self._sync_history), self.sync_actions, self.sync_max_distance
            ):
                self.state = DecoderState.COLLECT_FRAME
                self._data_actions.clear()
                self._data_confidences.clear()
            return self._event(slot_index, normalized, self.state, sync_score_value=score)

        if self.state == DecoderState.COLLECT_FRAME:
            self._data_actions.append(normalized)
            self._data_confidences.append(confidence)
            if len(self._data_actions) < DATA_ACTION_COUNT:
                return self._event(slot_index, normalized, self.state)

            self.state = DecoderState.BCH_DECODE
            result = decode_data_actions(self._data_actions, self._data_confidences, self.action_mapping)
            outcome_state = DecoderState.ACCEPT_FRAME if result.accepted else DecoderState.REJECT_FRAME
            if result.accepted and result.seq is not None:
                missing = self._missing_frames(result.seq)
                result = replace(result, possible_missing_frames=missing)
                self._last_sequence = result.seq
            self.last_result = result
            event = self._event(slot_index, normalized, outcome_state, result=result)
            event["next_state"] = DecoderState.SEARCH_SYNC.value
            self._data_actions.clear()
            self._data_confidences.clear()
            self._sync_history.clear()
            self.state = DecoderState.SEARCH_SYNC
            return event

        self.state = DecoderState.ERROR
        return self._event(slot_index, normalized, self.state)

    def _missing_frames(self, sequence: int) -> int:
        if self._last_sequence is None:
            return 0
        expected = (self._last_sequence + 1) % 4
        return (sequence - expected) % 4

    def _event(
        self,
        slot_index: int | None,
        action: str,
        reported_state: DecoderState,
        sync_score_value: float | None = None,
        result: FrameDecodeResult | None = None,
    ) -> dict[str, Any]:
        return {
            "slot_index": slot_index,
            "action": action,
            "symbol": actions_to_symbols((action,), self.action_mapping)[0],
            "decoder_state": reported_state.value,
            "next_state": self.state.value,
            "sync_score": sync_score_value if sync_score_value is not None else (
                sync_score(tuple(self._sync_history), self.sync_actions) if len(self._sync_history) == len(self.sync_actions) else 0.0
            ),
            "matched_sync_history": list(self._sync_history),
            "frame_collection_progress": len(self._data_actions),
            "result": result.to_dict() if result else None,
        }
