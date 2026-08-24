"""JSON boundary used by the Kotlin/Chaquopy facade."""

import json
from itertools import combinations
from typing import Any

from .decoder import ProtocolDecoder
from .frame import decode_data_actions, encode_frame
from .profiles import validate_profile

_decoder = ProtocolDecoder()


def _json(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def start() -> str:
    _decoder.start()
    return _json({"ok": True, "state": _decoder.state.value})


def pause() -> str:
    _decoder.pause()
    return _json({"ok": True, "state": _decoder.state.value})


def reset() -> str:
    _decoder.reset()
    return _json({"ok": True, "state": _decoder.state.value})


def process_slot_json(request_json: str) -> str:
    try:
        request = json.loads(request_json)
        event = _decoder.process_slot(
            action=request["action"],
            confidence=request.get("confidence", 0.0),
            slot_index=request.get("slotIndex"),
        )
        return _json({"ok": True, **event})
    except Exception as error:
        return _json({"ok": False, "error": type(error).__name__, "message": str(error)})


def decode_actions_json(request_json: str) -> str:
    try:
        request = json.loads(request_json)
        result = decode_data_actions(request["actions"], request.get("confidences"))
        return _json({"ok": True, "result": result.to_dict()})
    except Exception as error:
        return _json({"ok": False, "error": type(error).__name__, "message": str(error)})


def decode_replay_candidates_json(request_json: str) -> str:
    """BCH-check blind visual candidates without consulting payload labels."""
    try:
        request = json.loads(request_json)
        candidates = request["candidates"]
        if not isinstance(candidates, list) or len(candidates) > 4096:
            raise ValueError("Replay candidate count must be between 0 and 4096")
        results = []
        for index, candidate in enumerate(candidates):
            actions = list(candidate["actions"])
            confidences = list(candidate.get("confidences", [0.0] * len(actions)))
            alternatives = list(candidate.get("alternatives", actions))
            if len(actions) != 32 or len(confidences) != 32 or len(alternatives) != 32:
                raise ValueError("Each replay candidate must contain 32 actions, confidences, and alternatives")
            variants = {tuple(actions): (0, 0.0)}
            uncertain = sorted(range(len(actions)), key=lambda position: confidences[position])[:12]
            for change_count in range(1, 4):
                for positions in combinations(uncertain, change_count):
                    changed = actions.copy()
                    for position in positions:
                        changed[position] = alternatives[position]
                    if changed != actions:
                        key = tuple(changed)
                        penalty = sum(confidences[position] for position in positions)
                        prior_variant = variants.get(key)
                        if prior_variant is None or (penalty, change_count) < (prior_variant[1], prior_variant[0]):
                            variants[key] = (change_count, penalty)
            for position in uncertain[:7]:
                for replacement in ("H", "F", "L", "R"):
                    if replacement != actions[position]:
                        changed = actions.copy()
                        changed[position] = replacement
                        key = tuple(changed)
                        penalty = confidences[position]
                        prior_variant = variants.get(key)
                        if prior_variant is None or (penalty, 1) < (prior_variant[1], prior_variant[0]):
                            variants[key] = (1, penalty)
            accepted_by_payload = {}
            for variant_actions, (variant_cost, variant_penalty) in variants.items():
                result = decode_data_actions(
                    variant_actions,
                    confidences,
                    _decoder.action_mapping,
                )
                decoded_results = [(result, list(variant_actions))] if result.accepted else []
                for result, accepted_actions in decoded_results:
                    result_data = result.to_dict()
                    payload_key = (
                        result_data.get("payload_hex"),
                        result_data.get("seq"),
                        result_data.get("length"),
                    )
                    item = {
                        "index": index,
                        "variantCost": variant_cost,
                        "variantPenalty": variant_penalty,
                        "actions": accepted_actions,
                        **result_data,
                    }
                    prior = accepted_by_payload.get(payload_key)
                    if prior is None or (variant_penalty, variant_cost, result.corrected_bit_errors) < (
                        prior["variantPenalty"], prior["variantCost"], prior["corrected_bit_errors"]
                    ):
                        accepted_by_payload[payload_key] = item
            results.extend(accepted_by_payload.values())
        return _json({"ok": True, "results": results})
    except Exception as error:
        return _json({"ok": False, "error": type(error).__name__, "message": str(error)})


def encode_frame_json(request_json: str) -> str:
    try:
        request = json.loads(request_json)
        payload = bytes.fromhex(request["payloadHex"])
        actions = encode_frame(payload, int(request["seq"]))
        return _json({"ok": True, "actions": list(actions)})
    except Exception as error:
        return _json({"ok": False, "error": type(error).__name__, "message": str(error)})


def validate_profile_json(profile_json: str) -> str:
    try:
        errors = validate_profile(json.loads(profile_json))
        return _json({"ok": not errors, "errors": errors})
    except Exception as error:
        return _json({"ok": False, "errors": [str(error)]})


def configure_profile_json(profile_json: str) -> str:
    global _decoder
    try:
        profile = json.loads(profile_json)
        errors = validate_profile(profile)
        if errors:
            return _json({"ok": False, "errors": errors})
        _decoder = ProtocolDecoder(
            sync_actions=tuple(profile["sync"]),
            action_mapping=dict(profile["actionMapping"]),
        )
        return _json({"ok": True, "state": _decoder.state.value})
    except Exception as error:
        return _json({"ok": False, "errors": [str(error)]})
