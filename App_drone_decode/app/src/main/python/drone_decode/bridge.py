"""JSON boundary used by the Kotlin/Chaquopy facade."""

import json
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
            variants = [(0, actions)]
            uncertain = sorted(range(len(actions)), key=lambda position: confidences[position])[:8]
            for position in uncertain:
                for replacement in ("H", "F", "L", "R"):
                    if replacement != actions[position]:
                        changed = actions.copy()
                        changed[position] = replacement
                        variants.append((1, changed))
            for variant_cost, variant_actions in variants:
                result = decode_data_actions(
                    variant_actions,
                    confidences,
                    _decoder.action_mapping,
                )
                if result.accepted:
                    results.append({
                        "index": index,
                        "variantCost": variant_cost,
                        "actions": variant_actions,
                        **result.to_dict(),
                    })
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
