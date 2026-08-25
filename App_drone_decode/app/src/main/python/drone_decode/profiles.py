"""Validation for versioned decoder profiles crossing the Android bridge."""

from typing import Any

from .constants import PROTOCOL_VERSION
from .frame import normalize_action


DEFAULT_PROFILE: dict[str, Any] = {
    "profileVersion": 4,
    "protocolVersion": PROTOCOL_VERSION,
    "actionDurationMs": 500,
    "idleDurationMs": 500,
    "stableWindowFraction": 0.60,
    "minimumSamplesPerSlot": 5,
    "erasureThreshold": 0.55,
    "sync": ["R", "L", "H", "R", "F", "L", "F", "H"],
    "actionMapping": {"F": "00", "H": "11", "R": "10", "L": "01"},
}


def validate_profile(profile: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if not isinstance(profile.get("profileVersion"), int) or profile["profileVersion"] < 1:
        errors.append("profileVersion must be a positive integer")
    if profile.get("protocolVersion") not in {1, 2, PROTOCOL_VERSION}:
        errors.append("Unsupported protocol version")

    action_duration = profile.get("actionDurationMs")
    if not isinstance(action_duration, int) or not 100 <= action_duration <= 2000:
        errors.append("actionDurationMs must be an integer from 100 to 2000")
    idle_duration = profile.get("idleDurationMs")
    if not isinstance(idle_duration, int) or not 0 <= idle_duration <= 2000:
        errors.append("idleDurationMs must be an integer from 0 to 2000")
    if isinstance(action_duration, int) and isinstance(idle_duration, int) and action_duration + idle_duration > 4000:
        errors.append("actionDurationMs plus idleDurationMs must not exceed 4000")
    stable_fraction = profile.get("stableWindowFraction")
    if not isinstance(stable_fraction, (int, float)) or not 0.2 <= stable_fraction <= 0.9:
        errors.append("stableWindowFraction must be from 0.2 to 0.9")
    minimum_samples = profile.get("minimumSamplesPerSlot")
    if not isinstance(minimum_samples, int) or not 1 <= minimum_samples <= 60:
        errors.append("minimumSamplesPerSlot must be from 1 to 60")

    mapping = profile.get("actionMapping")
    if not isinstance(mapping, dict) or set(mapping) != {"H", "F", "L", "R"}:
        errors.append("actionMapping must define H, F, L, and R")
    elif set(mapping.values()) != {"00", "01", "10", "11"}:
        errors.append("Action codes must be unique and use 00, 01, 10, and 11")

    sync = profile.get("sync")
    if not isinstance(sync, list) or len(sync) != 8:
        errors.append("SYNC must contain exactly eight actions")
    else:
        try:
            normalized = [normalize_action(value) for value in sync]
            if "?" in normalized:
                errors.append("SYNC cannot contain UNKNOWN")
        except ValueError:
            errors.append("SYNC contains an invalid action")
    return errors
