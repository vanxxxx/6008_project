"""Confidence-aware synchronization helpers."""

from typing import Sequence

from .constants import SYNC_ACTIONS
from .frame import normalize_action


def sync_score(observed: Sequence[str], expected: Sequence[str] = SYNC_ACTIONS) -> float:
    if len(observed) != len(expected) or not expected:
        return 0.0
    matches = 0.0
    for actual, wanted in zip(observed, expected):
        normalized = normalize_action(actual)
        if normalized == wanted:
            matches += 1.0
    return matches / len(expected)


def sync_matches(
    observed: Sequence[str],
    expected: Sequence[str] = SYNC_ACTIONS,
    max_distance: int = 0,
) -> bool:
    if len(observed) != len(expected):
        return False
    distance = sum(normalize_action(actual) != normalize_action(wanted) for actual, wanted in zip(observed, expected))
    return distance <= max_distance
