"""Versioned constants for visual motion protocol version 3."""

PROTOCOL_VERSION = 3
BCH_N = 63
BCH_K = 45
BCH_T = 3
BCH_PARITY_BITS = BCH_N - BCH_K
BCH_GENERATOR = 0x782CF
MAX_ERASED_BITS = 6
TRANSPORT_BITS = 64
DATA_ACTION_COUNT = 32
SYNC_ACTIONS = ("R", "L", "H", "R", "F", "L", "F", "H")

ACTION_TO_BITS = {
    "F": (0, 0),
    "L": (0, 1),
    "R": (1, 0),
    "H": (1, 1),
}
BITS_TO_ACTION = {value: key for key, value in ACTION_TO_BITS.items()}

ACTION_ALIASES = {
    "H": "H",
    "HOVER": "H",
    "F": "F",
    "FORWARD": "F",
    "L": "L",
    "YAW_LEFT": "L",
    "R": "R",
    "YAW_RIGHT": "R",
    "?": "?",
    "UNKNOWN": "?",
}
