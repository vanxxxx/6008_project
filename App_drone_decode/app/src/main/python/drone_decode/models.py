"""Protocol data models with JSON-compatible conversion helpers."""

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any


class DecoderState(str, Enum):
    IDLE = "IDLE"
    SEARCH_SYNC = "SEARCH_SYNC"
    COLLECT_FRAME = "COLLECT_FRAME"
    BCH_DECODE = "BCH_DECODE"
    ACCEPT_FRAME = "ACCEPT_FRAME"
    REJECT_FRAME = "REJECT_FRAME"
    ERROR = "ERROR"


class RejectionReason(str, Enum):
    INVALID_INPUT = "INVALID_INPUT"
    INCOMPLETE_DATA_SLOTS = "INCOMPLETE_DATA_SLOTS"
    EXCESSIVE_ERASURES = "EXCESSIVE_ERASURES"
    BCH_UNCORRECTABLE = "BCH_UNCORRECTABLE"
    BCH_AMBIGUOUS = "BCH_AMBIGUOUS"
    INVALID_LENGTH = "INVALID_LENGTH"
    INVALID_TRANSPORT_PADDING = "INVALID_TRANSPORT_PADDING"
    NONZERO_DATA_PADDING = "NONZERO_DATA_PADDING"


@dataclass(frozen=True)
class BchDecodeResult:
    codeword_bits: tuple[int, ...]
    information_bits: tuple[int, ...]
    corrected_bit_errors: int
    erased_bits: int


@dataclass(frozen=True)
class FrameDecodeResult:
    accepted: bool
    seq: int | None = None
    length: int | None = None
    payload_hex: str = ""
    payload_ascii: str = ""
    corrected_bit_errors: int = 0
    erased_bits: int = 0
    mean_confidence: float = 0.0
    padding_bit_valid: bool | None = None
    possible_missing_frames: int = 0
    rejection_reason: str | None = None
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class DecodeFailure(ValueError):
    def __init__(self, reason: RejectionReason, message: str):
        super().__init__(message)
        self.reason = reason
