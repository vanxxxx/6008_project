"""Frame assembly, action mapping, and mandatory acceptance checks."""

from statistics import fmean
from typing import Iterable, Sequence

from . import bch
from .constants import (
    ACTION_ALIASES,
    ACTION_TO_BITS,
    BITS_TO_ACTION,
    DATA_ACTION_COUNT,
    SYNC_ACTIONS,
)
from .models import DecodeFailure, FrameDecodeResult, RejectionReason


def _integer_bits(value: int, width: int) -> tuple[int, ...]:
    if value < 0 or value >= 1 << width:
        raise ValueError(f"Value does not fit in {width} bits")
    return tuple((value >> shift) & 1 for shift in range(width - 1, -1, -1))


def _bits_to_bytes(bits: Sequence[int]) -> bytes:
    if len(bits) % 8:
        raise ValueError("Byte conversion requires a multiple of eight bits")
    return bytes(bch.bits_to_int(bits[index:index + 8]) for index in range(0, len(bits), 8))


def normalize_action(action: str) -> str:
    normalized = ACTION_ALIASES.get(str(action).strip().upper())
    if normalized is None:
        raise ValueError(f"Unknown action: {action}")
    return normalized


def encode_information(payload: bytes, seq: int) -> tuple[int, ...]:
    if not 0 <= len(payload) <= 5:
        raise ValueError("Payload length must be between zero and five bytes")
    if not 0 <= seq <= 3:
        raise ValueError("Sequence number must be between zero and three")
    padded_payload = payload + bytes(5 - len(payload))
    data_bits = tuple(bit for byte in padded_payload for bit in _integer_bits(byte, 8))
    return _integer_bits(len(payload), 3) + _integer_bits(seq, 2) + data_bits


def encode_data_actions(payload: bytes, seq: int) -> tuple[str, ...]:
    codeword = bch.encode(encode_information(payload, seq))
    transmitted = codeword + (0,)
    return tuple(BITS_TO_ACTION[transmitted[index:index + 2]] for index in range(0, 64, 2))


def encode_frame(payload: bytes, seq: int) -> tuple[str, ...]:
    return SYNC_ACTIONS + encode_data_actions(payload, seq)


def decode_data_actions(
    actions: Sequence[str],
    confidences: Sequence[float] | None = None,
    action_mapping: dict[str, str] | None = None,
) -> FrameDecodeResult:
    if len(actions) != DATA_ACTION_COUNT:
        return FrameDecodeResult(
            accepted=False,
            rejection_reason=RejectionReason.INCOMPLETE_DATA_SLOTS.value,
            diagnostics={"received_data_slots": len(actions), "expected_data_slots": DATA_ACTION_COUNT},
        )

    transport_bits: list[int | None] = []
    mapping = ACTION_TO_BITS if action_mapping is None else {
        key: tuple(int(bit) for bit in value) for key, value in action_mapping.items()
    }
    try:
        for action in actions:
            normalized = normalize_action(action)
            transport_bits.extend((None, None) if normalized == "?" else mapping[normalized])
    except ValueError as error:
        return FrameDecodeResult(
            accepted=False,
            rejection_reason=RejectionReason.INVALID_INPUT.value,
            diagnostics={"message": str(error)},
        )

    bch_positions = transport_bits[:63]
    erased_bits = sum(bit is None for bit in bch_positions)
    padding_bit_valid = transport_bits[63] in (0, None)
    confidence_values = [float(value) for value in (confidences or ()) if 0.0 <= float(value) <= 1.0]
    mean_confidence = fmean(confidence_values) if confidence_values else 0.0

    try:
        decoded = bch.decode(bch_positions)
    except DecodeFailure as error:
        return FrameDecodeResult(
            accepted=False,
            erased_bits=erased_bits,
            mean_confidence=mean_confidence,
            padding_bit_valid=padding_bit_valid,
            rejection_reason=error.reason.value,
            diagnostics={"message": str(error)},
        )

    if not padding_bit_valid:
        return FrameDecodeResult(
            accepted=False,
            corrected_bit_errors=decoded.corrected_bit_errors,
            erased_bits=decoded.erased_bits,
            mean_confidence=mean_confidence,
            padding_bit_valid=False,
            rejection_reason=RejectionReason.INVALID_TRANSPORT_PADDING.value,
        )

    information = decoded.information_bits
    length = bch.bits_to_int(information[:3])
    seq = bch.bits_to_int(information[3:5])
    if length > 5:
        return FrameDecodeResult(
            accepted=False,
            corrected_bit_errors=decoded.corrected_bit_errors,
            erased_bits=decoded.erased_bits,
            mean_confidence=mean_confidence,
            padding_bit_valid=padding_bit_valid,
            rejection_reason=RejectionReason.INVALID_LENGTH.value,
            diagnostics={"decoded_length": length},
        )

    padded_payload = _bits_to_bytes(information[5:45])
    if any(padded_payload[length:]):
        return FrameDecodeResult(
            accepted=False,
            seq=seq,
            length=length,
            corrected_bit_errors=decoded.corrected_bit_errors,
            erased_bits=decoded.erased_bits,
            mean_confidence=mean_confidence,
            padding_bit_valid=True,
            rejection_reason=RejectionReason.NONZERO_DATA_PADDING.value,
            diagnostics={"nonzero_padding_bytes": sum(byte != 0 for byte in padded_payload[length:])},
        )
    payload = padded_payload[:length]
    return FrameDecodeResult(
        accepted=True,
        seq=seq,
        length=length,
        payload_hex=payload.hex().upper(),
        payload_ascii=payload.decode("utf-8", errors="replace"),
        corrected_bit_errors=decoded.corrected_bit_errors,
        erased_bits=decoded.erased_bits,
        mean_confidence=mean_confidence,
        padding_bit_valid=padding_bit_valid,
    )


def actions_to_symbols(
    actions: Iterable[str],
    action_mapping: dict[str, str] | None = None,
) -> tuple[str, ...]:
    symbols = []
    mapping = {key: "".join(map(str, value)) for key, value in ACTION_TO_BITS.items()} \
        if action_mapping is None else action_mapping
    for action in actions:
        normalized = normalize_action(action)
        symbols.append("??" if normalized == "?" else mapping[normalized])
    return tuple(symbols)
