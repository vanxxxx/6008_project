"""Systematic BCH(63,45) encoder and bounded error/erasure decoder."""

from functools import lru_cache
from itertools import combinations, product
from typing import Iterable, Sequence

from .constants import (
    BCH_GENERATOR,
    BCH_K,
    BCH_N,
    BCH_PARITY_BITS,
    BCH_T,
    MAX_ERASED_BITS,
)
from .models import BchDecodeResult, DecodeFailure, RejectionReason


def _validate_bits(bits: Sequence[int], expected_length: int) -> tuple[int, ...]:
    if len(bits) != expected_length or any(bit not in (0, 1) for bit in bits):
        raise ValueError(f"Expected {expected_length} binary bits")
    return tuple(int(bit) for bit in bits)


def bits_to_int(bits: Iterable[int]) -> int:
    value = 0
    for bit in bits:
        value = (value << 1) | int(bit)
    return value


def int_to_bits(value: int, width: int) -> tuple[int, ...]:
    return tuple((value >> shift) & 1 for shift in range(width - 1, -1, -1))


def polynomial_remainder(value: int, generator: int = BCH_GENERATOR) -> int:
    generator_degree = generator.bit_length() - 1
    while value and value.bit_length() - 1 >= generator_degree:
        shift = value.bit_length() - 1 - generator_degree
        value ^= generator << shift
    return value


def encode(information_bits: Sequence[int]) -> tuple[int, ...]:
    information = _validate_bits(information_bits, BCH_K)
    shifted = bits_to_int(information) << BCH_PARITY_BITS
    parity = polynomial_remainder(shifted)
    return int_to_bits(shifted | parity, BCH_N)


def is_codeword(bits: Sequence[int]) -> bool:
    try:
        codeword = _validate_bits(bits, BCH_N)
    except ValueError:
        return False
    return polynomial_remainder(bits_to_int(codeword)) == 0


@lru_cache(maxsize=4)
def _syndrome_table(max_weight: int) -> dict[int, int]:
    if not 0 <= max_weight <= BCH_T:
        raise ValueError("Unsupported BCH correction weight")
    table = {0: 0}
    for weight in range(1, max_weight + 1):
        for positions in combinations(range(BCH_N), weight):
            mask = 0
            for position in positions:
                mask |= 1 << (BCH_N - 1 - position)
            syndrome = polynomial_remainder(mask)
            previous = table.get(syndrome)
            if previous is not None and previous != mask:
                raise RuntimeError("BCH syndrome collision inside correction radius")
            table[syndrome] = mask
    return table


def decode(
    received_bits: Sequence[int | None],
    erasure_positions: Sequence[int] | None = None,
) -> BchDecodeResult:
    if len(received_bits) != BCH_N:
        raise DecodeFailure(RejectionReason.INVALID_INPUT, "Expected 63 BCH positions")

    inferred = {index for index, bit in enumerate(received_bits) if bit is None}
    supplied = set(erasure_positions or ())
    erasures = sorted(inferred | supplied)
    if any(position < 0 or position >= BCH_N for position in erasures):
        raise DecodeFailure(RejectionReason.INVALID_INPUT, "Erasure position is outside the BCH word")
    if len(erasures) > MAX_ERASED_BITS:
        raise DecodeFailure(RejectionReason.EXCESSIVE_ERASURES, "More than six BCH bits are erased")

    known_bits: list[int] = []
    for index, bit in enumerate(received_bits):
        if index in erasures:
            known_bits.append(0)
        elif bit in (0, 1):
            known_bits.append(int(bit))
        else:
            raise DecodeFailure(RejectionReason.INVALID_INPUT, "A non-erased bit is not binary")

    base_value = bits_to_int(known_bits)
    allowed_unknown_errors = (2 * BCH_T - len(erasures)) // 2
    syndrome_table = _syndrome_table(allowed_unknown_errors)
    candidates: dict[int, int] = {}

    for assignment in product((0, 1), repeat=len(erasures)):
        assigned_value = base_value
        for position, bit in zip(erasures, assignment):
            if bit:
                assigned_value |= 1 << (BCH_N - 1 - position)
        syndrome = polynomial_remainder(assigned_value)
        error_mask = syndrome_table.get(syndrome)
        if error_mask is None:
            continue
        corrected = assigned_value ^ error_mask
        if polynomial_remainder(corrected) != 0:
            continue
        error_count = error_mask.bit_count()
        candidates[corrected] = min(candidates.get(corrected, BCH_T + 1), error_count)

    if not candidates:
        raise DecodeFailure(RejectionReason.BCH_UNCORRECTABLE, "No valid BCH codeword is within the correction bound")
    if len(candidates) != 1:
        raise DecodeFailure(RejectionReason.BCH_AMBIGUOUS, "Erasure assignments produce multiple valid BCH codewords")

    corrected_value, corrected_errors = next(iter(candidates.items()))
    codeword = int_to_bits(corrected_value, BCH_N)
    return BchDecodeResult(
        codeword_bits=codeword,
        information_bits=codeword[:BCH_K],
        corrected_bit_errors=corrected_errors,
        erased_bits=len(erasures),
    )
