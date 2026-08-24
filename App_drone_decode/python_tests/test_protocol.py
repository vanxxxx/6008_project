import json

import pytest

from drone_decode import bch
from drone_decode.constants import ACTION_TO_BITS, BITS_TO_ACTION
from drone_decode.decoder import ProtocolDecoder
from drone_decode.frame import decode_data_actions, encode_data_actions, encode_frame, encode_information
from drone_decode.models import DecodeFailure
from drone_decode import bridge


HELLO_INFO = "101000100100001100101011011000110110001101111"
HELLO_PARITY = "100100101101010011"
HELLO_CODEWORD = HELLO_INFO + HELLO_PARITY
HELLO_PADDED = HELLO_CODEWORD + "0"
HELLO_DATA_ACTIONS = (
    "R", "R", "H", "R", "F", "H", "H", "L",
    "H", "R", "R", "L", "F", "R", "H", "L",
    "F", "R", "H", "L", "F", "L", "L", "H",
    "R", "F", "F", "R", "R", "R", "F", "R",
)


def test_hello_reference_vector():
    information = encode_information(b"Hello", 0)
    assert "".join(map(str, information)) == HELLO_INFO
    codeword = bch.encode(information)
    assert "".join(map(str, codeword)) == HELLO_CODEWORD
    assert encode_data_actions(b"Hello", 0) == HELLO_DATA_ACTIONS
    result = decode_data_actions(HELLO_DATA_ACTIONS, [1.0] * 32)
    assert result.accepted
    assert result.payload_ascii == "Hello"
    assert result.seq == 0
    assert result.corrected_bit_errors == 0


@pytest.mark.parametrize("erased_indices", [(4,), (4, 12, 20)])
def test_complete_action_erasures_inside_bound_are_accepted(erased_indices):
    damaged = list(HELLO_DATA_ACTIONS)
    for index in erased_indices:
        damaged[index] = "?"
    result = decode_data_actions(damaged)
    assert result.accepted
    assert result.payload_ascii == "Hello"
    assert result.erased_bits == len(erased_indices) * 2


def test_four_erased_actions_are_rejected_without_slot_shifting():
    damaged = list(HELLO_DATA_ACTIONS)
    for index in (3, 9, 15, 21):
        damaged[index] = "?"
    result = decode_data_actions(damaged)
    assert not result.accepted
    assert result.rejection_reason == "EXCESSIVE_ERASURES"


def test_one_misclassified_action_is_corrected():
    damaged = list(HELLO_DATA_ACTIONS)
    damaged[0] = "L"
    result = decode_data_actions(damaged)
    assert result.accepted
    assert result.payload_ascii == "Hello"
    assert result.corrected_bit_errors == 1


def test_three_unknown_bit_errors_are_corrected():
    bits = [int(bit) for bit in HELLO_CODEWORD]
    for index in (2, 18, 52):
        bits[index] ^= 1
    result = bch.decode(bits)
    assert "".join(map(str, result.information_bits)) == HELLO_INFO
    assert result.corrected_bit_errors == 3


def test_a_four_bit_error_pattern_is_not_assumed_correctable():
    original = [int(bit) for bit in HELLO_CODEWORD]
    rejected = False
    for fourth in range(3, 63):
        damaged = original.copy()
        for index in (0, 1, 2, fourth):
            damaged[index] ^= 1
        try:
            decoded = bch.decode(damaged)
            rejected = decoded.information_bits != tuple(int(bit) for bit in HELLO_INFO)
        except DecodeFailure:
            rejected = True
        if rejected:
            break
    assert rejected


def test_one_erased_action_plus_two_bit_errors_is_accepted():
    transport: list[int | None] = []
    for action in HELLO_DATA_ACTIONS:
        transport.extend(ACTION_TO_BITS[action])
    transport[8] = None
    transport[9] = None
    transport[20] ^= 1
    transport[44] ^= 1
    decoded = bch.decode(transport[:63])
    assert "".join(map(str, decoded.information_bits)) == HELLO_INFO
    assert decoded.erased_bits == 2
    assert decoded.corrected_bit_errors == 2


def test_sequence_discontinuity_reports_a_possible_missing_frame():
    decoder = ProtocolDecoder()
    decoder.start()
    accepted = []
    for sequence in (0, 1, 3):
        for action in encode_frame(b"Hello", sequence):
            event = decoder.process_slot(action, 1.0)
            if event["result"]:
                accepted.append(event["result"])
    assert [result["seq"] for result in accepted] == [0, 1, 3]
    assert accepted[-1]["possible_missing_frames"] == 1


def test_incomplete_data_slots_are_rejected():
    result = decode_data_actions(HELLO_DATA_ACTIONS[:-1])
    assert not result.accepted
    assert result.rejection_reason == "INCOMPLETE_DATA_SLOTS"


def test_reference_padded_bits_are_exact():
    actions = encode_data_actions(b"Hello", 0)
    bits = "".join("".join(map(str, ACTION_TO_BITS[action])) for action in actions)
    assert bits == HELLO_PADDED


def test_replay_candidate_batch_uses_generic_bch_validation():
    damaged = list(HELLO_DATA_ACTIONS)
    damaged[0] = "L"
    response = json.loads(bridge.decode_replay_candidates_json(json.dumps({
        "candidates": [
            {"actions": list(HELLO_DATA_ACTIONS), "confidences": [1.0] * 32},
            {"actions": damaged, "confidences": [0.8] * 32},
        ]
    })))

    assert response["ok"]
    assert {item["index"] for item in response["results"]} == {0, 1}
    assert {item["payload_ascii"] for item in response["results"]} == {"Hello"}


def test_replay_soft_decision_recovers_one_low_confidence_action():
    damaged = list(HELLO_DATA_ACTIONS)
    damaged[0] = "F"
    damaged[1] = "H"
    damaged[2] = "R"
    assert not decode_data_actions(damaged).accepted
    confidences = [0.9] * 32
    confidences[0] = 0.01
    response = json.loads(bridge.decode_replay_candidates_json(json.dumps({
        "candidates": [{"actions": damaged, "confidences": confidences}]
    })))

    assert response["ok"]
    assert any(item["payload_ascii"] == "Hello" for item in response["results"])


def test_replay_soft_decision_combines_multiple_visual_alternatives():
    damaged = list(HELLO_DATA_ACTIONS)
    alternatives = damaged.copy()
    confidences = [0.9] * 32
    one_bit_alternative = {"H": "F", "F": "H", "L": "R", "R": "L"}
    for position in range(6):
        alternatives[position] = damaged[position]
        damaged[position] = one_bit_alternative[damaged[position]]
        confidences[position] = 0.01 + position * 0.01
    assert not decode_data_actions(damaged).accepted
    response = json.loads(bridge.decode_replay_candidates_json(json.dumps({
        "candidates": [{
            "actions": damaged,
            "confidences": confidences,
            "alternatives": alternatives,
        }]
    })))

    assert response["ok"]
    assert any(item["payload_ascii"] == "Hello" for item in response["results"])
    assert any(item["variantCost"] >= 3 for item in response["results"] if item["payload_ascii"] == "Hello")


def test_nonzero_transport_padding_is_rejected():
    damaged = list(HELLO_DATA_ACTIONS)
    damaged[-1] = "L"  # R=10 to L=11 changes only the fixed transport padding bit.

    result = decode_data_actions(damaged)

    assert not result.accepted
    assert result.rejection_reason == "INVALID_TRANSPORT_PADDING"


def test_unused_payload_bytes_must_be_canonical_zero_padding():
    information = tuple(int(bit) for bit in "00000" + "1" + "0" * 39)
    transport = bch.encode(information) + (0,)
    actions = tuple(BITS_TO_ACTION[transport[index:index + 2]] for index in range(0, 64, 2))

    result = decode_data_actions(actions)

    assert not result.accepted
    assert result.rejection_reason == "NONZERO_DATA_PADDING"
