# Visual Motion Communication Decode/Encode Algorithm

## 1. Scope

This document specifies a compact four-action visual motion communication protocol for controlled experiments with a camera-observed drone or mobile robot.

The protocol is designed for:

- short messages;
- fixed-time-slot decoding;
- recovery from missed or uncertain actions;
- correction of a small number of action classification errors;
- frame-level resynchronization;
- simple future extension to larger action alphabets.

It is **not** intended as an evasion or anti-detection mechanism. The design goal is reliable visible motion communication.

---

## 2. Design Summary

The protocol uses four distinguishable actions. Each action represents one 2-bit symbol.

A complete frame contains:

```text
+----------------------+-----------------------------+
| 8 action SYNC word   | BCH-coded data             |
| 8 actions            | 32 actions                 |
+----------------------+-----------------------------+
                                      Total: 40 actions
```

The coded data is:

```text
45 information bits
    |
    v
BCH(63,45), t = 3
    |
    v
63 coded bits
    |
    + one fixed padding bit = 0
    v
64 transmitted bits
    |
    v
32 four-action symbols
```

Therefore:

```text
1 frame = 8 SYNC actions + 32 data actions = 40 actions
```

A frame carries up to five payload bytes.

---

## 3. Action Alphabet

The final protocol v3 four-action mapping is:

| 2-bit symbol | Action | Short name |
|---|---|---|
| `00` | Forward | `F` |
| `01` | Translate left | `L` |
| `10` | Translate right | `R` |
| `11` | Backward translation | `H` (legacy wire name) |

The mapping assigns Hamming distance 2 to opposite action pairs (`F/H` and
`L/R`). Cross-axis confusions differ by one bit, so a typical non-opposite
misclassification contributes one BCH bit error instead of two.

The letters and bit mapping define wire protocol version 3. In physical profile
version 4, `H` no longer means hover and `L/R` no longer mean yaw. The
idle interval is not a fifth symbol and carries no bits.

### Important rule for backward motion and idle

Loss of tracking or an idle interval must **never** be interpreted as `H`.

The receiver must distinguish:

```text
H = target is visible and confidently classified as translating backward
idle = the configured no-action reference interval after a symbol action
? = action cannot be classified / target was missed
```

`?` is an erasure, not a valid action.

---

## 4. Fixed Time Slots

All symbols are transmitted in fixed-duration cycles of length `T`:

```text
T = action_duration A + idle_duration I
```

The physical profile version 4 default is `A = 500 ms` and `I = 500 ms`.
The action is applied only during the first phase; the transmitter then holds
position during the idle phase. The receiver preserves one symbol position per
complete action-plus-idle cycle and uses the idle phase as a local motion baseline.

Example:

```text
| slot 0 | slot 1 | slot 2 | slot 3 | ...
|   R    |   F    |   ?    |   L    | ...
```

The receiver must preserve slot positions.

If one action is missed:

```text
Correct:
R F ? L H ...

Incorrect:
R F L H ...
```

The incorrect behavior would shift all later symbols and make the remainder of the frame undecodable.

Each detected slot should internally be represented as something similar to:

```text
(slot_index, action, confidence)
```

For example:

```text
(12, MOVE_LEFT, 0.94)
(13, UNKNOWN, 0.21)
(14, FORWARD, 0.91)
```

Low-confidence observations should be converted to erasures instead of being guessed.

---

## 5. Frame Format

### 5.1 Synchronization Word

Each frame starts with the following provisional eight-action SYNC word:

```text
R L H R F L F H
```

The SYNC word carries no user data.

Its purposes are:

1. locating the beginning of a frame;
2. establishing slot phase;
3. allowing recovery after a completely missed or rejected frame.

The receiver searches for SYNC only while in the `SEARCH_SYNC` state. Once it enters `COLLECT_FRAME`, a SYNC-like pattern inside payload data must not restart the decoder.

The exact production SYNC word should eventually be selected using measured action-confusion probabilities and low shifted autocorrelation.

---

### 5.2 Information Block

The BCH encoder receives exactly 45 information bits:

```text
LEN[2:0] || SEQ[1:0] || DATA[39:0]
```

Field definitions:

| Field | Size | Description |
|---|---:|---|
| `LEN` | 3 bits | Number of valid payload bytes, `0..5` |
| `SEQ` | 2 bits | Frame sequence number, modulo 4 |
| `DATA` | 40 bits | Up to five payload bytes |

Total:

```text
3 + 2 + 40 = 45 bits
```

For payloads shorter than five bytes, unused bytes are filled with `0x00`.

The decoder returns only the first `LEN` bytes.

### Sequence number behavior

`SEQ` increments modulo four:

```text
00 -> 01 -> 10 -> 11 -> 00 -> ...
```

A discontinuity can indicate a missed frame.

Example:

```text
received SEQ: 00, 01, 11
```

The receiver can infer that frame `10` was not accepted.

Because the counter is only two bits, it is intended for short streams. A future protocol version may increase the sequence field for long-running streams.

---

## 6. BCH Code

The protocol uses a binary systematic:

```text
BCH(63,45), t = 3
```

with designed minimum distance 7.

It can correct up to three unknown bit errors in a 63-bit codeword.

For error-and-erasure decoding, the standard correction bound is:

```text
2E + S <= 6
```

where:

- `E` = number of unknown erroneous bits;
- `S` = number of known erased bit positions.

Because one action represents two bits, one completely missed action contributes:

```text
S = 2
```

Examples:

| Channel damage | BCH guarantee |
|---|---|
| 1 bit error | Correctable |
| 2 bit errors | Correctable |
| 3 bit errors | Correctable |
| 1 erased action = 2 erased bits | Correctable |
| 2 erased actions = 4 erased bits | Correctable |
| 3 erased actions = 6 erased bits | Correctable |
| 1 erased action + 2 bit errors | Correctable (`2*2 + 2 = 6`) |
| 4 erased actions | Not guaranteed |
| 4 unknown bit errors | Not guaranteed |

A cross-axis misclassification creates one bit error. Confusing an action with
its opposite (`F/H` or `L/R`) creates two bit errors, but the protocol v3
mapping assumes those opposite-action confusions are substantially less likely.

---

## 7. BCH Definition and Bit Ordering

For reproducible test vectors in this document, use the binary generator polynomial:

```text
g(x) =
x^18 + x^17 + x^16 + x^15
+ x^9 + x^7 + x^6
+ x^3 + x^2 + x + 1
```

Equivalent low-order coefficient representation:

```text
g = 0x782CF
```

This is a degree-18 generator for a primitive binary BCH(63,45) construction with `t = 3`.

A compatible GF(2^6) primitive polynomial for an algebraic decoder is:

```text
p(x) = x^6 + x + 1
```

### Systematic encoding convention

Interpret the 45-bit information block as the coefficients of `m(x)`, with the leftmost transmitted information bit being the most significant bit.

Compute:

```text
r(x) = [m(x) * x^18] mod g(x)
```

Then:

```text
c(x) = m(x) * x^18 + r(x)
```

The transmitted 63-bit codeword is therefore:

```text
45 information bits || 18 parity bits
```

Bits are transmitted from left to right.

---

## 8. Encoder Algorithm

### Input

```text
payload: byte array, length 0..5
seq: integer 0..3
```

### Output

```text
40 actions
```

### Procedure

1. Compute `LEN = len(payload)`.
2. Pad `payload` with zero bytes to exactly five bytes.
3. Build the 45-bit information block:

   ```text
   LEN(3) || SEQ(2) || DATA(40)
   ```

4. BCH-encode the 45 bits to a systematic 63-bit codeword.
5. Append one fixed zero padding bit:

   ```text
   63 bits || 0
   ```

6. Split the resulting 64 bits into 32 two-bit symbols.
7. Map each symbol to one physical action.
8. Prepend the eight-action SYNC word.
9. Execute each action for `A`, then hold with no action for `I` before the next symbol.

### Encoder pseudocode

```python
def encode_frame(payload, seq):
    assert 0 <= len(payload) <= 5
    assert 0 <= seq <= 3

    length = len(payload)
    data = payload + b"\x00" * (5 - length)

    info_bits = (
        bits(length, 3)
        + bits(seq, 2)
        + bits_from_bytes(data)
    )
    assert len(info_bits) == 45

    codeword = bch_63_45_encode(info_bits)
    assert len(codeword) == 63

    tx_bits = codeword + [0]
    assert len(tx_bits) == 64

    data_actions = []
    for i in range(0, 64, 2):
        symbol = tx_bits[i:i+2]
        data_actions.append(symbol_to_action(symbol))

    return SYNC_ACTIONS + data_actions
```

---

## 9. Receiver State Machine

Recommended receiver states:

```text
SEARCH_SYNC
    |
    v
COLLECT_FRAME
    |
    v
BCH_DECODE
    |
    +---- success ----> ACCEPT_FRAME ----> SEARCH_SYNC
    |
    +---- failure ----> REJECT_FRAME ----> SEARCH_SYNC
```

### 9.1 SEARCH_SYNC

The receiver continuously classifies fixed-duration action slots.

It compares the observed action history with:

```text
R L H R F L F H
```

A practical implementation should use a confidence-weighted or Hamming-distance threshold instead of requiring perfect equality.

Once SYNC is accepted:

```text
data_slot_index = 0
state = COLLECT_FRAME
```

---

### 9.2 COLLECT_FRAME

Collect exactly 32 data action slots.

Each action becomes:

```text
F -> 00
L -> 01
R -> 10
H -> 11
? -> ??
```

Two erased bits are recorded for every unknown action.

After 32 actions:

1. reconstruct 64 bit positions;
2. remove the final padding position;
3. retain 63 BCH positions;
4. pass the codeword plus erasure locations to the BCH decoder.

---

### 9.3 BCH_DECODE

Decode BCH(63,45).

Reject the frame if:

- the decoder reports uncorrectable damage;
- correction does not produce a valid BCH codeword;
- `LEN > 5`.
- the fixed transport padding bit is not zero;
- any unused payload byte after `LEN` is nonzero.

On success, extract:

```text
LEN  = decoded[0:3]
SEQ  = decoded[3:5]
DATA = decoded[5:45]
```

Return:

```text
DATA[0 : LEN bytes]
```

The receiver should also report metadata such as:

```text
sequence number
number of corrected errors
number of erased bits
average classifier confidence
```

---

## 10. Practical Erasure Decoder

Some BCH libraries correct errors but do not expose erasure decoding.

For this short protocol, erasures can still be handled efficiently.

If there are `S <= 6` erased bit positions:

1. enumerate all `2^S` assignments to erased bits;
2. for each assignment, run the ordinary BCH decoder;
3. permit at most:

   ```text
   floor((6 - S) / 2)
   ```

   additional unknown bit errors;
4. retain only valid BCH codewords;
5. accept only if the result is unique.

The maximum intended erasure case is:

```text
S = 6
2^6 = 64 candidates
```

which is small enough for a prototype decoder.

If more than six BCH bits are erased, reject the frame immediately.

The fixed padding bit is not part of the BCH codeword and must not be counted as one of the 63 BCH positions.

---

## 11. Complete `"Hello"` Test Vector

### 11.1 Input

```text
payload = ASCII "Hello"
SEQ     = 0
```

ASCII bytes:

```text
H = 0x48 = 01001000
e = 0x65 = 01100101
l = 0x6C = 01101100
l = 0x6C = 01101100
o = 0x6F = 01101111
```

Payload:

```text
01001000 01100101 01101100 01101100 01101111
```

Because the payload contains five bytes:

```text
LEN = 101
SEQ = 00
```

### 11.2 45-bit information block

```text
LEN | SEQ | DATA
101 | 00  | 0100100001100101011011000110110001101111
```

Concatenated:

```text
101000100100001100101011011000110110001101111
```

Length:

```text
45 bits
```

---

### 11.3 BCH parity

Using the generator polynomial specified above, the 18 parity bits are:

```text
100100101101010011
```

Therefore the complete 63-bit BCH codeword is:

```text
101000100100001100101011011000110110001101111100100101101010011
```

Verification:

```text
codeword mod g(x) = 0
```

---

### 11.4 Add transport padding

Append one fixed zero:

```text
1010001001000011001010110110001101100011011111001001011010100110
```

Length:

```text
64 bits
```

Split into 2-bit symbols:

```text
10 10 00 10 01 00 00 11
00 10 10 11 01 10 00 11
01 10 00 11 01 11 11 00
10 01 01 10 10 10 01 10
```

Map to actions:

```text
R R F R L F F H
F R R H L R F H
L R F H L H H F
R L L R R R L R
```

These are the 32 data actions.

---

### 11.5 Add SYNC

SYNC:

```text
R L H R F L F H
```

Full transmitted frame:

```text
SYNC:
R L H R F L F H

DATA:
R R F R L F F H
F R R H L R F H
L R F H L H H F
R L L R R R L R
```

As a single 40-action sequence:

```text
R L H R F L F H
R R F R L F F H
F R R H L R F H
L R F H L H H F
R L L R R R L R
```

Expected decoder output:

```text
SEQ     = 0
LEN     = 5
PAYLOAD = "Hello"
```

---

## 12. Test Cases

### Test 1 — Clean frame

Input:

```text
full 40-action "Hello" sequence
```

Expected result:

```text
ACCEPT
SEQ = 0
DATA = "Hello"
corrected_errors = 0
```

---

### Test 2 — One erased data action

Replace one data action with `?`.

Example:

```text
... H R R L F R H L ...
            |
            ?
```

One erased action produces two known erased BCH bit positions.

Expected result:

```text
ACCEPT
DATA = "Hello"
```

because:

```text
S = 2
2E + S = 2 <= 6
```

with no additional errors.

---

### Test 3 — Three erased data actions

Replace three data actions with `?`.

This creates:

```text
S = 6
```

Expected result:

```text
ACCEPT
```

provided there are no additional bit errors and all erased positions are inside the BCH codeword.

This is the maximum guaranteed complete-action erasure case for this code.

---

### Test 4 — Four erased data actions

Four erased actions correspond to:

```text
S = 8
```

Expected behavior:

```text
REJECT
```

Recovery is not guaranteed.

The receiver must not shift later symbols or silently guess the missing actions.

---

### Test 5 — One misclassified action

Suppose one transmitted action is classified as another action.

Depending on the two symbols, this creates one or two bit errors.

Expected result:

```text
ACCEPT
DATA = "Hello"
```

because BCH(63,45) corrects up to three unknown bit errors.

---

### Test 6 — Three unknown bit errors

Flip any three BCH bits.

Expected result:

```text
ACCEPT
DATA = "Hello"
```

---

### Test 7 — Four unknown bit errors

Flip four BCH bits.

Expected behavior:

```text
REJECT OR UNCORRECTABLE
```

Correction is not guaranteed outside the BCH bound. The application must never assume that a frame outside the correction bound is valid.

---

### Test 8 — One erased action plus two bit errors

One erased action gives:

```text
S = 2
```

Two additional bit errors give:

```text
E = 2
```

Therefore:

```text
2E + S = 2*2 + 2 = 6
```

Expected result:

```text
ACCEPT
```

---

### Test 9 — Entire frame is missed

Suppose the receiver accepts:

```text
SEQ 00
SEQ 01
SEQ 11
```

Expected result:

```text
SEQ 10 was missed or rejected
```

Decoding of `SEQ 11` remains independent and does not shift because every frame contains its own SYNC word and BCH block.

---

### Test 10 — Tracking loss or idle must not become backward motion

Camera tracking is lost for one action window, or the classifier is sampling the idle window.

Incorrect classifier result:

```text
H
```

Correct classifier result:

```text
?
```

Expected protocol behavior:

```text
record two erasures
preserve slot position
attempt BCH error/erasure decoding
```

---

## 13. Timing

Let:

```text
A = duration of the active motion window
I = duration of the no-action reference window
T = A + I = duration of one transmitted symbol cycle
```

Every frame contains exactly 40 actions, therefore:

```text
frame_duration = 40 * T
```

Examples:

| Action `A` | Idle `I` | Symbol cycle `T` | Time for one five-byte frame |
|---:|---:|---:|---:|
| 0.50 s | 0.50 s | 1.00 s | 40.0 s |
| 0.50 s | 0.25 s | 0.75 s | 30.0 s |
| 0.40 s | 0.20 s | 0.60 s | 24.0 s |
| 0.25 s | 0.25 s | 0.50 s | 20.0 s |

For a 30 FPS camera:

| Window duration | Approx. frames observed per window |
|---:|---:|
| 0.50 s | 15 |
| 0.25 s | 7-8 |
| 0.20 s | 6 |
| 0.10 s | 3 |

A reasonable initial laboratory test point is:

```text
A = 0.5 s
I = 0.5 s
T = 1.0 s
```

which gives approximately 15 camera frames in each action and idle window and:

```text
"Hello" transmission time = 40 seconds
```

After the motion classifier is validated, shorter action and idle windows can be tested.

---

## 14. Decoder Acceptance Rules

A production receiver should accept a frame only if all mandatory checks pass:

```text
1. SYNC detected
2. exactly 32 data slots collected
3. no more erasures than the decoder supports
4. BCH decoding succeeds
5. corrected codeword is a valid BCH codeword
6. LEN is in the range 0..5
7. the fixed transport padding bit is zero
8. all unused DATA bytes after LEN are zero
```

If any check fails:

```text
discard frame
return to SEARCH_SYNC
```

Never delete an unknown slot and continue decoding. Unknown slots must remain explicit erasures.

---

## 15. Recommended Receiver Output

Instead of returning only decoded text, expose structured diagnostics:

```text
{
    "accepted": true,
    "seq": 0,
    "length": 5,
    "payload_hex": "48656c6c6f",
    "payload_ascii": "Hello",
    "corrected_bit_errors": 1,
    "erased_bits": 2,
    "mean_confidence": 0.91
}
```

This makes camera/classifier tuning much easier.

---

## 16. Longer Messages

A frame carries at most five bytes.

Longer messages are divided into independent frames.

Example: 13 bytes

```text
Frame 0: first 5 bytes
Frame 1: next  5 bytes
Frame 2: final 3 bytes
```

Sequence numbers:

```text
00 -> 01 -> 10
```

Each frame receives its own:

```text
SYNC + BCH codeword
```

Therefore loss of one complete frame does not shift the decoder for the following frame.

---

## 17. Implementation Notes

### Classification should prefer erasures over guesses

For each slot, use two confidence thresholds:

```text
high confidence:
    output H/F/L/R

low confidence:
    output ?
```

For a forward-error-corrected protocol, declaring an uncertain symbol as an erasure is often safer than converting an uncertain observation into a confident but incorrect symbol.

### Keep synchronization and payload decoding separate

Do not continuously search for SYNC while a frame is already being collected.

Recommended behavior:

```text
SEARCH_SYNC -> COLLECT_EXACTLY_32_SLOTS -> DECODE -> SEARCH_SYNC
```

This prevents payload data that resembles the SYNC pattern from causing a false restart.

### Measure the action confusion matrix

During calibration, record how often the classifier confuses every pair:

```text
          classified as
          H    F    L    R
actual H
actual F
actual L
actual R
```

Use this matrix to verify the protocol v3 assumption that opposite-action
confusions are rare. The v3 mapping is fixed; any future reassignment requires
a new wire-protocol version and new reference vectors.

### Tune the SYNC sequence empirically

The provisional SYNC sequence is:

```text
R L H R F L F H
```

Before deployment in an experiment, evaluate candidate SYNC words against:

- shifted self-correlation;
- measured action-confusion probabilities;
- false-detection probability;
- tolerance to one erased SYNC action;
- camera frame-rate and slot timing.

---

## 18. Reference Constants

```text
ACTION MAP (PROTOCOL V3)
00 = F
01 = L
10 = R
11 = H

SYNC
R L H R F L F H

INFORMATION BLOCK
LEN:  3 bits
SEQ:  2 bits
DATA: 40 bits
TOTAL: 45 bits

BCH
n = 63
k = 45
t = 3
designed distance = 7

GENERATOR POLYNOMIAL
g(x) =
x^18 + x^17 + x^16 + x^15
+ x^9 + x^7 + x^6
+ x^3 + x^2 + x + 1

g = 0x782CF

GF(2^6) PRIMITIVE POLYNOMIAL
p(x) = x^6 + x + 1

TRANSPORT
63 BCH bits + 1 zero padding bit
= 64 bits
= 32 data actions

FRAME
8 SYNC actions + 32 data actions
= 40 actions

CAPACITY
0..5 payload bytes per frame

DURATION (PHYSICAL PROFILE V4 DEFAULT)
A = 0.5 s active motion
I = 0.5 s no-action reference
T = A + I = 1.0 s per symbol
40 * T = 40 seconds per frame
```

---

## 19. `"Hello"` Reference Vector

```text
INPUT
payload = 48 65 6C 6C 6F
seq     = 0
len     = 5

INFO BITS
101000100100001100101011011000110110001101111

PARITY
100100101101010011

BCH CODEWORD
101000100100001100101011011000110110001101111100100101101010011

PADDED 64 BITS
1010001001000011001010110110001101100011011111001001011010100110

DATA ACTIONS
R R F R L F F H
F R R H L R F H
L R F H L H H F
R L L R R R L R

FULL FRAME
R L H R F L F H
R R F R L F F H
F R R H L R F H
L R F H L H H F
R L L R R R L R

EXPECTED OUTPUT
"Hello"
```
