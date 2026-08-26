# AGENTS.md — Webots UAV Motion Encoder and Robot Window

## 1. Mission

Build and maintain the Webots transmitter used by the UAV visible-motion communication experiment.

The transmitter workflow is:

```text
UTF-8 text entered in the Robot Window
-> versioned frame encoder in C
-> BCH(63,45) codeword and H/F/L/R symbols
-> fixed action phase + return-to-moving-centerline phase
-> existing Mavic 2 Pro flight-control targets
```

This project is the transmitter and protocol-inspection UI. It must remain compatible with the Android visual receiver without giving the receiver simulator position, telemetry, hidden labels, network truth, or any other ground-truth shortcut.

Do not describe the system as undetectable, RF-proof, or formally covert. The intended claim is only that the data is represented by visible drone motion rather than an intentional radio or modulated-light data link.

## 2. Scope and instruction precedence

This file applies to everything under `Webot/`, including the controller, protocol module, Robot Window, tests, and worlds.

Follow this precedence:

1. The current user request.
2. This `AGENTS.md` for the Webots transmitter and Web UI integration contract.
3. `../Decode_algorithm.md` for BCH construction, information-field layout, bit ordering, frame length, synchronization sequence, and the clean `"Hello"` bit vector.
4. Existing source behavior when it does not conflict with the above.

The current protocol is **version 3**. Version 3 deliberately overrides only the legacy physical action-to-symbol semantics from `../Decode_algorithm.md`. It does not change the BCH polynomial, systematic bit order, information fields, padding rule, or SYNC characters.

Never silently mix protocol versions. If a requested change affects symbol meaning, SYNC, transmitted bits, timing interpretation, or accepted frames:

- increment `MOTION_PROTOCOL_VERSION`;
- update `motion_protocol.c/.h`;
- update the C reference tests;
- update the Robot Window mapping and displayed version;
- update cache-busting query strings for edited Web assets;
- update the Android receiver profile, tests, and migration behavior;
- document compatibility consequences.

## 3. Project layout

```text
Webot/
  AGENTS.md
  controllers/mavic2pro/
    mavic2pro.c                 # Webots lifecycle, flight control, WWI bridge, action scheduler
    motion_protocol.h           # Versioned protocol types and limits
    motion_protocol.c           # BCH encoder, frame construction, symbol mapping
    tests/motion_protocol_test.c
    Makefile
  plugins/robot_windows/motion_encoder/
    motion_encoder.html         # English Robot Window structure
    motion_encoder.css          # Local UI styling
    motion_encoder.js           # WWI client and rendering; not the authoritative encoder
  worlds/Drone.wbt              # Mavic world using window "motion_encoder"
```

Keep protocol encoding separate from Webots devices and UI code. `motion_protocol.c` must remain independently testable without starting Webots.

## 4. Runtime architecture

Webots serves the custom Robot Window at a URL similar to:

```text
http://localhost:1234/robot_windows/motion_encoder/motion_encoder.html?name=Mavic%202%20PRO
```

The browser page communicates with the controller only through Webots WWI:

```text
Robot Window JavaScript
  -> robotWindow.send(text command)
  -> wb_robot_wwi_receive_text()
  -> C encoder and scheduler
  -> wb_robot_wwi_send_text(JSON)
  -> Robot Window rendering
```

The C implementation is authoritative. JavaScript must not independently implement BCH or construct a competing action sequence. It may convert the input string to UTF-8 hex, validate basic UI ranges, send commands, and render the C response.

Do not add a separate unauthenticated HTTP control server. Use the Webots Robot Window bridge for local simulator control.

## 5. Protocol v3 invariants

Unless the user explicitly requests another versioned protocol revision, preserve the following.

### 5.1 Wire symbol and physical-action mapping

```text
00 = F = FORWARD    = keyboard Up behavior
01 = L = MOVE_LEFT  = keyboard Q behavior
10 = R = MOVE_RIGHT = keyboard E behavior
11 = H = BACKWARD   = keyboard Down behavior
```

`MOTION_ACTION_HOVER` is a legacy C enum identifier for the `H` character. In protocol v3 it means **BACKWARD**, not hover and not idle. Do not infer physical meaning from that legacy identifier.

Idle/position hold is not one of the four symbols. During idle, the controller reports `action: "-"` and applies no horizontal movement target. Never encode idle as H, append it to the 40-symbol frame, or let the receiver treat it as a data symbol or erasure.

### 5.2 Synchronization and frame structure

```text
SYNC = R L H R F L F H

1 frame = 8 SYNC actions + 32 data actions = 40 encoded actions

information = LEN[2:0] || SEQ[1:0] || DATA[39:0]
LEN = 0..5 payload bytes
SEQ = modulo 4
unused DATA bytes = 0x00
```

### 5.3 BCH and transport

```text
BCH(63,45), t = 3, designed distance = 7
generator polynomial g = 0x782CF
primitive polynomial = x^6 + x + 1

45 information bits || 18 parity bits = 63-bit systematic codeword
63-bit codeword || one fixed zero padding bit = 64 transmitted bits
64 bits / 2 = 32 data actions
```

Bits are transmitted left to right, most-significant bit first. Do not reinterpret the fixed transport padding bit as part of the BCH word.

### 5.4 Clean `"Hello"` reference

The clean bit-level reference remains:

```text
payload = 48 65 6C 6C 6F
SEQ = 0

INFO
101000100100001100101011011000110110001101111

PARITY
100100101101010011

BCH CODEWORD
101000100100001100101011011000110110001101111100100101101010011

PADDED
1010001001000011001010110110001101100011011111001001011010100110
```

With the v3 wire mapping, the full 40-action sequence is:

```text
RLHRFLFHRRFRLFFHFRRHLRFHLRFHLHHFRLLRRRLR
```

The first eight characters are SYNC. The remaining 32 characters are the data actions. Any action-mapping change must update this expectation without changing the bit vector unless the BCH/frame format also changes.

## 6. Physical flight-control mapping

The automatic transmitter must reuse the same control targets as the existing keyboard paths in `mavic2pro.c`:

```text
F / Up:
  manual_pitch_target = +k_move_tilt_angle

H / Down:
  manual_pitch_target = -k_move_tilt_angle

L / Q:
  manual_roll_target = -k_qe_tilt_angle

R / E:
  manual_roll_target = +k_qe_tilt_angle
```

Automatic L/R actions must not write `yaw_disturbance`. The left/right arrow keys may continue to provide manual yaw control, but that path is not part of protocol transmission.

During an automatic sequence:

- automatic horizontal targets take precedence over manual horizontal/yaw input;
- altitude controls and the P-key emergency stop remain available;
- in physical profile v5, the horizontal reference advances from captured A to configured B for the entire transmission;
- v5 derives nominal speed from route distance and complete message duration; do not reject a valid route solely because that derived speed exceeds a fixed cap;
- the return phase clears data movement targets and tracks the current advancing centerline point;
- stopping a sequence returns to position hold; it does not cut the motors;
- pressing P cancels the active transmission and performs the existing emergency motor stop.

Do not use simulator ground-truth values to decide which symbol was observed by the Android receiver. GPS and IMU may remain part of the transmitter's existing stabilization controller.

## 7. Timing contract

Every encoded action slot has two phases:

```text
slot wall time = actionDurationMs + secondaryPhaseDurationMs

action phase:
  apply the encoded F/L/R/H movement target

return phase (physical profile v5):
  apply no data movement target and track the current A→B centerline point
```

Current defaults and validation bounds are:

```text
actionDurationMs: default 500, range 100..5000
recoveryDurationMs (v5): default 500, range 0..5000
idleDurationMs (v4):     default 500, range 0..5000
```

At the defaults, one encoded slot takes 1.0 second and one 40-action frame takes 40 seconds.

The Android receiver must be configured for this two-phase transmitter schedule. In v5 it compares action and return windows to remove common centerline velocity. Return must not create an extra symbol, an H symbol, or a shifted slot.

Use Webots simulation time for scheduling. Do not infer action timing from controller-loop iteration counts.

## 8. Robot Window command contract

Commands sent from JavaScript to C are UTF-8 text.

```text
HELLO
STOP
ENCODE|<action_ms>|<idle_ms>|<UTF8_HEX>
START|<action_ms>|<idle_ms>|<UTF8_HEX>
ENCODE_ROUTE|<action_ms>|<return_ms>|<dest_x>|<dest_y>|<UTF8_HEX>
START_ROUTE|<action_ms>|<return_ms>|<dest_x>|<dest_y>|<UTF8_HEX>
```

Meanings:

- `HELLO`: request controller capabilities and the current staged encoding, if any.
- `ENCODE`: validate and encode for preview without starting flight actions.
- `START`: validate, encode, and start the sequence using Webots simulation time.
- `STOP`: cancel the current sequence and return to position hold.
- `ENCODE_ROUTE`: preview physical profile v5 using the current GPS horizontal position as A and the supplied absolute Webots X/Y as B.
- `START_ROUTE`: capture A and current altitude, align to B, then transmit on the advancing centerline.

The current parser accepts the legacy three-field form `ENCODE|<action_ms>|<UTF8_HEX>` and uses the default idle duration. Keep this only as a compatibility path; new UI code must send both durations.

Input is limited to 240 UTF-8 bytes. Messages are split into five-byte payload frames, and SEQ starts at zero and increments modulo four for each generated transmission.

Do not put raw user text into the pipe-delimited command. JavaScript must send uppercase or lowercase hexadecimal UTF-8 bytes so delimiters and Unicode characters cannot corrupt parsing.

## 9. Controller response contract

All controller-to-UI messages are JSON strings with a `type` field.

### 9.1 Ready

```json
{
  "type": "ready",
  "protocolVersion": 3,
  "maxBytes": 240,
  "defaultActionMs": 500,
  "minActionMs": 100,
  "maxActionMs": 5000,
  "defaultIdleMs": 500,
  "minIdleMs": 0,
  "maxIdleMs": 5000,
  "running": false
}
```

Legacy `defaultSlotMs`, `minSlotMs`, and `maxSlotMs` aliases are currently also returned. Do not remove them without a deliberate compatibility cleanup.

### 9.2 Encoding stream

An encoding preview is a message sequence:

```text
encoding-start
encoding-frame (one per frame)
encoding-end
```

`encoding-start` includes byte count, frame count, both durations, cycle duration, total encoded actions, and estimated wall-clock duration.

Each `encoding-frame` includes:

```text
index, seq, length, payloadHex,
informationBits, parityBits, codewordBits,
paddedBits, actions
```

### 9.3 Execution and status

Execution state messages use:

```json
{"type":"execution","state":"running"}
{"type":"execution","state":"complete"}
{"type":"execution","state":"stopped","reason":"user"}
```

Periodic status contains at least:

```text
frameIndex, frameCount, slotIndex, globalSlot, totalSlots,
phase, encodedAction, action, actionName,
slotProgress, phaseProgress, elapsedSeconds, totalSeconds
```

During the v5 return phase:

```text
phase = "return"
encodedAction = the slot's original H/F/L/R symbol
action = "-"
actionName = "Return to moving centerline"
```

Errors use:

```json
{"type":"error","code":"...","message":"..."}
```

Keep response keys stable. When adding fields, prefer backward-compatible additions over renaming existing fields.

## 10. Robot Window requirements

The Robot Window is English-only unless the user explicitly requests localization.

It must show:

- UTF-8 input and byte count;
- independently adjustable action and idle durations;
- a Reset timing action which restores 500 ms / 500 ms;
- run and stop/hold controls;
- connection, running, phase, frame, slot, and time status;
- protocol version and current v3 action-to-bit mapping;
- information bits, parity, 63-bit systematic codeword, padded 64 bits, and 40 action characters for every frame;
- a clear distinction between an encoded H action and idle position hold.

Keep the UI compact, responsive, keyboard-accessible, and usable inside a narrow Webots dock. Preserve visible focus states and semantic labels.

Do not use user-controlled strings with `innerHTML`. Render user data with `textContent` or DOM node APIs.

Whenever `motion_encoder.js` or `motion_encoder.css` changes, update the query-string version in `motion_encoder.html`. Webots' embedded browser can retain old assets aggressively. Also keep the no-cache meta elements. After controller or UI changes, instruct testers to restart the simulation/controller and hard-refresh or reopen the Robot Window.

Do not deploy this Robot Window as a standalone hosted site. It depends on the local Webots WWI bridge.

## 11. Android receiver integration

The Android project is under `../App_drone_decode/` and has its own `AGENTS.md`. Integration work must satisfy both projects' instructions.

For protocol v3:

- configure the receiver profile with `00=F`, `01=L`, `10=R`, `11=H`;
- use the same `R L H R F L F H` SYNC characters;
- preserve the BCH and information-bit reference vector exactly;
- classify H as backward translation, not stationary hover;
- classify L/R as lateral translation, not yaw;
- treat the legacy Android/C enum names as implementation history, not physical semantics;
- preserve one slot per action/return cycle and use the return phase only as a motion reference;
- reject or clearly warn on protocol-profile version mismatch;
- never silently reinterpret a stored v1/v2 profile as v3.

The Android receiver's existing `HOVER`, `YAW_LEFT`, and `YAW_RIGHT` enum names are potentially misleading for v3. Use a versioned mapping/profile or explicit adapter until those names can be migrated safely. Do not let enum names force the wrong classifier behavior.

The receiver must obtain actions only from camera observations. WWI messages, Webots GPS, controller state, and generated action arrays are allowed for transmitter tests and offline fixtures, but must never feed production visual decoding as ground truth.

## 12. Testing and verification

Before handing off a change, run the relevant checks.

### 12.1 Protocol unit test

The mandatory test verifies the unchanged `"Hello"` bit vector and the v3 40-action mapping:

```powershell
$webotsHome = 'D:\Program Files\Webots'
$env:Path = "$webotsHome\msys64\mingw64\bin\cpp;$webotsHome\msys64\usr\bin;$webotsHome\msys64\mingw64\bin;$env:Path"
& "$webotsHome\msys64\mingw64\bin\gcc.exe" `
  -std=c11 -Wall -Wextra -Werror -pedantic `
  motion_protocol.c tests\motion_protocol_test.c `
  -o build\motion_protocol_test.exe
& .\build\motion_protocol_test.exe
```

Expected output:

```text
motion_protocol_test: all tests passed
```

### 12.2 Webots controller build

From `controllers/mavic2pro/`:

```powershell
$webotsHome = 'D:\Program Files\Webots'
$env:Path = "$webotsHome\msys64\mingw64\bin\cpp;$webotsHome\msys64\usr\bin;$webotsHome\msys64\mingw64\bin;$env:Path"
& "$webotsHome\msys64\usr\bin\make.exe" `
  -j2 "WEBOTS_HOME=$webotsHome"
```

### 12.3 Web UI static checks

```powershell
node --check ..\..\plugins\robot_windows\motion_encoder\motion_encoder.js
rg -n "[一-龥]" ..\..\plugins\robot_windows\motion_encoder
```

The second command should return no matches while the UI is English-only.

### 12.4 Runtime checks

At minimum verify:

- the controller starts in `Drone.wbt`;
- the Robot Window reports protocol version 3;
- typing `Hello` produces the reference bit vector and v3 action sequence;
- Reset timing restores 500/500 ms;
- action and idle durations update estimated time;
- F drives the Up/forward path;
- H drives the Down/backward path;
- L drives the Q/left path;
- R drives the E/right path;
- L/R do not change heading through `yaw_disturbance`;
- idle reports `-` and applies position hold;
- Stop returns to position hold;
- P cancels transmission and performs emergency stop.

If a running controller predates the rebuilt executable, restart the simulation before evaluating behavior. Rebuilding the `.exe` does not replace code already loaded into the running controller process.

## 13. Performance and safety rules

- Keep the Webots control loop non-blocking.
- Do not allocate unbounded message, frame, or log buffers.
- Use Webots simulation time for action scheduling.
- Preserve the existing stabilization, motor sign conventions, slew limiting, position hold, altitude control, and emergency stop unless the user explicitly requests a flight-control change.
- Do not execute movement directly from unvalidated browser text.
- Reject malformed durations, malformed hex, oversized payloads, starts during emergency stop, and concurrent starts.
- Do not add credentials, machine-specific paths, or personal data to source files or ordinary logs.
- Keep generated `build/` files and large recordings out of version control.
- Keep source comments and UI text accurate when legacy enum identifiers no longer match physical semantics.

## 14. Change checklist

For protocol, mapping, or timing changes, update all affected surfaces together:

```text
motion_protocol.h version/constants
motion_protocol.c symbol mapping and names
motion_protocol_test.c reference actions
mavic2pro.c scheduler and physical driver mapping
Robot Window labels/version/cache keys
WWI command/response compatibility
Android DecoderProfile mapping/version/migration
Android Python/Kotlin reference tests
integration documentation
```

Do not call the Webots transmitter, Android receiver, or end-to-end integration complete until their protocol versions, mapping, timing windows, and clean reference vectors agree.
