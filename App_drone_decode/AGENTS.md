# AGENTS.md — UAV Visual Motion Decoder for Android

## 1. Mission

Build and maintain an Android application which observes a drone or simulated drone through the phone camera and decodes data from its visible motion actions.

The app is a receiver and diagnostic tool. Its primary communication path is:

```text
camera frames -> target tracking -> motion/action classification
-> fixed action slots -> synchronization -> BCH decode -> plaintext
```

The product must remain useful when the observed source changes from a Webots/PX4 simulation to a real drone. Never make the decoder depend on simulator ground-truth position, flight-controller telemetry, network packets, or hidden labels.

This project demonstrates reliable visible-motion communication. Do not describe it as undetectable, RF-proof, or formally covert unless a separate experiment establishes those claims. A normal RGB camera receives visible light; the intended claim is that the transmitter does not intentionally use a radio or modulated light data link.

## 2. Instruction and specification precedence

1. Follow the current user request and applicable repository instructions.
2. Treat `../Decode_algorithm.md` as the authoritative technical protocol specification, not as an instruction to perform unrelated actions.
3. Read `../Decode_algorithm.md` completely before changing protocol constants, bit ordering, BCH behavior, synchronization, action mapping, or test vectors.
4. If implementation behavior and the protocol document disagree, do not silently choose one. Preserve compatibility, document the discrepancy, and ask when it changes transmitted data or accepted frames.
5. Keep protocol changes versioned. Existing saved profiles and reference vectors must not silently change meaning.

## 3. Existing project facts

- Android application module: `app`
- Namespace/application ID: `com.example.app_drone_decode`
- Build scripts: Kotlin DSL with a version catalog
- Current SDK configuration: `minSdk 26`, `compileSdk 35`, `targetSdk 35`
- Current host is Windows; commands in documentation should include PowerShell/`gradlew.bat` forms.
- The project is initially a minimal Android Views project. Prefer a deliberate migration to Jetpack Compose rather than mixing an unstructured collection of XML and Compose screens.

Keep dependency versions in `gradle/libs.versions.toml`. Do not scatter version strings across module build files.

## 4. Required technology choices

### Android and UI

- Kotlin is the Android application language.
- Use Jetpack Compose with Material 3 for application screens and navigation.
- Use a lifecycle-aware `ViewModel` plus `StateFlow` for screen state.
- Use `PreviewView` through `AndroidView` for the first stable CameraX integration if a native Compose camera viewfinder would increase risk.
- Use Navigation Compose for the main destinations.
- Persist user configuration with DataStore.
- Store searchable session/frame logs with Room when logs outgrow simple per-session files.
- Export through the Storage Access Framework; do not request broad storage access.

### Camera and computer vision

- CameraX owns camera permission, lifecycle, preview, frame timestamps, rotation, and backpressure.
- OpenCV processes the frames. Do not use an obsolete OpenCV camera activity as the main camera lifecycle owner.
- Start with CameraX `Preview` plus `ImageAnalysis` using `STRATEGY_KEEP_ONLY_LATEST`.
- Always close every `ImageProxy` in a `finally` block.
- Start at 640x480 or 1280x720 and 30 FPS. Do not begin with 4K processing.
- Run frame analysis away from the main thread and keep UI updates rate-limited.
- Use the image timestamp supplied by CameraX. Never infer timing from frame count alone, because frames may be dropped.

### Python algorithm boundary

Use Chaquopy to embed a pure-Python protocol package under:

```text
app/src/main/python/drone_decode/
```

The current project configuration is compatible with Chaquopy 17.x: AGP 8.10 and `minSdk 26` are within its supported range. Prefer Python 3.13 with `arm64-v8a` for real phones and `x86_64` for the emulator unless a tested dependency requires another supported Python version.

Python should own the portions most likely to be changed during research:

- protocol constants and profiles;
- action-to-symbol mapping validation;
- fixed-slot observation aggregation after compact observations are produced;
- synchronization search and decoder state machine;
- BCH(63,45) encoding/decoding and error/erasure handling;
- frame parsing, sequence tracking, plaintext/hex formatting;
- offline reference-vector generation and experiment evaluation.

Kotlin should own latency-sensitive platform work:

- CameraX and permissions;
- YUV/RGBA conversion and OpenCV `Mat` lifetime;
- per-frame target detection/tracking;
- drawing overlays;
- Android persistence, export, API, Bluetooth, and USB integration;
- lifecycle, threading, and UI state.

Do not send full camera frames through the Kotlin/Python bridge at 30 FPS. Kotlin/OpenCV should emit compact observations such as position, orientation, linear/angular velocity, action likelihoods, confidence, visibility, and timestamp. Call Python at slot boundaries or with small batches of observations. This boundary is required for predictable latency and testability.

Keep Python modules free of Android imports. Define a small JSON-compatible or typed bridge and validate every field at the boundary. Python exceptions must become structured decoder errors, not application crashes.

## 5. Proposed source layout

Use package names which reveal architectural responsibility. A suitable layout is:

```text
app/src/main/java/com/example/app_drone_decode/
  app/                 # Application setup and navigation
  ui/monitor/          # Main observation screen
  ui/decoderconfig/    # Protocol and classifier configuration
  ui/functionconfig/   # Camera, storage and transport configuration
  ui/logs/             # Session/log browser and export
  camera/              # CameraX frame source and transforms
  vision/              # OpenCV tracking and action classification
  decoder/             # Kotlin facade and Python bridge
  domain/model/        # Stable domain models
  data/config/         # DataStore repositories
  data/logs/           # Room/session log repositories
  connectivity/        # Remote, Bluetooth and wired transport ports
  diagnostics/         # Timing, FPS and health metrics

app/src/main/python/drone_decode/
  constants.py
  models.py
  bch.py
  sync.py
  frame.py
  decoder.py
  profiles.py
  diagnostics.py

app/src/test/...
app/src/androidTest/...
python_tests/           # Host-side tests for the pure Python package
```

Avoid a single large Activity, ViewModel, analyzer, or Python file. Protocol code must not depend on Compose or CameraX classes.

## 6. Stable domain model

Use explicit states and do not encode important meaning in nullable strings.

```kotlin
enum class ActionClass {
    HOVER,
    FORWARD,
    YAW_LEFT,
    YAW_RIGHT,
    UNKNOWN
}

data class MotionObservation(
    val timestampNs: Long,
    val visible: Boolean,
    val centerX: Float?,
    val centerY: Float?,
    val normalizedLinearVelocity: Float?,
    val yawRate: Float?,
    val actionProbabilities: Map<ActionClass, Float>,
    val confidence: Float
)

data class SlotObservation(
    val slotIndex: Long,
    val action: ActionClass,
    val confidence: Float,
    val erased: Boolean,
    val sampleCount: Int
)
```

Use an explicit decoder state such as:

```text
IDLE / SEARCH_SYNC / COLLECT_FRAME / BCH_DECODE
/ ACCEPT_FRAME / REJECT_FRAME / ERROR
```

`UNKNOWN` is a first-class erasure. Tracking loss, insufficient samples, low confidence, or an ambiguous classification must never become `HOVER`.

## 7. Protocol invariants from Decode_algorithm.md

These are mandatory unless the user explicitly requests a protocol revision:

```text
ACTION MAP
00 = HOVER (H)
01 = FORWARD (F)
11 = YAW_LEFT (L)
10 = YAW_RIGHT (R)

SYNC
R L H R F L F H

FRAME
8 sync actions + 32 data actions = 40 fixed-duration slots

INFORMATION
LEN: 3 bits, SEQ: 2 bits, DATA: 40 bits = 45 bits

FEC
BCH(63,45), t=3, designed distance 7
g(x) = 0x782CF, primitive polynomial x^6 + x + 1

TRANSPORT
63 BCH bits + one fixed zero padding bit = 64 bits = 32 actions

CAPACITY
0..5 payload bytes per frame
```

Required decoder behavior:

- Preserve every fixed slot, including unknown slots.
- Never delete a missed action and shift later actions.
- Search for SYNC only in `SEARCH_SYNC`.
- After synchronization, collect exactly 32 data slots before decoding.
- Each erased action produces two known erased bit positions, except that the fixed transport padding bit is outside the BCH word.
- Enforce the error/erasure bound `2E + S <= 6` for guaranteed correction.
- If the ordinary BCH implementation lacks erasure support, enumerate at most `2^S` assignments for `S <= 6`, retain only valid BCH codewords, and accept only a unique result.
- Reject on failed BCH validation, excessive erasures, invalid `LEN`, incomplete data slots, or ambiguous correction.
- Append plaintext to the accepted-message log only after every mandatory acceptance check passes.
- Track `SEQ` modulo four and report possible missing frames without shifting subsequent frames.

The initial slot duration is `T = 0.5 s`. Make it configurable with safe bounds and a clear warning when the selected camera rate gives too few frames per slot.

## 8. Motion classifier requirements

Implement the visual pipeline behind interfaces so classifiers can be replaced:

```kotlin
interface TargetTracker {
    fun process(frame: CvFrame): TrackingResult
}

interface ActionClassifier {
    fun classify(history: List<MotionObservation>): ActionLikelihoods
}
```

Recommended development order:

1. A high-contrast color or ArUco marker tracker for deterministic laboratory tests.
2. Position and orientation filtering with timestamp-aware velocity estimates.
3. Four-action classification using visible target state, longitudinal speed, and signed yaw rate.
4. Markerless or learned tracking only after the protocol pipeline is verified.

The initial action semantics are:

- `HOVER`: target is visible, confidently tracked, and both linear and angular motion are below calibrated thresholds.
- `FORWARD`: confidently positive longitudinal motion in the calibrated forward direction.
- `YAW_LEFT`: confidently signed left yaw motion.
- `YAW_RIGHT`: confidently signed right yaw motion.
- `UNKNOWN`: no target, too few samples, conflicting evidence, or confidence below threshold.

Do not hard-code screen-left as physical forward or yaw-left. Provide a calibration step which establishes camera orientation, longitudinal image axis, yaw sign, target scale, and stationary noise. When a camera view cannot distinguish forward translation from scale/depth change, expose that limitation instead of guessing.

Slot aggregation must use the stable center of each slot rather than transition edges. Preserve raw per-frame observations for diagnostics, but decode from one classified action or erasure per slot.

## 9. Required screens and adaptive layout

The app must have at least these top-level destinations:

### 9.1 Monitor / 主观测

This is the default destination.

Required content:

- live rear-camera preview;
- optional recording with an obvious recording indicator;
- target bounding box, center, orientation/trajectory overlay, current action, confidence, velocity/yaw estimates, slot progress, and current ROI;
- camera state, effective FPS, target visibility, decoder state, synchronization confidence, current slot index, and frame collection progress;
- raw action stream, for example `R L H ? F ...`;
- raw two-bit/erasure stream, for example `10 11 00 ?? 01 ...`;
- synchronization status and matched SYNC history;
- accepted plaintext, payload hex, sequence number, corrected-error count, erased-bit count, and mean confidence;
- start/pause decoding, reset session, mark event, open configuration, and export-session actions.

Adaptive layout:

- Portrait: camera/overlay on top, diagnostics and scrolling logs below.
- Landscape: camera/overlay on the left, diagnostics and logs in a resizable right panel.
- Preserve decoder/camera state across rotation; do not restart a frame because of a configuration change.
- Use `WindowSizeClass` or equivalent measured constraints. Do not branch only on a hard-coded device model.

Logs must be bounded in memory and auto-scroll only when the user is already at the end. Do not steal scroll position while a user is inspecting older entries.

### 9.2 Decoder configuration / 解码配置

Provide validated controls for:

- mapping `H/F/L/R` to the four unique codes `00/01/10/11`;
- provisional SYNC action sequence;
- slot duration and stable sampling window;
- minimum samples per slot;
- high/low confidence thresholds and erasure threshold;
- stationary linear-speed threshold;
- forward-speed threshold and sign;
- left/right yaw-rate thresholds;
- smoothing window, outlier rejection, hysteresis and debounce parameters;
- ROI and tracker/classifier profile;
- BCH/profile information as read-only for the initial protocol;
- restore defaults, duplicate profile, import/export profile, and validate profile.

Reject duplicate action codes and malformed SYNC words. Display consequences before changing a mapping which invalidates interoperability with saved data or the reference vector.

### 9.3 Function configuration / 功能配置

Provide sections for:

- camera: lens, requested resolution/FPS, torch, focus/exposure behavior, orientation, ROI, calibration;
- recording and privacy: video recording off by default, storage limits, retention, session naming;
- logging: verbosity, raw observation retention, CSV/JSON format, export destination;
- remote debug API: endpoint, transport type, connection state, authentication token, timeout, reconnect policy and test button;
- Bluetooth: disabled-by-default entry, permission/status UI and selectable paired device;
- wired transport: USB/serial entry, device/status UI and explicit connect/disconnect;
- developer tools: synthetic slot injection, replay mode, known-payload BER mode, timing diagnostics.

Remote/Bluetooth/USB transports are for configuration, log transfer, test-vector injection, and diagnostics. They must not silently feed simulator truth into the production visual decoder. Clearly label sessions which use injected data.

### 9.4 Logs / 日志

Provide a session list and detail view with:

- start/end time and configuration profile;
- accepted and rejected frames;
- raw actions and symbol stream;
- decoder transitions and reasons for rejection;
- tracking gaps and confidence statistics;
- decoded plaintext and payload hex;
- export to CSV and structured JSON through the system file picker.

## 10. Visual design: modern, Linear-like, not a clone

Use a compact, quiet, information-dense visual language inspired by modern developer tools such as Linear, without copying proprietary branding or assets.

- Prefer dark neutral surfaces with restrained indigo/violet or cool-blue accents.
- Use thin low-contrast borders, modest 8–12 dp corner radii, and shallow/no shadows.
- Keep spacing on a consistent 4/8 dp grid.
- Use sentence case, concise labels, clear hierarchy, and tabular/monospaced numerals for logs and bit streams.
- Reserve strong colors for semantic state: accepted, warning, rejected, recording, target lost.
- Use subtle transitions; never animate continuously in a way that obscures camera analysis or wastes battery.
- Support light and dark themes, system font scaling, TalkBack labels, sufficient contrast, and touch targets of at least 48 dp.
- Do not rely on color alone for decoder state.

The monitor must prioritize the camera and decoding evidence over decorative cards. Avoid dashboard clutter, gradients, oversized headings, glassmorphism, and excessive rounded containers.

## 11. Configuration and persistence rules

Create a versioned `DecoderProfile`. Include at least:

- profile ID/name/version;
- action mapping and SYNC;
- slot and stable-window timing;
- confidence, velocity, yaw, hysteresis and debounce thresholds;
- camera calibration metadata;
- tracker/classifier implementation ID and version;
- BCH/protocol version.

Validate profiles at creation, import, activation, and Python bridge entry. Migrate old profiles explicitly. Never reinterpret an old field with new units.

Keep units in names where ambiguity is possible, such as `slotDurationMs`, `timestampNs`, `yawRateDegPerSec`, or `normalizedVelocityPerSec`.

## 12. Logs, recording, privacy, and diagnostics

- Camera permission is required; request it at runtime with a clear explanation.
- Do not request microphone permission unless audio recording becomes an explicit feature.
- Do not record video by default. Make recording visible and stoppable.
- Do not put access tokens, raw frames, or personal paths in ordinary logs.
- Bound log size and define retention behavior.
- Use monotonic timestamps for motion calculations and wall-clock timestamps only for human-facing session records.
- Log structured rejection reasons, not only free-form strings.
- Export deterministic UTF-8 CSV/JSON with schema/version metadata.

At minimum log:

```text
timestamp, session, frame, slot, visible, action, symbol,
confidence, position, velocity, yaw_rate, decoder_state,
sync_score, accepted, rejection_reason
```

## 13. Connectivity extension points

Keep connectivity behind a replaceable port such as:

```kotlin
interface DebugTransport {
    val state: StateFlow<TransportState>
    val incoming: Flow<DebugEnvelope>
    suspend fun connect(config: TransportConfig)
    suspend fun send(message: DebugEnvelope)
    suspend fun disconnect()
}
```

Provide implementations in stages:

1. `FakeDebugTransport` for deterministic tests.
2. HTTP/WebSocket debug client for a configured development endpoint.
3. Bluetooth transport only after permission and lifecycle tests exist.
4. USB/serial transport only after device discovery and detach handling exist.

All remote transports are off by default. Never hard-code credentials, addresses, certificates, device IDs, or tokens. Generate or request secrets and store them with Android security APIs. Do not expose a phone-hosted unauthenticated server on all interfaces.

The decoder core must continue to work with every transport disconnected and with the phone in airplane mode.

## 14. Testing requirements

### Pure Python protocol tests

Run the Python decoder independently of Android. The mandatory clean reference is:

```text
payload = 48 65 6C 6C 6F
SEQ = 0
expected plaintext = "Hello"

INFO
101000100100001100101011011000110110001101111

PARITY
100100101101010011

BCH CODEWORD
101000100100001100101011011000110110001101111100100101101010011

PADDED
1010001001000011001010110110001101100011011111001001011010100110
```

Verify all damage cases specified in `../Decode_algorithm.md`, including:

- clean frame;
- one and three erased actions accepted when within the bound;
- four erased actions rejected;
- one misclassified action;
- three bit errors;
- four bit errors not assumed correctable;
- one erased action plus two bit errors;
- complete missed frame detected through sequence discontinuity;
- tracking loss remains `UNKNOWN`, never `HOVER`.

### Kotlin and integration tests

- Unit-test timestamp-aware velocity/yaw estimation and slot boundaries.
- Test that dropped frames preserve real time and do not shift slots.
- Test every decoder state transition and rejection reason.
- Test Kotlin/Python serialization, invalid inputs, exceptions and process restart.
- Test profile validation and migration.
- Test portrait/landscape layouts, rotation during collection, permission denial, camera interruption, background/foreground, and target loss.
- Use prerecorded deterministic video or observation fixtures before live-camera tests.
- Add a debug-only known-payload mode to calculate BER; never show BER in a live unknown-message session as though ground truth were available.

### Build and quality commands

From the project root on Windows, use the relevant subset:

```powershell
.\gradlew.bat testDebugUnitTest
.\gradlew.bat lintDebug
.\gradlew.bat assembleDebug
.\gradlew.bat connectedDebugAndroidTest
py -3.13 -m pytest python_tests
```

If a command cannot run because no device, camera, SDK component, or matching host Python is available, run all unaffected checks and report the exact missing prerequisite.

## 15. Performance and reliability budgets

- Camera preview must remain responsive while decoding.
- Use backpressure, bounded queues and bounded logs. No unbounded frame buffering.
- Avoid allocations in the per-frame hot path where practical.
- Never block the main thread on Python, OpenCV, file I/O, Room, network, Bluetooth or USB.
- Publish diagnostics for analyzed FPS, dropped frames, per-stage latency, Python call latency and queue depth.
- Prefer a deliberate erasure over a low-confidence guess.
- Handle camera interruption, app pause/resume and transport disconnect without corrupting the current persistent session.

For the first milestone, target stable 30 FPS acquisition at 640x480 and one reliable slot decision at `T = 0.5 s`. Optimize only after measuring.

## 16. Implementation sequence

Work in small vertical slices:

1. Enable Compose/Material 3, navigation, theme, and adaptive monitor/config shells.
2. Implement the pure-Python protocol engine and all reference-vector/error-erasure tests.
3. Add Chaquopy and a typed Kotlin/Python facade with contract tests.
4. Add CameraX preview, permission flow and diagnostics with a no-op analyzer.
5. Add OpenCV target tracking and an overlay using prerecorded fixtures first.
6. Add position/orientation filtering, action likelihoods and slot aggregation.
7. Connect slot observations to synchronization, BCH decoding and the monitor logs.
8. Add validated profiles, calibration and DataStore persistence.
9. Add session persistence and SAF export.
10. Add fake, remote API, Bluetooth and USB transport entry points in that order.

Every slice must build and have focused tests before beginning the next. Do not scaffold all features as nonfunctional placeholders and call the application complete.

## 17. Definition of done for the first usable release

The first usable release is complete only when:

- it builds and launches on an `arm64-v8a` Android phone;
- camera permission, preview, rotation and lifecycle behavior work;
- the monitor is usable in portrait and landscape;
- OpenCV continuously tracks a controlled test target;
- target loss produces `UNKNOWN/??`, not `HOVER/00`;
- raw actions, raw symbols, sync state and slot progress are visible;
- the complete 40-action `Hello` vector decodes to `Hello`;
- mandatory BCH error/erasure tests pass;
- accepted and rejected frames include structured diagnostics;
- profiles survive app restart and invalid mappings are rejected;
- a session can be exported as UTF-8 CSV and JSON;
- the decoder works while remote API, Bluetooth and USB are disabled;
- build, unit tests and lint pass, with any device-only limitation documented.

## 18. Working rules for coding agents

- Inspect existing files and preserve user changes before editing.
- Prefer small, reviewable changes; do not rewrite the project without need.
- Keep generated build outputs, SDKs, recordings and large test videos out of version control.
- Do not place machine-specific `local.properties`, secrets, absolute paths or tokens in committed files.
- Pin dependencies and document why a nonstandard library is needed.
- Prefer official AndroidX/OpenCV/Chaquopy APIs over abandoned samples.
- Use fakes at hardware boundaries and deterministic fixtures at algorithm boundaries.
- Keep the receiver operational without a network connection.
- Do not add transmitter/drone-control features unless the user explicitly expands scope.
- When changing protocol behavior, update Python tests, Kotlin bridge tests, profile versioning, UI descriptions and reference documentation together.
- Before handing off, run the relevant build/tests/lint and report both what passed and what could not be exercised.

