# UAV Visual Motion Decoder

This Android receiver observes a controlled drone target through the rear camera and decodes a four-action visible-motion protocol. It does not use simulator ground truth, flight-controller telemetry, or a network data channel for production decoding.

## Implemented receiver path

```text
CameraX RGBA frames or timestamped frames from a selected video
  -> OpenCV high-contrast green-marker tracker
  -> timestamp-aware position and orientation estimates
  -> HOVER / FORWARD / YAW_LEFT / YAW_RIGHT / UNKNOWN likelihoods
  -> fixed-duration stable-center slot aggregation
  -> Python synchronization state machine
  -> BCH(63,45) error/erasure decoding
  -> accepted plaintext and structured diagnostics
```

Tracking loss is always represented as `UNKNOWN / ??`; it is never converted to backward motion `H / 11`.

Protocol v3 uses the fixed wire mapping `F=00`, `L=01`, `R=10`, and
`H=11`. Thus the opposite pairs `F/H` and `L/R` differ by two bits, while
cross-axis action confusions differ by one bit.

## Application areas

- **Monitor**: camera preview, tracking overlay, motion evidence, slot/synchronization state, accepted and rejected frames, compact bounded live events, recording, selected-video analysis, and session export. Portrait controls use two fixed rows so every action remains visible.
- **Decoder**: versioned action mapping, SYNC, timing, thresholds, filtering, ROI, calibration metadata, profile validation, and profile import/export.
- **Functions**: camera request, privacy and recording controls, logging options, disabled transport entry points, timing metrics, video replay information, and labeled `Hello` reference injection.
- **Logs**: a persistent session list with date/time, duration, event count, source, import, and per-session JSON/CSV export. Selecting a session opens its decode summary, raw streams, diagnostics, and event detail.

Bluetooth, USB/serial, and remote debug clients are intentionally disabled until their permission, lifecycle, discovery, authentication, and detach tests exist. They do not feed the visual decoder.

## Laboratory target

The initial tracker selects the largest sufficiently sized green region. Use a matte, high-saturation green marker under stable lighting, keep it inside the configured ROI, and calibrate the longitudinal axis, yaw sign, scale, and stationary noise for the camera geometry used in the experiment. The default axis is provisional and must not be treated as a physical ground truth.

The default protocol cycle uses 500 ms of action followed by 500 ms of idle. At 30 FPS, each phase contains approximately 15 camera frames and a 40-action frame lasts 40 seconds.

Use **Monitor > Video** to select a recording through the Android system picker. Replay uses sequential `MediaExtractor`/`MediaCodec` decoding, displays sampled frames and progress live, preserves media timestamps, and runs the production visual pipeline. It does not consume embedded telemetry or simulator labels. The active decoder profile and camera calibration also apply to replayed video.

A video must contain the complete configured SYNC and 32 data slots before plaintext can be accepted. With the default 500 ms action plus 500 ms idle cycle, a full protocol frame lasts 40 seconds. Shorter recordings are still useful for tracking and action-classifier diagnostics; the replay result reports the duration limitation instead of presenting the missing frame as a generic decoder failure.

## Build and test on Windows

From this directory in PowerShell:

```powershell
py -3.13 -m pip install -r requirements-dev.txt
py -3.13 -m pytest python_tests
.\gradlew.bat testDebugUnitTest
.\gradlew.bat lintDebug
.\gradlew.bat assembleDebug
```

Run device tests when an Android device or emulator is connected:

```powershell
.\gradlew.bat connectedDebugAndroidTest
```

The debug APK is generated at `app/build/outputs/apk/debug/app-debug.apk`.

## Reference vector

The developer-tools screen can inject the complete protocol v3 `Hello` frame. Injected sessions are labeled and bypass the visual classifier so the Kotlin/Python bridge and protocol engine can be diagnosed independently of the camera. The host-side Python tests cover the clean vector, errors, erasures, the guaranteed `2E + S <= 6` bound, incomplete data, and sequence discontinuity.
