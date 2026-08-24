"""Small deterministic diagnostics helpers used by host and Android tooling."""


def frames_per_slot(camera_fps: float, slot_duration_ms: int) -> float:
    if camera_fps <= 0 or slot_duration_ms <= 0:
        return 0.0
    return camera_fps * slot_duration_ms / 1000.0


def timing_warning(camera_fps: float, slot_duration_ms: int, minimum_frames: int = 5) -> str | None:
    observed = frames_per_slot(camera_fps, slot_duration_ms)
    if observed < minimum_frames:
        return f"Only {observed:.1f} camera frames are expected per action slot"
    return None
