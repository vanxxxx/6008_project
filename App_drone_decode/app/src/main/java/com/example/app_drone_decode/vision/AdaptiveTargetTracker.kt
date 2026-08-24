package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.NormalizedRect

/**
 * Prefers the aircraft silhouette and falls back to the deterministic green
 * laboratory marker. A simulation coordinate gizmo can contain a large green
 * arrow, so color alone must not override a concurrently visible aircraft.
 */
class AdaptiveTargetTracker(
    roiProvider: () -> NormalizedRect = { NormalizedRect(0f, 0f, 1f, 1f) },
) : TargetTracker {
    private val markerTracker = GreenMarkerTracker(roiProvider = roiProvider)
    private val aircraftTracker = NeutralAircraftTracker(roiProvider = roiProvider)

    override fun process(frame: CvFrame): TrackingResult {
        val aircraft = aircraftTracker.process(frame)
        if (aircraft.visible) {
            markerTracker.reset()
            return aircraft
        }
        return markerTracker.process(frame)
    }

    override fun reset() {
        markerTracker.reset()
        aircraftTracker.reset()
    }
}
