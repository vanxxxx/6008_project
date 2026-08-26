package com.example.app_drone_decode.domain.model

import kotlin.math.abs

data class CameraCalibration(
    val longitudinalAxisX: Float = 0f,
    val longitudinalAxisY: Float = -1f,
    val yawSign: Float = 1f,
    val targetScale: Float = 1f,
    val calibrated: Boolean = false,
)

enum class MotionReferenceMode {
    /** Legacy v4 behavior: the second phase holds the last stationary reference point. */
    STATIONARY_HOLD,
    /** Physical profile v5: the second phase returns to an A→B centerline which continues moving. */
    MOVING_CENTERLINE_RETURN,
}

data class DecoderProfile(
    val profileId: String = "protocol-v3-centerline-v5",
    val profileName: String = "Protocol v3 moving-centerline motion",
    val profileVersion: Int = 5,
    val protocolVersion: Int = 3,
    val actionMapping: Map<ActionClass, String> = mapOf(
        ActionClass.HOVER to "11",
        ActionClass.FORWARD to "00",
        ActionClass.YAW_LEFT to "01",
        ActionClass.YAW_RIGHT to "10",
    ),
    val syncActions: List<ActionClass> = listOf(
        ActionClass.YAW_RIGHT,
        ActionClass.YAW_LEFT,
        ActionClass.HOVER,
        ActionClass.YAW_RIGHT,
        ActionClass.FORWARD,
        ActionClass.YAW_LEFT,
        ActionClass.FORWARD,
        ActionClass.HOVER,
    ),
    val actionDurationMs: Int = 500,
    /** v5 return phase. It is deliberately distinct from v4's idleDurationMs. */
    val recoveryDurationMs: Int = 500,
    val motionReferenceMode: MotionReferenceMode = MotionReferenceMode.MOVING_CENTERLINE_RETURN,
    /** v4 stationary-hold phase; retained to avoid reinterpreting saved profiles. */
    val idleDurationMs: Int = 500,
    val stableWindowFraction: Float = 0.60f,
    val minimumSamplesPerSlot: Int = 5,
    val highConfidenceThreshold: Float = 0.80f,
    val lowConfidenceThreshold: Float = 0.55f,
    val erasureThreshold: Float = 0.55f,
    val stationaryLinearSpeedPerSec: Float = 0.035f,
    val forwardSpeedThresholdPerSec: Float = 0.08f,
    val leftYawRateDegPerSec: Float = 18f,
    val rightYawRateDegPerSec: Float = -18f,
    val smoothingWindow: Int = 5,
    val outlierRejectionSigma: Float = 2.5f,
    val hysteresisFraction: Float = 0.15f,
    val debounceSamples: Int = 2,
    val roi: NormalizedRect = NormalizedRect(0.10f, 0.10f, 0.90f, 0.90f),
    val trackerImplementationId: String = "adaptive-appearance-opencv",
    val trackerImplementationVersion: Int = 2,
    val classifierImplementationId: String = "centerline-action-return-v3",
    val classifierImplementationVersion: Int = 3,
    val bchProfile: String = "BCH(63,45), t=3, g=0x782CF",
    val calibration: CameraCalibration = CameraCalibration(),
) {
    val secondaryPhaseDurationMs: Int get() = when (motionReferenceMode) {
        MotionReferenceMode.STATIONARY_HOLD -> idleDurationMs
        MotionReferenceMode.MOVING_CENTERLINE_RETURN -> recoveryDurationMs
    }
    val symbolDurationMs: Int get() = actionDurationMs + secondaryPhaseDurationMs

    companion object {
        fun legacyStationaryHoldProfile() = DecoderProfile(
            profileId = "protocol-v3-default",
            profileName = "Protocol v3 stationary-hold motion (legacy)",
            profileVersion = 4,
            recoveryDurationMs = 500,
            motionReferenceMode = MotionReferenceMode.STATIONARY_HOLD,
            classifierImplementationId = "pulse-pause-translation-v2",
            classifierImplementationVersion = 2,
        )
    }
}

data class FunctionSettings(
    val requestedResolution: String = "640x480",
    val requestedFps: Int = 30,
    val torchEnabled: Boolean = false,
    val recordingEnabledByDefault: Boolean = false,
    val recordingLimitMb: Int = 500,
    val retentionDays: Int = 14,
    val sessionNamePrefix: String = "motion-session",
    val retainRawObservations: Boolean = true,
    val exportFormat: String = "JSON",
    val remoteEndpoint: String = "",
    val remoteEnabled: Boolean = false,
    val bluetoothEnabled: Boolean = false,
    val usbEnabled: Boolean = false,
    val replayMode: Boolean = false,
    val knownPayloadBerMode: Boolean = false,
)

data class ProfileValidation(
    val valid: Boolean,
    val errors: List<String>,
    val interoperabilityWarning: String? = null,
)

object DecoderProfileValidator {
    private val allowedCodes = setOf("00", "01", "10", "11")

    fun validate(profile: DecoderProfile): ProfileValidation {
        val errors = buildList {
            if (profile.profileVersion < 1) add("Profile version must be positive")
            if (profile.protocolVersion !in 1..3) add("Unsupported protocol version")
            if (profile.actionMapping.keys != setOf(
                    ActionClass.HOVER,
                    ActionClass.FORWARD,
                    ActionClass.YAW_LEFT,
                    ActionClass.YAW_RIGHT,
                )
            ) add("Mapping must define H, F, L, and R")
            if (profile.actionMapping.values.toSet() != allowedCodes) {
                add("Action codes must be unique and use 00, 01, 10, and 11")
            }
            if (profile.syncActions.size != 8 || profile.syncActions.any { it == ActionClass.UNKNOWN }) {
                add("SYNC must contain exactly eight known actions")
            }
            if (profile.actionDurationMs !in 100..2_000) add("Action duration must be 100–2000 ms")
            if (profile.idleDurationMs !in 0..2_000) add("Idle duration must be 0–2000 ms")
            if (profile.recoveryDurationMs !in 0..2_000) add("Recovery duration must be 0–2000 ms")
            if (profile.symbolDurationMs > 4_000) add("Action plus idle duration must not exceed 4000 ms")
            if (profile.stableWindowFraction !in 0.2f..0.9f) add("Stable window must be 20–90%")
            if (profile.minimumSamplesPerSlot !in 1..60) add("Minimum samples must be 1–60")
            if (profile.erasureThreshold !in 0f..1f) add("Erasure threshold must be 0–1")
            if (profile.highConfidenceThreshold < profile.lowConfidenceThreshold) {
                add("High confidence threshold must not be below the low threshold")
            }
            if (profile.leftYawRateDegPerSec <= 0f || profile.rightYawRateDegPerSec >= 0f) {
                add("Left yaw must be positive and right yaw must be negative")
            }
            if (profile.smoothingWindow !in 2..30) add("Smoothing window must be 2–30")
            if (profile.debounceSamples !in 1..20) add("Debounce samples must be 1–20")
            if (profile.roi.left !in 0f..1f || profile.roi.top !in 0f..1f ||
                profile.roi.right !in 0f..1f || profile.roi.bottom !in 0f..1f ||
                profile.roi.left >= profile.roi.right || profile.roi.top >= profile.roi.bottom
            ) add("ROI must be a valid normalized rectangle")
            if (abs(profile.calibration.longitudinalAxisX) + abs(profile.calibration.longitudinalAxisY) < 0.01f) {
                add("Calibration longitudinal axis cannot be zero")
            }
            if (profile.calibration.yawSign !in setOf(-1f, 1f)) add("Calibration yaw sign must be -1 or 1")
            if (profile.calibration.targetScale <= 0f) add("Calibration target scale must be positive")
        }
        val defaults = DecoderProfile()
        val mappingChanged = profile.protocolVersion != defaults.protocolVersion ||
            profile.actionMapping != defaults.actionMapping || profile.syncActions != defaults.syncActions
        return ProfileValidation(
            valid = errors.isEmpty(),
            errors = errors,
            interoperabilityWarning = if (mappingChanged) {
                "This profile differs from the protocol v3 wire mapping and is incompatible with its reference vector."
            } else {
                null
            },
        )
    }
}
