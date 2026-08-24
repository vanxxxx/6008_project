package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.CameraCalibration
import com.example.app_drone_decode.domain.model.MotionObservation
import kotlin.math.abs
import kotlin.math.hypot

class MotionEstimator {
    private var previous: MotionObservation? = null

    fun reset() {
        previous = null
    }

    fun estimate(
        tracking: TrackingResult,
        timestampNs: Long,
        calibration: CameraCalibration,
    ): MotionObservation {
        val centerX = tracking.boundingBox?.centerX
        val centerY = tracking.boundingBox?.centerY
        val prior = previous
        val elapsedSec = prior?.let { (timestampNs - it.timestampNs) / 1_000_000_000f }
        val validDelta = tracking.visible && prior?.visible == true && elapsedSec != null && elapsedSec in 0.001f..0.5f

        val targetVelocity = if (validDelta && centerX != null && centerY != null) {
            val deltaX = centerX - requireNotNull(prior.centerX)
            val deltaY = centerY - requireNotNull(prior.centerY)
            deltaX / elapsedSec to deltaY / elapsedSec
        } else {
            null
        }
        val relativeVelocity = targetVelocity?.let { velocity ->
            val referenceX = tracking.sceneVelocityXPerSec ?: 0f
            val referenceY = tracking.sceneVelocityYPerSec ?: 0f
            (velocity.first - referenceX) to (velocity.second - referenceY)
        }
        val linearVelocity = relativeVelocity?.let { velocity ->
            val axisLength = hypot(calibration.longitudinalAxisX, calibration.longitudinalAxisY)
            if (axisLength > 0.001f) {
                (velocity.first * calibration.longitudinalAxisX + velocity.second * calibration.longitudinalAxisY) /
                    axisLength
            } else {
                null
            }
        }
        val relativeAcceleration = if (validDelta && relativeVelocity != null &&
            prior.relativeVelocityXPerSec != null && prior.relativeVelocityYPerSec != null
        ) {
            ((relativeVelocity.first - prior.relativeVelocityXPerSec) / elapsedSec) to
                ((relativeVelocity.second - prior.relativeVelocityYPerSec) / elapsedSec)
        } else null

        val yawRate = if (validDelta && tracking.orientationDeg != null && prior.orientationDeg != null) {
            var delta = tracking.orientationDeg - prior.orientationDeg
            while (delta > 90f) delta -= 180f
            while (delta < -90f) delta += 180f
            (delta / elapsedSec * calibration.yawSign).takeIf { abs(it) <= MAX_PLAUSIBLE_YAW_RATE_DEG_PER_SEC }
        } else {
            null
        }

        val observation = MotionObservation(
            timestampNs = timestampNs,
            visible = tracking.visible,
            centerX = centerX,
            centerY = centerY,
            normalizedLinearVelocityPerSec = linearVelocity,
            yawRateDegPerSec = yawRate,
            actionProbabilities = mapOf(ActionClass.UNKNOWN to 1f),
            confidence = tracking.confidence,
            boundingBox = tracking.boundingBox,
            orientationDeg = tracking.orientationDeg,
            sceneVelocityXPerSec = tracking.sceneVelocityXPerSec,
            sceneVelocityYPerSec = tracking.sceneVelocityYPerSec,
            sceneRotationDegPerSec = tracking.sceneRotationDegPerSec,
            sceneScaleRatePerSec = tracking.sceneScaleRatePerSec,
            targetAreaFraction = tracking.targetAreaFraction,
            relativeVelocityXPerSec = relativeVelocity?.first,
            relativeVelocityYPerSec = relativeVelocity?.second,
            relativeAccelerationXPerSec2 = relativeAcceleration?.first,
            relativeAccelerationYPerSec2 = relativeAcceleration?.second,
        )
        previous = observation
        return observation
    }

    private companion object {
        const val MAX_PLAUSIBLE_YAW_RATE_DEG_PER_SEC = 180f
    }
}
