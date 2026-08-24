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

        val linearVelocity = if (validDelta && centerX != null && centerY != null) {
            val deltaX = centerX - requireNotNull(prior.centerX)
            val deltaY = centerY - requireNotNull(prior.centerY)
            val axisLength = hypot(calibration.longitudinalAxisX, calibration.longitudinalAxisY)
            if (axisLength > 0.001f) {
                (deltaX * calibration.longitudinalAxisX + deltaY * calibration.longitudinalAxisY) /
                    axisLength / elapsedSec
            } else {
                null
            }
        } else {
            null
        }

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
        )
        previous = observation
        return observation
    }

    private companion object {
        const val MAX_PLAUSIBLE_YAW_RATE_DEG_PER_SEC = 180f
    }
}
