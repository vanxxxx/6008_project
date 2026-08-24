package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.CameraCalibration
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.NormalizedRect
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

class MotionEstimatorTest {
    @Test
    fun velocityAndYawUseCameraTimestamps() {
        val estimator = MotionEstimator()
        val calibration = CameraCalibration(
            longitudinalAxisX = 1f,
            longitudinalAxisY = 0f,
            yawSign = 1f,
            calibrated = true,
        )
        estimator.estimate(visibleTracking(0.4f, 0.5f, 10f), 1_000_000_000L, calibration)
        val result = estimator.estimate(visibleTracking(0.5f, 0.5f, 20f), 1_500_000_000L, calibration)

        assertEquals(0.2f, result.normalizedLinearVelocityPerSec!!, 0.0001f)
        assertEquals(20f, result.yawRateDegPerSec!!, 0.0001f)
    }

    @Test
    fun longDroppedFrameGapDoesNotCreateMotionGuess() {
        val estimator = MotionEstimator()
        val calibration = CameraCalibration(calibrated = true)
        estimator.estimate(visibleTracking(0.4f, 0.5f, 10f), 1_000_000_000L, calibration)
        val result = estimator.estimate(visibleTracking(0.8f, 0.5f, 80f), 2_000_000_000L, calibration)

        assertNull(result.normalizedLinearVelocityPerSec)
        assertNull(result.yawRateDegPerSec)
    }

    @Test
    fun groundReferenceRemovesCameraMotionAndProducesRelativeAcceleration() {
        val estimator = MotionEstimator()
        val calibration = CameraCalibration(
            longitudinalAxisX = 1f,
            longitudinalAxisY = 0f,
            calibrated = true,
        )
        estimator.estimate(visibleTracking(0.40f, 0.5f, 0f, 0.04f, 0f), 1_000_000_000L, calibration)
        val second = estimator.estimate(
            visibleTracking(0.50f, 0.5f, 0f, 0.04f, 0f),
            1_500_000_000L,
            calibration,
        )
        val third = estimator.estimate(
            visibleTracking(0.62f, 0.5f, 0f, 0.04f, 0f),
            2_000_000_000L,
            calibration,
        )

        assertEquals(0.16f, second.relativeVelocityXPerSec!!, 0.0001f)
        assertNull(second.relativeAccelerationXPerSec2)
        assertEquals(0.20f, third.normalizedLinearVelocityPerSec!!, 0.0001f)
        assertEquals(0.08f, third.relativeAccelerationXPerSec2!!, 0.0001f)
    }

    @Test
    fun implausibleContourAngleFlipIsRejected() {
        val estimator = MotionEstimator()
        val calibration = CameraCalibration(calibrated = true)
        estimator.estimate(visibleTracking(0.5f, 0.5f, 0f), 1_000_000_000L, calibration)

        val result = estimator.estimate(visibleTracking(0.5f, 0.5f, 30f), 1_033_000_000L, calibration)

        assertNull(result.yawRateDegPerSec)
    }

    @Test
    fun trackingLossClassifiesAsUnknownNeverBackward() {
        val classifier = ThresholdActionClassifier { DecoderProfile() }
        val observation = MotionEstimator().estimate(
            TrackingResult(visible = false, confidence = 0f),
            1_000_000_000L,
            CameraCalibration(),
        )

        assertEquals(ActionClass.UNKNOWN, classifier.classify(listOf(observation)).mostLikely.first)
    }

    @Test
    fun stationaryIdleIsUnknownRatherThanBackward() {
        val classifier = ThresholdActionClassifier {
            DecoderProfile(debounceSamples = 1, calibration = CameraCalibration(calibrated = true))
        }
        val first = classifiedObservation(0f, 0f, 1_000_000_000L)
        val observation = classifiedObservation(0f, 0f, 1_033_000_000L)

        assertEquals(ActionClass.UNKNOWN, classifier.classify(listOf(first, observation)).mostLikely.first)
    }

    @Test
    fun backwardTranslationRequiresConfiguredDebounceSamples() {
        val classifier = ThresholdActionClassifier {
            DecoderProfile(debounceSamples = 2, calibration = CameraCalibration(calibrated = true))
        }
        val first = classifiedObservation(-0.2f, 0f, 1_000_000_000L)
        val second = classifiedObservation(-0.2f, 0f, 1_033_000_000L)
        val third = classifiedObservation(-0.2f, 0f, 1_066_000_000L)

        assertEquals(ActionClass.UNKNOWN, classifier.classify(listOf(first)).mostLikely.first)
        assertEquals(ActionClass.UNKNOWN, classifier.classify(listOf(first, second)).mostLikely.first)
        assertEquals(ActionClass.HOVER, classifier.classify(listOf(first, second, third)).mostLikely.first)
    }

    private fun visibleTracking(
        x: Float,
        y: Float,
        orientation: Float,
        sceneVelocityX: Float? = null,
        sceneVelocityY: Float? = null,
    ) = TrackingResult(
        visible = true,
        confidence = 0.95f,
        boundingBox = NormalizedRect(x - 0.05f, y - 0.05f, x + 0.05f, y + 0.05f),
        orientationDeg = orientation,
        sceneVelocityXPerSec = sceneVelocityX,
        sceneVelocityYPerSec = sceneVelocityY,
    )

    private fun classifiedObservation(
        velocity: Float,
        yawRate: Float,
        timestampNs: Long = 1_000_000_000L,
    ) = MotionObservation(
        timestampNs = timestampNs,
        visible = true,
        centerX = 0.5f,
        centerY = 0.5f,
        normalizedLinearVelocityPerSec = velocity,
        yawRateDegPerSec = yawRate,
        actionProbabilities = emptyMap(),
        confidence = 1f,
    )
}
