package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.NormalizedRect
import org.opencv.core.Mat

data class CvFrame(
    val rgba: Mat,
    val timestampNs: Long,
)

data class TrackingResult(
    val visible: Boolean,
    val confidence: Float,
    val boundingBox: NormalizedRect? = null,
    val orientationDeg: Float? = null,
    val sceneVelocityXPerSec: Float? = null,
    val sceneVelocityYPerSec: Float? = null,
    val sceneRotationDegPerSec: Float? = null,
    val sceneScaleRatePerSec: Float? = null,
    val targetAreaFraction: Float? = null,
)

data class ActionLikelihoods(
    val probabilities: Map<ActionClass, Float>,
) {
    val mostLikely: Pair<ActionClass, Float>
        get() = probabilities.maxByOrNull { it.value }?.toPair() ?: (ActionClass.UNKNOWN to 1f)
}

interface TargetTracker {
    fun process(frame: CvFrame): TrackingResult

    /** Clears temporal tracking state when the image source changes or ends. */
    fun reset() = Unit
}

interface ActionClassifier {
    fun classify(history: List<MotionObservation>): ActionLikelihoods
}
