package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import kotlin.math.abs
import kotlin.math.sqrt

class ThresholdActionClassifier(
    private val profileProvider: () -> DecoderProfile,
) : ActionClassifier {
    private var pendingAction = ActionClass.UNKNOWN
    private var pendingSamples = 0
    private var latchedAction = ActionClass.UNKNOWN

    override fun classify(history: List<MotionObservation>): ActionLikelihoods {
        val latest = history.lastOrNull() ?: return unknown()
        val profile = profileProvider()
        if (!profile.calibration.calibrated) return unknown()
        if (!latest.visible || latest.confidence < maxOf(profile.lowConfidenceThreshold, profile.erasureThreshold)) {
            resetLatch()
            return unknown()
        }
        val velocity = filteredMean(
            history.mapNotNull(MotionObservation::normalizedLinearVelocityPerSec),
            profile.outlierRejectionSigma,
        ) ?: return unknown()
        val lateralVelocity = filteredMean(
            lateralVelocities(history, profile),
            profile.outlierRejectionSigma,
        ) ?: return unknown()

        val forward = velocity >= profile.forwardSpeedThresholdPerSec
        val backward = velocity <= -profile.forwardSpeedThresholdPerSec
        val left = lateralVelocity <= -profile.forwardSpeedThresholdPerSec
        val right = lateralVelocity >= profile.forwardSpeedThresholdPerSec
        val movingSignals = listOf(forward, backward, left, right).count { it }
        val rawAction = when {
            movingSignals > 1 -> ActionClass.UNKNOWN
            backward -> ActionClass.HOVER
            left -> ActionClass.YAW_LEFT
            right -> ActionClass.YAW_RIGHT
            forward -> ActionClass.FORWARD
            matchesRelaxed(latchedAction, velocity, lateralVelocity, profile) -> latchedAction
            else -> ActionClass.UNKNOWN
        }
        if (rawAction != ActionClass.UNKNOWN && rawAction != latchedAction &&
            latest.confidence < profile.highConfidenceThreshold
        ) {
            pendingAction = ActionClass.UNKNOWN
            pendingSamples = 0
            return unknown()
        }
        val action = debounce(rawAction, profile.debounceSamples)
        if (action == ActionClass.UNKNOWN) return unknown()

        val confidence = latest.confidence.coerceIn(profile.lowConfidenceThreshold, 1f)
        val remainder = (1f - confidence) / 4f
        return ActionLikelihoods(ActionClass.entries.associateWith { candidate ->
            if (candidate == action) confidence else remainder
        })
    }

    private fun unknown() = ActionLikelihoods(
        ActionClass.entries.associateWith { if (it == ActionClass.UNKNOWN) 1f else 0f },
    )

    private fun debounce(candidate: ActionClass, requiredSamples: Int): ActionClass {
        if (candidate == ActionClass.UNKNOWN) {
            pendingAction = ActionClass.UNKNOWN
            pendingSamples = 0
            latchedAction = ActionClass.UNKNOWN
            return ActionClass.UNKNOWN
        }
        if (candidate == pendingAction) {
            pendingSamples++
        } else {
            pendingAction = candidate
            pendingSamples = 1
        }
        if (pendingSamples >= requiredSamples.coerceAtLeast(1)) latchedAction = candidate
        return latchedAction
    }

    private fun resetLatch() {
        pendingAction = ActionClass.UNKNOWN
        pendingSamples = 0
        latchedAction = ActionClass.UNKNOWN
    }

    private fun matchesRelaxed(
        action: ActionClass,
        velocity: Float,
        lateralVelocity: Float,
        profile: DecoderProfile,
    ): Boolean {
        val relaxed = (1f - profile.hysteresisFraction).coerceIn(0.5f, 1f)
        return when (action) {
            ActionClass.HOVER -> velocity <= -profile.forwardSpeedThresholdPerSec * relaxed
            ActionClass.FORWARD -> velocity >= profile.forwardSpeedThresholdPerSec * relaxed
            ActionClass.YAW_LEFT -> lateralVelocity <= -profile.forwardSpeedThresholdPerSec * relaxed
            ActionClass.YAW_RIGHT -> lateralVelocity >= profile.forwardSpeedThresholdPerSec * relaxed
            ActionClass.UNKNOWN -> false
        }
    }

    private fun lateralVelocities(history: List<MotionObservation>, profile: DecoderProfile): List<Float> {
        val axisX = profile.calibration.longitudinalAxisX
        val axisY = profile.calibration.longitudinalAxisY
        val axisLength = kotlin.math.hypot(axisX, axisY)
        if (axisLength <= 0.001f) return emptyList()
        return history.zipWithNext().mapNotNull { (prior, current) ->
            val elapsedSec = (current.timestampNs - prior.timestampNs) / 1_000_000_000f
            val priorX = prior.centerX
            val priorY = prior.centerY
            val currentX = current.centerX
            val currentY = current.centerY
            if (!prior.visible || !current.visible || elapsedSec !in 0.001f..0.5f ||
                priorX == null || priorY == null || currentX == null || currentY == null
            ) {
                null
            } else {
                val deltaX = currentX - priorX
                val deltaY = currentY - priorY
                (-deltaX * axisY + deltaY * axisX) / axisLength / elapsedSec
            }
        }
    }

    private fun filteredMean(values: List<Float>, sigma: Float): Float? {
        if (values.isEmpty()) return null
        if (values.size < 3) return values.average().toFloat()
        val mean = values.average().toFloat()
        val standardDeviation = sqrt(values.sumOf { value ->
            val delta = value - mean
            (delta * delta).toDouble()
        } / values.size).toFloat()
        if (standardDeviation < 0.00001f) return mean
        val filtered = values.filter { abs(it - mean) <= standardDeviation * sigma.coerceAtLeast(1f) }
        return if (filtered.isEmpty()) null else filtered.average().toFloat()
    }
}
