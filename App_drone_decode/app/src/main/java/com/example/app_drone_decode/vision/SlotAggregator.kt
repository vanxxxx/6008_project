package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.MotionReferenceMode
import com.example.app_drone_decode.domain.model.SlotObservation
import kotlin.math.abs
import kotlin.math.hypot

/** Converts camera observations into one fixed symbol per action-plus-secondary-phase cycle. */
class SlotAggregator(profile: DecoderProfile) {
    private var profile = profile
    private var actionDurationNs = profile.actionDurationMs * 1_000_000L
    private var secondaryDurationNs = profile.secondaryPhaseDurationMs * 1_000_000L
    private var symbolDurationNs = profile.symbolDurationMs * 1_000_000L
    private var stableWindowFraction = profile.stableWindowFraction
    private var minimumSamples = profile.minimumSamplesPerSlot
    private var erasureThreshold = profile.erasureThreshold
    private var anchorTimestampNs: Long? = null
    private var activeSlotIndex = 0L
    private val actionSamples = mutableListOf<MotionObservation>()
    private val returnSamples = mutableListOf<MotionObservation>()

    @Synchronized
    fun updateProfile(profile: DecoderProfile) {
        this.profile = profile
        actionDurationNs = profile.actionDurationMs * 1_000_000L
        secondaryDurationNs = profile.secondaryPhaseDurationMs * 1_000_000L
        symbolDurationNs = profile.symbolDurationMs * 1_000_000L
        stableWindowFraction = profile.stableWindowFraction
        minimumSamples = profile.minimumSamplesPerSlot
        erasureThreshold = profile.erasureThreshold
    }

    @Synchronized
    fun reset(anchorTimestampNs: Long? = null) {
        this.anchorTimestampNs = anchorTimestampNs
        activeSlotIndex = 0L
        actionSamples.clear()
        returnSamples.clear()
    }

    @Synchronized
    fun add(observation: MotionObservation): List<SlotObservation> {
        val anchor = anchorTimestampNs ?: observation.timestampNs.also { anchorTimestampNs = it }
        val elapsed = observation.timestampNs - anchor
        if (elapsed < 0L) return emptyList()
        val observedSlot = elapsed / symbolDurationNs
        if (observedSlot < activeSlotIndex) return emptyList()

        val completed = mutableListOf<SlotObservation>()
        if (observedSlot > activeSlotIndex) {
            completed += finalizeSlot(activeSlotIndex)
            for (missingSlot in (activeSlotIndex + 1) until observedSlot) completed += unknownSlot(missingSlot, 0)
            actionSamples.clear()
            returnSamples.clear()
            activeSlotIndex = observedSlot
        }

        val withinCycleNs = elapsed % symbolDurationNs
        when {
            withinCycleNs < actionDurationNs && inStableWindow(withinCycleNs, actionDurationNs) -> actionSamples += observation
            profile.motionReferenceMode == MotionReferenceMode.MOVING_CENTERLINE_RETURN &&
                withinCycleNs >= actionDurationNs && secondaryDurationNs > 0L &&
                inStableWindow(withinCycleNs - actionDurationNs, secondaryDurationNs) -> returnSamples += observation
        }
        return completed
    }

    @Synchronized
    fun flush(): List<SlotObservation> {
        if (anchorTimestampNs == null) return emptyList()
        val completed = listOf(finalizeSlot(activeSlotIndex))
        actionSamples.clear()
        returnSamples.clear()
        anchorTimestampNs = null
        return completed
    }

    private fun inStableWindow(offsetNs: Long, phaseDurationNs: Long): Boolean {
        val fraction = offsetNs.toDouble() / phaseDurationNs.toDouble()
        val halfWindow = stableWindowFraction / 2f
        return fraction >= 0.5 - halfWindow && fraction <= 0.5 + halfWindow
    }

    private fun finalizeSlot(slotIndex: Long): SlotObservation = when (profile.motionReferenceMode) {
        MotionReferenceMode.STATIONARY_HOLD -> finalizeLegacySlot(slotIndex)
        MotionReferenceMode.MOVING_CENTERLINE_RETURN -> finalizeCenterlineSlot(slotIndex)
    }

    private fun finalizeLegacySlot(slotIndex: Long): SlotObservation {
        if (actionSamples.size < minimumSamples || actionSamples.count { it.visible } < minimumSamples) {
            return unknownSlot(slotIndex, actionSamples.size)
        }
        val candidates = listOf(ActionClass.HOVER, ActionClass.FORWARD, ActionClass.YAW_LEFT, ActionClass.YAW_RIGHT)
        val scores = candidates.associateWith { action ->
            actionSamples.map { it.actionProbabilities[action] ?: 0f }.average().toFloat()
        }
        val best = scores.maxByOrNull { it.value } ?: return unknownSlot(slotIndex, actionSamples.size)
        return if (best.value < erasureThreshold) unknownSlot(slotIndex, actionSamples.size) else SlotObservation(
            slotIndex = slotIndex,
            action = best.key,
            confidence = best.value,
            erased = false,
            sampleCount = actionSamples.size,
        )
    }

    private fun finalizeCenterlineSlot(slotIndex: Long): SlotObservation {
        val sampleCount = actionSamples.size + returnSamples.size
        if (actionSamples.size < minimumSamples || returnSamples.size < minimumSamples) return unknownSlot(slotIndex, sampleCount)
        val actionVelocity = medianVelocity(actionSamples) ?: return unknownSlot(slotIndex, sampleCount)
        val returnVelocity = medianVelocity(returnSamples) ?: return unknownSlot(slotIndex, sampleCount)
        val axisX = profile.calibration.longitudinalAxisX
        val axisY = profile.calibration.longitudinalAxisY
        val axisLength = hypot(axisX, axisY)
        if (!profile.calibration.calibrated || axisLength <= 0.001f) return unknownSlot(slotIndex, sampleCount)

        val contrastX = actionVelocity.first - returnVelocity.first
        val contrastY = actionVelocity.second - returnVelocity.second
        val longitudinal = (contrastX * axisX + contrastY * axisY) / axisLength
        val lateral = (-contrastX * axisY + contrastY * axisX) / axisLength
        val threshold = profile.forwardSpeedThresholdPerSec
        val longitudinalActive = abs(longitudinal) >= threshold
        val lateralActive = abs(lateral) >= threshold
        if (longitudinalActive == lateralActive) return unknownSlot(slotIndex, sampleCount)

        val action = when {
            longitudinal >= threshold -> ActionClass.FORWARD
            longitudinal <= -threshold -> ActionClass.HOVER
            lateral <= -threshold -> ActionClass.YAW_LEFT
            lateral >= threshold -> ActionClass.YAW_RIGHT
            else -> ActionClass.UNKNOWN
        }
        if (action == ActionClass.UNKNOWN) return unknownSlot(slotIndex, sampleCount)
        val trackingConfidence = (actionSamples + returnSamples).map { it.confidence }.average().toFloat()
        val signal = maxOf(abs(longitudinal), abs(lateral))
        val confidence = trackingConfidence * (signal / (threshold * 2f)).coerceIn(0f, 1f)
        return if (confidence < erasureThreshold) unknownSlot(slotIndex, sampleCount) else SlotObservation(
            slotIndex = slotIndex,
            action = action,
            confidence = confidence,
            erased = false,
            sampleCount = sampleCount,
        )
    }

    private fun medianVelocity(samples: List<MotionObservation>): Pair<Float, Float>? {
        val valid = samples.filter { it.visible && it.relativeVelocityXPerSec != null && it.relativeVelocityYPerSec != null }
        if (valid.size < minimumSamples) return null
        return median(valid.map { requireNotNull(it.relativeVelocityXPerSec) }) to
            median(valid.map { requireNotNull(it.relativeVelocityYPerSec) })
    }

    private fun median(values: List<Float>): Float {
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) / 2f else sorted[middle]
    }

    private fun unknownSlot(slotIndex: Long, samples: Int) = SlotObservation(
        slotIndex = slotIndex,
        action = ActionClass.UNKNOWN,
        confidence = 0f,
        erased = true,
        sampleCount = samples,
    )
}
