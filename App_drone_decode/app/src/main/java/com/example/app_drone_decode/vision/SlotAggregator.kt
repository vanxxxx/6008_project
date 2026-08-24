package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.SlotObservation

class SlotAggregator(profile: DecoderProfile) {
    private var actionDurationNs = profile.actionDurationMs * 1_000_000L
    private var symbolDurationNs = profile.symbolDurationMs * 1_000_000L
    private var stableWindowFraction = profile.stableWindowFraction
    private var minimumSamples = profile.minimumSamplesPerSlot
    private var erasureThreshold = profile.erasureThreshold
    private var anchorTimestampNs: Long? = null
    private var activeSlotIndex = 0L
    private val stableSamples = mutableListOf<MotionObservation>()

    @Synchronized
    fun updateProfile(profile: DecoderProfile) {
        actionDurationNs = profile.actionDurationMs * 1_000_000L
        symbolDurationNs = profile.symbolDurationMs * 1_000_000L
        stableWindowFraction = profile.stableWindowFraction
        minimumSamples = profile.minimumSamplesPerSlot
        erasureThreshold = profile.erasureThreshold
    }

    @Synchronized
    fun reset(anchorTimestampNs: Long? = null) {
        this.anchorTimestampNs = anchorTimestampNs
        activeSlotIndex = 0L
        stableSamples.clear()
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
            for (missingSlot in (activeSlotIndex + 1) until observedSlot) {
                completed += unknownSlot(missingSlot, 0)
            }
            stableSamples.clear()
            activeSlotIndex = observedSlot
        }

        val withinCycleNs = elapsed % symbolDurationNs
        if (withinCycleNs >= actionDurationNs) return completed
        val withinAction = withinCycleNs.toDouble() / actionDurationNs.toDouble()
        val halfWindow = stableWindowFraction / 2f
        if (withinAction >= 0.5 - halfWindow && withinAction <= 0.5 + halfWindow) {
            stableSamples += observation
        }
        return completed
    }

    /** Finalizes the active slot when a finite replay source reaches its end. */
    @Synchronized
    fun flush(): List<SlotObservation> {
        if (anchorTimestampNs == null) return emptyList()
        val completed = listOf(finalizeSlot(activeSlotIndex))
        stableSamples.clear()
        anchorTimestampNs = null
        return completed
    }

    private fun finalizeSlot(slotIndex: Long): SlotObservation {
        if (stableSamples.size < minimumSamples) return unknownSlot(slotIndex, stableSamples.size)
        val visibleSamples = stableSamples.filter { it.visible }
        if (visibleSamples.size < minimumSamples) return unknownSlot(slotIndex, stableSamples.size)

        val candidates = listOf(
            ActionClass.HOVER,
            ActionClass.FORWARD,
            ActionClass.YAW_LEFT,
            ActionClass.YAW_RIGHT,
        )
        val scores = candidates.associateWith { action ->
            stableSamples.map { it.actionProbabilities[action] ?: 0f }.average().toFloat()
        }
        val best = scores.maxByOrNull { it.value } ?: return unknownSlot(slotIndex, stableSamples.size)
        if (best.value < erasureThreshold) return unknownSlot(slotIndex, stableSamples.size)
        return SlotObservation(
            slotIndex = slotIndex,
            action = best.key,
            confidence = best.value,
            erased = false,
            sampleCount = stableSamples.size,
        )
    }

    private fun unknownSlot(slotIndex: Long, samples: Int) = SlotObservation(
        slotIndex = slotIndex,
        action = ActionClass.UNKNOWN,
        confidence = 0f,
        erased = true,
        sampleCount = samples,
    )
}
