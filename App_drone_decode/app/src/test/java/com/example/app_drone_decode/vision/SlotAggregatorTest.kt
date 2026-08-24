package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SlotAggregatorTest {
    private val profile = DecoderProfile(
        actionDurationMs = 100,
        idleDurationMs = 0,
        stableWindowFraction = 0.6f,
        minimumSamplesPerSlot = 1,
        erasureThreshold = 0.55f,
    )

    @Test
    fun stableSlotCenterProducesOneAction() {
        val aggregator = SlotAggregator(profile)
        aggregator.add(observation(0L, ActionClass.HOVER))
        aggregator.add(observation(50_000_000L, ActionClass.FORWARD))
        val result = aggregator.add(observation(100_000_000L, ActionClass.YAW_LEFT))

        assertEquals(1, result.size)
        assertEquals(0L, result.single().slotIndex)
        assertEquals(ActionClass.FORWARD, result.single().action)
        assertFalse(result.single().erased)
    }

    @Test
    fun droppedFramesPreserveMissingSlotPositions() {
        val aggregator = SlotAggregator(profile)
        aggregator.add(observation(0L, ActionClass.HOVER))
        aggregator.add(observation(50_000_000L, ActionClass.HOVER))
        val result = aggregator.add(observation(350_000_000L, ActionClass.HOVER))

        assertEquals(listOf(0L, 1L, 2L), result.map { it.slotIndex })
        assertEquals(ActionClass.HOVER, result.first().action)
        assertTrue(result[1].erased)
        assertTrue(result[2].erased)
    }

    @Test
    fun trackingLossRemainsExplicitErasure() {
        val aggregator = SlotAggregator(profile)
        aggregator.add(observation(0L, ActionClass.UNKNOWN, visible = false))
        aggregator.add(observation(50_000_000L, ActionClass.UNKNOWN, visible = false))
        val result = aggregator.add(observation(100_000_000L, ActionClass.HOVER))

        assertEquals(ActionClass.UNKNOWN, result.single().action)
        assertTrue(result.single().erased)
    }

    @Test
    fun flushFinalizesLastReplaySlot() {
        val aggregator = SlotAggregator(profile)
        aggregator.add(observation(0L, ActionClass.HOVER))
        aggregator.add(observation(50_000_000L, ActionClass.FORWARD))

        val result = aggregator.flush()

        assertEquals(1, result.size)
        assertEquals(0L, result.single().slotIndex)
        assertEquals(ActionClass.FORWARD, result.single().action)
        assertTrue(aggregator.flush().isEmpty())
    }

    @Test
    fun idleWindowIsIgnoredAndDoesNotBecomeBackward() {
        val pulseProfile = profile.copy(actionDurationMs = 100, idleDurationMs = 100)
        val aggregator = SlotAggregator(pulseProfile)
        aggregator.reset(0L)
        aggregator.add(observation(50_000_000L, ActionClass.FORWARD))
        aggregator.add(observation(150_000_000L, ActionClass.HOVER))

        val result = aggregator.add(observation(250_000_000L, ActionClass.YAW_LEFT))

        assertEquals(ActionClass.FORWARD, result.single().action)
        assertFalse(result.single().erased)
    }

    private fun observation(timestampNs: Long, action: ActionClass, visible: Boolean = true) = MotionObservation(
        timestampNs = timestampNs,
        visible = visible,
        centerX = if (visible) 0.5f else null,
        centerY = if (visible) 0.5f else null,
        normalizedLinearVelocityPerSec = if (visible) 0f else null,
        yawRateDegPerSec = if (visible) 0f else null,
        actionProbabilities = ActionClass.entries.associateWith { if (it == action) 1f else 0f },
        confidence = if (visible) 1f else 0f,
    )
}
