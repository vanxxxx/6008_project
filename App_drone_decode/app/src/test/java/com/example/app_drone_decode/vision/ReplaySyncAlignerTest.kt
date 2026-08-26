package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.MotionReferenceMode
import com.example.app_drone_decode.domain.model.NormalizedRect
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ReplaySyncAlignerTest {
    private val profile = DecoderProfile(
        actionDurationMs = 100,
        idleDurationMs = 0,
        motionReferenceMode = MotionReferenceMode.STATIONARY_HOLD,
        stableWindowFraction = 0.6f,
        minimumSamplesPerSlot = 3,
        erasureThreshold = 0.55f,
    )
    private val frame = profile.syncActions + List(32) { index ->
        listOf(ActionClass.HOVER, ActionClass.FORWARD, ActionClass.YAW_LEFT, ActionClass.YAW_RIGHT)[index % 4]
    }

    @Test
    fun findsSyncAfterLeadingPartialSlotAndIdleVideo() {
        val syncStartNs = 235_000_000L
        val observations = observations(syncStartNs, 0L, 4_400_000_000L)

        val result = ReplaySyncAligner(profile).search(observations)

        val alignment = assertNotNull(result.alignment).let { result.alignment!! }
        assertTrue(kotlin.math.abs(syncStartNs - alignment.syncStartTimestampNs) < 100_000_000L)
        assertEquals(frame, alignment.frameSlots.map { it.action })
        assertTrue(alignment.syncScore > 0.9f)
    }

    @Test
    fun timestampsNeedNotStartAtZeroOrAtAnActionBoundary() {
        val videoStartNs = 9_017_000_000L
        val syncStartNs = videoStartNs + 318_000_000L
        val observations = observations(syncStartNs, videoStartNs, videoStartNs + 4_450_000_000L)

        val result = ReplaySyncAligner(profile).search(observations)
        val alignment = result.alignment

        assertNotNull("${result.reason}; score=${result.bestScore}", alignment)
        assertTrue(kotlin.math.abs(syncStartNs - alignment!!.syncStartTimestampNs) < 100_000_000L)
    }

    @Test
    fun selfCalibratesFromPublicSyncWithoutClassifiedActions() {
        val syncStartNs = 0L
        val observations = generateSequence(0L) { it + 5_000_000L }
            .takeWhile { it < 4_000_000_000L }
            .map { timestamp ->
                val slot = ((timestamp - syncStartNs) / 100_000_000L).toInt()
                val action = if (timestamp >= syncStartNs && slot in frame.indices) {
                    frame[slot]
                } else {
                    ActionClass.HOVER
                }
                MotionObservation(
                    timestampNs = timestamp,
                    visible = true,
                    centerX = 0.5f,
                    centerY = 0.5f,
                    normalizedLinearVelocityPerSec = 0f,
                    yawRateDegPerSec = if (slot >= profile.syncActions.size) {
                        20f + slot * 0.7f
                    } else {
                        when (action) {
                            ActionClass.YAW_RIGHT -> 8f
                            ActionClass.YAW_LEFT -> -8f
                            ActionClass.FORWARD -> 3f
                            else -> 0f
                        }
                    },
                    actionProbabilities = emptyMap(),
                    confidence = 1f,
                )
            }
            .toList()

        val result = ReplaySyncAligner(profile, phaseSteps = 2).search(observations)
        val alignment = result.alignment

        assertNotNull("${result.reason}; score=${result.bestScore}", alignment)
        assertTrue(alignment!!.selfCalibrated)
        assertEquals(profile.syncActions, alignment.frameSlots.take(8).map { it.action })
        assertTrue(alignment.frameSlots.take(8).none { it.erased })
    }

    @Test
    fun pulsePauseSearchUsesIdleAsAZeroMotionReference() {
        val pulseProfile = profile.copy(actionDurationMs = 100, idleDurationMs = 100)
        val syncStartNs = 235_000_000L
        val observations = generateSequence(0L) { it + 10_000_000L }
            .takeWhile { it <= syncStartNs + frame.size * 200_000_000L + 250_000_000L }
            .map { timestamp ->
                val elapsed = timestamp - syncStartNs
                val slot = (elapsed / 200_000_000L).toInt()
                val inAction = elapsed >= 0L && elapsed % 200_000_000L < 100_000_000L
                val action = frame.getOrNull(slot)
                val motion = if (inAction && action != null) {
                    when (action) {
                        ActionClass.HOVER -> 0f to -2f
                        ActionClass.FORWARD -> 0f to 2f
                        ActionClass.YAW_LEFT -> -2f to 0f
                        ActionClass.YAW_RIGHT -> 2f to 0f
                        ActionClass.UNKNOWN -> 0f to 0f
                    }
                } else {
                    0f to 0f
                }
                MotionObservation(
                    timestampNs = timestamp,
                    visible = true,
                    centerX = 0.5f,
                    centerY = 0.5f,
                    normalizedLinearVelocityPerSec = 0f,
                    yawRateDegPerSec = 0f,
                    actionProbabilities = emptyMap(),
                    confidence = 1f,
                    boundingBox = NormalizedRect(0.45f, 0.45f, 0.55f, 0.55f),
                    orientationDeg = 0f,
                    sceneVelocityXPerSec = motion.first,
                    sceneVelocityYPerSec = motion.second,
                    sceneRotationDegPerSec = 0f,
                    sceneScaleRatePerSec = 0f,
                )
            }.toList()

        val result = ReplaySyncAligner(pulseProfile, phaseSteps = 2).search(observations)

        assertTrue(result.candidates.isNotEmpty())
        assertEquals(8, result.candidates.maxOf { it.syncMatches })
        assertTrue(result.candidates.any { candidate -> candidate.frameSlots.map { it.action } == frame })
    }

    private fun observations(syncStartNs: Long, fromNs: Long, untilNs: Long): List<MotionObservation> {
        return generateSequence(fromNs) { it + 5_000_000L }
            .takeWhile { it <= untilNs }
            .map { timestamp ->
                val slot = ((timestamp - syncStartNs) / 100_000_000L).toInt()
                val action = if (timestamp < syncStartNs || slot !in frame.indices) {
                    ActionClass.HOVER
                } else {
                    frame[slot]
                }
                MotionObservation(
                    timestampNs = timestamp,
                    visible = true,
                    centerX = 0.5f,
                    centerY = 0.5f,
                    normalizedLinearVelocityPerSec = 0f,
                    yawRateDegPerSec = 0f,
                    actionProbabilities = ActionClass.entries.associateWith { candidate ->
                        if (candidate == action) 1f else 0f
                    },
                    confidence = 1f,
                )
            }
            .toList()
    }
}
