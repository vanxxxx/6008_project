package com.example.app_drone_decode.vision

import com.example.app_drone_decode.decoder.PythonDecoderFacade.ReplayCandidateDecode
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.SlotObservation
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test

class ReplayCandidateConsensusTest {
    @Test
    fun tiedVoteCountUsesPublicSyncModelMarginRatherThanPayloadContent() {
        val candidates = listOf(
            alignment(0, 8, 0.20f),
            alignment(1, 6, 0.90f),
            alignment(2, 6, 0.70f),
            alignment(3, 6, 0.60f),
            alignment(4, 6, 0.55f),
        )
        val decodes = listOf(
            decode(0, accepted = false, payload = ""),
            decode(1, accepted = true, payload = "41424344"),
            decode(2, accepted = true, payload = "41424344"),
            decode(3, accepted = true, payload = "00112233"),
            decode(4, accepted = true, payload = "00112233"),
        )

        val result = ReplayCandidateConsensus.select(candidates, decodes)

        assertNotNull(result.alignment)
        assertEquals(1_000_000L, result.alignment!!.syncStartTimestampNs)
        assertEquals(2, result.support)
    }

    private fun alignment(index: Int, matches: Int, margin: Float) = ReplaySyncAlignment(
        phaseOffsetMs = index.toFloat(),
        syncStartTimestampNs = index * 1_000_000L,
        syncScore = matches / 8f,
        runnerUpScore = 0f,
        frameSlots = List(40) { slot ->
            val signatureAction = if (slot == 8) {
                listOf(ActionClass.HOVER, ActionClass.FORWARD, ActionClass.YAW_LEFT, ActionClass.YAW_RIGHT)[index % 4]
            } else {
                ActionClass.HOVER
            }
            SlotObservation(slot.toLong(), signatureAction, 1f, false, 4)
        },
        selfCalibrated = true,
        syncMatches = matches,
        modelMargin = margin,
    )

    private fun decode(index: Int, accepted: Boolean, payload: String) = ReplayCandidateDecode(
        index = index,
        accepted = accepted,
        sequence = if (accepted) 0 else null,
        length = if (accepted) 4 else null,
        payloadText = "",
        payloadHex = payload,
        correctedBitErrors = 0,
        rejectionReason = if (accepted) null else "UNCORRECTABLE",
    )
}
