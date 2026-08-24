package com.example.app_drone_decode.vision

import com.example.app_drone_decode.decoder.PythonDecoderFacade.ReplayCandidateDecode
import java.nio.ByteBuffer
import java.nio.charset.CodingErrorAction
import java.nio.charset.StandardCharsets

data class ReplayConsensusSelection(
    val alignment: ReplaySyncAlignment?,
    val support: Int,
    val eligibleAcceptedCandidates: Int,
    val reason: String,
    val payloadGroups: List<ReplayPayloadGroupSummary> = emptyList(),
)

data class ReplayPayloadGroupSummary(
    val payloadHex: String,
    val payloadText: String,
    val sequence: Int?,
    val support: Int,
    val bestSyncMatches: Int,
    val bestMargin: Float,
)

/** Selects a payload supported by independent blind visual models and BCH. */
object ReplayCandidateConsensus {
    fun select(
        candidates: List<ReplaySyncAlignment>,
        decodes: List<ReplayCandidateDecode>,
    ): ReplayConsensusSelection {
        if (candidates.isEmpty()) return failure("No visual replay candidates were produced")
        val maximumSyncMatches = candidates.maxOf(ReplaySyncAlignment::syncMatches)
        val minimumSyncMatches = (maximumSyncMatches - SYNC_MATCH_TOLERANCE).coerceAtLeast(0)
        val eligible = decodes.mapNotNull { decode ->
            val alignment = candidates.getOrNull(decode.index) ?: return@mapNotNull null
            if (!decode.accepted || alignment.syncMatches < minimumSyncMatches) null else Evaluated(alignment, decode)
        }
        if (eligible.isEmpty()) {
            return failure("No BCH-valid candidate remained near the best public-SYNC match")
        }
        val groups = eligible.groupBy { evaluated ->
            PayloadKey(
                payloadHex = evaluated.decode.payloadHex,
                sequence = evaluated.decode.sequence,
                length = evaluated.decode.length,
            )
        }.values.map { rawMembers ->
            val members = rawMembers.groupBy { evaluated ->
                evaluated.alignment.frameSlots.drop(8).joinToString("") { it.action.shortName }
            }.values.map { variants ->
                variants.minBy { it.decode.variantCost }
            }
            PayloadGroup(
                members = members,
                bestMargin = members.maxOf { it.alignment.modelMargin },
                bestSyncMatches = members.maxOf { it.alignment.syncMatches },
                textQuality = textQuality(members.first().decode.payloadHex),
            )
        }.sortedWith(
            compareByDescending<PayloadGroup> { it.members.size + it.textQuality * TEXT_QUALITY_WEIGHT }
                .thenByDescending { it.members.size }
                .thenByDescending { it.bestMargin }
                .thenByDescending { it.bestSyncMatches },
        )
        val winner = groups.first()
        val summaries = groups.take(MAXIMUM_REPORTED_GROUPS).map(::summarize)
        if (winner.members.size < MINIMUM_SUPPORT) {
            return failure("BCH produced no visual consensus across the replay models", eligible.size, summaries)
        }
        val runner = groups.getOrNull(1)
        if (runner != null && runner.members.size == winner.members.size &&
            winner.bestMargin - runner.bestMargin < MINIMUM_TIED_MARGIN
        ) {
            return failure(
                "Two BCH-valid payloads have indistinguishable visual support",
                eligible.size,
                summaries,
            )
        }
        val selected = winner.members.sortedWith(
            compareByDescending<Evaluated> { it.alignment.syncMatches }
                .thenByDescending { it.alignment.modelMargin }
                .thenBy { it.decode.correctedBitErrors },
        ).first()
        return ReplayConsensusSelection(
            alignment = selected.alignment.withDataActions(selected.decode.actions),
            support = winner.members.size,
            eligibleAcceptedCandidates = eligible.size,
            reason = "${winner.members.size} blind visual models agree after BCH validation",
            payloadGroups = summaries,
        )
    }

    private fun summarize(group: PayloadGroup): ReplayPayloadGroupSummary {
        val representative = group.members.first().decode
        return ReplayPayloadGroupSummary(
            payloadHex = representative.payloadHex,
            payloadText = representative.payloadText,
            sequence = representative.sequence,
            support = group.members.size,
            bestSyncMatches = group.bestSyncMatches,
            bestMargin = group.bestMargin,
        )
    }

    private fun ReplaySyncAlignment.withDataActions(actions: List<com.example.app_drone_decode.domain.model.ActionClass>): ReplaySyncAlignment {
        if (actions.size != 32) return this
        val updated = frameSlots.mapIndexed { index, slot ->
            if (index < 8) slot else slot.copy(action = actions[index - 8], erased = false)
        }
        return copy(frameSlots = updated)
    }

    private fun textQuality(payloadHex: String): Int {
        if (payloadHex.isEmpty() || payloadHex.length % 2 != 0) return 0
        val bytes = runCatching {
            ByteArray(payloadHex.length / 2) { index -> payloadHex.substring(index * 2, index * 2 + 2).toInt(16).toByte() }
        }.getOrNull() ?: return 0
        val text = runCatching {
            StandardCharsets.UTF_8.newDecoder()
                .onMalformedInput(CodingErrorAction.REPORT)
                .onUnmappableCharacter(CodingErrorAction.REPORT)
                .decode(ByteBuffer.wrap(bytes)).toString()
        }.getOrNull() ?: return 0
        return if (text.isNotEmpty() && text.none { character ->
                Character.isISOControl(character) && character !in setOf('\n', '\r', '\t')
            }
        ) {
            2
        } else {
            1
        }
    }

    private fun failure(
        reason: String,
        eligible: Int = 0,
        summaries: List<ReplayPayloadGroupSummary> = emptyList(),
    ) = ReplayConsensusSelection(
        alignment = null,
        support = 0,
        eligibleAcceptedCandidates = eligible,
        reason = reason,
        payloadGroups = summaries,
    )

    private data class Evaluated(
        val alignment: ReplaySyncAlignment,
        val decode: ReplayCandidateDecode,
    )
    private data class PayloadKey(val payloadHex: String, val sequence: Int?, val length: Int?)
    private data class PayloadGroup(
        val members: List<Evaluated>,
        val bestMargin: Float,
        val bestSyncMatches: Int,
        val textQuality: Int,
    )

    private const val SYNC_MATCH_TOLERANCE = 4
    private const val MINIMUM_SUPPORT = 2
    private const val MINIMUM_TIED_MARGIN = 0.05f
    private const val MAXIMUM_REPORTED_GROUPS = 5
    private const val TEXT_QUALITY_WEIGHT = 0.75f
}
