package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.SlotObservation
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * Aligns action-plus-idle recordings without any payload labels. Each model is
 * calibrated exclusively from the public SYNC actions. The idle half of every
 * cycle acts as a local zero-motion reference.
 */
internal class PulsePauseReplayAligner(
    private val profile: DecoderProfile,
    private val requestedPhaseSteps: Int,
) {
    fun search(observations: List<MotionObservation>): ReplaySyncSearchResult {
        val signals = buildSignals(observations)
        if (signals.size < MINIMUM_SIGNAL_SAMPLES) {
            return ReplaySyncSearchResult(null, 0f, "Too few usable pulse/pause observations")
        }
        val cycleNs = profile.symbolDurationMs * 1_000_000L
        val frameDurationNs = FRAME_ACTION_COUNT * cycleNs
        val medianIntervalNs = medianLong(signals.zipWithNext { first, second -> second.timestampNs - first.timestampNs })
            .coerceAtLeast(1L)
        val lastStartNs = signals.last().timestampNs + medianIntervalNs - frameDurationNs
        if (lastStartNs < signals.first().timestampNs) {
            return ReplaySyncSearchResult(null, 0f, "The video does not contain one complete 40-symbol frame")
        }

        val phaseSteps = requestedPhaseSteps.coerceAtLeast(MINIMUM_PHASE_STEPS)
        val stepNs = (cycleNs / phaseSteps).coerceAtLeast(1L)
        val candidates = ArrayList<ModelCandidate>()
        var startNs = signals.first().timestampNs
        while (startNs <= lastStartNs) {
            val rawProfiles = buildProfiles(signals, startNs, cycleNs)
            if (rawProfiles != null) {
                CHANNEL_SETS.forEachIndexed { channelSetIndex, channels ->
                    val profileColumns = IntArray(BIN_COUNT * channels.size) { index ->
                        val phase = index / channels.size
                        phase * SIGNAL_CHANNEL_COUNT + channels[index % channels.size]
                    }
                    val deltaColumns = IntArray(channels.size) { index ->
                        BIN_COUNT * SIGNAL_CHANNEL_COUNT + channels[index]
                    }
                    val actionColumns = IntArray(ACTION_BIN_COUNT * channels.size) { index ->
                        val phase = index / channels.size
                        phase * SIGNAL_CHANNEL_COUNT + channels[index % channels.size]
                    }
                    listOf(
                        profileColumns,
                        deltaColumns,
                        profileColumns + deltaColumns,
                        actionColumns,
                        actionColumns + deltaColumns,
                    ).forEachIndexed { modeIndex, columns ->
                        calibrate(rawProfiles, columns)?.let { calibrated ->
                            if (calibrated.syncMatches >= MINIMUM_CANDIDATE_MATCHES) {
                                candidates += ModelCandidate(
                                    startNs = startNs,
                                    syncMatches = calibrated.syncMatches,
                                    margin = calibrated.margin,
                                    score = calibrated.score,
                                    actions = calibrated.actions,
                                    alternativeActions = calibrated.alternativeActions,
                                    confidences = calibrated.confidences,
                                    sampleCounts = rawProfiles.sampleCounts,
                                    modelOrder = channelSetIndex * 5 + modeIndex,
                                )
                            }
                        }
                    }
                }
            }
            startNs += stepNs
        }

        if (candidates.isEmpty()) {
            return ReplaySyncSearchResult(null, 0f, "No visually separable pulse/pause SYNC model was found")
        }
        val retained = candidates
            .sortedWith(
                compareByDescending<ModelCandidate> { it.syncMatches }
                    .thenByDescending { it.margin }
                    .thenBy { it.startNs }
                    .thenBy { it.modelOrder },
            )
            .take(MAXIMUM_REPLAY_CANDIDATES)
            .map { it.toAlignment(signals.first().timestampNs) }
        val best = retained.first()
        return ReplaySyncSearchResult(
            alignment = best,
            bestScore = best.syncScore,
            reason = "Pulse/pause candidates calibrated from the public SYNC word",
            candidates = retained,
        )
    }

    private fun buildSignals(observations: List<MotionObservation>): List<SignalSample> {
        val ordered = observations.sortedBy(MotionObservation::timestampNs)
        if (ordered.size < 3) return emptyList()
        val base = Array(ordered.size) { FloatArray(SOURCE_CHANNEL_COUNT) { Float.NaN } }
        ordered.forEachIndexed { index, observation ->
            base[index][0] = observation.sceneVelocityXPerSec ?: Float.NaN
            base[index][1] = observation.sceneVelocityYPerSec ?: Float.NaN
            base[index][2] = observation.sceneRotationDegPerSec ?: Float.NaN
            base[index][3] = observation.sceneScaleRatePerSec ?: Float.NaN
            base[index][4] = observation.centerX ?: Float.NaN
            base[index][5] = observation.centerY ?: Float.NaN
            base[index][6] = observation.orientationDeg ?: Float.NaN
            val box = observation.boundingBox
            if (box != null) {
                val width = (box.right - box.left).coerceAtLeast(0f)
                val height = (box.bottom - box.top).coerceAtLeast(0f)
                base[index][7] = sqrt(observation.targetAreaFraction ?: (width * height))
                base[index][8] = width
                base[index][9] = height
            }
        }
        fillMissing(base)
        val smooth = smooth(base)
        return ordered.indices.map { index ->
            val priorIndex = (index - 1).coerceAtLeast(0)
            val nextIndex = (index + 1).coerceAtMost(ordered.lastIndex)
            val elapsedSec = ((ordered[nextIndex].timestampNs - ordered[priorIndex].timestampNs) /
                1_000_000_000f).coerceAtLeast(0.001f)
            val values = FloatArray(SIGNAL_CHANNEL_COUNT)
            for (channel in 0 until SCENE_CHANNEL_COUNT) values[channel] = smooth[index][channel]
            for (channel in 0 until SCENE_CHANNEL_COUNT) {
                values[4 + channel] = (smooth[nextIndex][channel] - smooth[priorIndex][channel]) / elapsedSec
            }
            for (channel in 0 until POSE_CHANNEL_COUNT) {
                val sourceChannel = SCENE_CHANNEL_COUNT + channel
                values[8 + channel] = (smooth[nextIndex][sourceChannel] - smooth[priorIndex][sourceChannel]) /
                    elapsedSec
            }
            SignalSample(ordered[index].timestampNs, values)
        }
    }

    private fun fillMissing(rows: Array<FloatArray>) {
        for (column in 0 until SOURCE_CHANNEL_COUNT) {
            val validIndices = rows.indices.filter { rows[it][column].isFinite() }
            if (validIndices.isEmpty()) {
                rows.forEach { it[column] = 0f }
                continue
            }
            val first = validIndices.first()
            val last = validIndices.last()
            for (index in 0 until first) rows[index][column] = rows[first][column]
            for (index in last + 1 until rows.size) rows[index][column] = rows[last][column]
            validIndices.zipWithNext().forEach { (left, right) ->
                val leftValue = rows[left][column]
                val rightValue = rows[right][column]
                for (index in left + 1 until right) {
                    val fraction = (index - left).toFloat() / (right - left)
                    rows[index][column] = leftValue + (rightValue - leftValue) * fraction
                }
            }
        }
    }

    private fun smooth(rows: Array<FloatArray>): Array<FloatArray> = Array(rows.size) { index ->
        FloatArray(SOURCE_CHANNEL_COUNT) { channel ->
            var weighted = 0f
            var total = 0f
            SMOOTHING_WEIGHTS.forEachIndexed { weightIndex, weight ->
                val sourceIndex = (index + weightIndex - SMOOTHING_RADIUS).coerceIn(rows.indices)
                weighted += rows[sourceIndex][channel] * weight
                total += weight
            }
            weighted / total
        }
    }

    private fun buildProfiles(
        signals: List<SignalSample>,
        startNs: Long,
        cycleNs: Long,
    ): RawProfiles? {
        val rows = Array(FRAME_ACTION_COUNT) { FloatArray(PROFILE_WIDTH) }
        val sampleCounts = IntArray(FRAME_ACTION_COUNT)
        for (slot in 0 until FRAME_ACTION_COUNT) {
            val cycleStart = startNs + slot * cycleNs
            val phaseValues = Array(BIN_COUNT) { FloatArray(SIGNAL_CHANNEL_COUNT) }
            for (phase in 0 until BIN_COUNT) {
                val left = cycleStart + cycleNs * phase / BIN_COUNT
                val right = cycleStart + cycleNs * (phase + 1) / BIN_COUNT
                val from = lowerBound(signals, left)
                val until = lowerBound(signals, right)
                if (until - from < MINIMUM_BIN_SAMPLES) return null
                sampleCounts[slot] += until - from
                for (channel in 0 until SIGNAL_CHANNEL_COUNT) {
                    phaseValues[phase][channel] = median((from until until).map { signals[it].values[channel] })
                }
            }
            for (phase in 0 until BIN_COUNT) {
                for (channel in 0 until SIGNAL_CHANNEL_COUNT) {
                    rows[slot][phase * SIGNAL_CHANNEL_COUNT + channel] = phaseValues[phase][channel]
                }
            }
            val deltaOffset = BIN_COUNT * SIGNAL_CHANNEL_COUNT
            for (channel in 0 until SIGNAL_CHANNEL_COUNT) {
                val actionMedian = median((0 until ACTION_BIN_COUNT).map { phaseValues[it][channel] })
                val idleMedian = median((ACTION_BIN_COUNT until BIN_COUNT).map { phaseValues[it][channel] })
                rows[slot][deltaOffset + channel] = actionMedian - idleMedian
            }
        }
        return RawProfiles(rows, sampleCounts)
    }

    private fun calibrate(rawProfiles: RawProfiles, columns: IntArray): CalibratedModel? {
        if (columns.isEmpty()) return null
        val selected = Array(FRAME_ACTION_COUNT) { row -> FloatArray(columns.size) { rawProfiles.rows[row][columns[it]] } }
        for (column in columns.indices) {
            val center = median(selected.map { it[column] })
            val scale = (median(selected.map { abs(it[column] - center) }) * MAD_SCALE)
                .takeIf { it >= MINIMUM_SCALE } ?: 1f
            selected.forEach { row -> row[column] = ((row[column] - center) / scale).coerceIn(-12f, 12f) }
        }

        var matches = 0
        val margins = ArrayList<Float>(profile.syncActions.size)
        profile.syncActions.forEachIndexed { index, expected ->
            val prototypes = ACTIONS.associateWith { action ->
                meanVector(profile.syncActions.indices.filter { other ->
                    other != index && profile.syncActions[other] == action
                }.map { selected[it] }) ?: return null
            }
            val costs = ACTIONS.associateWith { action -> squaredDistance(selected[index], prototypes.getValue(action)) }
            val orderedCosts = costs.entries.sortedBy { it.value }
            if (orderedCosts.first().key == expected) matches++
            margins += ((orderedCosts.getOrNull(1)?.value ?: orderedCosts.first().value) - orderedCosts.first().value)
                .coerceAtLeast(0f)
        }

        val prototypes = ACTIONS.associateWith { action ->
            meanVector(profile.syncActions.indices.filter { profile.syncActions[it] == action }.map { selected[it] })
                ?: return null
        }
        val actions = ArrayList<ActionClass>(FRAME_ACTION_COUNT)
        val alternativeActions = ArrayList<ActionClass>(FRAME_ACTION_COUNT)
        val confidences = FloatArray(FRAME_ACTION_COUNT)
        selected.forEachIndexed { index, vector ->
            val costs = ACTIONS.map { action -> action to squaredDistance(vector, prototypes.getValue(action)) }
                .sortedBy { it.second }
            actions += costs.first().first
            alternativeActions += costs[1].first
            val gap = (costs[1].second - costs[0].second).coerceAtLeast(0f)
            confidences[index] = (1f - exp(-gap.toDouble())).toFloat().coerceIn(0.05f, 0.98f)
        }
        val margin = median(margins)
        val score = (matches.toFloat() / profile.syncActions.size * 0.80f +
            (1f - exp(-margin.toDouble())).toFloat() * 0.20f).coerceIn(0f, 1f)
        return CalibratedModel(matches, margin, score, actions, alternativeActions, confidences)
    }

    private fun ModelCandidate.toAlignment(firstTimestampNs: Long): ReplaySyncAlignment {
        val slots = actions.mapIndexed { index, action ->
            SlotObservation(
                slotIndex = index.toLong(),
                action = profile.syncActions.getOrNull(index) ?: action,
                confidence = confidences[index],
                erased = false,
                sampleCount = sampleCounts[index],
            )
        }
        return ReplaySyncAlignment(
            phaseOffsetMs = (startNs - firstTimestampNs) / 1_000_000f,
            syncStartTimestampNs = startNs,
            syncScore = score,
            runnerUpScore = 0f,
            frameSlots = slots,
            selfCalibrated = true,
            syncMatches = syncMatches,
            modelMargin = margin,
            alternativeActions = alternativeActions,
        )
    }

    private fun lowerBound(values: List<SignalSample>, timestampNs: Long): Int {
        var low = 0
        var high = values.size
        while (low < high) {
            val middle = (low + high) ushr 1
            if (values[middle].timestampNs < timestampNs) low = middle + 1 else high = middle
        }
        return low
    }

    private fun median(values: List<Float>): Float {
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) / 2f else sorted[middle]
    }

    private fun medianLong(values: List<Long>): Long {
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) / 2L else sorted[middle]
    }

    private fun meanVector(samples: List<FloatArray>): FloatArray? {
        if (samples.isEmpty()) return null
        return FloatArray(samples.first().size) { column -> samples.map { it[column] }.average().toFloat() }
    }

    private fun squaredDistance(first: FloatArray, second: FloatArray): Float = first.indices.sumOf { index ->
        val difference = first[index] - second[index]
        (difference * difference).toDouble()
    }.toFloat() / first.size

    private data class SignalSample(val timestampNs: Long, val values: FloatArray)
    private data class RawProfiles(val rows: Array<FloatArray>, val sampleCounts: IntArray)
    private data class CalibratedModel(
        val syncMatches: Int,
        val margin: Float,
        val score: Float,
        val actions: List<ActionClass>,
        val alternativeActions: List<ActionClass>,
        val confidences: FloatArray,
    )
    private data class ModelCandidate(
        val startNs: Long,
        val syncMatches: Int,
        val margin: Float,
        val score: Float,
        val actions: List<ActionClass>,
        val alternativeActions: List<ActionClass>,
        val confidences: FloatArray,
        val sampleCounts: IntArray,
        val modelOrder: Int,
    )

    private companion object {
        const val FRAME_ACTION_COUNT = 40
        const val BIN_COUNT = 8
        const val ACTION_BIN_COUNT = 4
        const val SCENE_CHANNEL_COUNT = 4
        const val POSE_CHANNEL_COUNT = 6
        const val SOURCE_CHANNEL_COUNT = SCENE_CHANNEL_COUNT + POSE_CHANNEL_COUNT
        const val SIGNAL_CHANNEL_COUNT = 14
        const val PROFILE_WIDTH = BIN_COUNT * SIGNAL_CHANNEL_COUNT + SIGNAL_CHANNEL_COUNT
        const val MINIMUM_SIGNAL_SAMPLES = 80
        const val MINIMUM_BIN_SAMPLES = 1
        const val MINIMUM_PHASE_STEPS = 32
        const val MINIMUM_CANDIDATE_MATCHES = 4
        const val MAXIMUM_REPLAY_CANDIDATES = 4096
        const val MAD_SCALE = 1.4826f
        const val MINIMUM_SCALE = 0.001f
        const val SMOOTHING_RADIUS = 2
        val SMOOTHING_WEIGHTS = floatArrayOf(1f, 2f, 3f, 2f, 1f)
        val ACTIONS = listOf(
            ActionClass.HOVER,
            ActionClass.FORWARD,
            ActionClass.YAW_LEFT,
            ActionClass.YAW_RIGHT,
        )
        val CHANNEL_SETS = listOf(
            intArrayOf(0, 1),
            intArrayOf(0, 1, 2, 3),
            intArrayOf(4, 5),
            intArrayOf(4, 5, 6, 7),
            intArrayOf(0, 1, 4, 5),
            intArrayOf(0, 1, 2, 3, 4, 5, 6, 7),
            intArrayOf(8, 9),
            intArrayOf(8, 9, 10, 11),
            intArrayOf(8, 9, 10, 11, 12, 13),
            IntArray(SIGNAL_CHANNEL_COUNT) { it },
        )
    }
}
