package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.SlotObservation
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sqrt

data class ReplaySyncAlignment(
    val phaseOffsetMs: Float,
    val syncStartTimestampNs: Long,
    val syncScore: Float,
    val runnerUpScore: Float,
    val frameSlots: List<SlotObservation>,
    val selfCalibrated: Boolean = false,
    val syncMatches: Int = 0,
    val modelMargin: Float = 0f,
    val alternativeActions: List<ActionClass> = emptyList(),
)

data class ReplaySyncSearchResult(
    val alignment: ReplaySyncAlignment?,
    val bestScore: Float,
    val reason: String,
    val candidates: List<ReplaySyncAlignment> = emptyList(),
)

/**
 * Searches the complete recording for slot phase and SYNC. When an imported
 * video has no camera calibration, the search derives action prototypes only
 * from the public SYNC word. It never uses a filename, payload, or data-slot
 * label to classify the recording.
 */
class ReplaySyncAligner(
    private val profile: DecoderProfile,
    private val phaseSteps: Int = DEFAULT_PHASE_STEPS,
) {
    fun search(observations: List<MotionObservation>): ReplaySyncSearchResult {
        if (observations.size < profile.minimumSamplesPerSlot) {
            return failure(0f, "Too few visual observations for slot synchronization")
        }
        val ordered = observations.sortedBy(MotionObservation::timestampNs)
        if (profile.idleDurationMs > 0) {
            return PulsePauseReplayAligner(profile, phaseSteps).search(ordered)
        }
        val classified = searchClassified(ordered)
        if (classified.alignment != null) return classified

        val calibrated = searchWithSyncCalibration(ordered)
        if (calibrated.alignment != null) return calibrated
        return if (calibrated.bestScore >= classified.bestScore) calibrated else classified
    }

    private fun searchClassified(ordered: List<MotionObservation>): ReplaySyncSearchResult {
        val firstTimestampNs = ordered.first().timestampNs
        val slotDurationNs = profile.symbolDurationMs * 1_000_000L
        val candidates = mutableListOf<Candidate>()

        for (phaseIndex in 0 until phaseSteps.coerceAtLeast(2)) {
            val phaseOffsetNs = slotDurationNs * phaseIndex / phaseSteps.coerceAtLeast(2)
            val anchor = firstTimestampNs + phaseOffsetNs
            val aggregator = SlotAggregator(profile)
            aggregator.reset(anchor)
            val slots = buildList {
                ordered.forEach { addAll(aggregator.add(it)) }
                addAll(aggregator.flush())
            }
            if (slots.size < FRAME_ACTION_COUNT) continue
            for (startIndex in 0..slots.size - FRAME_ACTION_COUNT) {
                val syncWindow = slots.subList(startIndex, startIndex + profile.syncActions.size)
                val matches = syncWindow.zip(profile.syncActions).count { (slot, expected) ->
                    !slot.erased && slot.action == expected
                }
                val score = syncWindow.zip(profile.syncActions).sumOf { (slot, expected) ->
                    if (!slot.erased && slot.action == expected) slot.confidence.toDouble() else 0.0
                }.toFloat() / profile.syncActions.size
                candidates += Candidate(
                    phaseOffsetNs = phaseOffsetNs,
                    syncStartTimestampNs = anchor + slots[startIndex].slotIndex * slotDurationNs,
                    matches = matches,
                    score = score,
                    frameSlots = slots.subList(startIndex, startIndex + FRAME_ACTION_COUNT).toList(),
                    selfCalibrated = false,
                )
            }
        }
        return selectCandidate(candidates, "No complete high-confidence classified SYNC was found")
    }

    private fun searchWithSyncCalibration(ordered: List<MotionObservation>): ReplaySyncSearchResult {
        val firstTimestampNs = ordered.first().timestampNs
        val slotDurationNs = profile.symbolDurationMs * 1_000_000L
        val candidates = mutableListOf<Candidate>()

        for (phaseIndex in 0 until phaseSteps.coerceAtLeast(2)) {
            val phaseOffsetNs = slotDurationNs * phaseIndex / phaseSteps.coerceAtLeast(2)
            val anchor = firstTimestampNs + phaseOffsetNs
            val featureSlots = aggregateFeatures(ordered, anchor, slotDurationNs)
            if (featureSlots.size < FRAME_ACTION_COUNT) continue
            for (startIndex in 0..featureSlots.size - FRAME_ACTION_COUNT) {
                val frame = featureSlots.subList(
                    startIndex,
                    minOf(startIndex + FRAME_ACTION_COUNT + 1, featureSlots.size),
                )
                val calibrated = calibrateCandidate(frame) ?: continue
                candidates += Candidate(
                    phaseOffsetNs = phaseOffsetNs,
                    syncStartTimestampNs = anchor + startIndex * slotDurationNs,
                    matches = calibrated.syncMatches,
                    score = calibrated.score,
                    frameSlots = calibrated.slots,
                    selfCalibrated = true,
                )
            }
        }
        return selectCandidate(candidates, "No visually separable SYNC was found for replay self-calibration")
    }

    private fun aggregateFeatures(
        observations: List<MotionObservation>,
        anchorTimestampNs: Long,
        slotDurationNs: Long,
    ): List<FeatureSlot> {
        val grouped = linkedMapOf<Int, MutableList<MotionObservation>>()
        val halfWindow = profile.stableWindowFraction / 2f
        observations.forEach { observation ->
            val elapsed = observation.timestampNs - anchorTimestampNs
            if (elapsed < 0L) return@forEach
            val slotIndex = (elapsed / slotDurationNs).toInt()
            val progress = (elapsed % slotDurationNs).toDouble() / slotDurationNs
            if (progress in (0.5 - halfWindow)..(0.5 + halfWindow)) {
                grouped.getOrPut(slotIndex) { mutableListOf() } += observation
            }
        }
        val lastIndex = grouped.keys.maxOrNull() ?: return emptyList()
        return (0..lastIndex).map { index -> featureSlot(grouped[index].orEmpty()) }
    }

    private fun featureSlot(samples: List<MotionObservation>): FeatureSlot {
        val visible = samples.filter(MotionObservation::visible)
        if (visible.size < profile.minimumSamplesPerSlot) return FeatureSlot.invalid(visible.size)
        val velocityX = slope(visible) { it.centerX }
        val velocityY = slope(visible) { it.centerY }
        val speed = if (velocityX != null && velocityY != null) hypot(velocityX, velocityY) else null
        val yaw = median(visible.mapNotNull(MotionObservation::yawRateDegPerSec))
            ?: orientationSlope(visible)
        val scaleRate = slope(visible) { observation ->
            observation.boundingBox?.let { box ->
                sqrt(((box.right - box.left) * (box.bottom - box.top)).coerceAtLeast(0f))
            }
        }
        val sceneVelocityX = median(visible.mapNotNull(MotionObservation::sceneVelocityXPerSec))
        val sceneVelocityY = median(visible.mapNotNull(MotionObservation::sceneVelocityYPerSec))
        val sceneSpeed = if (sceneVelocityX != null && sceneVelocityY != null) {
            hypot(sceneVelocityX, sceneVelocityY)
        } else {
            0f
        }
        val sceneRotation = median(visible.mapNotNull(MotionObservation::sceneRotationDegPerSec)) ?: 0f
        val sceneScale = median(visible.mapNotNull(MotionObservation::sceneScaleRatePerSec)) ?: 0f
        return if (speed == null || yaw == null) {
            FeatureSlot.invalid(visible.size)
        } else {
            FeatureSlot(
                velocityX = requireNotNull(velocityX),
                velocityY = requireNotNull(velocityY),
                speed = speed,
                yawRate = yaw,
                scaleRate = scaleRate ?: 0f,
                sceneVelocityX = sceneVelocityX ?: 0f,
                sceneVelocityY = sceneVelocityY ?: 0f,
                sceneSpeed = sceneSpeed,
                sceneRotation = sceneRotation,
                sceneScaleRate = sceneScale,
                sceneValid = sceneVelocityX != null && sceneVelocityY != null,
                sampleCount = visible.size,
                valid = true,
            )
        }
    }

    private fun calibrateCandidate(frame: List<FeatureSlot>): CalibratedCandidate? {
        if (frame.take(profile.syncActions.size).any { !it.valid }) return null
        val bestModel = YAW_MEMORY_COEFFICIENTS.flatMap { memory ->
            FEATURE_SETS.mapNotNull { columns ->
                buildFeatureVectors(frame, memory, columns)?.let { vectors ->
                    scoreCalibration(vectors)?.let { calibration -> Triple(memory, vectors, calibration) }
                }
            }
        }.maxWithOrNull(
            compareBy<Triple<Float, List<FloatArray>, Calibration>> { it.third.matches }
                .thenBy { it.third.score },
        ) ?: return null

        val vectors = bestModel.second
        val calibration = bestModel.third

        val slots = vectors.mapIndexed { index, vector ->
            val distances = calibration.prototypes.mapValues { (_, prototype) -> squaredDistance(vector, prototype) }
            val ordered = distances.entries.sortedBy { it.value }
            val best = ordered.first()
            val second = ordered.getOrNull(1)?.value ?: best.value + 1f
            val gap = (second - best.value).coerceAtLeast(0f)
            val confidence = (1f - exp(-gap.toDouble())).toFloat().coerceIn(0f, 0.98f)
            val source = frame[index]
            val erased = !source.valid || confidence < profile.erasureThreshold
            val synchronizedAction = profile.syncActions.getOrNull(index)
            SlotObservation(
                slotIndex = index.toLong(),
                action = synchronizedAction ?: if (erased) ActionClass.UNKNOWN else best.key,
                confidence = confidence,
                erased = synchronizedAction == null && erased,
                sampleCount = source.sampleCount,
            )
        }
        return CalibratedCandidate(calibration.matches, calibration.score, slots)
    }

    private fun buildFeatureVectors(
        frame: List<FeatureSlot>,
        yawMemory: Float,
        columns: IntArray,
    ): List<FloatArray>? {
        val useSceneMotion = frame.take(FRAME_ACTION_COUNT).count(FeatureSlot::sceneValid) >=
            FRAME_ACTION_COUNT * 3 / 4
        val raw = frame.take(FRAME_ACTION_COUNT).mapIndexed { index, current ->
            val prior = frame.getOrNull(index - 1) ?: current
            val future = frame.getOrNull(index + 1) ?: current
            val targetYawResidual = current.yawRate - yawMemory * prior.yawRate
            val sceneYawResidual = current.sceneRotation - yawMemory * prior.sceneRotation
            val allFeatures = floatArrayOf(
                targetYawResidual,
                current.yawRate,
                sceneYawResidual,
                current.sceneRotation,
                targetYawResidual - sceneYawResidual,
                current.velocityX,
                current.velocityY,
                current.velocityX - prior.velocityX,
                current.velocityY - prior.velocityY,
                current.speed,
                current.speed - prior.speed,
                current.scaleRate,
                current.sceneVelocityX,
                current.sceneVelocityY,
                future.sceneVelocityX - current.sceneVelocityX,
                future.sceneVelocityY - current.sceneVelocityY,
                future.sceneVelocityX - 2f * current.sceneVelocityX + prior.sceneVelocityX,
                future.sceneVelocityY - 2f * current.sceneVelocityY + prior.sceneVelocityY,
                current.sceneSpeed,
                future.sceneSpeed - current.sceneSpeed,
                future.sceneSpeed - 2f * current.sceneSpeed + prior.sceneSpeed,
                current.sceneScaleRate,
                future.sceneScaleRate - current.sceneScaleRate,
            )
            if (useSceneMotion) {
                FloatArray(columns.size) { featureIndex -> allFeatures[columns[featureIndex]] }
            } else {
                floatArrayOf(
                current.yawRate - yawMemory * prior.yawRate,
                current.speed,
                current.speed - prior.speed,
                current.scaleRate,
                )
            }
        }
        val validRows = raw.filterIndexed { index, _ -> frame[index].valid }
        if (validRows.size < profile.syncActions.size) return null
        val centers = FloatArray(raw.first().size) { column -> median(validRows.map { it[column] }) ?: 0f }
        val scales = FloatArray(raw.first().size) { column ->
            val deviations = validRows.map { abs(it[column] - centers[column]) }
            ((median(deviations) ?: 0f) * MAD_SCALE).coerceAtLeast(MINIMUM_FEATURE_SCALE)
        }
        return raw.map { row -> FloatArray(row.size) { column -> (row[column] - centers[column]) / scales[column] } }
    }

    private fun scoreCalibration(vectors: List<FloatArray>): Calibration? {
        val syncCount = profile.syncActions.size
        val syncVectors = vectors.take(syncCount)
        val prototypes = ACTIONS.associateWith { action ->
            meanVector(syncVectors.filterIndexed { index, _ -> profile.syncActions[index] == action })
                ?: return null
        }
        var matches = 0
        var marginSum = 0f
        profile.syncActions.forEachIndexed { index, expected ->
            val leaveOneOut = ACTIONS.associateWith { action ->
                val samples = syncVectors.filterIndexed { otherIndex, _ ->
                    profile.syncActions[otherIndex] == action && otherIndex != index
                }
                meanVector(samples) ?: prototypes.getValue(action)
            }
            val distances = leaveOneOut.mapValues { (_, prototype) -> squaredDistance(syncVectors[index], prototype) }
            val ordered = distances.entries.sortedBy { it.value }
            if (ordered.first().key == expected) matches++
            marginSum += ((ordered.getOrNull(1)?.value ?: ordered.first().value) - ordered.first().value)
                .coerceAtLeast(0f)
        }
        val within = profile.syncActions.indices.sumOf { index ->
            squaredDistance(syncVectors[index], prototypes.getValue(profile.syncActions[index])).toDouble()
        }.toFloat() / syncCount
        val pairDistances = ACTIONS.indices.flatMap { first ->
            ((first + 1) until ACTIONS.size).map { second ->
                squaredDistance(prototypes.getValue(ACTIONS[first]), prototypes.getValue(ACTIONS[second]))
            }
        }
        val between = if (pairDistances.isEmpty()) 0f else pairDistances.average().toFloat()
        val separation = between / (within + 0.25f)
        val score = (
            matches.toFloat() / syncCount * 0.65f +
                (1f - exp(-separation.toDouble())).toFloat() * 0.25f +
                (1f - exp(-(marginSum / syncCount).toDouble())).toFloat() * 0.10f
            ).coerceIn(0f, 1f)
        return Calibration(matches, score, prototypes)
    }

    private fun selectCandidate(candidates: List<Candidate>, failureReason: String): ReplaySyncSearchResult {
        val best = candidates.maxWithOrNull(compareBy<Candidate> { it.matches }.thenBy { it.score })
            ?: return failure(0f, failureReason)
        val slotDurationNs = profile.symbolDurationMs * 1_000_000L
        val runnerUp = candidates.asSequence()
            .filter { candidate ->
                abs(candidate.syncStartTimestampNs - best.syncStartTimestampNs) >= slotDurationNs * 3 / 4
            }
            .maxOfOrNull(Candidate::score) ?: 0f
        val minimumMatches = if (best.selfCalibrated) MINIMUM_SELF_CALIBRATED_MATCHES else profile.syncActions.size
        val minimumScore = if (best.selfCalibrated) MINIMUM_SELF_CALIBRATED_SCORE else MINIMUM_SYNC_SCORE
        if (best.matches < minimumMatches || best.score < minimumScore) {
            return failure(best.score, failureReason)
        }
        if (runnerUp >= best.score - MINIMUM_UNIQUE_MARGIN) {
            return failure(best.score, "Multiple equally plausible SYNC positions were found")
        }
        return ReplaySyncSearchResult(
            alignment = ReplaySyncAlignment(
                phaseOffsetMs = best.phaseOffsetNs / 1_000_000f,
                syncStartTimestampNs = best.syncStartTimestampNs,
                syncScore = best.score,
                runnerUpScore = runnerUp,
                frameSlots = best.frameSlots,
                selfCalibrated = best.selfCalibrated,
                syncMatches = best.matches,
                modelMargin = best.score - runnerUp,
            ),
            bestScore = best.score,
            reason = if (best.selfCalibrated) "SYNC locked with replay self-calibration" else "SYNC locked",
        )
    }

    private fun slope(samples: List<MotionObservation>, selector: (MotionObservation) -> Float?): Float? {
        val points = samples.mapNotNull { sample -> selector(sample)?.let { sample.timestampNs to it } }
        if (points.size < profile.minimumSamplesPerSlot) return null
        val origin = points.first().first
        val times = points.map { (timestamp, _) -> (timestamp - origin) / 1_000_000_000f }
        val meanTime = times.average().toFloat()
        val meanValue = points.map { it.second }.average().toFloat()
        val denominator = times.sumOf { time ->
            val centered = time - meanTime
            (centered * centered).toDouble()
        }.toFloat()
        if (denominator <= 0.000001f) return null
        return times.indices.sumOf { index ->
            ((times[index] - meanTime) * (points[index].second - meanValue)).toDouble()
        }.toFloat() / denominator
    }

    private fun orientationSlope(samples: List<MotionObservation>): Float? {
        val unwrapped = mutableListOf<Pair<Long, Float>>()
        samples.forEach { sample ->
            val orientation = sample.orientationDeg ?: return@forEach
            val previous = unwrapped.lastOrNull()?.second
            var value = orientation
            if (previous != null) {
                while (value - previous > 90f) value -= 180f
                while (value - previous < -90f) value += 180f
            }
            unwrapped += sample.timestampNs to value
        }
        if (unwrapped.size < profile.minimumSamplesPerSlot) return null
        val synthetic = unwrapped.map { (timestamp, value) ->
            MotionObservation(timestamp, true, value, 0f, null, null, emptyMap(), 1f)
        }
        return slope(synthetic) { it.centerX }
    }

    private fun median(values: List<Float>): Float? {
        if (values.isEmpty()) return null
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) (sorted[middle - 1] + sorted[middle]) / 2f else sorted[middle]
    }

    private fun meanVector(samples: List<FloatArray>): FloatArray? {
        if (samples.isEmpty()) return null
        return FloatArray(samples.first().size) { column -> samples.map { it[column] }.average().toFloat() }
    }

    private fun squaredDistance(first: FloatArray, second: FloatArray): Float = first.indices.sumOf { index ->
        val difference = first[index] - second[index]
        (difference * difference).toDouble()
    }.toFloat() / first.size

    private fun failure(score: Float, reason: String) = ReplaySyncSearchResult(null, score, reason)

    private data class FeatureSlot(
        val velocityX: Float,
        val velocityY: Float,
        val speed: Float,
        val yawRate: Float,
        val scaleRate: Float,
        val sceneVelocityX: Float,
        val sceneVelocityY: Float,
        val sceneSpeed: Float,
        val sceneRotation: Float,
        val sceneScaleRate: Float,
        val sceneValid: Boolean,
        val sampleCount: Int,
        val valid: Boolean,
    ) {
        companion object {
            fun invalid(samples: Int) = FeatureSlot(
                velocityX = 0f,
                velocityY = 0f,
                speed = 0f,
                yawRate = 0f,
                scaleRate = 0f,
                sceneVelocityX = 0f,
                sceneVelocityY = 0f,
                sceneSpeed = 0f,
                sceneRotation = 0f,
                sceneScaleRate = 0f,
                sceneValid = false,
                sampleCount = samples,
                valid = false,
            )
        }
    }

    private data class Calibration(
        val matches: Int,
        val score: Float,
        val prototypes: Map<ActionClass, FloatArray>,
    )

    private data class CalibratedCandidate(
        val syncMatches: Int,
        val score: Float,
        val slots: List<SlotObservation>,
    )

    private data class Candidate(
        val phaseOffsetNs: Long,
        val syncStartTimestampNs: Long,
        val matches: Int,
        val score: Float,
        val frameSlots: List<SlotObservation>,
        val selfCalibrated: Boolean,
    )

    private companion object {
        val ACTIONS = listOf(
            ActionClass.HOVER,
            ActionClass.FORWARD,
            ActionClass.YAW_LEFT,
            ActionClass.YAW_RIGHT,
        )
        val YAW_MEMORY_COEFFICIENTS = listOf(0f, 0.35f, 0.60f, 0.80f, 1f, 1.15f)
        val FEATURE_SETS = listOf(
            intArrayOf(0),
            intArrayOf(2),
            intArrayOf(4),
            intArrayOf(0, 2, 4),
            intArrayOf(5, 6, 7, 8, 9, 10, 11),
            intArrayOf(12, 13, 14, 15),
            intArrayOf(12, 13, 14, 15, 16, 17),
            intArrayOf(18, 19, 20),
            intArrayOf(2, 4, 12, 13, 14, 15, 16, 17, 19, 20),
            IntArray(23) { it },
        )
        const val DEFAULT_PHASE_STEPS = 20
        const val FRAME_ACTION_COUNT = 40
        const val MINIMUM_SYNC_SCORE = 0.65f
        const val MINIMUM_SELF_CALIBRATED_SCORE = 0.72f
        const val MINIMUM_SELF_CALIBRATED_MATCHES = 7
        const val MINIMUM_UNIQUE_MARGIN = 0.08f
        const val MAD_SCALE = 1.4826f
        const val MINIMUM_FEATURE_SCALE = 0.0001f
    }
}
