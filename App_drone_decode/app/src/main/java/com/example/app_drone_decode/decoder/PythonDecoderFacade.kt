package com.example.app_drone_decode.decoder

import android.content.Context
import com.chaquo.python.Python
import com.chaquo.python.android.AndroidPlatform
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecodedFrame
import com.example.app_drone_decode.domain.model.DecoderBridgeEvent
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderState
import com.example.app_drone_decode.domain.model.SlotObservation
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONArray
import org.json.JSONObject

class PythonDecoderFacade(private val appContext: Context) {
    @Volatile
    private var activeActionMapping: Map<ActionClass, String> = DecoderProfile().actionMapping

    data class ReplayCandidateDecode(
        val index: Int,
        val accepted: Boolean,
        val sequence: Int?,
        val length: Int?,
        val payloadText: String,
        val payloadHex: String,
        val correctedBitErrors: Int,
        val rejectionReason: String?,
        val variantCost: Int = 0,
        val variantPenalty: Float = 0f,
        val actions: List<ActionClass> = emptyList(),
    )

    suspend fun start(): Result<Unit> = invokeSimple("start")

    suspend fun pause(): Result<Unit> = invokeSimple("pause")

    suspend fun reset(): Result<Unit> = invokeSimple("reset")

    suspend fun validateProfile(profile: DecoderProfile): Result<List<String>> = withContext(Dispatchers.Default) {
        runCatching {
            val response = JSONObject(module().callAttr("validate_profile_json", profile.toJson().toString()).toString())
            response.getJSONArray("errors").toStringList()
        }
    }

    suspend fun configureProfile(profile: DecoderProfile): Result<Unit> = withContext(Dispatchers.Default) {
        runCatching {
            val response = JSONObject(module().callAttr("configure_profile_json", profile.toJson().toString()).toString())
            check(response.optBoolean("ok")) {
                response.optJSONArray("errors")?.toStringList()?.joinToString() ?: "Profile configuration failed"
            }
            activeActionMapping = profile.actionMapping.toMap()
        }
    }

    suspend fun process(slot: SlotObservation): DecoderBridgeEvent = withContext(Dispatchers.Default) {
        val request = JSONObject()
            .put("slotIndex", slot.slotIndex)
            .put("action", slot.action.shortName)
            .put("confidence", slot.confidence)
        val response = runCatching {
            JSONObject(module().callAttr("process_slot_json", request.toString()).toString())
        }.getOrElse { error ->
            return@withContext errorEvent(slot, error.message ?: error.javaClass.simpleName)
        }
        if (!response.optBoolean("ok")) {
            return@withContext errorEvent(slot, response.optString("message", "Python decoder error"))
        }
        response.toBridgeEvent()
    }

    suspend fun encodeReferenceFrame(): Result<List<ActionClass>> = withContext(Dispatchers.Default) {
        runCatching {
            val request = JSONObject().put("payloadHex", "48656C6C6F").put("seq", 0)
            val response = JSONObject(module().callAttr("encode_frame_json", request.toString()).toString())
            check(response.getBoolean("ok")) { response.optString("message") }
            response.getJSONArray("actions").toStringList().map(ActionClass::fromShortName)
        }
    }

    suspend fun decodeReplayCandidates(
        candidates: List<com.example.app_drone_decode.vision.ReplaySyncAlignment>,
    ): Result<List<ReplayCandidateDecode>> = withContext(Dispatchers.Default) {
        runCatching {
            val request = JSONObject().put(
                "candidates",
                JSONArray().apply {
                    candidates.forEach { candidate ->
                        val dataSlots = candidate.frameSlots.drop(8)
                        val alternatives = candidate.alternativeActions.drop(8)
                        put(
                            JSONObject()
                                .put("actions", JSONArray(dataSlots.map { it.action.shortName }))
                                .put("confidences", JSONArray(dataSlots.map { it.confidence.toDouble() }))
                                .put("alternatives", JSONArray(alternatives.map { it.shortName })),
                        )
                    }
                },
            )
            val response = JSONObject(
                module().callAttr("decode_replay_candidates_json", request.toString()).toString(),
            )
            check(response.optBoolean("ok")) { response.optString("message", "Replay BCH validation failed") }
            val results = response.getJSONArray("results")
            List(results.length()) { index ->
                results.getJSONObject(index).let { item ->
                    ReplayCandidateDecode(
                        index = item.getInt("index"),
                        accepted = item.optBoolean("accepted"),
                        sequence = item.nullableInt("seq"),
                        length = item.nullableInt("length"),
                        payloadText = item.optString("payload_ascii"),
                        payloadHex = item.optString("payload_hex"),
                        correctedBitErrors = item.optInt("corrected_bit_errors"),
                        rejectionReason = item.nullableString("rejection_reason"),
                        variantCost = item.optInt("variantCost"),
                        variantPenalty = item.optDouble("variantPenalty", 0.0).toFloat(),
                        actions = item.optJSONArray("actions")?.toStringList()
                            ?.map(ActionClass::fromShortName).orEmpty(),
                    )
                }
            }
        }
    }

    private suspend fun invokeSimple(functionName: String): Result<Unit> = withContext(Dispatchers.Default) {
        runCatching {
            val response = JSONObject(module().callAttr(functionName).toString())
            check(response.optBoolean("ok")) { response.optString("message", "Decoder call failed") }
        }
    }

    private fun module() = synchronized(PythonDecoderFacade::class.java) {
        if (!Python.isStarted()) Python.start(AndroidPlatform(appContext))
        Python.getInstance().getModule("drone_decode.bridge")
    }

    private fun JSONObject.toBridgeEvent(): DecoderBridgeEvent {
        val resultObject = optJSONObject("result")
        val result = resultObject?.let {
            DecodedFrame(
                accepted = it.optBoolean("accepted"),
                sequence = it.nullableInt("seq"),
                payloadText = it.optString("payload_ascii"),
                payloadHex = it.optString("payload_hex"),
                correctedBitErrors = it.optInt("corrected_bit_errors"),
                erasedBits = it.optInt("erased_bits"),
                meanConfidence = it.optDouble("mean_confidence").toFloat(),
                possibleMissingFrames = it.optInt("possible_missing_frames"),
                rejectionReason = it.nullableString("rejection_reason"),
            )
        }
        return DecoderBridgeEvent(
            ok = true,
            slotIndex = nullableLong("slot_index"),
            action = ActionClass.fromShortName(optString("action", "?")),
            symbol = optString("symbol", "??"),
            decoderState = optString("decoder_state", DecoderState.ERROR.name).toDecoderState(),
            nextState = optString("next_state", DecoderState.ERROR.name).toDecoderState(),
            syncScore = optDouble("sync_score", 0.0).toFloat(),
            matchedSyncHistory = optJSONArray("matched_sync_history")?.toStringList().orEmpty(),
            frameCollectionProgress = optInt("frame_collection_progress", 0),
            result = result,
        )
    }

    private fun DecoderProfile.toJson() = JSONObject()
        .put("profileVersion", profileVersion)
        .put("protocolVersion", protocolVersion)
        .put("actionDurationMs", actionDurationMs)
        .put("recoveryDurationMs", recoveryDurationMs)
        .put("motionReferenceMode", motionReferenceMode.name)
        .put("idleDurationMs", idleDurationMs)
        .put("symbolDurationMs", symbolDurationMs)
        .put("stableWindowFraction", stableWindowFraction)
        .put("minimumSamplesPerSlot", minimumSamplesPerSlot)
        .put("erasureThreshold", erasureThreshold)
        .put("sync", JSONArray(syncActions.map { it.shortName }))
        .put(
            "actionMapping",
            JSONObject().apply {
                put("H", actionMapping.getValue(ActionClass.HOVER))
                put("F", actionMapping.getValue(ActionClass.FORWARD))
                put("L", actionMapping.getValue(ActionClass.YAW_LEFT))
                put("R", actionMapping.getValue(ActionClass.YAW_RIGHT))
            },
        )

    private fun errorEvent(slot: SlotObservation, message: String) = DecoderBridgeEvent(
        ok = false,
        slotIndex = slot.slotIndex,
        action = slot.action,
        symbol = activeActionMapping[slot.action] ?: "??",
        decoderState = DecoderState.ERROR,
        nextState = DecoderState.ERROR,
        syncScore = 0f,
        matchedSyncHistory = emptyList(),
        frameCollectionProgress = 0,
        result = null,
        errorMessage = message,
    )
}

private fun JSONArray.toStringList(): List<String> = List(length()) { index -> getString(index) }

private fun JSONObject.nullableInt(key: String): Int? = if (isNull(key)) null else getInt(key)

private fun JSONObject.nullableLong(key: String): Long? = if (isNull(key)) null else getLong(key)

private fun JSONObject.nullableString(key: String): String? = if (isNull(key)) null else getString(key)

private fun String.toDecoderState(): DecoderState = runCatching { DecoderState.valueOf(this) }
    .getOrDefault(DecoderState.ERROR)
