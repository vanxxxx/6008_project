package com.example.app_drone_decode.data.logs

import android.content.Context
import com.example.app_drone_decode.domain.model.DecoderLogEntry
import com.example.app_drone_decode.domain.model.DecoderState
import com.example.app_drone_decode.domain.model.LogSeverity
import com.example.app_drone_decode.domain.model.MotionObservation
import java.io.File
import java.time.Instant
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.util.UUID
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import org.json.JSONArray
import org.json.JSONObject

data class SessionSummary(
    val id: String,
    val startedEpochMs: Long,
    val endedEpochMs: Long,
    val durationMs: Long,
    val profileName: String,
    val eventCount: Int,
    val observationCount: Int,
    val dataSource: String,
    val sourceLabel: String?,
    val file: File,
)

data class PersistedSessionEvent(
    val timestampEpochMs: Long,
    val severity: LogSeverity,
    val decoderState: DecoderState,
    val message: String,
    val slotIndex: Long?,
    val action: String?,
    val symbol: String?,
    val confidence: Float?,
    val rejectionReason: String?,
)

data class SessionDetail(
    val summary: SessionSummary,
    val acceptedFrames: Int,
    val rejectedFrames: Int,
    val trackingGapObservations: Int,
    val meanConfidence: Float,
    val plaintextMessages: List<String>,
    val payloadHexValues: List<String>,
    val rawActions: List<String>,
    val rawSymbols: List<String>,
    val events: List<PersistedSessionEvent>,
)

class SessionLogStore(context: Context) {
    private val sessionDirectory = File(context.filesDir, "sessions").apply { mkdirs() }
    private val mutex = Mutex()

    suspend fun startSession(
        profileName: String,
        dataSource: String = "VISUAL_CAMERA",
        sourceLabel: String? = null,
    ): SessionSummary = withContext(Dispatchers.IO) {
        val started = System.currentTimeMillis()
        val id = "${started}-${UUID.randomUUID().toString().take(8)}"
        val file = File(sessionDirectory, "$id.jsonl")
        val header = JSONObject()
            .put("recordType", "session")
            .put("schemaVersion", SCHEMA_VERSION)
            .put("sessionId", id)
            .put("startedEpochMs", started)
            .put("profileName", profileName)
            .put("dataSource", dataSource)
            .put("sourceLabel", sourceLabel)
        file.writeText(header.toString() + "\n", Charsets.UTF_8)
        summarizeFile(file)
    }

    suspend fun append(session: SessionSummary, entry: DecoderLogEntry) = withContext(Dispatchers.IO) {
        mutex.withLock {
            val json = JSONObject()
                .put("recordType", "decoderEvent")
                .put("timestamp", entry.timestampEpochMs)
                .put("timestampEpochMs", entry.timestampEpochMs)
                .put("slot", entry.slotIndex)
                .put("severity", entry.severity.name)
                .put("decoder_state", entry.state.name)
                .put("message", entry.message)
                .put("action", entry.action?.shortName)
                .put("symbol", entry.symbol)
                .put("confidence", entry.confidence)
                .put("accepted", entry.severity == LogSeverity.ACCEPTED)
                .put("rejection_reason", entry.rejectionReason)
                .put("payload_text", entry.payloadText)
                .put("payload_hex", entry.payloadHex)
                .put("sequence", entry.sequence)
                .put("corrected_bit_errors", entry.correctedBitErrors)
                .put("erased_bits", entry.erasedBits)
            appendBounded(session.file, json)
        }
    }

    suspend fun appendObservation(
        session: SessionSummary,
        frameIndex: Long,
        slotIndex: Long,
        observation: MotionObservation,
        actionSymbol: String,
        decoderState: DecoderState,
        syncScore: Float,
    ) = withContext(Dispatchers.IO) {
        mutex.withLock {
            val json = JSONObject()
                .put("recordType", "motionObservation")
                .put("timestamp", observation.timestampNs)
                .put("timestampEpochMs", System.currentTimeMillis())
                .put("frame", frameIndex)
                .put("slot", slotIndex)
                .put("visible", observation.visible)
                .put("action", observation.actionProbabilities.maxByOrNull { it.value }?.key?.shortName ?: "?")
                .put("symbol", actionSymbol)
                .put("confidence", observation.confidence)
                .put("centerX", observation.centerX)
                .put("centerY", observation.centerY)
                .put("velocity", observation.normalizedLinearVelocityPerSec)
                .put("yaw_rate", observation.yawRateDegPerSec)
                .put("orientation_deg", observation.orientationDeg)
                .put("box_left", observation.boundingBox?.left)
                .put("box_top", observation.boundingBox?.top)
                .put("box_right", observation.boundingBox?.right)
                .put("box_bottom", observation.boundingBox?.bottom)
                .put("target_area_fraction", observation.targetAreaFraction)
                .put("scene_velocity_x", observation.sceneVelocityXPerSec)
                .put("scene_velocity_y", observation.sceneVelocityYPerSec)
                .put("scene_rotation_deg_per_sec", observation.sceneRotationDegPerSec)
                .put("scene_scale_rate_per_sec", observation.sceneScaleRatePerSec)
                .put("decoder_state", decoderState.name)
                .put("sync_score", syncScore)
                .put("accepted", false)
            appendBounded(session.file, json)
        }
    }

    suspend fun listSessions(): List<SessionSummary> = withContext(Dispatchers.IO) {
        sessionDirectory.listFiles { file -> file.extension == "jsonl" }
            .orEmpty()
            .mapNotNull { file -> runCatching { summarizeFile(file) }.getOrNull() }
            .sortedByDescending(SessionSummary::startedEpochMs)
    }

    suspend fun loadDetail(summary: SessionSummary): SessionDetail = withContext(Dispatchers.IO) {
        var accepted = 0
        var rejected = 0
        var trackingGaps = 0
        var confidenceTotal = 0.0
        var confidenceCount = 0
        val plaintext = mutableListOf<String>()
        val payloadHex = mutableListOf<String>()
        val actions = mutableListOf<String>()
        val symbols = mutableListOf<String>()
        val events = ArrayDeque<PersistedSessionEvent>()

        summary.file.useLines { lines ->
            lines.drop(1).filter(String::isNotBlank).forEach { line ->
                val record = JSONObject(line)
                when (record.optString("recordType")) {
                    "decoderEvent" -> {
                        val severity = record.optString("severity").toEnumOrDefault(LogSeverity.INFO)
                        if (severity == LogSeverity.ACCEPTED) accepted++
                        if (severity == LogSeverity.REJECTED) rejected++
                        record.optionalString("payload_text")?.takeIf(String::isNotEmpty)?.let(plaintext::add)
                        record.optionalString("payload_hex")?.takeIf(String::isNotEmpty)?.let(payloadHex::add)
                        record.optionalString("action")?.let(actions::add)
                        record.optionalString("symbol")?.let(symbols::add)
                        events.addLast(
                            PersistedSessionEvent(
                                timestampEpochMs = record.optLong("timestampEpochMs", summary.startedEpochMs),
                                severity = severity,
                                decoderState = record.optString("decoder_state").toEnumOrDefault(DecoderState.IDLE),
                                message = record.optString("message"),
                                slotIndex = record.optionalLong("slot"),
                                action = record.optionalString("action"),
                                symbol = record.optionalString("symbol"),
                                confidence = record.optionalFloat("confidence"),
                                rejectionReason = record.optionalString("rejection_reason"),
                            ),
                        )
                        if (events.size > MAX_DETAIL_EVENTS) events.removeFirst()
                    }
                    "motionObservation" -> {
                        if (!record.optBoolean("visible", false)) trackingGaps++
                        record.optionalFloat("confidence")?.let {
                            confidenceTotal += it
                            confidenceCount++
                        }
                    }
                }
            }
        }
        SessionDetail(
            summary = summarizeFile(summary.file),
            acceptedFrames = accepted,
            rejectedFrames = rejected,
            trackingGapObservations = trackingGaps,
            meanConfidence = if (confidenceCount == 0) 0f else (confidenceTotal / confidenceCount).toFloat(),
            plaintextMessages = plaintext.takeLast(50),
            payloadHexValues = payloadHex.takeLast(50),
            rawActions = actions.takeLast(160),
            rawSymbols = symbols.takeLast(160),
            events = events.toList(),
        )
    }

    suspend fun importJson(jsonText: String): SessionSummary = withContext(Dispatchers.IO) {
        require(jsonText.toByteArray(Charsets.UTF_8).size <= MAX_SESSION_BYTES) { "Imported session exceeds 25 MB" }
        val document = JSONObject(jsonText)
        require(document.getInt("schemaVersion") == SCHEMA_VERSION) { "Unsupported session schema" }
        val originalHeader = document.getJSONObject("session")
        val records = document.getJSONArray("records")
        require(records.length() <= MAX_IMPORTED_RECORDS) { "Imported session contains too many records" }

        val importedAt = System.currentTimeMillis()
        val id = "imported-$importedAt-${UUID.randomUUID().toString().take(8)}"
        val file = File(sessionDirectory, "$id.jsonl")
        val header = JSONObject()
            .put("recordType", "session")
            .put("schemaVersion", SCHEMA_VERSION)
            .put("sessionId", id)
            .put("startedEpochMs", originalHeader.optLong("startedEpochMs", importedAt))
            .put("profileName", originalHeader.optString("profileName", "Imported session"))
            .put("dataSource", "IMPORTED")
            .put(
                "sourceLabel",
                originalHeader.optionalString("sourceLabel")
                    ?: "Imported ${originalHeader.optString("sessionId", "session")}",
            )
            .put("importedEpochMs", importedAt)
            .put("originalSessionId", originalHeader.optString("sessionId"))
            .put("originalDataSource", originalHeader.optString("dataSource", "UNKNOWN"))

        mutex.withLock {
            file.writeText(header.toString() + "\n", Charsets.UTF_8)
            for (index in 0 until records.length()) appendBounded(file, records.getJSONObject(index))
        }
        summarizeFile(file)
    }

    suspend fun pruneOlderThan(retentionDays: Int) = withContext(Dispatchers.IO) {
        val cutoffEpochMs = System.currentTimeMillis() - retentionDays.coerceIn(1, 365) * 86_400_000L
        mutex.withLock {
            sessionDirectory.listFiles { file -> file.extension == "jsonl" }
                .orEmpty()
                .filter { it.lastModified() < cutoffEpochMs }
                .forEach(File::delete)
        }
    }

    suspend fun exportJson(summary: SessionSummary): String = withContext(Dispatchers.IO) {
        val lines = summary.file.readLines(Charsets.UTF_8).filter(String::isNotBlank)
        require(lines.isNotEmpty()) { "Session file is empty" }
        val header = JSONObject(lines.first())
        val records = JSONArray()
        lines.drop(1).forEach { records.put(JSONObject(it)) }
        JSONObject()
            .put("schemaVersion", header.getInt("schemaVersion"))
            .put("session", header)
            .put("records", records)
            .toString(2)
    }

    suspend fun exportCsv(summary: SessionSummary): String = withContext(Dispatchers.IO) {
        val keys = listOf(
            "timestamp",
            "session",
            "frame",
            "slot",
            "visible",
            "action",
            "symbol",
            "confidence",
            "position",
            "velocity",
            "yaw_rate",
            "decoder_state",
            "sync_score",
            "accepted",
            "rejection_reason",
        )
        val rows = summary.file.useLines { lines ->
            lines.drop(1).filter(String::isNotBlank).map { line ->
                val record = JSONObject(line)
                val position = if (record.has("centerX") || record.has("centerY")) {
                    "${record.cleanValue("centerX")}|${record.cleanValue("centerY")}"
                } else {
                    ""
                }
                val values = mapOf(
                    "timestamp" to (record.cleanValue("timestamp").ifEmpty { record.cleanValue("timestampEpochMs") }),
                    "session" to summary.id,
                    "frame" to record.cleanValue("frame"),
                    "slot" to record.cleanValue("slot"),
                    "visible" to record.cleanValue("visible"),
                    "action" to record.cleanValue("action"),
                    "symbol" to record.cleanValue("symbol"),
                    "confidence" to record.cleanValue("confidence"),
                    "position" to position,
                    "velocity" to record.cleanValue("velocity"),
                    "yaw_rate" to record.cleanValue("yaw_rate"),
                    "decoder_state" to record.cleanValue("decoder_state"),
                    "sync_score" to record.cleanValue("sync_score"),
                    "accepted" to record.cleanValue("accepted"),
                    "rejection_reason" to record.cleanValue("rejection_reason"),
                )
                keys.joinToString(",") { key -> csvEscape(values.getValue(key)) }
            }.toList()
        }
        (listOf("# schema_version=$SCHEMA_VERSION", keys.joinToString(",")) + rows).joinToString("\r\n")
    }

    private fun summarizeFile(file: File): SessionSummary {
        var header: JSONObject? = null
        var endedEpochMs = 0L
        var eventCount = 0
        var observationCount = 0
        file.useLines { lines ->
            lines.filter(String::isNotBlank).forEachIndexed { index, line ->
                val record = JSONObject(line)
                if (index == 0) {
                    header = record
                    endedEpochMs = record.optLong("startedEpochMs")
                } else {
                    when (record.optString("recordType")) {
                        "decoderEvent" -> eventCount++
                        "motionObservation" -> observationCount++
                    }
                    endedEpochMs = maxOf(endedEpochMs, record.optLong("timestampEpochMs", endedEpochMs))
                }
            }
        }
        val sessionHeader = requireNotNull(header) { "Session header is missing" }
        val startedEpochMs = sessionHeader.getLong("startedEpochMs")
        return SessionSummary(
            id = sessionHeader.getString("sessionId"),
            startedEpochMs = startedEpochMs,
            endedEpochMs = maxOf(endedEpochMs, startedEpochMs),
            durationMs = (endedEpochMs - startedEpochMs).coerceAtLeast(0),
            profileName = sessionHeader.optString("profileName", "Unknown profile"),
            eventCount = eventCount,
            observationCount = observationCount,
            dataSource = sessionHeader.optString("dataSource", "VISUAL_CAMERA"),
            sourceLabel = sessionHeader.optionalString("sourceLabel"),
            file = file,
        )
    }

    private fun appendBounded(file: File, json: JSONObject) {
        if (file.length() >= MAX_SESSION_BYTES) return
        file.appendText(json.toString() + "\n", Charsets.UTF_8)
    }

    companion object {
        private const val SCHEMA_VERSION = 1
        private const val MAX_SESSION_BYTES = 25L * 1024L * 1024L
        private const val MAX_IMPORTED_RECORDS = 250_000
        private const val MAX_DETAIL_EVENTS = 1_000

        fun formattedStartTime(epochMs: Long): String = DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss")
            .withZone(ZoneId.systemDefault())
            .format(Instant.ofEpochMilli(epochMs))

        fun formattedDuration(durationMs: Long): String {
            val totalSeconds = durationMs.coerceAtLeast(0) / 1_000
            val hours = totalSeconds / 3_600
            val minutes = (totalSeconds % 3_600) / 60
            val seconds = totalSeconds % 60
            return if (hours > 0) "%02d:%02d:%02d".format(hours, minutes, seconds) else "%02d:%02d".format(minutes, seconds)
        }

        private fun csvEscape(value: String): String = "\"${value.replace("\"", "\"\"")}\""
    }
}

private inline fun <reified T : Enum<T>> String.toEnumOrDefault(default: T): T =
    enumValues<T>().firstOrNull { it.name == this } ?: default

private fun JSONObject.optionalString(key: String): String? =
    if (!has(key) || isNull(key)) null else optString(key).takeUnless { it == "null" }

private fun JSONObject.optionalLong(key: String): Long? =
    if (!has(key) || isNull(key)) null else optLong(key)

private fun JSONObject.optionalFloat(key: String): Float? =
    if (!has(key) || isNull(key)) null else optDouble(key).toFloat()

private fun JSONObject.cleanValue(key: String): String =
    if (!has(key) || isNull(key)) "" else opt(key)?.toString().takeUnless { it == "null" }.orEmpty()
