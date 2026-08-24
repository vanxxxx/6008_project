package com.example.app_drone_decode.domain.model

enum class ActionClass(val shortName: String, val symbol: String) {
    HOVER("H", "00"),
    FORWARD("F", "01"),
    YAW_LEFT("L", "11"),
    YAW_RIGHT("R", "10"),
    UNKNOWN("?", "??");

    companion object {
        fun fromShortName(value: String): ActionClass = entries.firstOrNull {
            it.shortName == value.uppercase()
        } ?: UNKNOWN
    }
}

enum class DecoderState {
    IDLE,
    SEARCH_SYNC,
    COLLECT_FRAME,
    BCH_DECODE,
    ACCEPT_FRAME,
    REJECT_FRAME,
    ERROR,
}

data class NormalizedRect(
    val left: Float,
    val top: Float,
    val right: Float,
    val bottom: Float,
) {
    val centerX: Float get() = (left + right) / 2f
    val centerY: Float get() = (top + bottom) / 2f
}

data class MotionObservation(
    val timestampNs: Long,
    val visible: Boolean,
    val centerX: Float?,
    val centerY: Float?,
    val normalizedLinearVelocityPerSec: Float?,
    val yawRateDegPerSec: Float?,
    val actionProbabilities: Map<ActionClass, Float>,
    val confidence: Float,
    val boundingBox: NormalizedRect? = null,
    val orientationDeg: Float? = null,
    val sceneVelocityXPerSec: Float? = null,
    val sceneVelocityYPerSec: Float? = null,
    val sceneRotationDegPerSec: Float? = null,
    val sceneScaleRatePerSec: Float? = null,
    val targetAreaFraction: Float? = null,
)

data class SlotObservation(
    val slotIndex: Long,
    val action: ActionClass,
    val confidence: Float,
    val erased: Boolean,
    val sampleCount: Int,
)

data class DecodedFrame(
    val accepted: Boolean,
    val sequence: Int? = null,
    val payloadText: String = "",
    val payloadHex: String = "",
    val correctedBitErrors: Int = 0,
    val erasedBits: Int = 0,
    val meanConfidence: Float = 0f,
    val possibleMissingFrames: Int = 0,
    val rejectionReason: String? = null,
)

data class DecoderBridgeEvent(
    val ok: Boolean,
    val slotIndex: Long?,
    val action: ActionClass,
    val symbol: String,
    val decoderState: DecoderState,
    val nextState: DecoderState,
    val syncScore: Float,
    val matchedSyncHistory: List<String>,
    val frameCollectionProgress: Int,
    val result: DecodedFrame?,
    val errorMessage: String? = null,
)

enum class LogSeverity {
    INFO,
    ACCEPTED,
    WARNING,
    REJECTED,
    ERROR,
}

data class DecoderLogEntry(
    val timestampEpochMs: Long,
    val slotIndex: Long? = null,
    val severity: LogSeverity,
    val state: DecoderState,
    val message: String,
    val action: ActionClass? = null,
    val symbol: String? = null,
    val confidence: Float? = null,
    val rejectionReason: String? = null,
    val payloadText: String? = null,
    val payloadHex: String? = null,
    val sequence: Int? = null,
    val correctedBitErrors: Int? = null,
    val erasedBits: Int? = null,
)
