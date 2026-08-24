package com.example.app_drone_decode.ui.monitor

import android.graphics.Bitmap
import com.example.app_drone_decode.data.logs.SessionSummary
import com.example.app_drone_decode.data.logs.SessionDetail
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecodedFrame
import com.example.app_drone_decode.domain.model.DecoderLogEntry
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderState
import com.example.app_drone_decode.domain.model.FunctionSettings
import com.example.app_drone_decode.domain.model.NormalizedRect

data class MonitorUiState(
    val cameraState: String = "Permission required",
    val effectiveFps: Float = 0f,
    val analysisLatencyMs: Float = 0f,
    val targetVisible: Boolean = false,
    val boundingBox: NormalizedRect? = null,
    val orientationDeg: Float? = null,
    val trajectory: List<Pair<Float, Float>> = emptyList(),
    val currentAction: ActionClass = ActionClass.UNKNOWN,
    val actionConfidence: Float = 0f,
    val normalizedVelocityPerSec: Float? = null,
    val yawRateDegPerSec: Float? = null,
    val isDecoding: Boolean = false,
    val isRecording: Boolean = false,
    val recordingLabel: String? = null,
    val isReplayProcessing: Boolean = false,
    val replayProgress: Float = 0f,
    val replayFileName: String? = null,
    val replayStatus: String? = null,
    val replayPreviewFrame: Bitmap? = null,
    val injectedSession: Boolean = false,
    val knownPayloadBer: Float? = null,
    val decoderState: DecoderState = DecoderState.IDLE,
    val syncScore: Float = 0f,
    val matchedSyncHistory: List<String> = emptyList(),
    val currentSlotIndex: Long = 0,
    val frameCollectionProgress: Int = 0,
    val rawActions: List<String> = emptyList(),
    val rawSymbols: List<String> = emptyList(),
    val acceptedMessages: List<String> = emptyList(),
    val lastFrame: DecodedFrame? = null,
    val logs: List<DecoderLogEntry> = emptyList(),
    val profile: DecoderProfile = DecoderProfile(),
    val functionSettings: FunctionSettings = FunctionSettings(),
    val timingWarning: String? = null,
    val configMessage: String? = null,
    val sessions: List<SessionSummary> = emptyList(),
    val currentSessionId: String? = null,
    val selectedSession: SessionSummary? = null,
    val selectedSessionDetail: SessionDetail? = null,
    val logsMessage: String? = null,
)
