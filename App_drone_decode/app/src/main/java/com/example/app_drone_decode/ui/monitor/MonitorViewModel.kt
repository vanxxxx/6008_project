package com.example.app_drone_decode.ui.monitor

import android.app.Application
import android.net.Uri
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.example.app_drone_decode.AppContainer
import com.example.app_drone_decode.data.config.ProfileJsonCodec
import com.example.app_drone_decode.data.logs.SessionSummary
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderLogEntry
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderProfileValidator
import com.example.app_drone_decode.domain.model.DecoderState
import com.example.app_drone_decode.domain.model.FunctionSettings
import com.example.app_drone_decode.domain.model.LogSeverity
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.domain.model.SlotObservation
import com.example.app_drone_decode.vision.SlotAggregator
import com.example.app_drone_decode.vision.ReplaySyncAligner
import com.example.app_drone_decode.vision.ReplayCandidateConsensus
import com.example.app_drone_decode.vision.VideoReplayProcessor
import java.util.concurrent.atomic.AtomicLong
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext

class MonitorViewModel(
    application: Application,
    private val container: AppContainer,
) : AndroidViewModel(application) {
    private val mutableState = MutableStateFlow(MonitorUiState())
    val state: StateFlow<MonitorUiState> = mutableState.asStateFlow()
    private val slotAggregator = SlotAggregator(DecoderProfile())
    private val slotChannel = Channel<SlotObservation>(capacity = 64)
    private val observationLogChannel = Channel<PendingObservationLog>(
        capacity = 64,
        onBufferOverflow = BufferOverflow.DROP_OLDEST,
    )
    private val lastUiUpdateNs = AtomicLong(0L)
    private val lastEvidenceLogNs = AtomicLong(-REALTIME_LOG_INTERVAL_NS)
    private val analyzedFrameIndex = AtomicLong(0L)
    private var currentSession: SessionSummary? = null
    private var replayJob: Job? = null
    private val videoReplayProcessor = VideoReplayProcessor(application.applicationContext)
    private val decoderMutex = Mutex()

    init {
        viewModelScope.launch {
            container.profileRepository.activeProfile.collectLatest { profile ->
                slotAggregator.updateProfile(profile)
                mutableState.update {
                    it.copy(
                        profile = profile,
                        timingWarning = timingWarning(it.effectiveFps, profile),
                    )
                }
            }
        }
        viewModelScope.launch {
            container.functionSettingsRepository.settings.collectLatest { settings ->
                mutableState.update { it.copy(functionSettings = settings) }
            }
        }
        viewModelScope.launch {
            val initialProfile = container.profileRepository.activeProfile.first()
            val initialSettings = container.functionSettingsRepository.settings.first()
            container.sessionLogStore.pruneOlderThan(initialSettings.retentionDays)
            currentSession = container.sessionLogStore.startSession(initialProfile.profileName)
            mutableState.update { it.copy(currentSessionId = currentSession?.id) }
            refreshSessions()
        }
        viewModelScope.launch {
            for (slot in slotChannel) processSlot(slot)
        }
        viewModelScope.launch {
            for (pending in observationLogChannel) {
                pending.session?.let { session ->
                    container.sessionLogStore.appendObservation(
                        session = session,
                        frameIndex = pending.frameIndex,
                        slotIndex = pending.slotIndex,
                        observation = pending.observation,
                        actionSymbol = pending.actionSymbol,
                        decoderState = pending.decoderState,
                        syncScore = pending.syncScore,
                    )
                }
            }
        }
    }

    fun onCameraState(value: String) {
        mutableState.update { it.copy(cameraState = value) }
    }

    fun onRecordingState(recording: Boolean, label: String?) {
        if (recording) lastEvidenceLogNs.set(-REALTIME_LOG_INTERVAL_NS)
        mutableState.update { it.copy(isRecording = recording, recordingLabel = label) }
        appendLog(
            severity = if (recording) LogSeverity.WARNING else LogSeverity.INFO,
            message = if (recording) "Video recording started: $label" else "Video recording stopped: $label",
        )
    }

    fun onCameraObservation(observation: MotionObservation, fps: Float, latencyMs: Float) {
        if (mutableState.value.isReplayProcessing) return
        val frameIndex = analyzedFrameIndex.getAndIncrement()
        val current = mutableState.value
        if (current.isDecoding && !current.injectedSession) {
            slotAggregator.add(observation).forEach { slot ->
                if (slotChannel.trySend(slot).isFailure) {
                    appendLog(LogSeverity.ERROR, "Slot queue is full; decoder session was reset")
                    resetSession()
                }
            }
        }

        publishObservation(observation, fps, latencyMs, frameIndex, currentSession)
    }

    private fun publishObservation(
        observation: MotionObservation,
        fps: Float,
        latencyMs: Float,
        frameIndex: Long,
        session: SessionSummary?,
    ) {
        val current = mutableState.value
        val likely = observation.actionProbabilities.maxByOrNull { it.value }
            ?.toPair() ?: (ActionClass.UNKNOWN to 1f)
        publishRealtimeEvidenceLog(observation, likely, current)
        if (current.functionSettings.retainRawObservations) {
            val symbol = if (likely.first == ActionClass.UNKNOWN) {
                "??"
            } else {
                current.profile.actionMapping[likely.first] ?: "??"
            }
            observationLogChannel.trySend(
                PendingObservationLog(
                    session = session,
                    frameIndex = frameIndex,
                    slotIndex = current.currentSlotIndex,
                    observation = observation,
                    actionSymbol = symbol,
                    decoderState = current.decoderState,
                    syncScore = current.syncScore,
                ),
            )
        }
        val previousUpdate = lastUiUpdateNs.get()
        if (observation.timestampNs - previousUpdate < 100_000_000L ||
            !lastUiUpdateNs.compareAndSet(previousUpdate, observation.timestampNs)
        ) return
        mutableState.update { state ->
            val trajectory = if (observation.visible && observation.centerX != null && observation.centerY != null) {
                (state.trajectory + (observation.centerX to observation.centerY)).takeLast(40)
            } else {
                state.trajectory
            }
            state.copy(
                effectiveFps = fps,
                analysisLatencyMs = latencyMs,
                targetVisible = observation.visible,
                boundingBox = observation.boundingBox,
                orientationDeg = observation.orientationDeg,
                trajectory = trajectory,
                currentAction = likely.first,
                actionConfidence = likely.second,
                normalizedVelocityPerSec = observation.normalizedLinearVelocityPerSec,
                yawRateDegPerSec = observation.yawRateDegPerSec,
                relativeVelocityXPerSec = observation.relativeVelocityXPerSec,
                relativeVelocityYPerSec = observation.relativeVelocityYPerSec,
                relativeAccelerationXPerSec2 = observation.relativeAccelerationXPerSec2,
                relativeAccelerationYPerSec2 = observation.relativeAccelerationYPerSec2,
                timingWarning = timingWarning(fps, state.profile),
            )
        }
    }

    private fun publishRealtimeEvidenceLog(
        observation: MotionObservation,
        likely: Pair<ActionClass, Float>,
        state: MonitorUiState,
    ) {
        if (!state.isReplayProcessing && !state.isRecording && !state.isDecoding) return
        val previous = lastEvidenceLogNs.get()
        if (observation.timestampNs - previous < REALTIME_LOG_INTERVAL_NS ||
            !lastEvidenceLogNs.compareAndSet(previous, observation.timestampNs)
        ) return
        val source = when {
            state.isReplayProcessing -> "Replay ${"%.1f".format(observation.timestampNs / 1_000_000_000.0)} s"
            state.isRecording -> "Recording"
            else -> "Camera"
        }
        val relativeVelocity = formatVector(
            observation.relativeVelocityXPerSec,
            observation.relativeVelocityYPerSec,
            "/s",
        )
        val relativeAcceleration = formatVector(
            observation.relativeAccelerationXPerSec2,
            observation.relativeAccelerationYPerSec2,
            "/s²",
        )
        appendLog(
            LogSeverity.INFO,
            "$source · target=${if (observation.visible) "visible" else "lost"} · " +
                "action=${likely.first.shortName} ${(likely.second * 100).toInt()}% · " +
                "relative v=$relativeVelocity · a=$relativeAcceleration",
        )
    }

    private fun formatVector(x: Float?, y: Float?, unit: String): String =
        if (x != null && y != null) "(${"%.3f".format(x)}, ${"%.3f".format(y)}) $unit" else "—"

    fun startDecoding() {
        viewModelScope.launch {
            if (mutableState.value.isReplayProcessing) return@launch
            if (currentSession?.dataSource != "VISUAL_CAMERA") {
                currentSession = container.sessionLogStore.startSession(
                    profileName = mutableState.value.profile.profileName,
                    dataSource = "VISUAL_CAMERA",
                )
                mutableState.update { it.copy(currentSessionId = currentSession?.id) }
                refreshSessions()
            }
            val configured = decoderMutex.withLock {
                container.decoderFacade.configureProfile(mutableState.value.profile)
            }
            if (configured.isFailure) {
                appendLog(LogSeverity.ERROR, "Decoder profile failed to activate: ${configured.exceptionOrNull()?.message}")
                return@launch
            }
            val result = decoderMutex.withLock { container.decoderFacade.start() }
            if (result.isSuccess) {
                slotAggregator.reset()
                lastEvidenceLogNs.set(-REALTIME_LOG_INTERVAL_NS)
                mutableState.update {
                    it.copy(
                        isDecoding = true,
                        decoderState = DecoderState.SEARCH_SYNC,
                        injectedSession = false,
                        knownPayloadBer = null,
                        replayFileName = null,
                        replayStatus = null,
                        replayProgress = 0f,
                        replayPreviewFrame = null,
                        targetVisible = false,
                        boundingBox = null,
                        orientationDeg = null,
                        trajectory = emptyList(),
                        currentAction = ActionClass.UNKNOWN,
                        actionConfidence = 0f,
                    )
                }
                appendLog(LogSeverity.INFO, "Visual decoding started")
            } else {
                appendLog(LogSeverity.ERROR, "Decoder failed to start: ${result.exceptionOrNull()?.message}")
            }
        }
    }

    fun pauseDecoding() {
        if (mutableState.value.isReplayProcessing) {
            cancelVideoReplay()
            return
        }
        viewModelScope.launch {
            decoderMutex.withLock { container.decoderFacade.pause() }
            mutableState.update { it.copy(isDecoding = false, decoderState = DecoderState.IDLE) }
            appendLog(LogSeverity.INFO, "Decoding paused")
        }
    }

    fun resetSession() {
        replayJob?.cancel()
        viewModelScope.launch {
            decoderMutex.withLock { container.decoderFacade.reset() }
            slotAggregator.reset()
            if (currentSession?.dataSource != "VISUAL_CAMERA") {
                currentSession = container.sessionLogStore.startSession(
                    profileName = mutableState.value.profile.profileName,
                    dataSource = "VISUAL_CAMERA",
                )
                refreshSessions()
            }
            mutableState.update {
                it.copy(
                    currentSessionId = currentSession?.id,
                    isDecoding = false,
                    injectedSession = false,
                    knownPayloadBer = null,
                    decoderState = DecoderState.IDLE,
                    syncScore = 0f,
                    matchedSyncHistory = emptyList(),
                    currentSlotIndex = 0,
                    frameCollectionProgress = 0,
                    rawActions = emptyList(),
                    rawSymbols = emptyList(),
                    acceptedMessages = emptyList(),
                    lastFrame = null,
                    trajectory = emptyList(),
                    targetVisible = false,
                    boundingBox = null,
                    orientationDeg = null,
                    currentAction = ActionClass.UNKNOWN,
                    actionConfidence = 0f,
                    isReplayProcessing = false,
                    replayProgress = 0f,
                    replayFileName = null,
                    replayStatus = null,
                    replayPreviewFrame = null,
                )
            }
            appendLog(LogSeverity.INFO, "Session decoder state reset")
        }
    }

    fun markEvent() {
        appendLog(LogSeverity.WARNING, "User event marker")
    }

    fun processVideo(uri: Uri) {
        if (mutableState.value.isRecording) {
            appendLog(LogSeverity.ERROR, "Stop recording before opening a video file")
            return
        }
        replayJob?.cancel()
        mutableState.update {
            it.copy(
                isReplayProcessing = true,
                replayProgress = 0f,
                replayFileName = videoReplayProcessor.displayName(uri),
                replayStatus = "Preparing video",
                replayPreviewFrame = null,
            )
        }
        replayJob = viewModelScope.launch {
            val activeReplayJob = currentCoroutineContext()[Job]
            val profile = mutableState.value.profile
            val displayName = videoReplayProcessor.displayName(uri)
            try {
                while (slotChannel.tryReceive().isSuccess) {
                    // Discard live-camera slots captured before replay was selected.
                }
                decoderMutex.withLock {
                    container.decoderFacade.configureProfile(profile).getOrThrow()
                    container.decoderFacade.reset().getOrThrow()
                    container.decoderFacade.start().getOrThrow()
                }
                slotAggregator.updateProfile(profile)
                slotAggregator.reset()
                analyzedFrameIndex.set(0L)
                lastUiUpdateNs.set(-1_000_000_000L)
                lastEvidenceLogNs.set(-REALTIME_LOG_INTERVAL_NS)
                currentSession = container.sessionLogStore.startSession(
                    profileName = profile.profileName,
                    dataSource = "VIDEO_REPLAY",
                    sourceLabel = displayName,
                )
                mutableState.update {
                    it.copy(
                        currentSessionId = currentSession?.id,
                        isDecoding = true,
                        isReplayProcessing = true,
                        replayProgress = 0f,
                        replayFileName = displayName,
                        replayStatus = "Preparing video",
                        replayPreviewFrame = null,
                        injectedSession = false,
                        knownPayloadBer = null,
                        decoderState = DecoderState.SEARCH_SYNC,
                        syncScore = 0f,
                        matchedSyncHistory = emptyList(),
                        currentSlotIndex = 0L,
                        frameCollectionProgress = 0,
                        rawActions = emptyList(),
                        rawSymbols = emptyList(),
                        acceptedMessages = emptyList(),
                        lastFrame = null,
                        trajectory = emptyList(),
                        targetVisible = false,
                        boundingBox = null,
                        orientationDeg = null,
                        currentAction = ActionClass.UNKNOWN,
                        actionConfidence = 0f,
                        logs = emptyList(),
                    )
                }
                appendLog(LogSeverity.INFO, "Video replay started: $displayName")
                if (!profile.calibration.calibrated) {
                    appendLog(
                        LogSeverity.INFO,
                        "The active profile is not camera-calibrated; replay will self-calibrate from the public SYNC word",
                    )
                }
                val replaySession = currentSession
                val replayObservations = ArrayList<MotionObservation>()
                val requiredFrameDurationMs = 40L * profile.symbolDurationMs
                var durationLimitation: String? = null
                val result = videoReplayProcessor.process(
                    uri = uri,
                    profile = profile,
                    onMetadata = { durationMs ->
                        durationLimitation = if (durationMs < requiredFrameDurationMs) {
                            "${"%.1f".format(durationMs / 1_000f)} s video < ${"%.1f".format(requiredFrameDurationMs / 1_000f)} s frame"
                        } else {
                            null
                        }
                    },
                    onObservation = { observation, fps, latencyMs ->
                        val frameIndex = analyzedFrameIndex.getAndIncrement()
                        check(replayObservations.size < MAX_REPLAY_OBSERVATIONS) {
                            "Video is too long for bounded offline synchronization"
                        }
                        replayObservations += observation
                        publishObservation(observation, fps, latencyMs, frameIndex, replaySession)
                    },
                    onPreviewFrame = { bitmap ->
                        mutableState.update { it.copy(replayPreviewFrame = bitmap) }
                    },
                    onProgress = { progress ->
                        mutableState.update {
                            it.copy(
                                replayProgress = progress,
                                replayStatus = buildString {
                                    append("Analyzing · ${(progress * 100).toInt()}%")
                                    if (!profile.calibration.calibrated) append(" · collecting SYNC calibration")
                                    durationLimitation?.let { append(" · $it") }
                                },
                            )
                        }
                    },
                )
                mutableState.update { it.copy(replayStatus = "Searching slot phase and SYNC") }
                val syncSearch = ReplaySyncAligner(profile).search(replayObservations)
                var alignment = syncSearch.alignment
                var selectionReason = syncSearch.reason
                if (syncSearch.candidates.isNotEmpty()) {
                    mutableState.update { it.copy(replayStatus = "Validating blind motion candidates with BCH") }
                    val candidateDecodes = decoderMutex.withLock {
                        container.decoderFacade.decodeReplayCandidates(syncSearch.candidates).getOrThrow()
                    }
                    val selection = ReplayCandidateConsensus.select(syncSearch.candidates, candidateDecodes)
                    alignment = selection.alignment
                    selectionReason = selection.reason
                    appendLog(
                        if (alignment == null) LogSeverity.WARNING else LogSeverity.INFO,
                        "Replay candidate consensus: ${selection.reason}; " +
                            "${selection.eligibleAcceptedCandidates} BCH-valid candidate(s)",
                    )
                    selection.payloadGroups.forEachIndexed { index, group ->
                        appendLog(
                            LogSeverity.INFO,
                            "Replay BCH group ${index + 1}: payload=${group.payloadHex.ifEmpty { "<empty>" }} " +
                                "seq=${group.sequence ?: "?"} votes=${group.support} " +
                                "evidence=${"%.2f".format(group.evidenceScore)} " +
                                "sync=${group.bestSyncMatches}/8 margin=${"%.3f".format(group.bestMargin)}",
                        )
                    }
                }
                if (alignment != null) {
                    appendLog(
                        LogSeverity.INFO,
                        "Replay SYNC locked at " +
                            "${"%.3f".format(alignment.syncStartTimestampNs / 1_000_000_000.0)} s " +
                            "with ${"%.1f".format(alignment.phaseOffsetMs)} ms slot phase " +
                            "and ${"%.0f".format(alignment.syncScore * 100)}% confidence" +
                            if (alignment.selfCalibrated) " (SYNC self-calibrated)" else "",
                    )
                    appendLog(
                        LogSeverity.INFO,
                        "Replay selected actions: ${alignment.frameSlots.joinToString("") { it.action.shortName }}",
                    )
                    alignment.frameSlots.forEach { processSlot(it) }
                } else {
                    appendLog(
                        LogSeverity.WARNING,
                        "Replay synchronization failed: $selectionReason " +
                            "(best score ${"%.0f".format(syncSearch.bestScore * 100)}%)",
                    )
                }
                val accepted = mutableState.value.acceptedMessages.isNotEmpty()
                val completionReason = if (accepted) {
                    "Decoded ${mutableState.value.acceptedMessages.size} frame(s)"
                } else {
                    buildList {
                        durationLimitation?.let(::add)
                        if (alignment == null) add(selectionReason)
                        if (isEmpty()) add("The aligned frame did not pass BCH validation")
                    }.joinToString(separator = " · ", prefix = "No frame · ")
                }
                appendLog(
                    if (accepted) LogSeverity.ACCEPTED else LogSeverity.WARNING,
                    "Video replay completed in visual mode: ${result.sampledFrames} frames. $completionReason",
                )
                decoderMutex.withLock { container.decoderFacade.pause() }
                mutableState.update {
                    it.copy(
                        isDecoding = false,
                        isReplayProcessing = false,
                        replayProgress = 1f,
                        replayStatus = completionReason,
                        decoderState = DecoderState.IDLE,
                    )
                }
                refreshSessions()
            } catch (cancelled: CancellationException) {
                withContext(NonCancellable) {
                    decoderMutex.withLock { container.decoderFacade.pause() }
                    mutableState.update {
                        it.copy(
                            isDecoding = false,
                            isReplayProcessing = false,
                            replayStatus = "Analysis cancelled",
                            decoderState = DecoderState.IDLE,
                        )
                    }
                    appendLog(LogSeverity.WARNING, "Video replay cancelled")
                    refreshSessions()
                }
            } catch (error: Exception) {
                decoderMutex.withLock { container.decoderFacade.pause() }
                mutableState.update {
                    it.copy(
                        isDecoding = false,
                        isReplayProcessing = false,
                        replayStatus = "Analysis failed",
                        decoderState = DecoderState.ERROR,
                    )
                }
                appendLog(LogSeverity.ERROR, "Video replay failed: ${error.message}")
                refreshSessions()
            } finally {
                if (replayJob === activeReplayJob) replayJob = null
            }
        }
    }

    fun cancelVideoReplay() {
        replayJob?.cancel()
    }

    fun injectHelloReference() {
        viewModelScope.launch {
            val actions = decoderMutex.withLock {
                container.decoderFacade.configureProfile(DecoderProfile()).getOrElse { error ->
                    appendLog(LogSeverity.ERROR, "Reference profile failed to activate: ${error.message}")
                    return@launch
                }
                container.decoderFacade.reset()
                container.decoderFacade.start()
                container.decoderFacade.encodeReferenceFrame().getOrElse { error ->
                    appendLog(LogSeverity.ERROR, "Reference injection failed: ${error.message}")
                    return@launch
                }
            }
            mutableState.update {
                it.copy(
                    isDecoding = true,
                    injectedSession = true,
                    knownPayloadBer = if (it.functionSettings.knownPayloadBerMode) 0f else null,
                    decoderState = DecoderState.SEARCH_SYNC,
                    rawActions = emptyList(),
                    rawSymbols = emptyList(),
                )
            }
            appendLog(LogSeverity.WARNING, "Injected data session started with the protocol v3 Hello vector")
            actions.forEachIndexed { index, action ->
                slotChannel.send(SlotObservation(index.toLong(), action, 1f, false, 15))
            }
        }
    }

    fun saveProfile(profile: DecoderProfile) {
        if (mutableState.value.isDecoding) {
            mutableState.update { it.copy(configMessage = "Pause decoding before activating a profile.") }
            return
        }
        val validation = DecoderProfileValidator.validate(profile)
        if (!validation.valid) {
            mutableState.update { it.copy(configMessage = validation.errors.joinToString("\n")) }
            return
        }
        val activeProfile = mutableState.value.profile
        val transmittedMeaningChanged = profile.actionMapping != activeProfile.actionMapping ||
            profile.syncActions != activeProfile.syncActions
        if (transmittedMeaningChanged &&
            (profile.profileId == activeProfile.profileId || profile.profileVersion <= activeProfile.profileVersion)
        ) {
            mutableState.update {
                it.copy(configMessage = "Mapping or SYNC changes require a duplicated profile with a higher profile version.")
            }
            return
        }
        viewModelScope.launch {
            val pythonErrors = container.decoderFacade.validateProfile(profile).getOrElse { error ->
                listOf(error.message ?: "Python profile validation failed")
            }
            if (pythonErrors.isNotEmpty()) {
                mutableState.update { it.copy(configMessage = pythonErrors.joinToString("\n")) }
                return@launch
            }
            container.profileRepository.save(profile)
            mutableState.update { it.copy(configMessage = "Profile saved and activated.") }
        }
    }

    fun restoreDefaultProfile() {
        if (mutableState.value.isDecoding) {
            mutableState.update { it.copy(configMessage = "Pause decoding before restoring defaults.") }
            return
        }
        viewModelScope.launch {
            container.profileRepository.restoreDefaults()
            mutableState.update { it.copy(configMessage = "Protocol v3 defaults restored.") }
        }
    }

    fun restoreLegacyStationaryHoldProfile() {
        if (mutableState.value.isDecoding) {
            mutableState.update { it.copy(configMessage = "Pause decoding before restoring a legacy profile.") }
            return
        }
        viewModelScope.launch {
            container.profileRepository.restoreLegacyStationaryHold()
            mutableState.update { it.copy(configMessage = "Legacy stationary-hold v4 profile restored.") }
        }
    }

    fun saveFunctionSettings(settings: FunctionSettings) {
        viewModelScope.launch {
            runCatching { container.functionSettingsRepository.save(settings) }
                .onSuccess {
                    container.sessionLogStore.pruneOlderThan(settings.retentionDays)
                    mutableState.update { it.copy(configMessage = "Function settings saved.") }
                }
                .onFailure { error -> mutableState.update { it.copy(configMessage = error.message) } }
        }
    }

    fun clearConfigMessage() {
        mutableState.update { it.copy(configMessage = null) }
    }

    fun exportSession(uri: Uri, format: String) {
        val session = currentSession
        if (session == null) {
            appendLog(LogSeverity.ERROR, "The session log is not ready")
            return
        }
        exportStoredSession(session, uri, format, announceInMonitor = true)
    }

    fun exportStoredSession(
        session: SessionSummary,
        uri: Uri,
        format: String,
        announceInMonitor: Boolean = false,
    ) {
        viewModelScope.launch {
            runCatching {
                withContext(Dispatchers.IO) {
                    val content = if (format.uppercase() == "CSV") {
                        container.sessionLogStore.exportCsv(session)
                    } else {
                        container.sessionLogStore.exportJson(session)
                    }
                    getApplication<Application>().contentResolver.openOutputStream(uri)?.use { output ->
                        output.write(content.toByteArray(Charsets.UTF_8))
                    } ?: error("The selected export destination could not be opened")
                }
            }.onSuccess {
                if (announceInMonitor) {
                    appendLog(LogSeverity.INFO, "Session exported as ${format.uppercase()}")
                } else {
                    mutableState.update { it.copy(logsMessage = "Session exported as ${format.uppercase()}.") }
                }
            }.onFailure { error ->
                if (announceInMonitor) {
                    appendLog(LogSeverity.ERROR, "Export failed: ${error.message}")
                } else {
                    mutableState.update { it.copy(logsMessage = "Export failed: ${error.message}") }
                }
            }
        }
    }

    fun importSession(uri: Uri) {
        viewModelScope.launch {
            runCatching {
                val text = withContext(Dispatchers.IO) {
                    getApplication<Application>().contentResolver.openInputStream(uri)?.use { input ->
                        input.readBytes().toString(Charsets.UTF_8)
                    } ?: error("The selected session file could not be opened")
                }
                container.sessionLogStore.importJson(text)
            }.onSuccess { imported ->
                refreshSessions()
                val detail = container.sessionLogStore.loadDetail(imported)
                mutableState.update {
                    it.copy(
                        selectedSession = imported,
                        selectedSessionDetail = detail,
                        logsMessage = "Session imported.",
                    )
                }
            }.onFailure { error ->
                mutableState.update { it.copy(logsMessage = "Import failed: ${error.message}") }
            }
        }
    }

    fun exportProfile(uri: Uri) {
        viewModelScope.launch {
            runCatching {
                withContext(Dispatchers.IO) {
                    getApplication<Application>().contentResolver.openOutputStream(uri)?.use { output ->
                        output.write(ProfileJsonCodec.encode(mutableState.value.profile).toByteArray(Charsets.UTF_8))
                    } ?: error("The selected profile destination could not be opened")
                }
            }.onSuccess {
                mutableState.update { it.copy(configMessage = "Profile exported.") }
            }.onFailure { error ->
                mutableState.update { it.copy(configMessage = "Profile export failed: ${error.message}") }
            }
        }
    }

    fun importProfile(uri: Uri) {
        if (mutableState.value.isDecoding) {
            mutableState.update { it.copy(configMessage = "Pause decoding before importing a profile.") }
            return
        }
        viewModelScope.launch {
            runCatching {
                withContext(Dispatchers.IO) {
                    val text = getApplication<Application>().contentResolver.openInputStream(uri)?.use { input ->
                        input.readBytes().toString(Charsets.UTF_8)
                    } ?: error("The selected profile could not be opened")
                    ProfileJsonCodec.decode(text)
                }
            }.onSuccess(::saveProfile)
                .onFailure { error -> mutableState.update { it.copy(configMessage = "Profile import failed: ${error.message}") } }
        }
    }

    fun selectSession(summary: SessionSummary) {
        viewModelScope.launch {
            runCatching { container.sessionLogStore.loadDetail(summary) }
                .onSuccess { detail ->
                    mutableState.update { it.copy(selectedSession = detail.summary, selectedSessionDetail = detail) }
                }
                .onFailure { error ->
                    mutableState.update { it.copy(logsMessage = "Session read failed: ${error.message}") }
                }
        }
    }

    fun closeSessionDetail() {
        mutableState.update { it.copy(selectedSession = null, selectedSessionDetail = null) }
    }

    fun clearLogsMessage() {
        mutableState.update { it.copy(logsMessage = null) }
    }

    fun refreshSessionList() {
        viewModelScope.launch { refreshSessions() }
    }

    private suspend fun processSlot(slot: SlotObservation) {
        val event = decoderMutex.withLock { container.decoderFacade.process(slot) }
        val result = event.result
        val severity = when {
            !event.ok -> LogSeverity.ERROR
            result?.accepted == true -> LogSeverity.ACCEPTED
            result != null -> LogSeverity.REJECTED
            slot.erased -> LogSeverity.WARNING
            else -> LogSeverity.INFO
        }
        val message = when {
            !event.ok -> event.errorMessage ?: "Decoder bridge error"
            result?.accepted == true -> "Accepted frame ${result.sequence}: ${result.payloadText}"
            result != null -> "Rejected frame: ${result.rejectionReason}"
            slot.erased -> "Slot ${slot.slotIndex} is an explicit erasure"
            else -> "Slot ${slot.slotIndex}: ${slot.action.shortName} / ${event.symbol}"
        }
        val entry = DecoderLogEntry(
            timestampEpochMs = System.currentTimeMillis(),
            slotIndex = slot.slotIndex,
            severity = severity,
            state = event.decoderState,
            message = message,
            action = slot.action,
            symbol = event.symbol,
            confidence = slot.confidence,
            rejectionReason = result?.rejectionReason,
            payloadText = result?.payloadText,
            payloadHex = result?.payloadHex,
            sequence = result?.sequence,
            correctedBitErrors = result?.correctedBitErrors,
            erasedBits = result?.erasedBits,
        )
        mutableState.update { state ->
            state.copy(
                decoderState = event.decoderState,
                syncScore = event.syncScore,
                matchedSyncHistory = event.matchedSyncHistory,
                currentSlotIndex = slot.slotIndex,
                frameCollectionProgress = event.frameCollectionProgress,
                rawActions = (state.rawActions + slot.action.shortName).takeLast(120),
                rawSymbols = (state.rawSymbols + event.symbol).takeLast(120),
                acceptedMessages = if (result?.accepted == true) {
                    (state.acceptedMessages + result.payloadText).takeLast(50)
                } else {
                    state.acceptedMessages
                },
                lastFrame = result ?: state.lastFrame,
                logs = (state.logs + entry).takeLast(200),
            )
        }
        persist(entry)
    }

    private fun appendLog(severity: LogSeverity, message: String) {
        val entry = DecoderLogEntry(
            timestampEpochMs = System.currentTimeMillis(),
            severity = severity,
            state = mutableState.value.decoderState,
            message = message,
        )
        mutableState.update { it.copy(logs = (it.logs + entry).takeLast(200)) }
        viewModelScope.launch { persist(entry) }
    }

    private suspend fun persist(entry: DecoderLogEntry) {
        currentSession?.let { container.sessionLogStore.append(it, entry) }
    }

    private suspend fun refreshSessions() {
        mutableState.update { it.copy(sessions = container.sessionLogStore.listSessions()) }
    }

    private fun timingWarning(fps: Float, profile: DecoderProfile): String? {
        if (fps <= 0f) return null
        val actionFrames = fps * profile.actionDurationMs / 1_000f
        val idleFrames = fps * profile.secondaryPhaseDurationMs / 1_000f
        return when {
            actionFrames < 5f -> "Only ${"%.1f".format(actionFrames)} frames are expected in each action window."
            profile.secondaryPhaseDurationMs > 0 && idleFrames < 3f ->
                "Only ${"%.1f".format(idleFrames)} frames are expected in each return/reference window."
            else -> null
        }
    }

    private companion object {
        const val MAX_REPLAY_OBSERVATIONS = 200_000
        const val REALTIME_LOG_INTERVAL_NS = 1_000_000_000L
    }
}

private data class PendingObservationLog(
    val session: SessionSummary?,
    val frameIndex: Long,
    val slotIndex: Long,
    val observation: MotionObservation,
    val actionSymbol: String,
    val decoderState: DecoderState,
    val syncScore: Float,
)
