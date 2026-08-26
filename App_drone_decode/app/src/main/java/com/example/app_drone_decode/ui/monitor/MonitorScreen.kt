package com.example.app_drone_decode.ui.monitor

import android.Manifest
import android.content.pm.PackageManager
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.camera.view.PreviewView
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.detectHorizontalDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.requiredWidth
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.outlined.FiberManualRecord
import androidx.compose.material.icons.outlined.Flag
import androidx.compose.material.icons.outlined.Pause
import androidx.compose.material.icons.outlined.PlayArrow
import androidx.compose.material.icons.outlined.Refresh
import androidx.compose.material.icons.outlined.Stop
import androidx.compose.material.icons.outlined.VideoFile
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.ContextCompat
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import com.example.app_drone_decode.camera.CameraSession
import com.example.app_drone_decode.camera.MotionImageAnalyzer
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderLogEntry
import com.example.app_drone_decode.domain.model.LogSeverity
import com.example.app_drone_decode.ui.components.MetricRow
import com.example.app_drone_decode.ui.components.SectionSurface
import com.example.app_drone_decode.ui.theme.AcceptedGreen
import com.example.app_drone_decode.ui.theme.RecordingRed
import com.example.app_drone_decode.ui.theme.RejectedRed
import com.example.app_drone_decode.ui.theme.WarningAmber
import com.example.app_drone_decode.vision.AdaptiveTargetTracker
import com.example.app_drone_decode.vision.MotionEstimator
import com.example.app_drone_decode.vision.ThresholdActionClassifier
import com.example.app_drone_decode.R
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.cos
import kotlin.math.sin

@Composable
fun MonitorScreen(state: MonitorUiState, viewModel: MonitorViewModel) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current
    var permissionGranted by remember {
        mutableStateOf(ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
    }
    val permissionLauncher = rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) {
        permissionGranted = it
        viewModel.onCameraState(if (it) "Starting" else "Permission denied")
    }
    val currentProfile by rememberUpdatedState(state.profile)
    val currentUiState by rememberUpdatedState(state)
    val analyzer = remember {
        val estimator = MotionEstimator()
        MotionImageAnalyzer(
            tracker = AdaptiveTargetTracker(roiProvider = { currentProfile.roi }),
            estimator = estimator,
            classifier = ThresholdActionClassifier { currentProfile },
            profileProvider = { currentProfile },
            onObservation = viewModel::onCameraObservation,
        )
    }
    val cameraSession = remember {
        CameraSession(
            context = context.applicationContext,
            analyzer = analyzer,
            onCameraState = viewModel::onCameraState,
            onRecordingState = viewModel::onRecordingState,
        )
    }
    DisposableEffect(lifecycleOwner) {
        val observer = LifecycleEventObserver { _, event ->
            if (event == Lifecycle.Event.ON_STOP) {
                if (currentUiState.isDecoding) viewModel.pauseDecoding()
                if (currentUiState.isRecording) cameraSession.stopRecording()
            }
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        onDispose {
            lifecycleOwner.lifecycle.removeObserver(observer)
            cameraSession.close()
            if (currentUiState.isDecoding) viewModel.pauseDecoding()
        }
    }
    LaunchedEffect(state.functionSettings.torchEnabled, state.cameraState) {
        cameraSession.setTorch(state.functionSettings.torchEnabled)
    }
    LaunchedEffect(state.isReplayProcessing, state.replayFileName) {
        cameraSession.setAnalysisEnabled(!state.isReplayProcessing && state.replayFileName == null)
    }
    val exportLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("application/json"),
    ) { uri -> uri?.let { viewModel.exportSession(it, "JSON") } }
    val videoLauncher = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        uri?.let(viewModel::processVideo)
    }

    BoxWithConstraints(Modifier.fillMaxSize().padding(8.dp)) {
        val landscapeLayout = maxWidth >= 840.dp || maxWidth > maxHeight * 1.25f
        val totalWidthPx = with(LocalDensity.current) { maxWidth.toPx() }
        if (landscapeLayout) {
            var rightFraction by remember { mutableFloatStateOf(0.40f) }
            Row(Modifier.fillMaxSize()) {
                CameraAndControls(
                    state = state,
                    modifier = Modifier.weight(1f - rightFraction).fillMaxHeight(),
                    permissionGranted = permissionGranted,
                    requestPermission = { permissionLauncher.launch(Manifest.permission.CAMERA) },
                    bindPreview = { preview ->
                        cameraSession.bind(
                            lifecycleOwner,
                            preview,
                            state.functionSettings.requestedResolution,
                            state.functionSettings.requestedFps,
                        )
                    },
                    startRecording = { cameraSession.startRecording(state.functionSettings.recordingLimitMb) },
                    stopRecording = cameraSession::stopRecording,
                    viewModel = viewModel,
                    export = { exportLauncher.launch("motion-session.json") },
                    openVideo = { videoLauncher.launch(arrayOf("video/*")) },
                )
                Box(
                    Modifier
                        .requiredWidth(10.dp)
                        .fillMaxHeight()
                        .pointerInput(totalWidthPx) {
                            detectHorizontalDragGestures { _, dragAmount ->
                                rightFraction = (rightFraction - dragAmount / totalWidthPx.coerceAtLeast(1f))
                                    .coerceIn(0.32f, 0.58f)
                            }
                        },
                    contentAlignment = Alignment.Center,
                ) {
                    Box(Modifier.width(2.dp).fillMaxHeight().background(MaterialTheme.colorScheme.outline))
                }
                DiagnosticsPanel(state, Modifier.weight(rightFraction).fillMaxHeight())
            }
        } else {
            Column(
                Modifier.fillMaxSize().background(MaterialTheme.colorScheme.surface),
            ) {
                CameraAndControls(
                    state = state,
                    modifier = Modifier.fillMaxWidth().weight(1.02f),
                    permissionGranted = permissionGranted,
                    requestPermission = { permissionLauncher.launch(Manifest.permission.CAMERA) },
                    bindPreview = { preview ->
                        cameraSession.bind(
                            lifecycleOwner,
                            preview,
                            state.functionSettings.requestedResolution,
                            state.functionSettings.requestedFps,
                        )
                    },
                    startRecording = { cameraSession.startRecording(state.functionSettings.recordingLimitMb) },
                    stopRecording = cameraSession::stopRecording,
                    viewModel = viewModel,
                    export = { exportLauncher.launch("motion-session.json") },
                    openVideo = { videoLauncher.launch(arrayOf("video/*")) },
                )
                DiagnosticsPanel(state, Modifier.fillMaxWidth().weight(0.98f))
            }
        }
    }
}

@Composable
private fun CameraAndControls(
    state: MonitorUiState,
    modifier: Modifier,
    permissionGranted: Boolean,
    requestPermission: () -> Unit,
    bindPreview: (PreviewView) -> Unit,
    startRecording: () -> Unit,
    stopRecording: () -> Unit,
    viewModel: MonitorViewModel,
    export: () -> Unit,
    openVideo: () -> Unit,
) {
    Column(
        modifier = modifier.background(MaterialTheme.colorScheme.surface),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Box(
            Modifier
                .weight(1f)
                .fillMaxWidth()
                .clip(RoundedCornerShape(10.dp))
                .background(Color.Black, RoundedCornerShape(10.dp))
                .border(1.dp, MaterialTheme.colorScheme.outline, RoundedCornerShape(10.dp)),
        ) {
            if (state.replayFileName != null) {
                state.replayPreviewFrame?.let { bitmap ->
                    Image(
                        bitmap = bitmap.asImageBitmap(),
                        contentDescription = "Current decoded video frame",
                        modifier = Modifier.fillMaxSize(),
                        contentScale = ContentScale.FillBounds,
                    )
                    TrackingOverlay(state)
                } ?: Column(
                    Modifier.align(Alignment.Center).padding(24.dp),
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.spacedBy(8.dp),
                ) {
                    Text("Preparing video", color = Color.White, fontWeight = FontWeight.SemiBold)
                    Text(
                        state.replayFileName.orEmpty(),
                        color = Color.White.copy(alpha = 0.72f),
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            } else if (permissionGranted) {
                AndroidView(
                    factory = { context ->
                        PreviewView(context).apply {
                            scaleType = PreviewView.ScaleType.FILL_CENTER
                            implementationMode = PreviewView.ImplementationMode.COMPATIBLE
                            bindPreview(this)
                        }
                    },
                    modifier = Modifier.fillMaxSize(),
                )
                TrackingOverlay(state)
            } else {
                Column(
                    Modifier.align(Alignment.Center).padding(24.dp).widthIn(max = 420.dp),
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.spacedBy(12.dp),
                ) {
                    Text(stringResource(R.string.camera_permission_reason))
                    Button(onClick = requestPermission) { Text("Grant camera access") }
                }
            }

            Row(
                Modifier.align(Alignment.TopStart).padding(10.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                StatusChip(if (state.targetVisible) "Target visible" else "Target lost", state.targetVisible)
                if (state.injectedSession) StatusChip("Injected data", false, WarningAmber)
                if (state.replayFileName != null) {
                    StatusChip(if (state.isReplayProcessing) "Video replay" else "Video result", false, WarningAmber)
                }
                if (state.isRecording) StatusChip("REC", false, RecordingRed)
            }
            Surface(
                modifier = Modifier.align(Alignment.BottomEnd).padding(10.dp),
                color = Color.Black.copy(alpha = 0.65f),
                shape = RoundedCornerShape(8.dp),
            ) {
                Text(
                    "${state.currentAction.name.replace('_', ' ')}  ${"%.0f".format(state.actionConfidence * 100)}%",
                    modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp),
                    color = Color.White,
                    fontFamily = FontFamily.Monospace,
                    style = MaterialTheme.typography.labelMedium,
                )
            }
            if (state.isReplayProcessing) {
                LinearProgressIndicator(
                    progress = { state.replayProgress },
                    modifier = Modifier.align(Alignment.BottomStart).fillMaxWidth().height(4.dp),
                    color = WarningAmber,
                    trackColor = Color.Black.copy(alpha = 0.5f),
                )
            }
        }

        Surface(
            modifier = Modifier.fillMaxWidth(),
            color = MaterialTheme.colorScheme.surface,
        ) {
            Column(Modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                Button(
                    onClick = {
                        when {
                            state.isReplayProcessing -> viewModel.cancelVideoReplay()
                            state.isDecoding -> viewModel.pauseDecoding()
                            else -> viewModel.startDecoding()
                        }
                    },
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) {
                    Icon(
                        when {
                            state.isReplayProcessing -> Icons.Outlined.Stop
                            state.isDecoding -> Icons.Outlined.Pause
                            else -> Icons.Outlined.PlayArrow
                        },
                        contentDescription = null,
                    )
                    Spacer(Modifier.width(4.dp))
                    Text(if (state.isReplayProcessing) "Cancel" else if (state.isDecoding) "Pause" else "Start")
                }
                OutlinedButton(
                    onClick = viewModel::resetSession,
                    enabled = !state.isReplayProcessing,
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) {
                    Icon(Icons.Outlined.Refresh, contentDescription = null)
                    Spacer(Modifier.width(4.dp))
                    Text("Reset")
                }
                OutlinedButton(
                    onClick = if (state.isRecording) stopRecording else startRecording,
                    enabled = !state.isReplayProcessing && state.replayFileName == null,
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) {
                    Icon(
                        if (state.isRecording) Icons.Outlined.Stop else Icons.Outlined.FiberManualRecord,
                        contentDescription = null,
                        tint = if (state.isRecording) RecordingRed else MaterialTheme.colorScheme.onSurface,
                    )
                    Spacer(Modifier.width(4.dp))
                    Text(if (state.isRecording) "Stop" else "Record")
                }
            }
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                OutlinedButton(
                    onClick = viewModel::markEvent,
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) {
                    Icon(Icons.Outlined.Flag, contentDescription = null)
                    Spacer(Modifier.width(4.dp))
                    Text("Mark")
                }
                OutlinedButton(
                    onClick = openVideo,
                    enabled = !state.isReplayProcessing && !state.isRecording,
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) {
                    Icon(Icons.Outlined.VideoFile, contentDescription = null)
                    Spacer(Modifier.width(4.dp))
                    Text("Video")
                }
                OutlinedButton(
                    onClick = export,
                    modifier = Modifier.weight(1f).height(48.dp),
                    contentPadding = PaddingValues(horizontal = 6.dp),
                ) { Text("Export") }
                }
            }
        }
    }
}

@Composable
private fun TrackingOverlay(state: MonitorUiState) {
    val accent = if (state.targetVisible) AcceptedGreen else RejectedRed
    Canvas(
        Modifier
            .fillMaxSize()
            .semantics {
                contentDescription = if (state.targetVisible) "Tracked target overlay" else "Target not visible"
            },
    ) {
        val roi = state.profile.roi
        drawRect(
            color = Color.White.copy(alpha = 0.48f),
            topLeft = Offset(roi.left * size.width, roi.top * size.height),
            size = Size((roi.right - roi.left) * size.width, (roi.bottom - roi.top) * size.height),
            style = Stroke(width = 1.5f, pathEffect = PathEffect.dashPathEffect(floatArrayOf(10f, 8f))),
        )
        state.boundingBox?.let { box ->
            val left = box.left * size.width
            val top = box.top * size.height
            val width = (box.right - box.left) * size.width
            val height = (box.bottom - box.top) * size.height
            drawRect(accent, Offset(left, top), Size(width, height), style = Stroke(width = 3f))
            val center = Offset((box.centerX) * size.width, (box.centerY) * size.height)
            drawCircle(accent, radius = 5f, center = center)
            state.orientationDeg?.let { angle ->
                val radians = Math.toRadians(angle.toDouble())
                val end = center + Offset(cos(radians).toFloat() * 42f, sin(radians).toFloat() * 42f)
                drawLine(accent, center, end, strokeWidth = 3f)
            }
        }
        if (state.trajectory.size > 1) {
            val centerlineStart = state.trajectory.first()
            val centerlineEnd = state.trajectory.last()
            drawLine(
                color = Color(0xFF8BA7FF).copy(alpha = 0.72f),
                start = Offset(centerlineStart.first * size.width, centerlineStart.second * size.height),
                end = Offset(centerlineEnd.first * size.width, centerlineEnd.second * size.height),
                strokeWidth = 1.5f,
                pathEffect = PathEffect.dashPathEffect(floatArrayOf(9f, 7f)),
            )
            val path = Path()
            state.trajectory.forEachIndexed { index, point ->
                val offset = Offset(point.first * size.width, point.second * size.height)
                if (index == 0) path.moveTo(offset.x, offset.y) else path.lineTo(offset.x, offset.y)
            }
            drawPath(path, accent.copy(alpha = 0.65f), style = Stroke(width = 2f))
        }
    }
}

@Composable
private fun DiagnosticsPanel(state: MonitorUiState, modifier: Modifier) {
    val listState = rememberLazyListState()
    var previousLogCount by remember { mutableIntStateOf(0) }
    LaunchedEffect(state.logs.size) {
        val lastVisible = listState.layoutInfo.visibleItemsInfo.lastOrNull()?.index ?: -1
        val wasAtEnd = previousLogCount == 0 || lastVisible >= previousLogCount - 2
        if (wasAtEnd && state.logs.isNotEmpty()) listState.scrollToItem(state.logs.lastIndex)
        previousLogCount = state.logs.size
    }
    LazyColumn(
        modifier = modifier,
        state = listState,
        verticalArrangement = Arrangement.spacedBy(2.dp),
    ) {
        item {
            SectionSurface("Receiver") {
                MetricRow("Camera", state.cameraState, monospaced = false)
                MetricRow(
                    "Input",
                    if (state.isReplayProcessing || state.replayFileName != null) {
                        state.replayFileName ?: "Video file"
                    } else {
                        "Live camera"
                    },
                    monospaced = false,
                )
                state.replayStatus?.let { MetricRow("Replay", it, monospaced = false) }
                MetricRow("Analyzed FPS", "%.1f".format(state.effectiveFps))
                MetricRow("Analysis latency", "%.1f ms".format(state.analysisLatencyMs))
                MetricRow("Decoder state", state.decoderState.name)
                MetricRow("Slot", state.currentSlotIndex.toString())
                MetricRow("Frame progress", "${state.frameCollectionProgress} / 32")
                MetricRow("SYNC score", "%.0f%%".format(state.syncScore * 100f))
                MetricRow("SYNC history", state.matchedSyncHistory.joinToString(" ").ifEmpty { "—" })
                state.timingWarning?.let { Text(it, color = WarningAmber, style = MaterialTheme.typography.bodySmall) }
            }
        }
        item {
            SectionSurface("Motion evidence") {
                MetricRow("Target", if (state.targetVisible) "VISIBLE" else "LOST")
                MetricRow("Action", "${state.currentAction.shortName}  ${(state.actionConfidence * 100).toInt()}%")
                MetricRow("Velocity", state.normalizedVelocityPerSec?.let { "%.3f /s".format(it) } ?: "—")
                MetricRow(
                    "Ground-relative velocity",
                    if (state.relativeVelocityXPerSec != null && state.relativeVelocityYPerSec != null) {
                        "%.3f, %.3f /s".format(state.relativeVelocityXPerSec, state.relativeVelocityYPerSec)
                    } else "—",
                )
                MetricRow(
                    "Ground-relative acceleration",
                    if (state.relativeAccelerationXPerSec2 != null && state.relativeAccelerationYPerSec2 != null) {
                        "%.3f, %.3f /s²".format(
                            state.relativeAccelerationXPerSec2,
                            state.relativeAccelerationYPerSec2,
                        )
                    } else "—",
                )
                MetricRow("Orientation rate", state.yawRateDegPerSec?.let { "%.1f deg/s".format(it) } ?: "—")
                Text("Actions", style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
                StreamText(state.rawActions.joinToString(" ").ifEmpty { "—" })
                Text("Symbols", style = MaterialTheme.typography.labelSmall, color = MaterialTheme.colorScheme.onSurfaceVariant)
                StreamText(state.rawSymbols.joinToString(" ").ifEmpty { "—" })
            }
        }
        item {
            SectionSurface("Decoded output") {
                val frame = state.lastFrame
                MetricRow("Status", when (frame?.accepted) { true -> "ACCEPTED"; false -> "REJECTED"; null -> "WAITING" })
                MetricRow("Plaintext", frame?.payloadText?.ifEmpty { "(empty)" } ?: "—", monospaced = false)
                MetricRow("Payload hex", frame?.payloadHex?.ifEmpty { "—" } ?: "—")
                MetricRow("Sequence", frame?.sequence?.toString() ?: "—")
                MetricRow("Corrected errors", frame?.correctedBitErrors?.toString() ?: "—")
                MetricRow("Erased bits", frame?.erasedBits?.toString() ?: "—")
                MetricRow("Mean confidence", frame?.let { "%.0f%%".format(it.meanConfidence * 100f) } ?: "—")
                frame?.rejectionReason?.let { Text(it, color = RejectedRed, style = MaterialTheme.typography.bodySmall) }
                if ((frame?.possibleMissingFrames ?: 0) > 0) {
                    Text("Possible missing frames: ${frame?.possibleMissingFrames}", color = WarningAmber)
                }
            }
        }
        item {
            Text(
                "Session events",
                style = MaterialTheme.typography.titleSmall,
                fontWeight = FontWeight.SemiBold,
                modifier = Modifier.padding(horizontal = 4.dp, vertical = 2.dp),
            )
        }
        items(state.logs, key = { "${it.timestampEpochMs}-${it.slotIndex}-${it.message}" }) { entry ->
            LogRow(entry)
        }
        item { Spacer(Modifier.height(4.dp)) }
    }
}

@Composable
private fun StreamText(value: String) {
    Text(
        value,
        modifier = Modifier.fillMaxWidth().background(MaterialTheme.colorScheme.surfaceVariant, RoundedCornerShape(6.dp)).padding(8.dp),
        fontFamily = FontFamily.Monospace,
        style = MaterialTheme.typography.bodySmall,
        maxLines = 3,
        overflow = TextOverflow.Ellipsis,
    )
}

@Composable
private fun LogRow(entry: DecoderLogEntry) {
    val color = when (entry.severity) {
        LogSeverity.ACCEPTED -> AcceptedGreen
        LogSeverity.WARNING -> WarningAmber
        LogSeverity.REJECTED, LogSeverity.ERROR -> RejectedRed
        LogSeverity.INFO -> MaterialTheme.colorScheme.onSurfaceVariant
    }
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 4.dp, vertical = 1.dp),
        horizontalArrangement = Arrangement.spacedBy(6.dp),
        verticalAlignment = Alignment.Top,
    ) {
        Text(
            SimpleDateFormat("HH:mm:ss", Locale.US).format(Date(entry.timestampEpochMs)),
            color = MaterialTheme.colorScheme.onSurfaceVariant,
            fontFamily = FontFamily.Monospace,
            style = MaterialTheme.typography.labelSmall,
        )
        Box(Modifier.padding(top = 4.dp).size(6.dp).background(color, CircleShape))
        Text(entry.message, style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
    }
}

@Composable
private fun StatusChip(label: String, positive: Boolean, overrideColor: Color? = null) {
    val color = overrideColor ?: if (positive) AcceptedGreen else RejectedRed
    Surface(color = Color.Black.copy(alpha = 0.68f), shape = RoundedCornerShape(6.dp)) {
        Row(
            Modifier.padding(horizontal = 8.dp, vertical = 5.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Box(Modifier.size(7.dp).background(color, CircleShape))
            Text(label, color = Color.White, style = MaterialTheme.typography.labelSmall)
        }
    }
}
