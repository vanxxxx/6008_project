package com.example.app_drone_decode.ui.logs

import androidx.activity.compose.BackHandler
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.outlined.ArrowBack
import androidx.compose.material.icons.outlined.FileDownload
import androidx.compose.material.icons.outlined.FileUpload
import androidx.compose.material.icons.outlined.Refresh
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import com.example.app_drone_decode.data.logs.PersistedSessionEvent
import com.example.app_drone_decode.data.logs.SessionDetail
import com.example.app_drone_decode.data.logs.SessionLogStore
import com.example.app_drone_decode.data.logs.SessionSummary
import com.example.app_drone_decode.domain.model.LogSeverity
import com.example.app_drone_decode.ui.components.MetricRow
import com.example.app_drone_decode.ui.components.SectionSurface
import com.example.app_drone_decode.ui.monitor.MonitorUiState
import com.example.app_drone_decode.ui.monitor.MonitorViewModel
import com.example.app_drone_decode.ui.theme.AcceptedGreen
import com.example.app_drone_decode.ui.theme.RejectedRed
import com.example.app_drone_decode.ui.theme.WarningAmber
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

@Composable
fun LogsScreen(state: MonitorUiState, viewModel: MonitorViewModel) {
    LaunchedEffect(Unit) { viewModel.refreshSessionList() }
    BackHandler(enabled = state.selectedSessionDetail != null) { viewModel.closeSessionDetail() }
    var exportTarget by remember { mutableStateOf<SessionSummary?>(null) }
    val jsonExport = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("application/json"),
    ) { uri ->
        val target = exportTarget
        if (uri != null && target != null) viewModel.exportStoredSession(target, uri, "JSON")
        exportTarget = null
    }
    val csvExport = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("text/csv"),
    ) { uri ->
        val target = exportTarget
        if (uri != null && target != null) viewModel.exportStoredSession(target, uri, "CSV")
        exportTarget = null
    }
    val importSession = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        uri?.let(viewModel::importSession)
    }
    val requestExport: (SessionSummary, String) -> Unit = { session, format ->
        exportTarget = session
        val safeId = session.id.replace(Regex("[^A-Za-z0-9_-]"), "-")
        if (format == "CSV") csvExport.launch("$safeId.csv") else jsonExport.launch("$safeId.json")
    }
    val requestImport = { importSession.launch(arrayOf("application/json", "text/json", "text/plain")) }

    Column(
        Modifier.fillMaxSize().padding(horizontal = 12.dp, vertical = 8.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        state.logsMessage?.let { message ->
            Surface(
                modifier = Modifier.fillMaxWidth().clickable(onClick = viewModel::clearLogsMessage),
                shape = RoundedCornerShape(8.dp),
                color = MaterialTheme.colorScheme.secondaryContainer,
            ) {
                Text(message, Modifier.padding(10.dp), style = MaterialTheme.typography.bodySmall)
            }
        }
        val detail = state.selectedSessionDetail
        if (detail == null) {
            SessionList(
                state = state,
                viewModel = viewModel,
                importSession = requestImport,
                exportSession = requestExport,
                modifier = Modifier.fillMaxSize(),
            )
        } else {
            SessionDetails(
                detail = detail,
                close = viewModel::closeSessionDetail,
                importSession = requestImport,
                exportSession = requestExport,
                modifier = Modifier.fillMaxSize(),
            )
        }
    }
}

@Composable
private fun SessionList(
    state: MonitorUiState,
    viewModel: MonitorViewModel,
    importSession: () -> Unit,
    exportSession: (SessionSummary, String) -> Unit,
    modifier: Modifier,
) {
    Column(modifier, verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Column(Modifier.weight(1f)) {
                Text("Logs", style = MaterialTheme.typography.headlineSmall)
                Text(
                    "${state.sessions.size} saved sessions",
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            IconButton(onClick = viewModel::refreshSessionList) {
                Icon(Icons.Outlined.Refresh, contentDescription = "Refresh sessions")
            }
            OutlinedButton(onClick = importSession) {
                Icon(Icons.Outlined.FileUpload, contentDescription = null)
                Spacer(Modifier.size(6.dp))
                Text("Import")
            }
        }
        if (state.sessions.isEmpty()) {
            Surface(
                Modifier.fillMaxWidth(),
                shape = RoundedCornerShape(10.dp),
                border = BorderStroke(1.dp, MaterialTheme.colorScheme.outline),
            ) {
                Column(Modifier.padding(20.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
                    Text("No saved sessions", fontWeight = FontWeight.SemiBold)
                    Text(
                        "Start decoding or import a session JSON file.",
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        style = MaterialTheme.typography.bodySmall,
                    )
                }
            }
        } else {
            LazyColumn(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                items(state.sessions, key = SessionSummary::id) { session ->
                    SessionListItem(
                        session = session,
                        active = session.id == state.currentSessionId,
                        open = { viewModel.selectSession(session) },
                        exportJson = { exportSession(session, "JSON") },
                        exportCsv = { exportSession(session, "CSV") },
                    )
                }
            }
        }
    }
}

@Composable
private fun SessionListItem(
    session: SessionSummary,
    active: Boolean,
    open: () -> Unit,
    exportJson: () -> Unit,
    exportCsv: () -> Unit,
) {
    Surface(
        modifier = Modifier.fillMaxWidth().clickable(onClick = open),
        shape = RoundedCornerShape(10.dp),
        color = MaterialTheme.colorScheme.surface,
        border = BorderStroke(1.dp, MaterialTheme.colorScheme.outline),
    ) {
        Column(Modifier.padding(horizontal = 12.dp, vertical = 10.dp), verticalArrangement = Arrangement.spacedBy(7.dp)) {
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text(
                    SessionLogStore.formattedStartTime(session.startedEpochMs),
                    fontFamily = FontFamily.Monospace,
                    fontWeight = FontWeight.SemiBold,
                )
                if (active) StatusLabel("Active", AcceptedGreen)
            }
            Text(
                session.sourceLabel ?: session.profileName,
                maxLines = 1,
                overflow = TextOverflow.Ellipsis,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                style = MaterialTheme.typography.bodySmall,
            )
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(18.dp)) {
                CompactValue("Duration", SessionLogStore.formattedDuration(session.durationMs))
                CompactValue("Events", session.eventCount.toString())
                CompactValue("Input", sourceLabel(session.dataSource))
            }
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.End,
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TextButton(onClick = open) { Text("Details") }
                TextButton(onClick = exportCsv) { Text("CSV") }
                TextButton(onClick = exportJson) {
                    Icon(Icons.Outlined.FileDownload, contentDescription = null)
                    Spacer(Modifier.size(4.dp))
                    Text("JSON")
                }
            }
        }
    }
}

@Composable
private fun SessionDetails(
    detail: SessionDetail,
    close: () -> Unit,
    importSession: () -> Unit,
    exportSession: (SessionSummary, String) -> Unit,
    modifier: Modifier,
) {
    Column(modifier, verticalArrangement = Arrangement.spacedBy(8.dp)) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                IconButton(onClick = close) {
                    Icon(Icons.AutoMirrored.Outlined.ArrowBack, contentDescription = "Back to logs")
                }
                Column {
                    Text("Session detail", style = MaterialTheme.typography.titleLarge)
                    Text(
                        SessionLogStore.formattedStartTime(detail.summary.startedEpochMs),
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        style = MaterialTheme.typography.labelSmall,
                        fontFamily = FontFamily.Monospace,
                    )
                }
            }
            OutlinedButton(onClick = importSession) {
                Icon(Icons.Outlined.FileUpload, contentDescription = null)
                Spacer(Modifier.size(4.dp))
                Text("Import")
            }
        }
        Box(Modifier.weight(1f)) {
            LazyColumn(
                Modifier.fillMaxSize(),
                verticalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                item {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        OutlinedButton(
                            onClick = { exportSession(detail.summary, "CSV") },
                            modifier = Modifier.weight(1f),
                        ) { Text("Export CSV") }
                        Button(
                            onClick = { exportSession(detail.summary, "JSON") },
                            modifier = Modifier.weight(1f),
                        ) { Text("Export JSON") }
                    }
                }
                item {
                    SectionSurface("Summary") {
                        MetricRow("Started", SessionLogStore.formattedStartTime(detail.summary.startedEpochMs))
                        MetricRow("Duration", SessionLogStore.formattedDuration(detail.summary.durationMs))
                        MetricRow("Events", detail.summary.eventCount.toString())
                        MetricRow("Observations", detail.summary.observationCount.toString())
                        MetricRow("Profile", detail.summary.profileName, monospaced = false)
                        MetricRow("Input", sourceLabel(detail.summary.dataSource), monospaced = false)
                        detail.summary.sourceLabel?.let { MetricRow("Source", it, monospaced = false) }
                    }
                }
                item {
                    SectionSurface("Decode summary") {
                        MetricRow("Accepted frames", detail.acceptedFrames.toString())
                        MetricRow("Rejected frames", detail.rejectedFrames.toString())
                        MetricRow("Tracking gaps", detail.trackingGapObservations.toString())
                        MetricRow("Mean confidence", "%.0f%%".format(detail.meanConfidence * 100f))
                        MetricRow(
                            "Plaintext",
                            detail.plaintextMessages.joinToString(" · ").ifEmpty { "—" },
                            monospaced = false,
                        )
                        MetricRow("Payload hex", detail.payloadHexValues.joinToString(" · ").ifEmpty { "—" })
                    }
                }
                item {
                    SectionSurface("Raw streams") {
                        Text("Actions", style = MaterialTheme.typography.labelSmall)
                        StreamValue(detail.rawActions.joinToString(" ").ifEmpty { "—" })
                        Text("Symbols", style = MaterialTheme.typography.labelSmall)
                        StreamValue(detail.rawSymbols.joinToString(" ").ifEmpty { "—" })
                    }
                }
                item {
                    Text("Events", style = MaterialTheme.typography.titleSmall, fontWeight = FontWeight.SemiBold)
                }
                if (detail.events.isEmpty()) {
                    item { Text("No decoder events.", color = MaterialTheme.colorScheme.onSurfaceVariant) }
                } else {
                    items(detail.events, key = { "${it.timestampEpochMs}-${it.slotIndex}-${it.message}" }) { event ->
                        EventRow(event)
                    }
                }
            }
        }
    }
}

@Composable
private fun CompactValue(label: String, value: String) {
    Column {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant, style = MaterialTheme.typography.labelSmall)
        Text(value, fontFamily = FontFamily.Monospace, style = MaterialTheme.typography.bodySmall)
    }
}

@Composable
private fun StreamValue(value: String) {
    Text(
        value,
        modifier = Modifier.fillMaxWidth().padding(vertical = 2.dp),
        fontFamily = FontFamily.Monospace,
        style = MaterialTheme.typography.bodySmall,
        color = MaterialTheme.colorScheme.onSurfaceVariant,
    )
}

@Composable
private fun EventRow(event: PersistedSessionEvent) {
    val color = when (event.severity) {
        LogSeverity.ACCEPTED -> AcceptedGreen
        LogSeverity.WARNING -> WarningAmber
        LogSeverity.REJECTED, LogSeverity.ERROR -> RejectedRed
        LogSeverity.INFO -> MaterialTheme.colorScheme.onSurfaceVariant
    }
    Row(
        Modifier.fillMaxWidth().padding(vertical = 2.dp),
        horizontalArrangement = Arrangement.spacedBy(7.dp),
        verticalAlignment = Alignment.Top,
    ) {
        Text(
            SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(Date(event.timestampEpochMs)),
            fontFamily = FontFamily.Monospace,
            style = MaterialTheme.typography.labelSmall,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Surface(Modifier.padding(top = 5.dp).size(6.dp), color = color, shape = CircleShape) {}
        Column(Modifier.weight(1f)) {
            Text(event.message, style = MaterialTheme.typography.bodySmall)
            if (event.rejectionReason != null) {
                Text(event.rejectionReason, color = RejectedRed, style = MaterialTheme.typography.labelSmall)
            }
        }
    }
}

@Composable
private fun StatusLabel(label: String, color: Color) {
    Surface(color = color.copy(alpha = 0.14f), shape = RoundedCornerShape(5.dp)) {
        Text(label, Modifier.padding(horizontal = 6.dp, vertical = 3.dp), color = color, style = MaterialTheme.typography.labelSmall)
    }
}

private fun sourceLabel(source: String): String = when (source) {
    "VIDEO_REPLAY" -> "Video"
    "IMPORTED" -> "Imported"
    else -> "Camera"
}
