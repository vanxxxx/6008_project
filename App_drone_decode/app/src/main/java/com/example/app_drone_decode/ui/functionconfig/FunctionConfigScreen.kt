package com.example.app_drone_decode.ui.functionconfig

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import com.example.app_drone_decode.domain.model.FunctionSettings
import com.example.app_drone_decode.ui.components.MetricRow
import com.example.app_drone_decode.ui.components.SectionSurface
import com.example.app_drone_decode.ui.monitor.MonitorUiState
import com.example.app_drone_decode.ui.monitor.MonitorViewModel
import com.example.app_drone_decode.ui.theme.WarningAmber

@Composable
fun FunctionConfigScreen(state: MonitorUiState, viewModel: MonitorViewModel) {
    var settings by remember { mutableStateOf(state.functionSettings) }
    var fpsText by remember { mutableStateOf(settings.requestedFps.toString()) }
    var recordingLimitText by remember { mutableStateOf(settings.recordingLimitMb.toString()) }
    var retentionText by remember { mutableStateOf(settings.retentionDays.toString()) }
    var authToken by remember { mutableStateOf("") }
    LaunchedEffect(state.functionSettings) {
        settings = state.functionSettings
        fpsText = settings.requestedFps.toString()
        recordingLimitText = settings.recordingLimitMb.toString()
        retentionText = settings.retentionDays.toString()
    }

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        item {
            Text("Function configuration", style = MaterialTheme.typography.headlineSmall)
            Text(
                "Camera, recording, logging, transports, and laboratory tools.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        item {
            SectionSurface("Camera") {
                MetricRow("Lens", "Rear camera")
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    listOf("640x480", "1280x720").forEach { resolution ->
                        OutlinedButton(
                            onClick = { settings = settings.copy(requestedResolution = resolution) },
                            modifier = Modifier.weight(1f),
                        ) { Text(if (settings.requestedResolution == resolution) "✓ $resolution" else resolution) }
                    }
                }
                OutlinedTextField(
                    value = fpsText,
                    onValueChange = { fpsText = it },
                    label = { Text("Requested FPS (15–60)") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                ToggleRow("Torch", settings.torchEnabled) { settings = settings.copy(torchEnabled = it) }
                MetricRow("Focus / exposure", "CameraX automatic")
                MetricRow("Orientation", "Sensor-aware")
                MetricRow("ROI", "Profile-controlled")
                MetricRow("Calibration", if (state.profile.calibration.calibrated) "CALIBRATED" else "REQUIRED")
                Text(
                    "Forward/backward and left/right interpretation is limited until camera axes, target scale, and stationary noise are calibrated.",
                    color = WarningAmber,
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }
        item {
            SectionSurface("Recording and privacy") {
                Text("Video recording is off by default and always shows a REC indicator.")
                OutlinedTextField(
                    value = recordingLimitText,
                    onValueChange = { recordingLimitText = it },
                    label = { Text("Storage limit (MB)") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                OutlinedTextField(
                    value = retentionText,
                    onValueChange = { retentionText = it },
                    label = { Text("Retention (days)") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                OutlinedTextField(
                    value = settings.sessionNamePrefix,
                    onValueChange = { settings = settings.copy(sessionNamePrefix = it) },
                    label = { Text("Session name prefix") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                MetricRow("Audio", "Not requested or recorded")
            }
        }
        item {
            SectionSurface("Logging") {
                ToggleRow("Retain raw compact observations", settings.retainRawObservations) {
                    settings = settings.copy(retainRawObservations = it)
                }
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    listOf("JSON", "CSV").forEach { format ->
                        OutlinedButton(
                            onClick = { settings = settings.copy(exportFormat = format) },
                            modifier = Modifier.weight(1f),
                        ) { Text(if (settings.exportFormat == format) "✓ $format" else format) }
                    }
                }
                Text("Exports use UTF-8 and the Android system file picker.", style = MaterialTheme.typography.bodySmall)
                Text("Each session log is capped at 25 MB; older sessions follow the retention setting.", style = MaterialTheme.typography.bodySmall)
            }
        }
        item {
            SectionSurface("Remote debug API") {
                MetricRow("Connection", "DISABLED")
                OutlinedTextField(
                    value = settings.remoteEndpoint,
                    onValueChange = { settings = settings.copy(remoteEndpoint = it) },
                    label = { Text("Development endpoint") },
                    placeholder = { Text("https://development-host.example") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                OutlinedTextField(
                    value = authToken,
                    onValueChange = { authToken = it },
                    label = { Text("Authentication token (not logged)") },
                    visualTransformation = PasswordVisualTransformation(),
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                Text(
                    "Remote transports are isolated from the production visual decoder and cannot inject simulator ground truth.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }
        item {
            SectionSurface("Bluetooth") {
                MetricRow("Status", "DISABLED")
                MetricRow("Permission", "Not requested")
                MetricRow("Paired device", "None")
                Text("Bluetooth support remains off until permission and lifecycle tests are present.")
            }
        }
        item {
            SectionSurface("Wired transport") {
                MetricRow("USB / serial", "DISABLED")
                MetricRow("Attached device", "None")
                Text("Wired support remains off until discovery and detach handling are implemented.")
            }
        }
        item {
            SectionSurface("Developer tools") {
                MetricRow("Video replay", "Monitor > Video")
                Text(
                    "A selected recording is analyzed with the same visual tracker, action classifier, slot timing, and BCH decoder as the live camera.",
                    style = MaterialTheme.typography.bodySmall,
                )
                ToggleRow("Known-payload BER mode", settings.knownPayloadBerMode) {
                    settings = settings.copy(knownPayloadBerMode = it)
                }
                Text(
                    "BER is meaningful only for labeled fixtures or injected data, never for an unknown live message.",
                    color = WarningAmber,
                    style = MaterialTheme.typography.bodySmall,
                )
                Button(onClick = viewModel::injectHelloReference, modifier = Modifier.fillMaxWidth()) {
                    Text("Inject protocol v1 Hello vector")
                }
                MetricRow(
                    "Data source",
                    when {
                        state.injectedSession -> "INJECTED — LABELED"
                        state.isReplayProcessing || state.replayFileName != null -> "VIDEO REPLAY — VISUAL"
                        else -> "VISUAL CAMERA"
                    },
                )
                MetricRow(
                    "Known-payload BER",
                    if (state.injectedSession && state.functionSettings.knownPayloadBerMode) {
                        state.knownPayloadBer?.let { "%.3f%%".format(it * 100f) } ?: "WAITING"
                    } else {
                        "UNAVAILABLE FOR LIVE DATA"
                    },
                )
                MetricRow("Analysis latency", "%.1f ms".format(state.analysisLatencyMs))
                MetricRow("Analyzed FPS", "%.1f".format(state.effectiveFps))
            }
        }
        item {
            state.configMessage?.let { Text(it, color = MaterialTheme.colorScheme.primary) }
            Button(
                onClick = {
                    viewModel.saveFunctionSettings(
                        settings.copy(
                            requestedFps = fpsText.toIntOrNull() ?: 0,
                            recordingLimitMb = recordingLimitText.toIntOrNull() ?: 0,
                            retentionDays = retentionText.toIntOrNull() ?: 0,
                        ),
                    )
                },
                modifier = Modifier.fillMaxWidth(),
            ) { Text("Save function settings") }
        }
    }
}

@Composable
private fun ToggleRow(label: String, checked: Boolean, onCheckedChange: (Boolean) -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, modifier = Modifier.weight(1f))
        Switch(checked = checked, onCheckedChange = onCheckedChange)
    }
}
