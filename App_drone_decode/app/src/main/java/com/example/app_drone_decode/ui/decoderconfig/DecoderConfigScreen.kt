package com.example.app_drone_decode.ui.decoderconfig

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
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
import androidx.compose.ui.Modifier
import androidx.compose.ui.Alignment
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.CameraCalibration
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderProfileValidator
import com.example.app_drone_decode.domain.model.NormalizedRect
import com.example.app_drone_decode.ui.components.MetricRow
import com.example.app_drone_decode.ui.components.SectionSurface
import com.example.app_drone_decode.ui.monitor.MonitorUiState
import com.example.app_drone_decode.ui.monitor.MonitorViewModel
import com.example.app_drone_decode.ui.theme.WarningAmber

@Composable
fun DecoderConfigScreen(state: MonitorUiState, viewModel: MonitorViewModel) {
    var editor by remember { mutableStateOf(ProfileEditor.from(state.profile)) }
    LaunchedEffect(state.profile) { editor = ProfileEditor.from(state.profile) }
    val profile = editor.toProfile(state.profile)
    val validation = DecoderProfileValidator.validate(profile)
    val exportLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("application/json"),
    ) { uri -> uri?.let(viewModel::exportProfile) }
    val importLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.OpenDocument(),
    ) { uri -> uri?.let(viewModel::importProfile) }

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        item {
            Text("Decoder configuration", style = MaterialTheme.typography.headlineSmall)
            Text(
                "Protocol, motion classifier, and fixed-slot aggregation settings.",
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                style = MaterialTheme.typography.bodyMedium,
            )
        }
        item {
            SectionSurface("Profile") {
                OutlinedTextField(
                    value = editor.profileName,
                    onValueChange = { editor = editor.copy(profileName = it) },
                    label = { Text("Profile name") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                MetricRow("Profile version", state.profile.profileVersion.toString())
                MetricRow("Protocol version", state.profile.protocolVersion.toString())
                MetricRow("Tracker", "${state.profile.trackerImplementationId} v${state.profile.trackerImplementationVersion}")
                MetricRow("Classifier", "${state.profile.classifierImplementationId} v${state.profile.classifierImplementationVersion}")
            }
        }
        item {
            SectionSurface("Action mapping") {
                Text(
                    "Each action must use one unique two-bit code.",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                MappingRow("Backward (H)", editor.mappingH) { editor = editor.copy(mappingH = it.take(2)) }
                MappingRow("Forward (F)", editor.mappingF) { editor = editor.copy(mappingF = it.take(2)) }
                MappingRow("Move left (L)", editor.mappingL) { editor = editor.copy(mappingL = it.take(2)) }
                MappingRow("Move right (R)", editor.mappingR) { editor = editor.copy(mappingR = it.take(2)) }
                OutlinedTextField(
                    value = editor.sync,
                    onValueChange = { editor = editor.copy(sync = it.uppercase().filter(Char::isLetter).take(8)) },
                    label = { Text("SYNC actions") },
                    supportingText = { Text("Exactly eight actions using H, F, L, and R") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                validation.interoperabilityWarning?.let {
                    Text(it, color = WarningAmber, style = MaterialTheme.typography.bodySmall)
                }
            }
        }
        item {
            SectionSurface("Action and idle timing") {
                NumberField("Action duration (ms)", editor.actionDurationMs) {
                    editor = editor.copy(actionDurationMs = it)
                }
                NumberField("Idle duration (ms)", editor.idleDurationMs) {
                    editor = editor.copy(idleDurationMs = it)
                }
                NumberField("Stable window (0.2–0.9)", editor.stableWindowFraction, decimal = true) {
                    editor = editor.copy(stableWindowFraction = it)
                }
                NumberField("Minimum samples per action", editor.minimumSamplesPerSlot) {
                    editor = editor.copy(minimumSamplesPerSlot = it)
                }
                val actionMs = editor.actionDurationMs.toFloatOrNull() ?: 0f
                val idleMs = editor.idleDurationMs.toFloatOrNull() ?: 0f
                val framesPerAction = state.functionSettings.requestedFps * actionMs / 1_000f
                val framesPerIdle = state.functionSettings.requestedFps * idleMs / 1_000f
                MetricRow("Frames in action window", "%.1f".format(framesPerAction))
                MetricRow("Frames in idle window", "%.1f".format(framesPerIdle))
                MetricRow("Symbol cycle", "%.0f ms".format(actionMs + idleMs))
                if (framesPerAction in 0f..4.99f || (idleMs > 0f && framesPerIdle < 5f)) {
                    Text("The selected camera rate provides too few frames in one timing window.", color = WarningAmber)
                }
            }
        }
        item {
            SectionSurface("Confidence and motion thresholds") {
                DecimalField("High confidence", editor.highConfidence) { editor = editor.copy(highConfidence = it) }
                DecimalField("Low confidence", editor.lowConfidence) { editor = editor.copy(lowConfidence = it) }
                DecimalField("Erasure threshold", editor.erasureThreshold) { editor = editor.copy(erasureThreshold = it) }
                DecimalField("Stationary speed (/s)", editor.stationarySpeed) { editor = editor.copy(stationarySpeed = it) }
                DecimalField("Translation speed (/s)", editor.forwardSpeed) { editor = editor.copy(forwardSpeed = it) }
                DecimalField("Legacy left threshold", editor.leftYaw) { editor = editor.copy(leftYaw = it) }
                DecimalField("Legacy right threshold", editor.rightYaw) { editor = editor.copy(rightYaw = it) }
            }
        }
        item {
            SectionSurface("Filtering and calibration") {
                NumberField("Smoothing window", editor.smoothingWindow) { editor = editor.copy(smoothingWindow = it) }
                DecimalField("Outlier rejection sigma", editor.outlierSigma) { editor = editor.copy(outlierSigma = it) }
                DecimalField("Hysteresis fraction", editor.hysteresis) { editor = editor.copy(hysteresis = it) }
                NumberField("Debounce samples", editor.debounce) { editor = editor.copy(debounce = it) }
                DecimalField("ROI left (0–1)", editor.roiLeft) { editor = editor.copy(roiLeft = it) }
                DecimalField("ROI top (0–1)", editor.roiTop) { editor = editor.copy(roiTop = it) }
                DecimalField("ROI right (0–1)", editor.roiRight) { editor = editor.copy(roiRight = it) }
                DecimalField("ROI bottom (0–1)", editor.roiBottom) { editor = editor.copy(roiBottom = it) }
                DecimalField("Longitudinal axis X", editor.axisX) { editor = editor.copy(axisX = it) }
                DecimalField("Longitudinal axis Y", editor.axisY) { editor = editor.copy(axisY = it) }
                DecimalField("Yaw sign (-1 or 1)", editor.yawSign) { editor = editor.copy(yawSign = it) }
                DecimalField("Target scale", editor.targetScale) { editor = editor.copy(targetScale = it) }
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Text("Calibration completed")
                    Switch(
                        checked = editor.calibrated,
                        onCheckedChange = { editor = editor.copy(calibrated = it) },
                    )
                }
                Text(
                    "The current default longitudinal axis is provisional. Run camera calibration before interpreting depth or forward motion in an experiment.",
                    color = WarningAmber,
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }
        item {
            SectionSurface("Forward error correction") {
                MetricRow("Code", "BCH(63,45)")
                MetricRow("Correction", "t = 3, distance = 7")
                MetricRow("Generator", "0x782CF")
                MetricRow("Transport", "63 bits + fixed zero pad")
                Text("Protocol v1 FEC fields are read-only.", style = MaterialTheme.typography.bodySmall)
            }
        }
        item {
            if (validation.errors.isNotEmpty()) {
                Text(validation.errors.joinToString("\n"), color = MaterialTheme.colorScheme.error)
            }
            state.configMessage?.let { Text(it, color = MaterialTheme.colorScheme.primary) }
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = { viewModel.saveProfile(profile) },
                        enabled = validation.valid,
                        modifier = Modifier.weight(1f),
                    ) { Text("Validate and save") }
                    OutlinedButton(
                        onClick = viewModel::restoreDefaultProfile,
                        modifier = Modifier.weight(1f),
                    ) { Text("Restore defaults") }
                }
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(
                        onClick = {
                            viewModel.saveProfile(
                                profile.copy(
                                    profileId = "${profile.profileId}-copy-${System.currentTimeMillis()}",
                                    profileName = "${profile.profileName} copy",
                                    profileVersion = profile.profileVersion + 1,
                                ),
                            )
                        },
                        enabled = validation.valid,
                        modifier = Modifier.weight(1f),
                    ) { Text("Duplicate") }
                    OutlinedButton(
                        onClick = { importLauncher.launch(arrayOf("application/json")) },
                        modifier = Modifier.weight(1f),
                    ) { Text("Import") }
                    OutlinedButton(
                        onClick = { exportLauncher.launch("decoder-profile.json") },
                        modifier = Modifier.weight(1f),
                    ) { Text("Export") }
                }
            }
        }
    }
}

@Composable
private fun MappingRow(label: String, value: String, onValueChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = { onValueChange(it.filter { character -> character == '0' || character == '1' }) },
        label = { Text(label) },
        singleLine = true,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        modifier = Modifier.fillMaxWidth(),
    )
}

@Composable
private fun NumberField(label: String, value: String, decimal: Boolean = false, onValueChange: (String) -> Unit) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text(label) },
        singleLine = true,
        keyboardOptions = KeyboardOptions(keyboardType = if (decimal) KeyboardType.Decimal else KeyboardType.Number),
        modifier = Modifier.fillMaxWidth(),
    )
}

@Composable
private fun DecimalField(label: String, value: String, onValueChange: (String) -> Unit) =
    NumberField(label, value, decimal = true, onValueChange)

private data class ProfileEditor(
    val profileName: String,
    val mappingH: String,
    val mappingF: String,
    val mappingL: String,
    val mappingR: String,
    val sync: String,
    val actionDurationMs: String,
    val idleDurationMs: String,
    val stableWindowFraction: String,
    val minimumSamplesPerSlot: String,
    val highConfidence: String,
    val lowConfidence: String,
    val erasureThreshold: String,
    val stationarySpeed: String,
    val forwardSpeed: String,
    val leftYaw: String,
    val rightYaw: String,
    val smoothingWindow: String,
    val outlierSigma: String,
    val hysteresis: String,
    val debounce: String,
    val roiLeft: String,
    val roiTop: String,
    val roiRight: String,
    val roiBottom: String,
    val axisX: String,
    val axisY: String,
    val yawSign: String,
    val targetScale: String,
    val calibrated: Boolean,
) {
    fun toProfile(base: DecoderProfile): DecoderProfile = base.copy(
        profileName = profileName,
        actionMapping = mapOf(
            ActionClass.HOVER to mappingH,
            ActionClass.FORWARD to mappingF,
            ActionClass.YAW_LEFT to mappingL,
            ActionClass.YAW_RIGHT to mappingR,
        ),
        syncActions = sync.map { ActionClass.fromShortName(it.toString()) },
        actionDurationMs = actionDurationMs.toIntOrNull() ?: 0,
        idleDurationMs = idleDurationMs.toIntOrNull() ?: -1,
        stableWindowFraction = stableWindowFraction.toFloatOrNull() ?: 0f,
        minimumSamplesPerSlot = minimumSamplesPerSlot.toIntOrNull() ?: 0,
        highConfidenceThreshold = highConfidence.toFloatOrNull() ?: -1f,
        lowConfidenceThreshold = lowConfidence.toFloatOrNull() ?: -1f,
        erasureThreshold = erasureThreshold.toFloatOrNull() ?: -1f,
        stationaryLinearSpeedPerSec = stationarySpeed.toFloatOrNull() ?: -1f,
        forwardSpeedThresholdPerSec = forwardSpeed.toFloatOrNull() ?: -1f,
        leftYawRateDegPerSec = leftYaw.toFloatOrNull() ?: 0f,
        rightYawRateDegPerSec = rightYaw.toFloatOrNull() ?: 0f,
        smoothingWindow = smoothingWindow.toIntOrNull() ?: 0,
        outlierRejectionSigma = outlierSigma.toFloatOrNull() ?: 0f,
        hysteresisFraction = hysteresis.toFloatOrNull() ?: 0f,
        debounceSamples = debounce.toIntOrNull() ?: 0,
        roi = NormalizedRect(
            left = roiLeft.toFloatOrNull() ?: -1f,
            top = roiTop.toFloatOrNull() ?: -1f,
            right = roiRight.toFloatOrNull() ?: -1f,
            bottom = roiBottom.toFloatOrNull() ?: -1f,
        ),
        calibration = CameraCalibration(
            longitudinalAxisX = axisX.toFloatOrNull() ?: 0f,
            longitudinalAxisY = axisY.toFloatOrNull() ?: 0f,
            yawSign = yawSign.toFloatOrNull() ?: 0f,
            targetScale = targetScale.toFloatOrNull() ?: 0f,
            calibrated = calibrated,
        ),
    )

    companion object {
        fun from(profile: DecoderProfile) = ProfileEditor(
            profileName = profile.profileName,
            mappingH = profile.actionMapping.getValue(ActionClass.HOVER),
            mappingF = profile.actionMapping.getValue(ActionClass.FORWARD),
            mappingL = profile.actionMapping.getValue(ActionClass.YAW_LEFT),
            mappingR = profile.actionMapping.getValue(ActionClass.YAW_RIGHT),
            sync = profile.syncActions.joinToString("") { it.shortName },
            actionDurationMs = profile.actionDurationMs.toString(),
            idleDurationMs = profile.idleDurationMs.toString(),
            stableWindowFraction = profile.stableWindowFraction.toString(),
            minimumSamplesPerSlot = profile.minimumSamplesPerSlot.toString(),
            highConfidence = profile.highConfidenceThreshold.toString(),
            lowConfidence = profile.lowConfidenceThreshold.toString(),
            erasureThreshold = profile.erasureThreshold.toString(),
            stationarySpeed = profile.stationaryLinearSpeedPerSec.toString(),
            forwardSpeed = profile.forwardSpeedThresholdPerSec.toString(),
            leftYaw = profile.leftYawRateDegPerSec.toString(),
            rightYaw = profile.rightYawRateDegPerSec.toString(),
            smoothingWindow = profile.smoothingWindow.toString(),
            outlierSigma = profile.outlierRejectionSigma.toString(),
            hysteresis = profile.hysteresisFraction.toString(),
            debounce = profile.debounceSamples.toString(),
            roiLeft = profile.roi.left.toString(),
            roiTop = profile.roi.top.toString(),
            roiRight = profile.roi.right.toString(),
            roiBottom = profile.roi.bottom.toString(),
            axisX = profile.calibration.longitudinalAxisX.toString(),
            axisY = profile.calibration.longitudinalAxisY.toString(),
            yawSign = profile.calibration.yawSign.toString(),
            targetScale = profile.calibration.targetScale.toString(),
            calibrated = profile.calibration.calibrated,
        )
    }
}
