package com.example.app_drone_decode.data.config

import android.content.Context
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.example.app_drone_decode.domain.model.FunctionSettings
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.functionSettingsDataStore by preferencesDataStore(name = "function_settings")

class FunctionSettingsRepository(private val context: Context) {
    val settings: Flow<FunctionSettings> = context.functionSettingsDataStore.data.map { preferences ->
        val defaults = FunctionSettings()
        defaults.copy(
            requestedResolution = preferences[Keys.resolution] ?: defaults.requestedResolution,
            requestedFps = preferences[Keys.fps] ?: defaults.requestedFps,
            torchEnabled = preferences[Keys.torch] ?: defaults.torchEnabled,
            recordingLimitMb = preferences[Keys.recordingLimit] ?: defaults.recordingLimitMb,
            retentionDays = preferences[Keys.retentionDays] ?: defaults.retentionDays,
            sessionNamePrefix = preferences[Keys.sessionPrefix] ?: defaults.sessionNamePrefix,
            retainRawObservations = preferences[Keys.retainRaw] ?: defaults.retainRawObservations,
            exportFormat = preferences[Keys.exportFormat] ?: defaults.exportFormat,
            remoteEndpoint = preferences[Keys.remoteEndpoint] ?: defaults.remoteEndpoint,
            remoteEnabled = preferences[Keys.remoteEnabled] ?: defaults.remoteEnabled,
            bluetoothEnabled = preferences[Keys.bluetoothEnabled] ?: defaults.bluetoothEnabled,
            usbEnabled = preferences[Keys.usbEnabled] ?: defaults.usbEnabled,
            replayMode = preferences[Keys.replayMode] ?: defaults.replayMode,
            knownPayloadBerMode = preferences[Keys.berMode] ?: defaults.knownPayloadBerMode,
        )
    }

    suspend fun save(settings: FunctionSettings) {
        require(settings.requestedFps in 15..60)
        require(settings.recordingLimitMb in 50..5_000)
        require(settings.retentionDays in 1..365)
        context.functionSettingsDataStore.edit { preferences ->
            preferences[Keys.resolution] = settings.requestedResolution
            preferences[Keys.fps] = settings.requestedFps
            preferences[Keys.torch] = settings.torchEnabled
            preferences[Keys.recordingLimit] = settings.recordingLimitMb
            preferences[Keys.retentionDays] = settings.retentionDays
            preferences[Keys.sessionPrefix] = settings.sessionNamePrefix
            preferences[Keys.retainRaw] = settings.retainRawObservations
            preferences[Keys.exportFormat] = settings.exportFormat
            preferences[Keys.remoteEndpoint] = settings.remoteEndpoint
            preferences[Keys.remoteEnabled] = settings.remoteEnabled
            preferences[Keys.bluetoothEnabled] = settings.bluetoothEnabled
            preferences[Keys.usbEnabled] = settings.usbEnabled
            preferences[Keys.replayMode] = settings.replayMode
            preferences[Keys.berMode] = settings.knownPayloadBerMode
        }
    }

    private object Keys {
        val resolution = stringPreferencesKey("resolution")
        val fps = intPreferencesKey("fps")
        val torch = booleanPreferencesKey("torch")
        val recordingLimit = intPreferencesKey("recording_limit_mb")
        val retentionDays = intPreferencesKey("retention_days")
        val sessionPrefix = stringPreferencesKey("session_prefix")
        val retainRaw = booleanPreferencesKey("retain_raw")
        val exportFormat = stringPreferencesKey("export_format")
        val remoteEndpoint = stringPreferencesKey("remote_endpoint")
        val remoteEnabled = booleanPreferencesKey("remote_enabled")
        val bluetoothEnabled = booleanPreferencesKey("bluetooth_enabled")
        val usbEnabled = booleanPreferencesKey("usb_enabled")
        val replayMode = booleanPreferencesKey("replay_mode")
        val berMode = booleanPreferencesKey("known_payload_ber_mode")
    }
}
