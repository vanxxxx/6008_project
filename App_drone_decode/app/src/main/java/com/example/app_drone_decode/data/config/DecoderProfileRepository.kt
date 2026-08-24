package com.example.app_drone_decode.data.config

import android.content.Context
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.floatPreferencesKey
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderProfileValidator
import java.io.IOException
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.map

private val Context.decoderProfileDataStore by preferencesDataStore(name = "decoder_profile")

class DecoderProfileRepository(private val context: Context) {
    val activeProfile: Flow<DecoderProfile> = context.decoderProfileDataStore.data
        .catch { error ->
            if (error is IOException) emit(androidx.datastore.preferences.core.emptyPreferences()) else throw error
        }
        .map(::profileFromPreferences)

    suspend fun save(profile: DecoderProfile) {
        val validation = DecoderProfileValidator.validate(profile)
        require(validation.valid) { validation.errors.joinToString() }
        context.decoderProfileDataStore.edit { preferences ->
            preferences[Keys.profileId] = profile.profileId
            preferences[Keys.profileName] = profile.profileName
            preferences[Keys.profileVersion] = profile.profileVersion
            preferences[Keys.protocolVersion] = profile.protocolVersion
            preferences[Keys.mappingH] = profile.actionMapping.getValue(ActionClass.HOVER)
            preferences[Keys.mappingF] = profile.actionMapping.getValue(ActionClass.FORWARD)
            preferences[Keys.mappingL] = profile.actionMapping.getValue(ActionClass.YAW_LEFT)
            preferences[Keys.mappingR] = profile.actionMapping.getValue(ActionClass.YAW_RIGHT)
            preferences[Keys.sync] = profile.syncActions.joinToString("") { it.shortName }
            preferences[Keys.actionDurationMs] = profile.actionDurationMs
            preferences[Keys.idleDurationMs] = profile.idleDurationMs
            preferences[Keys.stableWindowFraction] = profile.stableWindowFraction
            preferences[Keys.minimumSamplesPerSlot] = profile.minimumSamplesPerSlot
            preferences[Keys.highConfidenceThreshold] = profile.highConfidenceThreshold
            preferences[Keys.lowConfidenceThreshold] = profile.lowConfidenceThreshold
            preferences[Keys.erasureThreshold] = profile.erasureThreshold
            preferences[Keys.stationarySpeed] = profile.stationaryLinearSpeedPerSec
            preferences[Keys.forwardSpeed] = profile.forwardSpeedThresholdPerSec
            preferences[Keys.leftYawRate] = profile.leftYawRateDegPerSec
            preferences[Keys.rightYawRate] = profile.rightYawRateDegPerSec
            preferences[Keys.smoothingWindow] = profile.smoothingWindow
            preferences[Keys.outlierSigma] = profile.outlierRejectionSigma
            preferences[Keys.hysteresis] = profile.hysteresisFraction
            preferences[Keys.debounce] = profile.debounceSamples
            preferences[Keys.roiLeft] = profile.roi.left
            preferences[Keys.roiTop] = profile.roi.top
            preferences[Keys.roiRight] = profile.roi.right
            preferences[Keys.roiBottom] = profile.roi.bottom
            preferences[Keys.axisX] = profile.calibration.longitudinalAxisX
            preferences[Keys.axisY] = profile.calibration.longitudinalAxisY
            preferences[Keys.yawSign] = profile.calibration.yawSign
            preferences[Keys.targetScale] = profile.calibration.targetScale
            preferences[Keys.calibrated] = profile.calibration.calibrated
            preferences[Keys.trackerId] = profile.trackerImplementationId
            preferences[Keys.trackerVersion] = profile.trackerImplementationVersion
            preferences[Keys.classifierId] = profile.classifierImplementationId
            preferences[Keys.classifierVersion] = profile.classifierImplementationVersion
            preferences[Keys.bchProfile] = profile.bchProfile
        }
    }

    suspend fun restoreDefaults() = save(DecoderProfile())

    private fun profileFromPreferences(preferences: Preferences): DecoderProfile {
        val defaults = DecoderProfile()
        val storedProfileId = preferences[Keys.profileId]
        val storedProfileVersion = preferences[Keys.profileVersion]
        val storedTrackerId = preferences[Keys.trackerId]
        // Version 3 changes the built-in physical motion profile from one
        // continuous 500 ms action to a 500 ms action followed by 500 ms idle.
        // Custom profiles retain their old duration as an action-only cycle.
        val migrateBuiltInProfile = storedProfileId == defaults.profileId &&
            storedProfileVersion != null && storedProfileVersion < defaults.profileVersion
        val sync = preferences[Keys.sync]
            ?.map { ActionClass.fromShortName(it.toString()) }
            ?.takeIf { it.size == 8 && ActionClass.UNKNOWN !in it }
            ?: defaults.syncActions
        return defaults.copy(
            profileId = storedProfileId ?: defaults.profileId,
            profileName = preferences[Keys.profileName] ?: defaults.profileName,
            profileVersion = if (migrateBuiltInProfile) defaults.profileVersion else
                storedProfileVersion ?: defaults.profileVersion,
            protocolVersion = preferences[Keys.protocolVersion] ?: defaults.protocolVersion,
            actionMapping = mapOf(
                ActionClass.HOVER to (preferences[Keys.mappingH] ?: "00"),
                ActionClass.FORWARD to (preferences[Keys.mappingF] ?: "01"),
                ActionClass.YAW_LEFT to (preferences[Keys.mappingL] ?: "11"),
                ActionClass.YAW_RIGHT to (preferences[Keys.mappingR] ?: "10"),
            ),
            syncActions = sync,
            actionDurationMs = if (migrateBuiltInProfile) {
                defaults.actionDurationMs
            } else {
                preferences[Keys.actionDurationMs]
                    ?: preferences[Keys.legacySlotDurationMs]
                    ?: defaults.actionDurationMs
            },
            idleDurationMs = if (migrateBuiltInProfile || storedProfileId == null) {
                defaults.idleDurationMs
            } else {
                preferences[Keys.idleDurationMs] ?: 0
            },
            stableWindowFraction = preferences[Keys.stableWindowFraction] ?: defaults.stableWindowFraction,
            minimumSamplesPerSlot = preferences[Keys.minimumSamplesPerSlot] ?: defaults.minimumSamplesPerSlot,
            highConfidenceThreshold = preferences[Keys.highConfidenceThreshold] ?: defaults.highConfidenceThreshold,
            lowConfidenceThreshold = preferences[Keys.lowConfidenceThreshold] ?: defaults.lowConfidenceThreshold,
            erasureThreshold = preferences[Keys.erasureThreshold] ?: defaults.erasureThreshold,
            stationaryLinearSpeedPerSec = preferences[Keys.stationarySpeed] ?: defaults.stationaryLinearSpeedPerSec,
            forwardSpeedThresholdPerSec = preferences[Keys.forwardSpeed] ?: defaults.forwardSpeedThresholdPerSec,
            leftYawRateDegPerSec = preferences[Keys.leftYawRate] ?: defaults.leftYawRateDegPerSec,
            rightYawRateDegPerSec = preferences[Keys.rightYawRate] ?: defaults.rightYawRateDegPerSec,
            smoothingWindow = preferences[Keys.smoothingWindow] ?: defaults.smoothingWindow,
            outlierRejectionSigma = preferences[Keys.outlierSigma] ?: defaults.outlierRejectionSigma,
            hysteresisFraction = preferences[Keys.hysteresis] ?: defaults.hysteresisFraction,
            debounceSamples = preferences[Keys.debounce] ?: defaults.debounceSamples,
            roi = com.example.app_drone_decode.domain.model.NormalizedRect(
                left = preferences[Keys.roiLeft] ?: defaults.roi.left,
                top = preferences[Keys.roiTop] ?: defaults.roi.top,
                right = preferences[Keys.roiRight] ?: defaults.roi.right,
                bottom = preferences[Keys.roiBottom] ?: defaults.roi.bottom,
            ),
            calibration = com.example.app_drone_decode.domain.model.CameraCalibration(
                longitudinalAxisX = preferences[Keys.axisX] ?: defaults.calibration.longitudinalAxisX,
                longitudinalAxisY = preferences[Keys.axisY] ?: defaults.calibration.longitudinalAxisY,
                yawSign = preferences[Keys.yawSign] ?: defaults.calibration.yawSign,
                targetScale = preferences[Keys.targetScale] ?: defaults.calibration.targetScale,
                calibrated = preferences[Keys.calibrated] ?: defaults.calibration.calibrated,
            ),
            trackerImplementationId = if (migrateBuiltInProfile) defaults.trackerImplementationId else
                storedTrackerId ?: defaults.trackerImplementationId,
            trackerImplementationVersion = if (migrateBuiltInProfile) defaults.trackerImplementationVersion else
                preferences[Keys.trackerVersion] ?: defaults.trackerImplementationVersion,
            classifierImplementationId = if (migrateBuiltInProfile) defaults.classifierImplementationId else
                preferences[Keys.classifierId] ?: defaults.classifierImplementationId,
            classifierImplementationVersion = if (migrateBuiltInProfile) defaults.classifierImplementationVersion else
                preferences[Keys.classifierVersion] ?: defaults.classifierImplementationVersion,
            bchProfile = preferences[Keys.bchProfile] ?: defaults.bchProfile,
        )
    }

    private object Keys {
        val profileId = stringPreferencesKey("profile_id")
        val profileName = stringPreferencesKey("profile_name")
        val profileVersion = intPreferencesKey("profile_version")
        val protocolVersion = intPreferencesKey("protocol_version")
        val mappingH = stringPreferencesKey("mapping_h")
        val mappingF = stringPreferencesKey("mapping_f")
        val mappingL = stringPreferencesKey("mapping_l")
        val mappingR = stringPreferencesKey("mapping_r")
        val sync = stringPreferencesKey("sync")
        val actionDurationMs = intPreferencesKey("action_duration_ms")
        val idleDurationMs = intPreferencesKey("idle_duration_ms")
        val legacySlotDurationMs = intPreferencesKey("slot_duration_ms")
        val stableWindowFraction = floatPreferencesKey("stable_window_fraction")
        val minimumSamplesPerSlot = intPreferencesKey("minimum_samples_per_slot")
        val highConfidenceThreshold = floatPreferencesKey("high_confidence_threshold")
        val lowConfidenceThreshold = floatPreferencesKey("low_confidence_threshold")
        val erasureThreshold = floatPreferencesKey("erasure_threshold")
        val stationarySpeed = floatPreferencesKey("stationary_speed_per_sec")
        val forwardSpeed = floatPreferencesKey("forward_speed_per_sec")
        val leftYawRate = floatPreferencesKey("left_yaw_rate_deg_per_sec")
        val rightYawRate = floatPreferencesKey("right_yaw_rate_deg_per_sec")
        val smoothingWindow = intPreferencesKey("smoothing_window")
        val outlierSigma = floatPreferencesKey("outlier_sigma")
        val hysteresis = floatPreferencesKey("hysteresis")
        val debounce = intPreferencesKey("debounce")
        val roiLeft = floatPreferencesKey("roi_left")
        val roiTop = floatPreferencesKey("roi_top")
        val roiRight = floatPreferencesKey("roi_right")
        val roiBottom = floatPreferencesKey("roi_bottom")
        val axisX = floatPreferencesKey("calibration_axis_x")
        val axisY = floatPreferencesKey("calibration_axis_y")
        val yawSign = floatPreferencesKey("calibration_yaw_sign")
        val targetScale = floatPreferencesKey("calibration_target_scale")
        val calibrated = booleanPreferencesKey("calibration_complete")
        val trackerId = stringPreferencesKey("tracker_id")
        val trackerVersion = intPreferencesKey("tracker_version")
        val classifierId = stringPreferencesKey("classifier_id")
        val classifierVersion = intPreferencesKey("classifier_version")
        val bchProfile = stringPreferencesKey("bch_profile")
    }
}
