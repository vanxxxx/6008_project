package com.example.app_drone_decode.data.config

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.CameraCalibration
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.NormalizedRect
import com.example.app_drone_decode.domain.model.MotionReferenceMode
import org.json.JSONArray
import org.json.JSONObject

object ProfileJsonCodec {
    fun encode(profile: DecoderProfile): String = JSONObject()
        .put("schemaVersion", 3)
        .put("profileId", profile.profileId)
        .put("profileName", profile.profileName)
        .put("profileVersion", profile.profileVersion)
        .put("protocolVersion", profile.protocolVersion)
        .put("actionMapping", JSONObject().apply {
            put("H", profile.actionMapping.getValue(ActionClass.HOVER))
            put("F", profile.actionMapping.getValue(ActionClass.FORWARD))
            put("L", profile.actionMapping.getValue(ActionClass.YAW_LEFT))
            put("R", profile.actionMapping.getValue(ActionClass.YAW_RIGHT))
        })
        .put("sync", JSONArray(profile.syncActions.map { it.shortName }))
        .put("actionDurationMs", profile.actionDurationMs)
        .put("recoveryDurationMs", profile.recoveryDurationMs)
        .put("motionReferenceMode", profile.motionReferenceMode.name)
        .put("idleDurationMs", profile.idleDurationMs)
        .put("stableWindowFraction", profile.stableWindowFraction)
        .put("minimumSamplesPerSlot", profile.minimumSamplesPerSlot)
        .put("highConfidenceThreshold", profile.highConfidenceThreshold)
        .put("lowConfidenceThreshold", profile.lowConfidenceThreshold)
        .put("erasureThreshold", profile.erasureThreshold)
        .put("stationaryLinearSpeedPerSec", profile.stationaryLinearSpeedPerSec)
        .put("forwardSpeedThresholdPerSec", profile.forwardSpeedThresholdPerSec)
        .put("leftYawRateDegPerSec", profile.leftYawRateDegPerSec)
        .put("rightYawRateDegPerSec", profile.rightYawRateDegPerSec)
        .put("smoothingWindow", profile.smoothingWindow)
        .put("outlierRejectionSigma", profile.outlierRejectionSigma)
        .put("hysteresisFraction", profile.hysteresisFraction)
        .put("debounceSamples", profile.debounceSamples)
        .put("trackerImplementationId", profile.trackerImplementationId)
        .put("trackerImplementationVersion", profile.trackerImplementationVersion)
        .put("classifierImplementationId", profile.classifierImplementationId)
        .put("classifierImplementationVersion", profile.classifierImplementationVersion)
        .put("bchProfile", profile.bchProfile)
        .put("roi", JSONObject().apply {
            put("left", profile.roi.left)
            put("top", profile.roi.top)
            put("right", profile.roi.right)
            put("bottom", profile.roi.bottom)
        })
        .put("calibration", JSONObject().apply {
            put("longitudinalAxisX", profile.calibration.longitudinalAxisX)
            put("longitudinalAxisY", profile.calibration.longitudinalAxisY)
            put("yawSign", profile.calibration.yawSign)
            put("targetScale", profile.calibration.targetScale)
            put("calibrated", profile.calibration.calibrated)
        })
        .toString(2)

    fun decode(text: String): DecoderProfile {
        val json = JSONObject(text)
        val schemaVersion = json.getInt("schemaVersion")
        require(schemaVersion in 1..3) { "Unsupported profile schema" }
        val defaults = DecoderProfile()
        val mapping = json.getJSONObject("actionMapping")
        val sync = json.getJSONArray("sync")
        val roi = json.optJSONObject("roi")
        val calibration = json.optJSONObject("calibration")
        return defaults.copy(
            profileId = json.getString("profileId"),
            profileName = json.getString("profileName"),
            profileVersion = json.getInt("profileVersion"),
            protocolVersion = json.getInt("protocolVersion"),
            actionMapping = mapOf(
                ActionClass.HOVER to mapping.getString("H"),
                ActionClass.FORWARD to mapping.getString("F"),
                ActionClass.YAW_LEFT to mapping.getString("L"),
                ActionClass.YAW_RIGHT to mapping.getString("R"),
            ),
            syncActions = List(sync.length()) { ActionClass.fromShortName(sync.getString(it)) },
            actionDurationMs = if (schemaVersion >= 2) {
                json.getInt("actionDurationMs")
            } else {
                json.getInt("slotDurationMs")
            },
            recoveryDurationMs = if (schemaVersion >= 3) {
                json.optInt("recoveryDurationMs", defaults.recoveryDurationMs)
            } else {
                json.optInt("idleDurationMs", defaults.recoveryDurationMs)
            },
            motionReferenceMode = if (schemaVersion >= 3) {
                MotionReferenceMode.entries.firstOrNull { it.name == json.optString("motionReferenceMode") }
                    ?: defaults.motionReferenceMode
            } else {
                MotionReferenceMode.STATIONARY_HOLD
            },
            idleDurationMs = if (schemaVersion >= 2) json.optInt("idleDurationMs", 0) else 0,
            stableWindowFraction = json.getDouble("stableWindowFraction").toFloat(),
            minimumSamplesPerSlot = json.getInt("minimumSamplesPerSlot"),
            highConfidenceThreshold = json.getDouble("highConfidenceThreshold").toFloat(),
            lowConfidenceThreshold = json.getDouble("lowConfidenceThreshold").toFloat(),
            erasureThreshold = json.getDouble("erasureThreshold").toFloat(),
            stationaryLinearSpeedPerSec = json.getDouble("stationaryLinearSpeedPerSec").toFloat(),
            forwardSpeedThresholdPerSec = json.getDouble("forwardSpeedThresholdPerSec").toFloat(),
            leftYawRateDegPerSec = json.getDouble("leftYawRateDegPerSec").toFloat(),
            rightYawRateDegPerSec = json.getDouble("rightYawRateDegPerSec").toFloat(),
            smoothingWindow = json.getInt("smoothingWindow"),
            outlierRejectionSigma = json.getDouble("outlierRejectionSigma").toFloat(),
            hysteresisFraction = json.getDouble("hysteresisFraction").toFloat(),
            debounceSamples = json.getInt("debounceSamples"),
            trackerImplementationId = json.optString("trackerImplementationId", defaults.trackerImplementationId),
            trackerImplementationVersion = json.optInt("trackerImplementationVersion", defaults.trackerImplementationVersion),
            classifierImplementationId = json.optString("classifierImplementationId", defaults.classifierImplementationId),
            classifierImplementationVersion = json.optInt("classifierImplementationVersion", defaults.classifierImplementationVersion),
            bchProfile = json.optString("bchProfile", defaults.bchProfile),
            roi = NormalizedRect(
                left = roi?.float("left", defaults.roi.left) ?: defaults.roi.left,
                top = roi?.float("top", defaults.roi.top) ?: defaults.roi.top,
                right = roi?.float("right", defaults.roi.right) ?: defaults.roi.right,
                bottom = roi?.float("bottom", defaults.roi.bottom) ?: defaults.roi.bottom,
            ),
            calibration = CameraCalibration(
                longitudinalAxisX = calibration?.float(
                    "longitudinalAxisX",
                    defaults.calibration.longitudinalAxisX,
                ) ?: defaults.calibration.longitudinalAxisX,
                longitudinalAxisY = calibration?.float(
                    "longitudinalAxisY",
                    defaults.calibration.longitudinalAxisY,
                ) ?: defaults.calibration.longitudinalAxisY,
                yawSign = calibration?.float("yawSign", defaults.calibration.yawSign)
                    ?: defaults.calibration.yawSign,
                targetScale = calibration?.float("targetScale", defaults.calibration.targetScale)
                    ?: defaults.calibration.targetScale,
                calibrated = calibration?.optBoolean("calibrated", defaults.calibration.calibrated)
                    ?: defaults.calibration.calibrated,
            ),
        )
    }

    private fun JSONObject.float(key: String, fallback: Float): Float = optDouble(key, fallback.toDouble()).toFloat()
}
