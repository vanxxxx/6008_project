package com.example.app_drone_decode.camera

import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import com.example.app_drone_decode.vision.ActionClassifier
import com.example.app_drone_decode.vision.CvFrame
import com.example.app_drone_decode.vision.MotionEstimator
import com.example.app_drone_decode.vision.TargetTracker
import java.util.ArrayDeque
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.android.OpenCVLoader

class MotionImageAnalyzer(
    private val tracker: TargetTracker,
    private val estimator: MotionEstimator,
    private val classifier: ActionClassifier,
    private val profileProvider: () -> DecoderProfile,
    private val onObservation: (MotionObservation, Float, Float) -> Unit,
) : ImageAnalysis.Analyzer {
    private val history = ArrayDeque<MotionObservation>()
    private val recentFrameTimestampsNs = ArrayDeque<Long>()
    private val openCvReady by lazy(LazyThreadSafetyMode.SYNCHRONIZED) { OpenCVLoader.initLocal() }

    override fun analyze(image: ImageProxy) {
        val startedNs = System.nanoTime()
        var source: Mat? = null
        var cropped: Mat? = null
        var rotated: Mat? = null
        try {
            if (!openCvReady) return
            val plane = image.planes.firstOrNull() ?: return
            val buffer = plane.buffer
            buffer.rewind()
            val bytes = ByteArray(buffer.remaining())
            buffer.get(bytes)
            val columnsWithPadding = plane.rowStride / plane.pixelStride
            source = Mat(image.height, columnsWithPadding, CvType.CV_8UC4)
            source.put(0, 0, bytes)
            cropped = source.submat(0, image.height, 0, image.width)
            rotated = rotate(cropped, image.imageInfo.rotationDegrees)

            val timestampNs = image.imageInfo.timestamp
            val tracking = tracker.process(CvFrame(rotated, timestampNs))
            val estimated = estimator.estimate(tracking, timestampNs, profileProvider().calibration)
            history.addLast(estimated)
            while (history.size > profileProvider().smoothingWindow.coerceAtLeast(2)) history.removeFirst()
            val likelihoods = classifier.classify(history.toList())
            val observation = estimated.copy(actionProbabilities = likelihoods.probabilities)
            history.removeLast()
            history.addLast(observation)

            recentFrameTimestampsNs.addLast(timestampNs)
            val cutoff = timestampNs - 1_000_000_000L
            while (recentFrameTimestampsNs.size > 1 && recentFrameTimestampsNs.first() < cutoff) {
                recentFrameTimestampsNs.removeFirst()
            }
            val fps = if (recentFrameTimestampsNs.size < 2) 0f else {
                val span = recentFrameTimestampsNs.last() - recentFrameTimestampsNs.first()
                if (span <= 0L) 0f else (recentFrameTimestampsNs.size - 1) * 1_000_000_000f / span
            }
            val latencyMs = (System.nanoTime() - startedNs) / 1_000_000f
            onObservation(observation, fps, latencyMs)
        } catch (_: RuntimeException) {
            // A malformed camera buffer is treated as a dropped analysis frame.
        } finally {
            if (rotated !== cropped) rotated?.release()
            cropped?.release()
            source?.release()
            image.close()
        }
    }

    private fun rotate(source: Mat, degrees: Int): Mat {
        if (degrees == 0) return source
        val destination = Mat()
        when (degrees) {
            90 -> Core.rotate(source, destination, Core.ROTATE_90_CLOCKWISE)
            180 -> Core.rotate(source, destination, Core.ROTATE_180)
            270 -> Core.rotate(source, destination, Core.ROTATE_90_COUNTERCLOCKWISE)
            else -> source.copyTo(destination)
        }
        return destination
    }
}
