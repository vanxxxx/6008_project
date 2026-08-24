package com.example.app_drone_decode.vision

import android.content.Context
import android.graphics.Bitmap
import android.media.Image
import android.media.MediaCodec
import android.media.MediaExtractor
import android.media.MediaFormat
import android.net.Uri
import android.os.SystemClock
import android.provider.OpenableColumns
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.MotionObservation
import java.util.ArrayDeque
import kotlin.math.ceil
import kotlin.math.max
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.currentCoroutineContext
import kotlinx.coroutines.delay
import kotlinx.coroutines.ensureActive
import kotlinx.coroutines.withContext
import org.opencv.android.OpenCVLoader
import org.opencv.android.Utils
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

data class VideoReplayResult(
    val displayName: String,
    val durationMs: Long,
    val sampledFrames: Int,
)

/** Sequentially decodes a selected video and runs the production visual pipeline. */
class VideoReplayProcessor(private val context: Context) {
    suspend fun process(
        uri: Uri,
        profile: DecoderProfile,
        onMetadata: (Long) -> Unit,
        onObservation: suspend (MotionObservation, Float, Float) -> Unit,
        onPreviewFrame: (Bitmap) -> Unit,
        onProgress: (Float) -> Unit,
    ): VideoReplayResult = withContext(Dispatchers.Default) {
        check(OpenCVLoader.initLocal()) { "OpenCV could not be initialized" }
        val extractor = MediaExtractor()
        var codec: MediaCodec? = null
        var activeTracker: TargetTracker? = null
        try {
            setExtractorDataSource(extractor, uri)
            val trackIndex = findVideoTrack(extractor)
            check(trackIndex >= 0) { "The selected file contains no decodable video track" }
            extractor.selectTrack(trackIndex)
            val inputFormat = extractor.getTrackFormat(trackIndex)
            val mime = inputFormat.getString(MediaFormat.KEY_MIME)
                ?: error("The selected video track has no MIME type")
            val durationUs = inputFormat.longValue(MediaFormat.KEY_DURATION)
                .takeIf { it > 0L }
                ?: error("The selected file has no readable video duration")
            onMetadata(durationUs / 1_000L)
            val rotationDegrees = inputFormat.intValue(MediaFormat.KEY_ROTATION).normalizeRotation()
            val targetFps = recommendedSamplingFps(profile)
            val samplingIntervalUs = 1_000_000L / targetFps
            val previewIntervalUs = 1_000_000L / PREVIEW_FPS
            val expectedSamples = ceil(durationUs.toDouble() / samplingIntervalUs).toInt().coerceAtLeast(1)
            val tracker = AdaptiveTargetTracker(roiProvider = { profile.roi })
            activeTracker = tracker
            val estimator = MotionEstimator()
            val classifier = ThresholdActionClassifier { profile }
            val history = ArrayDeque<MotionObservation>()
            val bufferInfo = MediaCodec.BufferInfo()

            codec = MediaCodec.createDecoderByType(mime).apply {
                configure(inputFormat, null, null, 0)
                start()
            }
            var inputFinished = false
            var outputFinished = false
            var nextSampleUs = 0L
            var nextPreviewUs = 0L
            var firstPresentationUs: Long? = null
            var sampledFrames = 0
            val replayStartedMs = SystemClock.elapsedRealtime()

            while (!outputFinished) {
                currentCoroutineContext().ensureActive()
                if (!inputFinished) {
                    val inputIndex = codec.dequeueInputBuffer(CODEC_TIMEOUT_US)
                    if (inputIndex >= 0) {
                        val inputBuffer = codec.getInputBuffer(inputIndex)
                            ?: error("Video decoder input buffer is unavailable")
                        val sampleSize = extractor.readSampleData(inputBuffer, 0)
                        if (sampleSize < 0) {
                            codec.queueInputBuffer(
                                inputIndex,
                                0,
                                0,
                                0L,
                                MediaCodec.BUFFER_FLAG_END_OF_STREAM,
                            )
                            inputFinished = true
                        } else {
                            codec.queueInputBuffer(
                                inputIndex,
                                0,
                                sampleSize,
                                extractor.sampleTime,
                                0,
                            )
                            extractor.advance()
                        }
                    }
                }

                val outputIndex = codec.dequeueOutputBuffer(bufferInfo, CODEC_TIMEOUT_US)
                when {
                    outputIndex >= 0 -> {
                        val presentationUs = bufferInfo.presentationTimeUs.coerceAtLeast(0L)
                        val shouldSample = bufferInfo.size > 0 && presentationUs >= nextSampleUs
                        if (shouldSample) {
                            val startedNs = System.nanoTime()
                            val image = codec.getOutputImage(outputIndex)
                                ?: error("Video decoder did not expose a YUV output image")
                            try {
                                val sourceRgba = image.toRgbaMat()
                                val displayRgba = sourceRgba.rotateAndScale(rotationDegrees)
                                if (displayRgba !== sourceRgba) sourceRgba.release()
                                try {
                                    val observation = processFrame(
                                        displayRgba,
                                        presentationUs * 1_000L,
                                        tracker,
                                        estimator,
                                        classifier,
                                        profile,
                                        history,
                                    )
                                    sampledFrames++
                                    val latencyMs = (System.nanoTime() - startedNs) / 1_000_000f
                                    onObservation(observation, targetFps.toFloat(), latencyMs)
                                    if (presentationUs >= nextPreviewUs) {
                                        onPreviewFrame(displayRgba.toBitmap())
                                        nextPreviewUs = presentationUs + previewIntervalUs
                                        onProgress((presentationUs.toFloat() / durationUs).coerceIn(0f, 1f))
                                    }
                                } finally {
                                    displayRgba.release()
                                }
                            } finally {
                                image.close()
                            }
                            do {
                                nextSampleUs += samplingIntervalUs
                            } while (nextSampleUs <= presentationUs)
                            val basePresentationUs = firstPresentationUs ?: presentationUs.also {
                                firstPresentationUs = it
                            }
                            paceReplay(presentationUs - basePresentationUs, replayStartedMs)
                        }
                        outputFinished = bufferInfo.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0
                        codec.releaseOutputBuffer(outputIndex, false)
                    }
                    outputIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> Unit
                }
            }
            onProgress(1f)
            check(sampledFrames > 0) { "No readable frames were found in the selected video" }
            check(sampledFrames >= minOf(2, expectedSamples)) { "Too few frames were decoded from the selected video" }
            VideoReplayResult(displayName(uri), durationUs / 1_000L, sampledFrames)
        } finally {
            activeTracker?.reset()
            runCatching { codec?.stop() }
            codec?.release()
            extractor.release()
        }
    }

    private suspend fun paceReplay(mediaElapsedUs: Long, replayStartedMs: Long) {
        val targetElapsedMs = mediaElapsedUs / 1_000L / MAX_REPLAY_SPEED
        val actualElapsedMs = SystemClock.elapsedRealtime() - replayStartedMs
        val waitMs = targetElapsedMs - actualElapsedMs
        if (waitMs > 0L) delay(waitMs.coerceAtMost(50L))
    }

    private fun processFrame(
        rgba: Mat,
        timestampNs: Long,
        tracker: TargetTracker,
        estimator: MotionEstimator,
        classifier: ActionClassifier,
        profile: DecoderProfile,
        history: ArrayDeque<MotionObservation>,
    ): MotionObservation {
        val tracking = tracker.process(CvFrame(rgba, timestampNs))
        val estimated = estimator.estimate(tracking, timestampNs, profile.calibration)
        history.addLast(estimated)
        while (history.size > profile.smoothingWindow.coerceAtLeast(2)) history.removeFirst()
        val likelihoods = classifier.classify(history.toList())
        return estimated.copy(actionProbabilities = likelihoods.probabilities).also { classified ->
            history.removeLast()
            history.addLast(classified)
        }
    }

    private fun Image.toRgbaMat(): Mat {
        check(planes.size >= 3) { "Unsupported decoder image format: $format" }
        val crop = cropRect
        val width = crop.width() and -2
        val height = crop.height() and -2
        check(width > 0 && height > 0) { "Decoded video frame has an invalid crop" }
        val nv21 = ByteArray(width * height * 3 / 2)
        copyLumaPlane(planes[0], crop.left, crop.top, width, height, nv21)
        val conversionCode = copyChromaPlanes(
            uPlane = planes[1],
            vPlane = planes[2],
            cropLeft = crop.left / 2,
            cropTop = crop.top / 2,
            width = width / 2,
            height = height / 2,
            output = nv21,
            outputOffset = width * height,
        )
        val yuv = Mat(height + height / 2, width, CvType.CV_8UC1)
        val rgba = Mat()
        return try {
            yuv.put(0, 0, nv21)
            Imgproc.cvtColor(yuv, rgba, conversionCode)
            rgba
        } catch (error: RuntimeException) {
            rgba.release()
            throw error
        } finally {
            yuv.release()
        }
    }

    private fun copyLumaPlane(
        plane: Image.Plane,
        cropLeft: Int,
        cropTop: Int,
        width: Int,
        height: Int,
        output: ByteArray,
    ) {
        val buffer = plane.buffer.duplicate()
        for (row in 0 until height) {
            val rowStart = (cropTop + row) * plane.rowStride + cropLeft * plane.pixelStride
            if (plane.pixelStride == 1) {
                buffer.position(rowStart)
                buffer.get(output, row * width, width)
            } else {
                for (column in 0 until width) {
                    output[row * width + column] = buffer.get(rowStart + column * plane.pixelStride)
                }
            }
        }
    }

    private fun copyChromaPlanes(
        uPlane: Image.Plane,
        vPlane: Image.Plane,
        cropLeft: Int,
        cropTop: Int,
        width: Int,
        height: Int,
        output: ByteArray,
        outputOffset: Int,
    ): Int {
        val uBuffer = uPlane.buffer.duplicate()
        val vBuffer = vPlane.buffer.duplicate()
        if (uPlane.pixelStride == 1 && vPlane.pixelStride == 1) {
            copyPlanarPlane(uPlane, cropLeft, cropTop, width, height, output, outputOffset)
            copyPlanarPlane(
                vPlane,
                cropLeft,
                cropTop,
                width,
                height,
                output,
                outputOffset + width * height,
            )
            return Imgproc.COLOR_YUV2RGBA_I420
        }
        var destination = outputOffset
        if (uPlane.pixelStride == 2 && vPlane.pixelStride == 2 && uPlane.rowStride == vPlane.rowStride) {
            val interleavedRowBytes = width * 2
            var copiedAllRows = true
            for (row in 0 until height) {
                val vRow = (cropTop + row) * vPlane.rowStride + cropLeft * vPlane.pixelStride
                if (vRow + interleavedRowBytes > vBuffer.limit()) {
                    copiedAllRows = false
                    break
                }
                vBuffer.position(vRow)
                vBuffer.get(output, destination, interleavedRowBytes)
                destination += interleavedRowBytes
            }
            if (copiedAllRows) return Imgproc.COLOR_YUV2RGBA_NV21
            destination = outputOffset
        }
        for (row in 0 until height) {
            val uRow = (cropTop + row) * uPlane.rowStride + cropLeft * uPlane.pixelStride
            val vRow = (cropTop + row) * vPlane.rowStride + cropLeft * vPlane.pixelStride
            for (column in 0 until width) {
                output[destination++] = vBuffer.get(vRow + column * vPlane.pixelStride)
                output[destination++] = uBuffer.get(uRow + column * uPlane.pixelStride)
            }
        }
        return Imgproc.COLOR_YUV2RGBA_NV21
    }

    private fun copyPlanarPlane(
        plane: Image.Plane,
        cropLeft: Int,
        cropTop: Int,
        width: Int,
        height: Int,
        output: ByteArray,
        outputOffset: Int,
    ) {
        val buffer = plane.buffer.duplicate()
        for (row in 0 until height) {
            val rowStart = (cropTop + row) * plane.rowStride + cropLeft
            buffer.position(rowStart)
            buffer.get(output, outputOffset + row * width, width)
        }
    }

    private fun Mat.rotateAndScale(rotationDegrees: Int): Mat {
        var current = this
        if (rotationDegrees != 0) {
            val rotated = Mat()
            when (rotationDegrees) {
                90 -> Core.rotate(current, rotated, Core.ROTATE_90_CLOCKWISE)
                180 -> Core.rotate(current, rotated, Core.ROTATE_180)
                270 -> Core.rotate(current, rotated, Core.ROTATE_90_COUNTERCLOCKWISE)
            }
            current = rotated
        }
        val longestEdge = max(current.cols(), current.rows())
        if (longestEdge <= MAX_FRAME_EDGE) return current
        val scale = MAX_FRAME_EDGE.toDouble() / longestEdge
        val scaled = Mat()
        Imgproc.resize(current, scaled, Size(current.cols() * scale, current.rows() * scale), 0.0, 0.0, Imgproc.INTER_AREA)
        if (current !== this) current.release()
        return scaled
    }

    private fun Mat.toBitmap(): Bitmap = Bitmap.createBitmap(cols(), rows(), Bitmap.Config.ARGB_8888).also { bitmap ->
        Utils.matToBitmap(this, bitmap)
    }

    private fun setExtractorDataSource(extractor: MediaExtractor, uri: Uri) {
        if (uri.scheme == "file") {
            extractor.setDataSource(requireNotNull(uri.path) { "The selected file path is invalid" })
        } else {
            extractor.setDataSource(context, uri, null)
        }
    }

    private fun findVideoTrack(extractor: MediaExtractor): Int = (0 until extractor.trackCount).firstOrNull { index ->
        extractor.getTrackFormat(index).getString(MediaFormat.KEY_MIME)?.startsWith("video/") == true
    } ?: -1

    private fun recommendedSamplingFps(profile: DecoderProfile): Int {
        val stableSeconds = profile.actionDurationMs / 1_000f * profile.stableWindowFraction
        val minimumForWindow = ceil(profile.minimumSamplesPerSlot / stableSeconds.coerceAtLeast(0.02f)).toInt()
        // Pulse/idle profiles need several samples in each 125 ms phase bin.
        // Thirty FPS is the minimum replay rate for stable blind calibration.
        return max(30, minimumForWindow + 1).coerceAtMost(60)
    }

    fun displayName(uri: Uri): String {
        return runCatching {
            context.contentResolver.query(uri, arrayOf(OpenableColumns.DISPLAY_NAME), null, null, null)?.use { cursor ->
                if (cursor.moveToFirst()) cursor.getString(0) else null
            }
        }.getOrNull() ?: uri.lastPathSegment ?: "Selected video"
    }

    private fun MediaFormat.longValue(key: String): Long = if (containsKey(key)) getLong(key) else 0L

    private fun MediaFormat.intValue(key: String): Int = if (containsKey(key)) getInteger(key) else 0

    private fun Int.normalizeRotation(): Int = ((this % 360) + 360) % 360

    private companion object {
        const val CODEC_TIMEOUT_US = 10_000L
        const val MAX_FRAME_EDGE = 480
        const val PREVIEW_FPS = 5
        const val MAX_REPLAY_SPEED = 4L
    }
}
