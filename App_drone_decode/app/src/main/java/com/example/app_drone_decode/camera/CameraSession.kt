package com.example.app_drone_decode.camera

import android.content.Context
import android.os.Environment
import android.util.Range
import android.util.Size
import androidx.camera.core.Camera
import androidx.camera.core.CameraSelector
import androidx.camera.core.CameraState
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.video.FileOutputOptions
import androidx.camera.video.FallbackStrategy
import androidx.camera.video.Quality
import androidx.camera.video.QualitySelector
import androidx.camera.video.Recorder
import androidx.camera.video.Recording
import androidx.camera.video.VideoCapture
import androidx.camera.video.VideoRecordEvent
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import java.io.File
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

class CameraSession(
    private val context: Context,
    private val analyzer: ImageAnalysis.Analyzer,
    private val onCameraState: (String) -> Unit,
    private val onRecordingState: (Boolean, String?) -> Unit,
) : AutoCloseable {
    private val analysisExecutor: ExecutorService = Executors.newSingleThreadExecutor()
    private var cameraProvider: ProcessCameraProvider? = null
    private var camera: Camera? = null
    private var videoCapture: VideoCapture<Recorder>? = null
    private var imageAnalysis: ImageAnalysis? = null
    private var recording: Recording? = null
    private val closed = AtomicBoolean(false)

    fun bind(
        owner: LifecycleOwner,
        previewView: PreviewView,
        requestedResolution: String,
        requestedFps: Int,
    ) {
        onCameraState("Starting")
        val future = ProcessCameraProvider.getInstance(context)
        future.addListener({
            try {
                val provider = future.get()
                if (closed.get()) {
                    provider.unbindAll()
                    return@addListener
                }
                val dimensions = requestedResolution.split("x").mapNotNull(String::toIntOrNull)
                val requestedSize = if (dimensions.size == 2) Size(dimensions[0], dimensions[1]) else Size(640, 480)
                val resolutionSelector = ResolutionSelector.Builder()
                    .setResolutionStrategy(
                        ResolutionStrategy(
                            requestedSize,
                            ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER,
                        ),
                    )
                    .build()
                val preview = Preview.Builder()
                    .setResolutionSelector(resolutionSelector)
                    .setTargetFrameRate(Range(15, requestedFps.coerceIn(15, 60)))
                    .build()
                    .also { it.surfaceProvider = previewView.surfaceProvider }
                val analysis = ImageAnalysis.Builder()
                    .setResolutionSelector(resolutionSelector)
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_RGBA_8888)
                    .build()
                    .also { it.setAnalyzer(analysisExecutor, analyzer) }
                imageAnalysis = analysis
                val recorder = Recorder.Builder()
                    .setQualitySelector(
                        QualitySelector.from(
                            Quality.SD,
                            FallbackStrategy.higherQualityOrLowerThan(Quality.SD),
                        ),
                    )
                    .build()
                videoCapture = VideoCapture.withOutput(recorder)

                provider.unbindAll()
                val boundCamera = provider.bindToLifecycle(
                    owner,
                    CameraSelector.DEFAULT_BACK_CAMERA,
                    preview,
                    analysis,
                    videoCapture,
                )
                camera = boundCamera
                boundCamera.cameraInfo.cameraState.observe(owner) { state ->
                    val label = when (state.type) {
                        CameraState.Type.OPEN -> "Ready"
                        CameraState.Type.OPENING -> "Starting"
                        CameraState.Type.PENDING_OPEN -> "Waiting for camera"
                        CameraState.Type.CLOSING -> "Stopping"
                        CameraState.Type.CLOSED -> if (state.error == null) "Stopped" else "Interrupted"
                    }
                    onCameraState(label)
                }
                cameraProvider = provider
                onCameraState("Ready")
            } catch (error: Exception) {
                onCameraState("Error: ${error.message ?: error.javaClass.simpleName}")
            }
        }, ContextCompat.getMainExecutor(context))
    }

    fun setTorch(enabled: Boolean) {
        camera?.cameraControl?.enableTorch(enabled)
    }

    fun setAnalysisEnabled(enabled: Boolean) {
        imageAnalysis?.let { analysis ->
            if (enabled) {
                analysis.setAnalyzer(analysisExecutor, analyzer)
            } else {
                analysis.clearAnalyzer()
            }
        }
    }

    fun startRecording(limitMb: Int) {
        if (recording != null) return
        val capture = videoCapture ?: run {
            onRecordingState(false, "Camera is not ready")
            return
        }
        val directory = context.getExternalFilesDir(Environment.DIRECTORY_MOVIES) ?: context.filesDir
        val timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"))
        val output = File(directory, "motion-session-$timestamp.mp4")
        val fileOptions = FileOutputOptions.Builder(output)
            .setFileSizeLimit(limitMb.coerceIn(50, 5_000) * 1024L * 1024L)
            .build()
        val pending = capture.output.prepareRecording(context, fileOptions)
        recording = pending.start(ContextCompat.getMainExecutor(context)) { event ->
            when (event) {
                is VideoRecordEvent.Start -> onRecordingState(true, output.name)
                is VideoRecordEvent.Finalize -> {
                    recording = null
                    if (event.hasError()) {
                        onRecordingState(false, "Recording error ${event.error}")
                    } else {
                        onRecordingState(false, output.name)
                    }
                }
            }
        }
    }

    fun stopRecording() {
        recording?.stop()
    }

    override fun close() {
        if (!closed.compareAndSet(false, true)) return
        recording?.stop()
        recording = null
        cameraProvider?.unbindAll()
        imageAnalysis = null
        analysisExecutor.shutdown()
    }
}
