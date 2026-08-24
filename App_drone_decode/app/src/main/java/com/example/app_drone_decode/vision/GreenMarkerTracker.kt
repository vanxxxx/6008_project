package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.NormalizedRect
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

class GreenMarkerTracker(
    private val minimumAreaFraction: Double = 0.0008,
    private val roiProvider: () -> NormalizedRect = { NormalizedRect(0f, 0f, 1f, 1f) },
) : TargetTracker {
    override fun process(frame: CvFrame): TrackingResult {
        val rgb = Mat()
        val hsv = Mat()
        val mask = Mat()
        val hierarchy = Mat()
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(5.0, 5.0))
        var hsvRoi: Mat? = null
        val contours = mutableListOf<MatOfPoint>()
        return try {
            Imgproc.cvtColor(frame.rgba, rgb, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
            val roi = roiProvider()
            val roiLeft = (roi.left.coerceIn(0f, 0.99f) * hsv.cols()).toInt()
            val roiTop = (roi.top.coerceIn(0f, 0.99f) * hsv.rows()).toInt()
            val roiRight = (roi.right.coerceIn(0.01f, 1f) * hsv.cols()).toInt().coerceAtLeast(roiLeft + 1)
            val roiBottom = (roi.bottom.coerceIn(0.01f, 1f) * hsv.rows()).toInt().coerceAtLeast(roiTop + 1)
            val activeRoi = hsv.submat(roiTop, roiBottom, roiLeft, roiRight)
            hsvRoi = activeRoi
            Core.inRange(activeRoi, Scalar(35.0, 70.0, 60.0), Scalar(90.0, 255.0, 255.0), mask)
            Imgproc.morphologyEx(
                mask,
                mask,
                Imgproc.MORPH_OPEN,
                kernel,
            )
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

            val largest = contours.maxByOrNull(Imgproc::contourArea)
                ?: return TrackingResult(visible = false, confidence = 0f)
            val frameArea = frame.rgba.rows().toDouble() * frame.rgba.cols().toDouble()
            val areaFraction = Imgproc.contourArea(largest) / frameArea
            if (areaFraction < minimumAreaFraction) {
                return TrackingResult(visible = false, confidence = 0f)
            }

            val rect = Imgproc.boundingRect(largest)
            val points = MatOfPoint2f(*largest.toArray())
            val orientation = try {
                val rotatedBox = Imgproc.minAreaRect(points)
                var normalizedAngle = rotatedBox.angle.toFloat()
                if (rotatedBox.size.width < rotatedBox.size.height) normalizedAngle += 90f
                while (normalizedAngle >= 90f) normalizedAngle -= 180f
                while (normalizedAngle < -90f) normalizedAngle += 180f
                normalizedAngle
            } finally {
                points.release()
            }
            val width = frame.rgba.cols().toFloat()
            val height = frame.rgba.rows().toFloat()
            val areaStrength = (areaFraction / minimumAreaFraction - 1.0).coerceIn(0.0, 1.0).toFloat()
            TrackingResult(
                visible = true,
                confidence = (0.65f + areaStrength * 0.30f).coerceIn(0f, 1f),
                boundingBox = NormalizedRect(
                    left = (rect.x + roiLeft) / width,
                    top = (rect.y + roiTop) / height,
                    right = (rect.x + roiLeft + rect.width) / width,
                    bottom = (rect.y + roiTop + rect.height) / height,
                ),
                orientationDeg = orientation,
            )
        } finally {
            contours.forEach(MatOfPoint::release)
            hierarchy.release()
            kernel.release()
            hsvRoi?.release()
            mask.release()
            hsv.release()
            rgb.release()
        }
    }
}
