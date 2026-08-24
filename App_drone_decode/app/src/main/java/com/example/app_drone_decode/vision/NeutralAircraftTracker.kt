package com.example.app_drone_decode.vision

import com.example.app_drone_decode.domain.model.NormalizedRect
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/** Tracks a low-saturation aircraft silhouette without simulator metadata. */
class NeutralAircraftTracker(
    private val minimumAreaFraction: Double = 0.00035,
    private val maximumAreaFraction: Double = 0.015,
    private val roiProvider: () -> NormalizedRect = { NormalizedRect(0f, 0f, 1f, 1f) },
) : TargetTracker {
    private var previousCenter: Point? = null
    private var previousSceneGrid: SceneGrid? = null
    private var previousSceneTimestampNs: Long? = null
    private var accumulatedOrientationDeg = 0f
    private var previousMeasuredOrientationDeg: Float? = null

    override fun process(frame: CvFrame): TrackingResult {
        val rgb = Mat()
        val hsv = Mat()
        val gray = Mat()
        val mask = Mat()
        val hierarchy = Mat()
        val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(3.0, 3.0))
        var hsvRoi: Mat? = null
        val contours = mutableListOf<MatOfPoint>()
        try {
            Imgproc.cvtColor(frame.rgba, rgb, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY)
            val roi = roiProvider()
            val left = (roi.left.coerceIn(0f, 0.99f) * hsv.cols()).toInt()
            val top = (roi.top.coerceIn(0f, 0.99f) * hsv.rows()).toInt()
            val right = (roi.right.coerceIn(0.01f, 1f) * hsv.cols()).toInt().coerceAtLeast(left + 1)
            val bottom = (roi.bottom.coerceIn(0.01f, 1f) * hsv.rows()).toInt().coerceAtLeast(top + 1)
            hsvRoi = hsv.submat(top, bottom, left, right)

            // The red/white checkerboard is chromatic while the aircraft body is neutral gray.
            Core.inRange(hsvRoi, Scalar(0.0, 0.0, 55.0), Scalar(180.0, 46.0, 232.0), mask)
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel)
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel)
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

            val frameArea = frame.rgba.rows().toDouble() * frame.rgba.cols().toDouble()
            val selected = contours
                .map { contour -> contour to Imgproc.boundingRect(contour) }
                .filter { (contour, rect) ->
                    val fraction = Imgproc.contourArea(contour) / frameArea
                    val aspect = rect.width.toDouble() / rect.height.coerceAtLeast(1)
                    fraction in minimumAreaFraction..maximumAreaFraction && aspect in 0.45..3.5
                }
                .maxByOrNull { (contour, _) -> candidateScore(contour, left, top) }
                ?: return lost(gray, frame.timestampNs)

            val contour = selected.first
            val rect = selected.second
            val moments = Imgproc.moments(contour)
            val center = if (abs(moments.m00) > 0.001) {
                Point(left + moments.m10 / moments.m00, top + moments.m01 / moments.m00)
            } else {
                Point(left + rect.x + rect.width / 2.0, top + rect.y + rect.height / 2.0)
            }
            val fullMask = Mat.zeros(gray.rows(), gray.cols(), org.opencv.core.CvType.CV_8UC1)
            val sceneMotion = estimateSceneMotion(gray, frame.timestampNs)
            try {
                val shifted = MatOfPoint(*contour.toArray().map { Point(it.x + left, it.y + top) }.toTypedArray())
                try {
                    Imgproc.drawContours(fullMask, listOf(shifted), 0, Scalar(255.0), Imgproc.FILLED)
                } finally {
                    shifted.release()
                }
                updateMeasuredOrientation(moments)
                previousCenter = center
            } finally {
                fullMask.release()
            }

            val areaFraction = Imgproc.contourArea(contour) / frameArea
            val areaStrength = ((areaFraction - minimumAreaFraction) /
                (minimumAreaFraction * 2.5)).coerceIn(0.0, 1.0).toFloat()
            return TrackingResult(
                visible = true,
                confidence = 0.62f + areaStrength * 0.30f,
                boundingBox = NormalizedRect(
                    left = (left + rect.x) / frame.rgba.cols().toFloat(),
                    top = (top + rect.y) / frame.rgba.rows().toFloat(),
                    right = (left + rect.x + rect.width) / frame.rgba.cols().toFloat(),
                    bottom = (top + rect.y + rect.height) / frame.rgba.rows().toFloat(),
                ),
                orientationDeg = accumulatedOrientationDeg,
                sceneVelocityXPerSec = sceneMotion?.velocityXPerSec,
                sceneVelocityYPerSec = sceneMotion?.velocityYPerSec,
                sceneRotationDegPerSec = sceneMotion?.rotationDegPerSec,
                sceneScaleRatePerSec = sceneMotion?.scaleRatePerSec,
                targetAreaFraction = areaFraction.toFloat(),
            )
        } finally {
            contours.forEach(MatOfPoint::release)
            hsvRoi?.release()
            kernel.release()
            hierarchy.release()
            mask.release()
            gray.release()
            hsv.release()
            rgb.release()
        }
    }

    /**
     * Measures background translation with a small grayscale grid and robust
     * block matching. Keeping this calculation in Kotlin avoids a second
     * native optical-flow invocation in the per-frame path, which was unstable
     * on the Android x86 OpenCV runtime. No video metadata is used.
     */
    private fun estimateSceneMotion(gray: Mat, timestampNs: Long): SceneMotion? {
        val current = sceneGrid(gray) ?: return null
        val prior = previousSceneGrid
        val priorTimestamp = previousSceneTimestampNs
        previousSceneGrid = current
        previousSceneTimestampNs = timestampNs
        if (prior == null || priorTimestamp == null || prior.width != current.width || prior.height != current.height) {
            return null
        }
        val elapsedSec = (timestampNs - priorTimestamp) / 1_000_000_000.0
        if (elapsedSec !in 0.001..0.5) return null
        val target = previousCenter
        val excludedLeft = target?.let { (it.x / gray.cols() * prior.width - prior.width * 0.24).toInt() }
        val excludedRight = target?.let { (it.x / gray.cols() * prior.width + prior.width * 0.20).toInt() }
        val excludedTop = target?.let { (it.y / gray.rows() * prior.height - prior.height * 0.17).toInt() }
        val excludedBottom = target?.let { (it.y / gray.rows() * prior.height + prior.height * 0.25).toInt() }
        val shifts = mutableListOf<SceneShift>()
        val usableLeft = SCENE_SEARCH_RADIUS
        val usableRight = prior.width - SCENE_SEARCH_RADIUS
        val usableTop = SCENE_SEARCH_RADIUS
        val usableBottom = prior.height - SCENE_SEARCH_RADIUS
        for (tileY in 0 until SCENE_TILE_ROWS) {
            val top = usableTop + (usableBottom - usableTop) * tileY / SCENE_TILE_ROWS
            val bottom = usableTop + (usableBottom - usableTop) * (tileY + 1) / SCENE_TILE_ROWS
            for (tileX in 0 until SCENE_TILE_COLUMNS) {
                val left = usableLeft + (usableRight - usableLeft) * tileX / SCENE_TILE_COLUMNS
                val right = usableLeft + (usableRight - usableLeft) * (tileX + 1) / SCENE_TILE_COLUMNS
                val centerX = (left + right) / 2.0
                val centerY = (top + bottom) / 2.0
                val overlapsTarget = excludedLeft != null && excludedRight != null &&
                    excludedTop != null && excludedBottom != null &&
                    centerX.toInt() in excludedLeft..excludedRight && centerY.toInt() in excludedTop..excludedBottom
                if (!overlapsTarget) {
                    matchSceneTile(prior, current, left, top, right, bottom)?.let { shift ->
                        shifts += shift.copy(x = centerX, y = centerY)
                    }
                }
            }
        }
        if (shifts.size < MINIMUM_SCENE_TILES) return null
        val medianDx = medianDouble(shifts.map(SceneShift::dx))
        val medianDy = medianDouble(shifts.map(SceneShift::dy))
        val residuals = shifts.map { shift ->
            val dx = shift.dx - medianDx
            val dy = shift.dy - medianDy
            dx * dx + dy * dy
        }
        val residualLimit = medianDouble(residuals) * 4.0 + 0.25
        val retained = shifts.filterIndexed { index, _ -> residuals[index] <= residualLimit }
        if (retained.size < MINIMUM_SCENE_TILES) return null
        val centerX = prior.width / 2.0
        val centerY = prior.height / 2.0
        var scaleNumerator = 0.0
        var rotationNumerator = 0.0
        var denominator = 0.0
        retained.forEach { shift ->
            val x = shift.x - centerX
            val y = shift.y - centerY
            val dx = shift.dx - medianDx
            val dy = shift.dy - medianDy
            scaleNumerator += x * dx + y * dy
            rotationNumerator += -y * dx + x * dy
            denominator += x * x + y * y
        }
        val scaleDelta = if (denominator > 1.0) scaleNumerator / denominator else 0.0
        val rotationRad = if (denominator > 1.0) rotationNumerator / denominator else 0.0
        return SceneMotion(
            velocityXPerSec = (medianDx / current.width / elapsedSec).toFloat(),
            velocityYPerSec = (medianDy / current.height / elapsedSec).toFloat(),
            rotationDegPerSec = (rotationRad * 180.0 / PI / elapsedSec).toFloat(),
            scaleRatePerSec = (scaleDelta / elapsedSec).toFloat(),
        )
    }

    private fun matchSceneTile(
        prior: SceneGrid,
        current: SceneGrid,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
    ): SceneShift? {
        val costs = Array(SCENE_SEARCH_RADIUS * 2 + 1) { DoubleArray(SCENE_SEARCH_RADIUS * 2 + 1) }
        for (dy in -SCENE_SEARCH_RADIUS..SCENE_SEARCH_RADIUS) {
            for (dx in -SCENE_SEARCH_RADIUS..SCENE_SEARCH_RADIUS) {
                var difference = 0L
                var samples = 0
                var y = top
                while (y < bottom) {
                    var x = left
                    while (x < right) {
                        val oldValue = prior.values[y * prior.width + x].toInt() and 0xff
                        val newValue = current.values[(y + dy) * current.width + x + dx].toInt() and 0xff
                        difference += abs(oldValue - newValue)
                        samples++
                        x += SCENE_SAMPLE_STEP
                    }
                    y += SCENE_SAMPLE_STEP
                }
                costs[dy + SCENE_SEARCH_RADIUS][dx + SCENE_SEARCH_RADIUS] =
                    if (samples == 0) Double.MAX_VALUE else difference.toDouble() / samples
            }
        }
        var bestColumn = 0
        var bestRow = 0
        var bestCost = Double.MAX_VALUE
        costs.indices.forEach { row ->
            costs[row].indices.forEach { column ->
                if (costs[row][column] < bestCost) {
                    bestCost = costs[row][column]
                    bestColumn = column
                    bestRow = row
                }
            }
        }
        if (!bestCost.isFinite() || bestCost > MAXIMUM_SCENE_MATCH_COST) return null
        val bestDx = bestColumn - SCENE_SEARCH_RADIUS
        val bestDy = bestRow - SCENE_SEARCH_RADIUS
        val refinedDx = bestDx + parabolicOffset(
            costs[bestRow].getOrNull(bestColumn - 1),
            bestCost,
            costs[bestRow].getOrNull(bestColumn + 1),
        )
        val refinedDy = bestDy + parabolicOffset(
            costs.getOrNull(bestRow - 1)?.get(bestColumn),
            bestCost,
            costs.getOrNull(bestRow + 1)?.get(bestColumn),
        )
        return SceneShift(0.0, 0.0, refinedDx, refinedDy)
    }

    private fun sceneGrid(gray: Mat): SceneGrid? {
        val source = ByteArray(gray.rows() * gray.cols())
        if (gray.get(0, 0, source) <= 0) return null
        val width = (gray.cols() / SCENE_GRID_STRIDE).coerceAtLeast(1)
        val height = (gray.rows() / SCENE_GRID_STRIDE).coerceAtLeast(1)
        val values = ByteArray(width * height)
        for (y in 0 until height) {
            val sourceY = (y * SCENE_GRID_STRIDE + SCENE_GRID_STRIDE / 2).coerceAtMost(gray.rows() - 1)
            for (x in 0 until width) {
                val sourceX = (x * SCENE_GRID_STRIDE + SCENE_GRID_STRIDE / 2).coerceAtMost(gray.cols() - 1)
                values[y * width + x] = source[sourceY * gray.cols() + sourceX]
            }
        }
        return SceneGrid(width, height, values)
    }

    private fun parabolicOffset(lower: Double?, center: Double, upper: Double?): Double {
        if (lower == null || upper == null) return 0.0
        val denominator = lower - 2.0 * center + upper
        if (abs(denominator) < 0.000001) return 0.0
        return (0.5 * (lower - upper) / denominator).coerceIn(-0.5, 0.5)
    }

    private fun medianDouble(values: List<Double>): Double {
        val sorted = values.sorted()
        val middle = sorted.size / 2
        return if (sorted.size % 2 == 0) {
            (sorted[middle - 1] + sorted[middle]) / 2.0
        } else {
            sorted[middle]
        }
    }

    private fun candidateScore(contour: MatOfPoint, absoluteX: Int, absoluteY: Int): Double {
        val area = Imgproc.contourArea(contour)
        val prior = previousCenter ?: return area
        val moments = Imgproc.moments(contour)
        val localX = if (abs(moments.m00) > 0.001) moments.m10 / moments.m00 else 0.0
        val localY = if (abs(moments.m00) > 0.001) moments.m01 / moments.m00 else 0.0
        val distanceSquared = (absoluteX + localX - prior.x) * (absoluteX + localX - prior.x) +
            (absoluteY + localY - prior.y) * (absoluteY + localY - prior.y)
        return area / (1.0 + distanceSquared * 0.01)
    }

    private fun updateMeasuredOrientation(moments: org.opencv.imgproc.Moments) {
        val measured = ((0.5 * atan2(2.0 * moments.mu11, moments.mu20 - moments.mu02) * 180.0 / PI) % 180.0)
            .toFloat()
        val previous = previousMeasuredOrientationDeg
        if (previous == null) {
            accumulatedOrientationDeg = measured
        } else {
            var delta = measured - previous
            while (delta > 90f) delta -= 180f
            while (delta < -90f) delta += 180f
            accumulatedOrientationDeg += delta
        }
        previousMeasuredOrientationDeg = measured
    }

    private fun lost(gray: Mat, timestampNs: Long): TrackingResult {
        sceneGrid(gray)?.let { grid ->
            previousSceneGrid = grid
            previousSceneTimestampNs = timestampNs
        }
        previousCenter = null
        return TrackingResult(visible = false, confidence = 0f)
    }

    override fun reset() {
        previousCenter = null
        previousSceneGrid = null
        previousSceneTimestampNs = null
        accumulatedOrientationDeg = 0f
        previousMeasuredOrientationDeg = null
    }

    private data class SceneMotion(
        val velocityXPerSec: Float,
        val velocityYPerSec: Float,
        val rotationDegPerSec: Float,
        val scaleRatePerSec: Float,
    )

    private data class SceneGrid(val width: Int, val height: Int, val values: ByteArray)

    private data class SceneShift(
        val x: Double,
        val y: Double,
        val dx: Double,
        val dy: Double,
    )

    private companion object {
        const val SCENE_GRID_STRIDE = 5
        const val SCENE_SEARCH_RADIUS = 5
        const val SCENE_SAMPLE_STEP = 2
        const val SCENE_TILE_COLUMNS = 4
        const val SCENE_TILE_ROWS = 3
        const val MINIMUM_SCENE_TILES = 4
        const val MAXIMUM_SCENE_MATCH_COST = 55.0
    }
}
