package com.example.app_drone_decode

import androidx.test.ext.junit.runners.AndroidJUnit4
import com.example.app_drone_decode.vision.CvFrame
import com.example.app_drone_decode.vision.GreenMarkerTracker
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.opencv.android.OpenCVLoader
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

@RunWith(AndroidJUnit4::class)
class OpenCvTrackerInstrumentedTest {
    @Test
    fun deterministicGreenMarkerFixtureIsTracked() {
        assertTrue(OpenCVLoader.initLocal())
        val rgba = Mat.zeros(480, 640, CvType.CV_8UC4)
        try {
            Imgproc.rectangle(
                rgba,
                Point(270.0, 190.0),
                Point(370.0, 290.0),
                Scalar(0.0, 255.0, 0.0, 255.0),
                Imgproc.FILLED,
            )
            val result = GreenMarkerTracker().process(CvFrame(rgba, 1_000_000_000L))

            assertTrue(result.visible)
            assertEquals(0.5f, result.boundingBox!!.centerX, 0.02f)
            assertEquals(0.5f, result.boundingBox!!.centerY, 0.02f)
        } finally {
            rgba.release()
        }
    }
}
