package com.example.app_drone_decode

import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import com.example.app_drone_decode.decoder.PythonDecoderFacade
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.SlotObservation
import kotlinx.coroutines.runBlocking
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class PythonBridgeInstrumentedTest {
    @Test
    fun helloReferenceVectorCrossesKotlinPythonBoundary() = runBlocking {
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        val facade = PythonDecoderFacade(context)
        val actions = facade.encodeReferenceFrame().getOrThrow()

        assertEquals(40, actions.size)
        assertEquals("R L H R F L F H", actions.take(8).joinToString(" ") { it.shortName })
        assertTrue(actions.none { it.name == "UNKNOWN" })

        facade.configureProfile(DecoderProfile()).getOrThrow()
        facade.start().getOrThrow()
        var acceptedText: String? = null
        actions.forEachIndexed { index, action ->
            val event = facade.process(
                SlotObservation(
                    slotIndex = index.toLong(),
                    action = action,
                    confidence = 1f,
                    erased = false,
                    sampleCount = 15,
                ),
            )
            if (event.result?.accepted == true) acceptedText = event.result.payloadText
        }
        assertEquals("Hello", acceptedText)
    }
}
