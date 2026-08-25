package com.example.app_drone_decode.domain

import com.example.app_drone_decode.domain.model.ActionClass
import com.example.app_drone_decode.domain.model.DecoderProfile
import com.example.app_drone_decode.domain.model.DecoderProfileValidator
import org.junit.Assert.assertFalse
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class DecoderProfileValidatorTest {
    @Test
    fun protocolDefaultsAreValid() {
        val profile = DecoderProfile()
        assertTrue(DecoderProfileValidator.validate(profile).valid)
        assertEquals(3, profile.protocolVersion)
        assertEquals("00", profile.actionMapping.getValue(ActionClass.FORWARD))
        assertEquals("11", profile.actionMapping.getValue(ActionClass.HOVER))
        assertEquals("10", profile.actionMapping.getValue(ActionClass.YAW_RIGHT))
        assertEquals("01", profile.actionMapping.getValue(ActionClass.YAW_LEFT))
    }

    @Test
    fun duplicateActionCodesAreRejected() {
        val profile = DecoderProfile(
            actionMapping = mapOf(
                ActionClass.HOVER to "00",
                ActionClass.FORWARD to "00",
                ActionClass.YAW_LEFT to "11",
                ActionClass.YAW_RIGHT to "10",
            ),
        )
        assertFalse(DecoderProfileValidator.validate(profile).valid)
    }

    @Test
    fun malformedSyncIsRejected() {
        val profile = DecoderProfile(syncActions = listOf(ActionClass.HOVER, ActionClass.UNKNOWN))
        assertFalse(DecoderProfileValidator.validate(profile).valid)
    }
}
