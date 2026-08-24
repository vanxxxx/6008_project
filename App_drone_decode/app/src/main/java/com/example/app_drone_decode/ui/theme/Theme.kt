package com.example.app_drone_decode.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

private val DarkColors = darkColorScheme(
    primary = Color(0xFF9EA8FF),
    onPrimary = Color(0xFF101432),
    primaryContainer = Color(0xFF252B55),
    onPrimaryContainer = Color(0xFFDDE0FF),
    secondary = Color(0xFF78B7FF),
    background = Color(0xFF0E0F12),
    onBackground = Color(0xFFE7E7EA),
    surface = Color(0xFF141519),
    onSurface = Color(0xFFE7E7EA),
    surfaceVariant = Color(0xFF1C1E24),
    onSurfaceVariant = Color(0xFFB6B8C2),
    outline = Color(0xFF343740),
    error = Color(0xFFFF8A8A),
)

private val LightColors = lightColorScheme(
    primary = Color(0xFF4D58C9),
    onPrimary = Color.White,
    primaryContainer = Color(0xFFE1E4FF),
    onPrimaryContainer = Color(0xFF171B52),
    secondary = Color(0xFF1A67AA),
    background = Color(0xFFF7F7F9),
    onBackground = Color(0xFF1A1B20),
    surface = Color(0xFFFFFFFF),
    onSurface = Color(0xFF1A1B20),
    surfaceVariant = Color(0xFFF0F1F4),
    onSurfaceVariant = Color(0xFF565861),
    outline = Color(0xFFD8DAE0),
    error = Color(0xFFBA1A1A),
)

val AcceptedGreen = Color(0xFF4FCB8D)
val WarningAmber = Color(0xFFF1B65B)
val RejectedRed = Color(0xFFFF6B73)
val RecordingRed = Color(0xFFFF4D5D)

@Composable
fun MotionDecoderTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = if (isSystemInDarkTheme()) DarkColors else LightColors,
        content = content,
    )
}
