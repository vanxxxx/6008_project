package com.example.app_drone_decode

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.outlined.ListAlt
import androidx.compose.material.icons.outlined.MonitorHeart
import androidx.compose.material.icons.outlined.Settings
import androidx.compose.material.icons.outlined.Tune
import androidx.compose.material3.Icon
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.example.app_drone_decode.ui.decoderconfig.DecoderConfigScreen
import com.example.app_drone_decode.ui.functionconfig.FunctionConfigScreen
import com.example.app_drone_decode.ui.logs.LogsScreen
import com.example.app_drone_decode.ui.monitor.MonitorScreen
import com.example.app_drone_decode.ui.monitor.MonitorViewModel
import com.example.app_drone_decode.ui.monitor.MonitorViewModelFactory
import com.example.app_drone_decode.ui.theme.MotionDecoderTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        val application = application as DroneDecodeApplication
        setContent {
            MotionDecoderTheme {
                val viewModel: MonitorViewModel = viewModel(
                    factory = MonitorViewModelFactory(application, application.container),
                )
                MotionDecoderApp(viewModel)
            }
        }
    }
}

private enum class Destination(
    val route: String,
    val label: String,
    val icon: @Composable () -> Unit,
) {
    Monitor("monitor", "Monitor", { Icon(Icons.Outlined.MonitorHeart, contentDescription = null) }),
    Decoder("decoder", "Decoder", { Icon(Icons.Outlined.Tune, contentDescription = null) }),
    Functions("functions", "Functions", { Icon(Icons.Outlined.Settings, contentDescription = null) }),
    Logs("logs", "Logs", { Icon(Icons.AutoMirrored.Outlined.ListAlt, contentDescription = null) }),
}

@Composable
private fun MotionDecoderApp(viewModel: MonitorViewModel) {
    val navController = rememberNavController()
    val backStackEntry by navController.currentBackStackEntryAsState()
    val currentRoute = backStackEntry?.destination?.route
    val state by viewModel.state.collectAsStateWithLifecycle()
    Scaffold(
        bottomBar = {
            NavigationBar {
                Destination.entries.forEach { destination ->
                    NavigationBarItem(
                        selected = currentRoute == destination.route,
                        onClick = {
                            navController.navigate(destination.route) {
                                popUpTo(navController.graph.findStartDestination().id) { saveState = true }
                                launchSingleTop = true
                                restoreState = true
                            }
                        },
                        icon = destination.icon,
                        label = { Text(destination.label) },
                    )
                }
            }
        },
    ) { padding ->
        NavHost(
            navController = navController,
            startDestination = Destination.Monitor.route,
            modifier = Modifier.padding(padding),
        ) {
            composable(Destination.Monitor.route) {
                MonitorScreen(state = state, viewModel = viewModel)
            }
            composable(Destination.Decoder.route) {
                DecoderConfigScreen(state = state, viewModel = viewModel)
            }
            composable(Destination.Functions.route) {
                FunctionConfigScreen(state = state, viewModel = viewModel)
            }
            composable(Destination.Logs.route) {
                LogsScreen(state = state, viewModel = viewModel)
            }
        }
    }
}
