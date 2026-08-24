package com.example.app_drone_decode

import android.app.Application
import com.example.app_drone_decode.data.config.DecoderProfileRepository
import com.example.app_drone_decode.data.config.FunctionSettingsRepository
import com.example.app_drone_decode.data.logs.SessionLogStore
import com.example.app_drone_decode.decoder.PythonDecoderFacade

class DroneDecodeApplication : Application() {
    lateinit var container: AppContainer
        private set

    override fun onCreate() {
        super.onCreate()
        container = AppContainer(this)
    }
}

class AppContainer(application: Application) {
    val profileRepository = DecoderProfileRepository(application)
    val functionSettingsRepository = FunctionSettingsRepository(application)
    val sessionLogStore = SessionLogStore(application)
    val decoderFacade = PythonDecoderFacade(application)
}
