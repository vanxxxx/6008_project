package com.example.app_drone_decode.ui.monitor

import android.app.Application
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.app_drone_decode.AppContainer

class MonitorViewModelFactory(
    private val application: Application,
    private val container: AppContainer,
) : ViewModelProvider.Factory {
    @Suppress("UNCHECKED_CAST")
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        require(modelClass.isAssignableFrom(MonitorViewModel::class.java))
        return MonitorViewModel(application, container) as T
    }
}
