package com.example.app_drone_decode.connectivity

import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow

enum class TransportState {
    DISABLED,
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR,
}

data class TransportConfig(
    val endpoint: String = "",
    val authenticationToken: String = "",
    val timeoutMs: Long = 5_000,
)

data class DebugEnvelope(
    val type: String,
    val payload: String,
    val injected: Boolean = false,
)

interface DebugTransport {
    val state: StateFlow<TransportState>
    val incoming: Flow<DebugEnvelope>
    suspend fun connect(config: TransportConfig)
    suspend fun send(message: DebugEnvelope)
    suspend fun disconnect()
}

class FakeDebugTransport : DebugTransport {
    private val mutableState = MutableStateFlow(TransportState.DISCONNECTED)
    private val mutableIncoming = MutableSharedFlow<DebugEnvelope>(extraBufferCapacity = 16)
    override val state: StateFlow<TransportState> = mutableState
    override val incoming: Flow<DebugEnvelope> = mutableIncoming

    override suspend fun connect(config: TransportConfig) {
        mutableState.value = TransportState.CONNECTED
    }

    override suspend fun send(message: DebugEnvelope) {
        check(mutableState.value == TransportState.CONNECTED)
        mutableIncoming.emit(message.copy(injected = true))
    }

    override suspend fun disconnect() {
        mutableState.value = TransportState.DISCONNECTED
    }
}
