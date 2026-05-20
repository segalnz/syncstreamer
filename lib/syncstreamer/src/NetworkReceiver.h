#pragma once
#include <stdint.h>
#include "JitterBuffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// NetworkReceiver — UDP socket receiver task
//
// Binds a UDP socket on UDP_PORT, receives SyncAudioPackets,
// validates them and pushes PCM blocks into JitterBuffer.
// Runs on Core 0 at priority 5 so audio (Core 1) is unaffected.
//
// Usage:
//   netReceiver.begin(&jitterBuffer, UDP_PORT);
// ============================================================

class NetworkReceiver {
public:
    bool begin(JitterBuffer* jbuf, uint16_t port);
    void stop();

    bool     isRunning()  const { return _running; }

    // ── Stats ─────────────────────────────────────────────────
    uint32_t statPacketsReceived;
    uint32_t statParseErrors;
    uint32_t statPushFailed;

private:
    static void _taskFunc(void* arg);
    void        _run();

    JitterBuffer*     _jbuf    = nullptr;
    uint16_t          _port    = 0;
    volatile bool     _running = false;
    TaskHandle_t      _task    = nullptr;
    int               _sock    = -1;
};
