#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// NetworkReceiver v2 — two UDP socket tasks
//
// wifi_rx_task  (Core 0, priority 18): port 5005 — audio packets
// ctrl_rx_task  (Core 0, priority 16): port 5006 — control messages
// ============================================================

#define NET_AUDIO_PORT  5005u
#define NET_CTRL_PORT   5006u

// FreeRTOS task bodies. Create from main.cpp.
void wifi_rx_task(void* pvParam);
void ctrl_rx_task(void* pvParam);
