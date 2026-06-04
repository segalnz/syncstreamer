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

// Reset stream sequence tracking. Call on STREAM_START, STREAM_STOP,
// and when entering ACQUIRING state. Avoids spurious seq_gaps on
// track change when the server resets its sequence counter.
void network_receiver_stream_reset(void);

// Packet statistics — exposed for dashboard / MQTT.
extern volatile uint32_t g_rx_packets;
extern volatile uint32_t g_rx_missed;
