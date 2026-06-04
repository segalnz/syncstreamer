#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── Include all public library headers ────────────────────────────────────────
#include "SyncPacket.h"
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include "Resampler.h"
#include "SyncController.h"
#include "AudioOutput.h"
#include "NetworkReceiver.h"
#include "StatusServer.h"

// ==============================================================================
// SyncStreamer v2 — Synchronized multi-room PCM audio client for ESP32-S3
//
// Single-include entry point.  Call syncstreamer_init() from setup() and
// syncstreamer_loop() from loop().  All subsystem init, WiFi, task creation,
// and server registration happens automatically.
//
// Quick start:
//
//   #include <syncstreamer.h>
//
//   static void my_mute(bool mute) {
//       digitalWrite(10, mute ? LOW : HIGH);  // PCM5102A XSMT
//   }
//
//   static const syncstreamer_config_t config = {
//       .wifi_ssid     = "MyNetwork",
//       .wifi_password = "secret",
//       .server_host   = "192.168.5.75",
//       .announce_port = 5006,
//       .mute_fn       = my_mute,
//   };
//
//   void setup() {
//       Serial.begin(115200);
//       syncstreamer_init(&config);
//   }
//
//   void loop() {
//       syncstreamer_loop();
//   }
// ==============================================================================

// Callback: library calls mute_fn(true) to mute, mute_fn(false) to unmute.
typedef void (*mute_fn_t)(bool mute);

// Callback: library calls wake_fn(true) on WAKE_DETECTED, wake_fn(false) after timeout.
// Use for visual indication (e.g. LED flash) when wakeword is recognised.
typedef void (*wake_fn_t)(bool wake);

typedef struct {
    const char* wifi_ssid;       // WiFi network name
    const char* wifi_password;   // WiFi network password
    const char* server_host;     // Server IP address (for SYNC_HELLO announce)
    uint16_t    announce_port;   // Server UDP port for client registration (default 5006)
    mute_fn_t   mute_fn;         // Called when library needs to mute/unmute; pass NULL for no-op
    wake_fn_t   wake_fn;         // Called on WAKE_DETECTED control message; pass NULL for no-op
} syncstreamer_config_t;

// Initialise WiFi, all subsystems, create FreeRTOS tasks, and register with
// the server.  Returns true on success.  Call once from setup().
bool syncstreamer_init(const syncstreamer_config_t* cfg);

// Drive periodic tasks: WiFi reconnect, MQTT, SYNC_HELLO.
// Call repeatedly from loop().
void syncstreamer_loop(void);

// Expose the internal AsyncWebServer so callers can register extra routes
// (e.g. OTA). Valid only after syncstreamer_init().
class AsyncWebServer;
AsyncWebServer* syncstreamer_get_server(void);

// Set mute state — calls the user-registered mute_fn and sets g_muted.
// true = mute, false = unmute.
void syncstreamer_set_mute(bool mute);

// Notify the library of a WAKE_DETECTED control message from the server.
// Called from NetworkReceiver; triggers wake_fn(true) and starts a 2.5s
// timeout after which wake_fn(false) is called.
void syncstreamer_notify_wake(void);
