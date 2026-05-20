#pragma once
#include <stdint.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <WiFiClient.h>

// ============================================================
// StatusServer v2 — web dashboard + ElegantOTA + MQTT publisher
//
// Routes:
//   GET  /           colourful dark-theme SPA dashboard
//   GET  /api/status JSON snapshot (fetched every 2s by page JS)
//   POST /api/config update target_fill_ms, mode
//   /update          ElegantOTA (registered by ElegantOTA.begin())
// ============================================================

#define MQTT_BROKER   "192.168.5.160"
#define MQTT_PORT     1883
#define MQTT_INTERVAL_MS 2000u

// Initialise web routes, MQTT client. Call after WiFi is connected.
// server must already be constructed; server.begin() called here.
void status_server_init(AsyncWebServer* server);

// Call from loop() — drives ElegantOTA and MQTT keep-alive + publish.
void status_server_loop(AsyncWebServer* server);
