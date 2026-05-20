// main.cpp — SyncStreamer v2  (digital-PLL + fractional resampler)
// ─────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "secrets.h"          // WIFI_SSID, WIFI_PASSWORD
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include "Resampler.h"
#include "SyncController.h"
#include "AudioOutput.h"
#include "NetworkReceiver.h"
#include "StatusServer.h"

// ── Global web server ─────────────────────────────────────────────────────────
static AsyncWebServer server(80);

// ── WiFi ──────────────────────────────────────────────────────────────────────
static void wifi_connect()
{
    log_i("WiFi: connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                     // disable modem-sleep for low-latency audio
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t kTimeout = 20000;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < kTimeout) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi: connected — IP %s  RSSI %d dBm",
              WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        log_w("WiFi: timed out — will retry in loop");
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);   // non-blocking CDC-TX: drop if host not reading
    delay(200);
    Serial.println("\n=== SyncStreamer v2 boot ===");

    // 1. WiFi
    wifi_connect();

    // 2. Subsystems (order matters — JB before resampler, offset before sync)
    jb_init();
    offset_estimator_init();
    resampler_init(&g_resampler);
    sync_controller_init();

    // 3. I2S / DAC
    audio_out_init();

    // 4. HTTP / OTA / MQTT
    status_server_init(&server);

    // 5. FreeRTOS tasks
    //    Core 0 — network (wifi_rx + ctrl_rx)
    //    Core 1 — real-time (audio_out + sync_task)
    xTaskCreatePinnedToCore(wifi_rx_task,   "wifi_rx",   8192, nullptr, 18, nullptr, 0);
    xTaskCreatePinnedToCore(ctrl_rx_task,   "ctrl_rx",   4096, nullptr, 16, nullptr, 0);
    xTaskCreatePinnedToCore(audio_out_task, "audio_out", 4096, nullptr, 22, nullptr, 1);
    xTaskCreatePinnedToCore(sync_task,      "sync_ctrl", 4096, nullptr, 15, nullptr, 1);

    log_i("setup() complete — tasks running");
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop()
{
    // Reconnect WiFi if dropped
    if (WiFi.status() != WL_CONNECTED) {
        log_w("WiFi lost — reconnecting");
        WiFi.reconnect();
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
            delay(250);
        }
    }

    // ElegantOTA keepalive + MQTT publish
    status_server_loop(&server);

    delay(10);
}
