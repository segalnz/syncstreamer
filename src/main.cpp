// main.cpp — SyncStreamer v2  (digital-PLL + fractional resampler)
// ─────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>
#include <arpa/inet.h>
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

// ── Client registration ───────────────────────────────────────────────────────
// Sends SYNC_HELLO to the server so it learns this device's IP and adds it
// to its active unicast list.  Called on boot and every 30 s as a keepalive.
static void send_announce()
{
    if (WiFi.status() != WL_CONNECTED) return;

    struct sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(ANNOUNCE_PORT);
    if (inet_aton(SERVER_HOST, &dest.sin_addr) == 0) {
        log_e("Announce: inet_aton failed for %s", SERVER_HOST);
        return;
    }

    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        log_e("Announce: socket() failed");
        return;
    }

    int reuse = 1;
    lwip_setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    for (int attempt = 1; attempt <= 5; ++attempt) {
        int n = lwip_sendto(sock, "SYNC_HELLO", 10, 0,
                            (struct sockaddr*)&dest, sizeof(dest));
        if (n == 10) {
            log_i("Announce -> %s:%d (attempt %d)", SERVER_HOST, ANNOUNCE_PORT, attempt);
            lwip_close(sock);
            return;
        }
        log_w("Announce: lwip_sendto failed (attempt %d/5)", attempt);
        if (attempt < 5) delay(250);
    }

    lwip_close(sock);
    log_e("Announce: gave up after 5 attempts");
}

// ── Announce task (periodic SYNC_HELLO keep-alive) ───────────────────────────
static void announce_task(void* arg) {
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(30000));
        send_announce();
    }
}

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
        send_announce();
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
    xTaskCreatePinnedToCore(sync_task,      "sync_ctrl", 4096, nullptr, 15, nullptr, 0);
    xTaskCreatePinnedToCore(announce_task,  "announce", 4096, nullptr,  5, nullptr, 0);

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

    // Disabled: MQTT/HTTP/ElegantOTA compete with audio UDP for lwIP buffers.
    // status_server_loop(&server);

    delay(10);
}
