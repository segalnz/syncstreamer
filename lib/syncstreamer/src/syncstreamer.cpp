#include "syncstreamer.h"
#include "SyncController.h"        // g_muted
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>

// ── Internal state ─────────────────────────────────────────────────────────────
static AsyncWebServer s_server(80);
static syncstreamer_config_t s_cfg;
static mute_fn_t       s_mute_fn = nullptr;
static wake_fn_t       s_wake_fn = nullptr;
static volatile bool   s_wake_active = false;
static volatile uint32_t s_wake_time = 0;   // millis() when WAKE_DETECTED was last received
#define WAKE_LED_MS     2500u               // how long the LED stays on after wake

// ── Wake notification ───────────────────────────────────────────────────────────
void syncstreamer_notify_wake(void)
{
    s_wake_active = true;
    s_wake_time = millis();
    if (s_wake_fn) s_wake_fn(true);
}

// ── Mute ───────────────────────────────────────────────────────────────────────
void syncstreamer_set_mute(bool mute)
{
    if (s_mute_fn) s_mute_fn(mute);
    g_muted = mute;
}

// ── Server accessor ────────────────────────────────────────────────────────────
AsyncWebServer* syncstreamer_get_server(void)
{
    return &s_server;
}

// ── SYNC_HELLO announce ───────────────────────────────────────────────────────
static void send_announce(void)
{
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiUDP udp;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        udp.beginPacket(s_cfg.server_host, s_cfg.announce_port);
        udp.write(reinterpret_cast<const uint8_t*>("SYNC_HELLO"), 10);
        if (udp.endPacket()) {
            log_i("Announce -> %s:%d  (our IP: %s, attempt %d)",
                  s_cfg.server_host, s_cfg.announce_port,
                  WiFi.localIP().toString().c_str(), attempt);
            return;
        }
        log_w("Announce: endPacket failed (attempt %d/5)", attempt);
        delay(250);
    }
    log_e("Announce: gave up after 5 attempts");
}

// ── WiFi connect ──────────────────────────────────────────────────────────────
static void wifi_connect(void)
{
    log_i("WiFi: connecting to \"%s\"", s_cfg.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(s_cfg.wifi_ssid, s_cfg.wifi_password);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    const uint32_t kTimeout = 20000;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < kTimeout) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi: connected -- IP %s  RSSI %d dBm",
              WiFi.localIP().toString().c_str(), WiFi.RSSI());
        send_announce();
    } else {
        log_w("WiFi: timed out -- will retry in loop");
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────
bool syncstreamer_init(const syncstreamer_config_t* cfg)
{
    s_cfg = *cfg;
    if (s_cfg.announce_port == 0) s_cfg.announce_port = 5006;
    s_mute_fn = cfg->mute_fn;
    s_wake_fn = cfg->wake_fn;
    s_wake_active = false;
    s_wake_time = 0;

    Serial.setTxTimeoutMs(0);
    delay(200);
    Serial.println("\n=== SyncStreamer v2 boot ===");

    wifi_connect();

    jb_init();
    offset_estimator_init();
    resampler_init(&g_resampler);
    sync_controller_init();
    audio_out_init();
    status_server_init(&s_server);

    xTaskCreatePinnedToCore(wifi_rx_task,   "wifi_rx",   8192, nullptr, 18, nullptr, 0);
    xTaskCreatePinnedToCore(ctrl_rx_task,   "ctrl_rx",   4096, nullptr, 16, nullptr, 0);
    xTaskCreatePinnedToCore(audio_out_task, "audio_out", 4096, nullptr, 22, nullptr, 1);
    xTaskCreatePinnedToCore(sync_task, "sync_ctrl", 4096, nullptr, 15, nullptr, 0);

    log_i("syncstreamer_init() complete -- tasks running");
    return true;
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void syncstreamer_loop(void)
{
    if (WiFi.status() != WL_CONNECTED) {
        log_w("WiFi lost -- reconnecting");
        WiFi.reconnect();
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < 10000) {
            delay(250);
        }
    }

    status_server_loop(&s_server);

    static uint32_t s_last_announce = 0;
    if (millis() - s_last_announce >= 30000) {
        s_last_announce = millis();
        send_announce();
    }

    // Wake LED timeout.
    if (s_wake_active && (millis() - s_wake_time >= WAKE_LED_MS)) {
        s_wake_active = false;
        if (s_wake_fn) s_wake_fn(false);
    }

    delay(10);
}
