#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

#include "secrets.h"
#include "NTPSync.h"
#include "JitterBuffer.h"
#include "NetworkReceiver.h"
#include "AudioOutput.h"
#include "PlaybackScheduler.h"
#include "SyncController.h"
#include "StatusServer.h"

// ── Global objects ────────────────────────────────────────────────────────────
static AsyncWebServer  server(80);
static JitterBuffer    jitterBuffer;
static NetworkReceiver netReceiver;
static AudioOutput     audioOutput;
static PlaybackScheduler scheduler;
static SyncController  syncController;
static StatusServer    statusServer;

// ── Server registration ───────────────────────────────────────────────────────
// Sends a UDP announce to SERVER_HOST:ANNOUNCE_PORT so the server records
// this device's current IP and adds it to the active unicast client set.
// Called on boot and every 30 s as a keepalive.
static void sendAnnounce()
{
    if (WiFi.status() != WL_CONNECTED) return;
    WiFiUDP udp;
    udp.beginPacket(SERVER_HOST, ANNOUNCE_PORT);
    udp.write((const uint8_t*)"SYNC_HELLO", 10);
    udp.endPacket();
    log_i("Announce → %s:%d  (our IP: %s)",
          SERVER_HOST, ANNOUNCE_PORT, WiFi.localIP().toString().c_str());
}

// ── WiFi helpers ──────────────────────────────────────────────────────────────
static void wifiConnect()
{
    log_i("WiFi: connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                        // disable modem sleep — needed for low-latency audio
    WiFi.setTxPower(WIFI_POWER_19_5dBm);         // maximum TX power
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const uint32_t timeout = 20000;
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        log_i("WiFi: connected — IP %s  RSSI %d dBm  BSSID %s  Ch %d",
              WiFi.localIP().toString().c_str(),
              WiFi.RSSI(),
              WiFi.BSSIDstr().c_str(),
              WiFi.channel());
        sendAnnounce();
    } else {
        log_w("WiFi: connection timed out — will retry in loop");
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    // USB-CDC TX becomes non-blocking: if the host isn't reading fast enough the
    // log data is silently dropped rather than blocking the caller.  Without this,
    // a full CDC TX buffer makes log_i() hold the esp_log mutex for seconds, which
    // in turn stalls any other task (e.g. PlaybackScheduler) that tries to log —
    // causing the observed audio dropouts that correlate with status printouts.
    Serial.setTxTimeoutMs(0);
    delay(200);
    Serial.println("\n\n=== SyncStreamer boot ===");

    // 1. WiFi
    wifiConnect();

    // 2. NTP (start polling immediately after WiFi up)
    ntpSync.begin(NTP_HOST);

    // 3. Audio output — must init before scheduler/sync
    if (!audioOutput.init()) {
        log_e("AudioOutput init failed — halting");
        while (true) delay(1000);
    }

    // 4. Jitter buffer (PSRAM)
    if (!jitterBuffer.init()) {
        log_e("JitterBuffer init failed — halting");
        while (true) delay(1000);
    }

    // 5. Network receiver (UDP socket + task)
    if (!netReceiver.begin(&jitterBuffer, UDP_PORT)) {
        log_e("NetworkReceiver init failed — halting");
        while (true) delay(1000);
    }

    // 6. Playback scheduler task
    scheduler.begin(&jitterBuffer, &audioOutput, &ntpSync);

    // 7. Sync controller task
    syncController.begin(&ntpSync, &audioOutput, &scheduler);

    // 8. Web server: ElegantOTA + status page
    ElegantOTA.begin(&server);
    statusServer.begin(&server, &ntpSync, &jitterBuffer, &netReceiver,
                       &audioOutput, &scheduler, &syncController);
    server.begin();

    log_i("SyncStreamer ready — http://%s/", WiFi.localIP().toString().c_str());
}

// ── loop ──────────────────────────────────────────────────────────────────────
static uint32_t _lastStatusMs    = 0;
static uint32_t _lastWifiCheckMs = 0;
static uint32_t _lastAnnounceMs  = 0;

void loop()
{
    // ElegantOTA requires regular calls during firmware updates.
    ElegantOTA.loop();

    uint32_t now = millis();

    // WiFi watchdog: reconnect if dropped.
    if (now - _lastWifiCheckMs > 5000) {
        _lastWifiCheckMs = now;
        if (WiFi.status() != WL_CONNECTED) {
            log_w("WiFi: disconnected — reconnecting");
            WiFi.disconnect(true);
            delay(100);
            wifiConnect();
        }
    }

    // Keepalive: re-announce every 30 s so the server refreshes our TTL.
    if (now - _lastAnnounceMs > 30000) {
        _lastAnnounceMs = now;
        sendAnnounce();
    }

    // Periodic serial status summary.
    if (now - _lastStatusMs > 10000) {
        _lastStatusMs = now;
        const SyncStatus& ss = syncController.getStatus();
        const char* stateStr =
            scheduler.getState() == SchedulerState::PLAYING  ? "PLAY"  :
            scheduler.getState() == SchedulerState::FILLING  ? "FILL"  : "RESET";
        log_i("Status | WiFi: %d dBm | NTP: %s | State: %s | Buf: %lld ms | Vol: %d%%",
              WiFi.RSSI(),
              ntpSync.isSynced() ? "OK" : "wait",
              stateStr,
              jitterBuffer.getFillLevelMicros() / 1000LL,
              audioOutput.getCurrentVolumePercent());
        log_i("       | rx=%u parse_err=%u push_fail=%u | "
              "jbuf ok=%u late=%u dup=%u ovf=%u | "
              "Phase: %lld µs (filt %lld µs) | Slips +%d/-%d",
              netReceiver.statPacketsReceived,
              netReceiver.statParseErrors,
              netReceiver.statPushFailed,
              jitterBuffer.statPushOk,
              jitterBuffer.statDropLate,
              jitterBuffer.statDropDuplicate,
              jitterBuffer.statDropOverflow,
              ss.phaseErrorUs,
              ss.filteredErrorUs,
              ss.totalSlipsInserted,
              ss.totalSlipsDropped);
    }

    delay(10);
}
