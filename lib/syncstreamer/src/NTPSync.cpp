#include "NTPSync.h"
#include "esp_sntp.h"
#include <sys/time.h>
#include <Arduino.h>

// Singleton definition
NTPSync ntpSync;

// ── SNTP sync callback ────────────────────────────────────────────────────────
static void sntpSyncCallback(struct timeval* tv)
{
    (void)tv;
    ntpSync._onSynced();
}

// ── Public API ────────────────────────────────────────────────────────────────
void NTPSync::begin(const char* server)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, server);
    sntp_set_time_sync_notification_cb(sntpSyncCallback);
    // SMOOTH mode: after the initial large step-sync (clock starts at 1970,
    // so the first sync always steps regardless), subsequent 30-second resyncs
    // use adjtime() to slew the clock gradually instead of stepping it.
    // This prevents periodic NTP corrections from causing sudden ~ms clock jumps
    // that would make buffered blocks appear late and trigger the late-flush path.
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    esp_sntp_set_sync_interval(30000);   // re-sync every 30 s
    esp_sntp_init();

    log_i("NTPSync: polling %s (smooth-resync mode)", server);
}

int64_t NTPSync::getNTPTimeMicros()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;
}

uint32_t NTPSync::getLastSyncAgeSec()
{
    if (!_synced) return UINT32_MAX;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint32_t)((uint32_t)tv.tv_sec - _lastSyncEpoch);
}

void NTPSync::_onSynced()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    _lastSyncEpoch = (uint32_t)tv.tv_sec;
    _synced = true;
    log_i("NTPSync: synced — epoch %u", _lastSyncEpoch);
}
