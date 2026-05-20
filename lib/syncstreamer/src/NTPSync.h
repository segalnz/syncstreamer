#pragma once
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// NTPSync — wraps ESP-IDF SNTP for high-resolution wall-clock
//
// Usage:
//   ntpSync.begin("192.168.5.4");
//   if (ntpSync.isSynced()) {
//       int64_t now = ntpSync.getNTPTimeMicros();
//   }
// ============================================================

class NTPSync {
public:
    // Start SNTP polling against the given server.
    // Call once after WiFi is connected.
    void begin(const char* server);

    // Microseconds since UNIX epoch (from gettimeofday).
    // Valid even before NTP sync — returns local RTC time.
    int64_t getNTPTimeMicros();

    // True once the first NTP sync has completed.
    bool isSynced() const { return _synced; }

    // Seconds elapsed since the most recent successful NTP sync.
    // Returns UINT32_MAX if never synced.
    uint32_t getLastSyncAgeSec();

    // Called internally by the SNTP sync callback — do not call directly.
    void _onSynced();

private:
    volatile bool     _synced         = false;
    volatile uint32_t _lastSyncEpoch  = 0;  // tv_sec at time of last sync
};

// Singleton — shared between all modules.
extern NTPSync ntpSync;
