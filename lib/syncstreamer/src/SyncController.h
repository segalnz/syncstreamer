#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "NTPSync.h"
#include "AudioOutput.h"
#include "PlaybackScheduler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// SyncController — phase error measurement and drift correction
//
// Runs every SYNC_INTERVAL_MS on Core 1 at priority 2.
//
// Phase error model:
//   expected_time = stream_epoch_us
//                 + (samples_written_since_epoch / 48000.0) * 1e6
//   phase_error   = NTP_now − (expected_time + DMA_LATENCY_US)
//
// A positive error means playback is behind wall clock (too slow)
// → drop 1 sample to speed up.
// A negative error means playback is ahead (too fast)
// → insert 1 sample to slow down.
//
// The error is low-pass filtered (IIR) to avoid chasing noise.
// A slip is only requested when the filtered error exceeds
// SLIP_THRESHOLD_US.
// ============================================================

// ── Tuning ───────────────────────────────────────────────────
//
// SYNC_INTERVAL_MS   — how often the controller wakes and measures phase error.
//                      250 ms gives 4 measurements/sec, adequate for crystal drift.
//
// SLIP_THRESHOLD_US  — dead zone: no correction below this filtered error.
//                      200 µs avoids chasing NTP/measurement noise.
//
// LPF_ALPHA          — IIR smoothing factor.  Larger = faster response.
//                      0.15 gives a ~1.7 s time constant at 250 ms intervals:
//                      fast enough to track a sudden 600 ms NTP step within ~10 s,
//                      slow enough to reject per-packet jitter.
//
// MAX_SLIPS_PER_RUN  — cap on proportional slip requests per interval.
//                      64 slips × 20.83 µs = 1.3 ms max correction per 250 ms run.
//                      AudioOutput drains slips at 1/block × 200 blocks/s:
//                      • small error (1 ms): corrects in ~2 s — imperceptible
//                      • NTP step (600 ms): corrects in ~145 s at 0.42 % pitch shift
//
static constexpr uint32_t SYNC_INTERVAL_MS   = 250;   // ms between measurements
static constexpr int64_t  SLIP_THRESHOLD_US  = 200;   // ±200 µs dead zone
static constexpr float    LPF_ALPHA          = 0.15f; // IIR coefficient
static constexpr int32_t  MAX_SLIPS_PER_RUN  = 64;    // cap per 250 ms interval

struct SyncStatus {
    int64_t  phaseErrorUs;          // raw last measurement
    int64_t  filteredErrorUs;       // low-pass filtered
    int32_t  totalSlipsInserted;
    int32_t  totalSlipsDropped;
    bool     active;                // true when stream epoch is valid
};

class SyncController {
public:
    bool begin(NTPSync* ntp, AudioOutput* audio, PlaybackScheduler* sched);
    void stop();

    const SyncStatus& getStatus() const { return _status; }

private:
    static void _taskFunc(void* arg);
    void        _run();

    NTPSync*           _ntp   = nullptr;
    AudioOutput*       _audio = nullptr;
    PlaybackScheduler* _sched = nullptr;

    volatile bool _running = false;
    TaskHandle_t  _task    = nullptr;

    float     _filteredError = 0.0f;  // running filtered phase error (µs)
    SyncStatus _status{};
};
