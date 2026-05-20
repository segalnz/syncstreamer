#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "JitterBuffer.h"
#include "AudioOutput.h"
#include "NTPSync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// PlaybackScheduler — wall-clock timed PCM block release
//
// States:
//   FILLING  — accumulate startup buffer (>= STARTUP_MIN_MS)
//   PLAYING  — pop blocks from JitterBuffer when NTP time allows
//   RESETTING — stream change detected; drain and re-fill
//
// The scheduler pins to Core 1 at priority 6 so it can feed the
// I2S DMA ahead of the sync controller (priority 2).
//
// i2s_write() blocks until DMA accepts data, so scheduling
// accuracy is naturally gated by the DMA buffer depth (~40 ms).
// ============================================================

// ── Startup / steady-state tuning ────────────────────────────
static constexpr uint32_t STARTUP_MIN_MS       = 300;   // min buffer before first play
static constexpr uint32_t STARTUP_MAX_MS       = 1000;  // log "still filling" interval
static constexpr uint32_t UNDERRUN_RESET_THRESH = 200;  // consec underruns → re-fill (200×5ms=1s)
static constexpr uint32_t LATE_RESET_THRESH     = 10;   // consec STALE blocks → flush+re-fill
static constexpr int64_t  LATE_STALE_US         = 2000000LL; // blocks > 2 s late are discarded
static constexpr int64_t  DMA_LATENCY_US    =
    (int64_t)AudioOutput::DMA_LATENCY_SAMPLES * 1000000LL / SAMPLE_RATE_HZ;

enum class SchedulerState { FILLING, PLAYING, RESETTING };

class PlaybackScheduler {
public:
    bool begin(JitterBuffer* jbuf, AudioOutput* audio, NTPSync* ntp);
    void stop();

    SchedulerState getState()         const { return _state; }
    int64_t  getStreamEpochUs()       const { return _streamEpochUs; }
    uint32_t getSamplesPlayedSync()   const { return _samplesPlayedAtEpoch
                                              + _audio->getSamplesWritten()
                                              - _samplesWrittenAtEpoch; }
    bool     isPlaying()              const { return _state == SchedulerState::PLAYING; }
    uint32_t getSamplesWrittenAtEpoch() const { return _samplesWrittenAtEpoch; }
    uint32_t getCurrentStreamId()       const { return _currentStreamId; }

    // Runtime-configurable startup fill threshold (ms). Thread-safe: volatile.
    void     setStartupMinMs(uint32_t ms) { _startupMinMs = ms; }
    uint32_t getStartupMinMs()  const     { return _startupMinMs; }

    uint32_t statUnderruns;
    uint32_t statLateBlocks;

private:
    static void _taskFunc(void* arg);
    void        _run();
    void        _resetToFilling(uint32_t new_stream_id);

    JitterBuffer*  _jbuf  = nullptr;
    AudioOutput*   _audio = nullptr;
    NTPSync*       _ntp   = nullptr;

    volatile SchedulerState _state = SchedulerState::FILLING;
    volatile bool           _running = false;
    TaskHandle_t            _task    = nullptr;

    uint32_t _currentStreamId      = 0;
    volatile uint32_t _startupMinMs = STARTUP_MIN_MS;  // runtime-configurable

    // Stream epoch tracking for SyncController.
    int64_t  _streamEpochUs        = 0;
    uint32_t _samplesPlayedAtEpoch = 0;
    uint32_t _samplesWrittenAtEpoch = 0;

    // Recovery counters.
    uint32_t _consecutiveUnderruns  = 0;
    uint32_t _consecutiveLateBlocks = 0;
    bool     _lateRunLogged         = false;  // suppress per-block log spam during a late episode
};
