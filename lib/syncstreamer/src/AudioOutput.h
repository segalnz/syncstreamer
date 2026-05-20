#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "JitterBuffer.h"   // for PCMBlock
#include "freertos/FreeRTOS.h"
#include "freertos/atomic.h"

// ============================================================
// AudioOutput — ESP-IDF I2S driver wrapper for PCM5102A DAC
//
// Pins (compile-time constants matching hardware):
//   BCK   = 15
//   WS    = 16
//   DATA  = 17
//   UNMUTE = 10  (active-high)
//
// Sample rate: 48 000 Hz, 16-bit, stereo (I2S always stereo).
// Mono packets are expanded L→L,R before writing.
//
// Volume duck/restore:
//   Driven by FLAGS_DUCKED on each block.
//   Duck : fast ramp (~50 ms) to DUCK_LEVEL_PERCENT.
//   Restore: slow ramp (~500 ms) back to 100%.
//   Volume applied via Q8 integer multiply (no float in hot path).
//
// Call init() once, then writeBlock() / writeSilenceMs() from the
// PlaybackScheduler task.
// ============================================================

// ── Hardware pin constants ────────────────────────────────────
static constexpr int  I2S_PIN_BCK    = 15;
static constexpr int  I2S_PIN_WS     = 16;
static constexpr int  I2S_PIN_DATA   = 17;
static constexpr int  I2S_PIN_UNMUTE = 10;

// ── DMA configuration ─────────────────────────────────────────
// 8 × 480-sample DMA buffers ≈ 80 ms DMA queue
static constexpr int  I2S_DMA_BUF_COUNT  = 8;
static constexpr int  I2S_DMA_BUF_SAMPLES = 480;  // samples (stereo pairs) per DMA buf

// ── Volume duck constants ─────────────────────────────────────
static constexpr int  DUCK_LEVEL_PERCENT   = 20;
static constexpr int  DUCK_RAMP_MS         = 50;
static constexpr int  RESTORE_RAMP_MS      = 500;
static constexpr int  SAMPLE_RATE_HZ       = 48000;

// Q8 fixed-point helpers (0–256 maps to 0–100%)
static constexpr int  VOL_Q8_MAX  = 256;
static constexpr int  VOL_Q8_DUCK = (DUCK_LEVEL_PERCENT * VOL_Q8_MAX) / 100;

class AudioOutput {
public:
    // Initialise I2S driver and unmute DAC. Returns false on error.
    bool init();

    // Write one PCM block to I2S.
    // Handles mono→stereo expansion and volume ramping.
    // Blocks until I2S DMA accepts all samples (i2s_write timeout = portMAX_DELAY).
    void writeBlock(const PCMBlock& block);

    // Write durationMs milliseconds of silence to keep DMA fed.
    void writeSilenceMs(uint32_t ms);

    // Total stereo sample pairs written to I2S DMA (monotonically increasing).
    uint32_t getSamplesWritten() const { return _samplesWritten; }

    // Estimated DMA latency in samples (fixed — actual queue depth not queried).
    static constexpr uint32_t DMA_LATENCY_SAMPLES =
        I2S_DMA_BUF_COUNT * I2S_DMA_BUF_SAMPLES / 2;

    // Current volume scale (0–100 integer percent) — for status display.
    int getCurrentVolumePercent() const {
        return (_volumeQ8 * 100) / VOL_Q8_MAX;
    }

    bool isDucked() const { return _ducked; }

    // Immediately restore volume to full — call on stream change so the incoming
    // stream is not inadvertently played at duck level.
    void clearDuck() {
        _ducked      = false;
        _ramping     = false;
        _volumeQ8    = VOL_Q8_MAX;
        _targetQ8    = VOL_Q8_MAX;
        _rampStepQ8  = 0;
    }

    // Runtime-configurable duck level (0–50%). Changes take effect on next transition.
    void setDuckLevelPercent(int pct) {
        pct = (pct < 0) ? 0 : (pct > 50) ? 50 : pct;
        _duckLevelQ8 = (pct * VOL_Q8_MAX) / 100;
    }
    int getDuckLevelPercent() const { return (_duckLevelQ8 * 100) / VOL_Q8_MAX; }

    // Request a single-sample slip at the next block boundary.
    // +1 = insert one silent pair (slow down), -1 = drop one pair (speed up).
    // Only one slip is applied per writeBlock() call; subsequent calls queue.
    void requestSlip(int direction) { _slipPending += direction; }

private:
    // Apply per-sample volume ramp step. Call once per stereo pair.
    void _stepVolume();

    volatile uint32_t _samplesWritten = 0;

    // Volume ramp state (not accessed from ISR — PlaybackScheduler task only).
    int  _volumeQ8     = VOL_Q8_MAX;   // current Q8 volume
    int  _targetQ8     = VOL_Q8_MAX;
    int  _rampStepQ8   = 0;            // signed step per sample
    bool _ducked       = false;        // current FLAGS_DUCKED state
    bool _ramping      = false;
    int  _slipPending  = 0;            // +N insert N pairs, -N drop N pairs
    int  _duckLevelQ8  = VOL_Q8_DUCK;  // runtime-configurable duck target
};
