#include "SyncController.h"
#include <Arduino.h>

// ── Begin / Stop ──────────────────────────────────────────────────────────────
bool SyncController::begin(NTPSync* ntp, AudioOutput* audio, PlaybackScheduler* sched)
{
    _ntp     = ntp;
    _audio   = audio;
    _sched   = sched;
    _running = true;
    _filteredError = 0.0f;
    memset(&_status, 0, sizeof(_status));

    xTaskCreatePinnedToCore(_taskFunc, "sync_ctrl", 4096, this, 2, &_task, 1);
    log_i("SyncController: started (threshold ±%lld µs, alpha=%.3f)",
          SLIP_THRESHOLD_US, LPF_ALPHA);
    return true;
}

void SyncController::stop()
{
    _running = false;
    if (_task) {
        vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS + 50));
        _task = nullptr;
    }
}

// ── Task ──────────────────────────────────────────────────────────────────────
void SyncController::_taskFunc(void* arg)
{
    static_cast<SyncController*>(arg)->_run();
    vTaskDelete(nullptr);
}

void SyncController::_run()
{
    while (_running) {
        vTaskDelay(pdMS_TO_TICKS(SYNC_INTERVAL_MS));

        // Only active while PlaybackScheduler has a valid epoch.
        if (!_sched->isPlaying() || _sched->getStreamEpochUs() == 0) {
            _filteredError    = 0.0f;
            _status.active    = false;
            continue;
        }

        // ── Compute phase error ───────────────────────────────
        // Samples written since stream epoch started.
        uint32_t samplesWritten  = _audio->getSamplesWritten();
        uint32_t samplesAtEpoch  = _sched->getSamplesWrittenAtEpoch();
        int64_t  epochUs         = _sched->getStreamEpochUs();

        // The sample at position (written - DMA_LATENCY_SAMPLES) is at the DAC now.
        // Its expected wall-clock output time = epoch_us + (samples_since_epoch - DMA_latency) / 48k
        int64_t samplesSinceEpoch = (int64_t)(samplesWritten - samplesAtEpoch)
                                  - (int64_t)AudioOutput::DMA_LATENCY_SAMPLES;

        int64_t expectedUs = epochUs
            + samplesSinceEpoch * 1000000LL / (int64_t)SAMPLE_RATE_HZ;

        // Actual NTP wall clock now.
        int64_t nowUs = _ntp->getNTPTimeMicros();

        // phase_error > 0: DAC is behind wall clock → too slow → drop sample
        // phase_error < 0: DAC is ahead of wall clock → too fast → insert sample
        int64_t phaseError = nowUs - expectedUs;

        // ── Low-pass filter ───────────────────────────────────
        _filteredError = (1.0f - LPF_ALPHA) * _filteredError
                       + LPF_ALPHA          * (float)phaseError;

        // ── Update status (for StatusServer) ─────────────────
        _status.phaseErrorUs    = phaseError;
        _status.filteredErrorUs = (int64_t)_filteredError;
        _status.active          = true;

        log_d("SyncCtrl: raw=%lld µs  filtered=%lld µs",
              phaseError, _status.filteredErrorUs);

        // ── Apply proportional slip correction ────────────────────────────────
        // Each slip = ±1 stereo sample = ±20.83 µs at 48 kHz.
        // AudioOutput queues slips and applies one per writeBlock() call, so large
        // requests drain gradually at ~200 slips/sec — 0.42 % speed change.
        if (!_ntp->isSynced()) continue;

        constexpr float US_PER_SLIP = 1000000.0f / (float)SAMPLE_RATE_HZ; // ≈20.83

        if (_filteredError > (float)SLIP_THRESHOLD_US) {
            // Playback behind wall clock → drop samples to speed up.
            int32_t slips = (int32_t)(_filteredError / US_PER_SLIP);
            if (slips > MAX_SLIPS_PER_RUN) slips = MAX_SLIPS_PER_RUN;
            _audio->requestSlip(-slips);
            _status.totalSlipsDropped += slips;
            log_d("SyncCtrl: %d drop-slip(s) requested (filtered=%lld µs)",
                  slips, _status.filteredErrorUs);
        } else if (_filteredError < -(float)SLIP_THRESHOLD_US) {
            // Playback ahead of wall clock → insert samples to slow down.
            int32_t slips = (int32_t)(-_filteredError / US_PER_SLIP);
            if (slips > MAX_SLIPS_PER_RUN) slips = MAX_SLIPS_PER_RUN;
            _audio->requestSlip(+slips);
            _status.totalSlipsInserted += slips;
            log_d("SyncCtrl: %d insert-slip(s) requested (filtered=%lld µs)",
                  slips, _status.filteredErrorUs);
        }
    }
}
