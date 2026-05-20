#include "PlaybackScheduler.h"
#include <Arduino.h>

// ── Begin / Stop ──────────────────────────────────────────────────────────────
bool PlaybackScheduler::begin(JitterBuffer* jbuf, AudioOutput* audio, NTPSync* ntp)
{
    _jbuf    = jbuf;
    _audio   = audio;
    _ntp     = ntp;
    _state   = SchedulerState::FILLING;
    _running = true;
    statUnderruns  = 0;
    statLateBlocks = 0;

    xTaskCreatePinnedToCore(_taskFunc, "playback", 8192, this, 6, &_task, 1);
    log_i("PlaybackScheduler: started");
    return true;
}

void PlaybackScheduler::stop()
{
    _running = false;
    if (_task) {
        vTaskDelay(pdMS_TO_TICKS(200));
        _task = nullptr;
    }
}

// ── Task ──────────────────────────────────────────────────────────────────────
void PlaybackScheduler::_taskFunc(void* arg)
{
    static_cast<PlaybackScheduler*>(arg)->_run();
    vTaskDelete(nullptr);
}

void PlaybackScheduler::_run()
{
    uint32_t fillWaitMs = 0;

    while (_running) {

        // ── FILLING: wait for startup buffer ──────────────────
        if (_state == SchedulerState::FILLING) {
            int64_t fill = _jbuf->getFillLevelMicros();
            if (fill >= (int64_t)_startupMinMs * 1000LL) {
                log_i("PlaybackScheduler: buffer ready (%lld ms), starting playback",
                      fill / 1000LL);
                _state    = SchedulerState::PLAYING;
                fillWaitMs = 0;
                // Stream epoch and sample counters set on first block below.
            } else {
                fillWaitMs += 10;
                if (fillWaitMs >= STARTUP_MAX_MS) {
                    // No data arrived — keep waiting, reset counter.
                    fillWaitMs = 0;
                    log_i("PlaybackScheduler: still filling (%lld ms), rx=%u",
                      fill / 1000LL, _jbuf->statPushOk);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            continue;
        }

        // ── RESETTING ─────────────────────────────────────────
        if (_state == SchedulerState::RESETTING) {
            _state = SchedulerState::FILLING;
            continue;
        }

        // ── PLAYING ───────────────────────────────────────────

        int64_t nextTime = 0;
        if (!_jbuf->peekNextPresentationTime(nextTime)) {
            // Buffer empty — underrun.
            statUnderruns++;
            if (++_consecutiveUnderruns >= UNDERRUN_RESET_THRESH) {
                log_w("PlaybackScheduler: prolonged underrun (%u ms) — resetting to FILLING",
                      _consecutiveUnderruns * 5u);
                _consecutiveUnderruns  = 0;
                _consecutiveLateBlocks = 0;
                _resetToFilling(_currentStreamId);
            } else {
                _audio->writeSilenceMs(5);
            }
            continue;
        }

        // Time at which we must start writing to I2S so DMA delivers on time.
        int64_t writeAt = nextTime - DMA_LATENCY_US;
        int64_t now     = _ntp->getNTPTimeMicros();
        int64_t waitUs  = writeAt - now;

        if (waitUs > 5000LL) {
            // More than 5 ms until write time — sleep 1 ms and re-check.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (waitUs > 0LL) {
            // Sub-5 ms wait — busy-spin via esp_timer.
            int64_t deadline = now + waitUs;
            while (_ntp->getNTPTimeMicros() < deadline) {
                taskYIELD();
            }
        }

        // Pop the block we already peeked at.
        PCMBlock block;
        if (!_jbuf->pop(block)) {
            // Raced with another consumer (shouldn't happen — single consumer).
            statUnderruns++;
            _audio->writeSilenceMs(5);
            continue;
        }

        // Detect stream change.
        if (_currentStreamId != 0 && block.stream_id != _currentStreamId) {
            log_i("PlaybackScheduler: stream change %u→%u, re-filling",
                  _currentStreamId, block.stream_id);
            _jbuf->release(block);
            // Reset AudioOutput duck state so new stream starts at full volume.
            _audio->clearDuck();
            _resetToFilling(block.stream_id);
            continue;
        }

        // ── Lateness check ───────────────────────────────────────────────────────
        // Blocks > LATE_STALE_US (2 s) old are discarded — they cannot contribute
        // to useful audio and indicate a genuine stream-restart or long WiFi outage.
        //
        // Blocks 20 ms – 2 s late are played immediately (no wait).  The phase
        // error is absorbed by the SyncController via proportional slip correction.
        // This is the normal recovery path for an NTP smooth-resync that nudged the
        // clock by a few hundred ms — audio is uninterrupted and the offset corrects
        // within ~30–120 s at 0.42 % playback speed adjustment.
        now = _ntp->getNTPTimeMicros();
        int64_t lateness = now - nextTime;

        if (lateness > LATE_STALE_US) {
            // Truly stale: discard and count toward flush threshold.
            statLateBlocks++;
            log_w("PlaybackScheduler: stale block seq=%u (%lld ms) — discarding",
                  block.sequence, lateness / 1000LL);
            _jbuf->release(block);
            if (++_consecutiveLateBlocks >= LATE_RESET_THRESH) {
                log_w("PlaybackScheduler: %u consecutive stale blocks — flushing and re-filling",
                      _consecutiveLateBlocks);
                _consecutiveLateBlocks = 0;
                _consecutiveUnderruns  = 0;
                _lateRunLogged         = false;
                _resetToFilling(_currentStreamId);
            }
            continue;
        }

        // Block is on-time or only moderately late — play it.
        _consecutiveLateBlocks = 0;
        if (lateness > 20000LL) {
            statLateBlocks++;
            if (!_lateRunLogged) {
                log_i("PlaybackScheduler: seq=%u running %lld ms late — playing; "
                      "SyncController will correct",
                      block.sequence, lateness / 1000LL);
                _lateRunLogged = true;
            }
        } else {
            _lateRunLogged = false;  // back on-time: reset for the next late episode
        }

        // First block of this stream: initialise epoch for SyncController.
        // Guard on _streamEpochUs==0 (not _samplesWrittenAtEpoch==0) to prevent
        // double-set when no I2S writes have completed yet.
        if (_currentStreamId == 0 || _streamEpochUs == 0) {
            _currentStreamId      = block.stream_id;
            _streamEpochUs        = block.presentation_time_us;
            _samplesPlayedAtEpoch = 0;
            _samplesWrittenAtEpoch = _audio->getSamplesWritten();
            log_i("PlaybackScheduler: epoch set stream=%u pt=%lld µs",
                  block.stream_id, _streamEpochUs);
        }

        _audio->writeBlock(block);
        _jbuf->release(block);
        _consecutiveUnderruns  = 0;
        _consecutiveLateBlocks = 0;
    }
}

// ── Reset ─────────────────────────────────────────────────────────────────────
void PlaybackScheduler::_resetToFilling(uint32_t new_stream_id)
{
    _jbuf->flush(new_stream_id);
    _currentStreamId       = new_stream_id;
    _streamEpochUs         = 0;
    _samplesPlayedAtEpoch  = 0;
    _samplesWrittenAtEpoch = 0;
    _lateRunLogged         = false;
    _state                 = SchedulerState::FILLING;
}
