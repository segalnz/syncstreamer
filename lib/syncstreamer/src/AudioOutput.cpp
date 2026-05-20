#include "AudioOutput.h"
#include "SyncPacket.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <string.h>

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

// ── Init ──────────────────────────────────────────────────────────────────────
bool AudioOutput::init()
{
    // Unmute PCM5102A: XSMT pin must be HIGH before I2S clocks start.
    pinMode(I2S_PIN_UNMUTE, OUTPUT);
    digitalWrite(I2S_PIN_UNMUTE, LOW);

    i2s_config_t cfg{};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = SAMPLE_RATE_HZ;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = I2S_DMA_BUF_COUNT;
    cfg.dma_buf_len          = I2S_DMA_BUF_SAMPLES;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;   // fill DMA with silence on underrun

    esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) {
        log_e("AudioOutput: i2s_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }

    i2s_pin_config_t pins{};
    pins.mck_io_num     = I2S_PIN_NO_CHANGE;
    pins.bck_io_num     = I2S_PIN_BCK;
    pins.ws_io_num      = I2S_PIN_WS;
    pins.data_out_num   = I2S_PIN_DATA;
    pins.data_in_num    = I2S_PIN_NO_CHANGE;

    err = i2s_set_pin(I2S_PORT, &pins);
    if (err != ESP_OK) {
        log_e("AudioOutput: i2s_set_pin failed: %s", esp_err_to_name(err));
        return false;
    }

    i2s_zero_dma_buffer(I2S_PORT);

    // Unmute after clocks are running.
    digitalWrite(I2S_PIN_UNMUTE, HIGH);

    log_i("AudioOutput: I2S started — BCK=%d WS=%d DATA=%d UNMUTE=%d",
          I2S_PIN_BCK, I2S_PIN_WS, I2S_PIN_DATA, I2S_PIN_UNMUTE);
    return true;
}

// ── Volume ramp ───────────────────────────────────────────────────────────────
void AudioOutput::_stepVolume()
{
    if (!_ramping) return;
    _volumeQ8 += _rampStepQ8;
    if (_rampStepQ8 > 0 && _volumeQ8 >= _targetQ8) {
        _volumeQ8 = _targetQ8;
        _ramping  = false;
    } else if (_rampStepQ8 < 0 && _volumeQ8 <= _targetQ8) {
        _volumeQ8 = _targetQ8;
        _ramping  = false;
    }
}

// ── writeBlock ────────────────────────────────────────────────────────────────
void AudioOutput::writeBlock(const PCMBlock& block)
{
    bool nowDucked = (block.flags & SYNC_PACKET_FLAGS_DUCKED) != 0;

    // Detect duck/restore transitions and set up ramp.
    if (nowDucked && !_ducked) {
        // 0 → 1 : begin duck (fast ramp down)
        _targetQ8   = _duckLevelQ8;
        int steps   = (DUCK_RAMP_MS * SAMPLE_RATE_HZ) / 1000;
        _rampStepQ8 = -(VOL_Q8_MAX - _duckLevelQ8 + steps - 1) / steps; // ceiling negative
        _ramping    = true;
        _ducked     = true;
        log_d("AudioOutput: duck started");
    } else if (!nowDucked && _ducked) {
        // 1 → 0 : begin restore (slow ramp up)
        _targetQ8   = VOL_Q8_MAX;
        int steps   = (RESTORE_RAMP_MS * SAMPLE_RATE_HZ) / 1000;
        _rampStepQ8 = (VOL_Q8_MAX - _duckLevelQ8 + steps - 1) / steps;  // ceiling positive
        _ramping    = true;
        _ducked     = false;
        log_d("AudioOutput: restore started");
    }

    bool mono = (block.flags & SYNC_PACKET_FLAGS_MONO) != 0;

    // Working buffer: always stereo output, max 480 pairs × 4 bytes.
    // Stereo pair = 2 × int16_t = 4 bytes.
    static int16_t outBuf[SYNC_PACKET_MAX_SAMPLES * 2];

    uint16_t pairs  = block.sample_count;
    for (uint16_t i = 0; i < pairs; i++) {
        int16_t L, R;
        if (mono) {
            L = R = block.pcm_data[i];
        } else {
            L = block.pcm_data[i * 2];
            R = block.pcm_data[i * 2 + 1];
        }
        // Apply Q8 volume scaling — intermediate via int32 to prevent overflow.
        L = (int16_t)(((int32_t)L * _volumeQ8) >> 8);
        R = (int16_t)(((int32_t)R * _volumeQ8) >> 8);

        outBuf[i * 2]     = L;
        outBuf[i * 2 + 1] = R;

        _stepVolume();
    }

    size_t outPairs = pairs;

    // Apply one pending slip at block boundary.
    if (_slipPending > 0) {
        // Insert: duplicate the last sample pair after the block.
        outBuf[outPairs * 2]     = outBuf[(outPairs - 1) * 2];
        outBuf[outPairs * 2 + 1] = outBuf[(outPairs - 1) * 2 + 1];
        outPairs++;
        _slipPending--;
        log_v("AudioOutput: slip insert");
    } else if (_slipPending < 0 && outPairs > 1) {
        // Drop: shorten by one pair (skip the first).
        outPairs--;
        memmove(outBuf, outBuf + 2, outPairs * 4u);
        _slipPending++;
        log_v("AudioOutput: slip drop");
    }

    size_t written = 0;
    i2s_write(I2S_PORT, outBuf, outPairs * 4u, &written, portMAX_DELAY);
    // Always count the logical stream frames (block.sample_count), NOT the
    // physical DMA samples (outPairs).  A drop-slip writes one fewer sample to
    // I2S, which speeds the DAC up by 20.83 µs; counting it as the full 240
    // keeps expectedUs tracking the stream timeline, not the hardware pipeline.
    // Using outPairs here creates a positive-feedback loop: drop-slips slow the
    // counter → expectedUs lags wall clock → phaseError grows → more drop-slips.
    _samplesWritten += pairs;
}

// ── writeSilenceMs ────────────────────────────────────────────────────────────
void AudioOutput::writeSilenceMs(uint32_t ms)
{
    // Number of stereo pairs for the requested duration.
    uint32_t pairs = ((uint32_t)SAMPLE_RATE_HZ * ms) / 1000u;
    if (pairs == 0) return;

    // Zero buffer in chunks to avoid large stack allocation.
    static const int16_t silenceChunk[I2S_DMA_BUF_SAMPLES * 2] = {};  // zeroed BSS

    while (pairs > 0) {
        uint32_t chunkPairs = (pairs > (uint32_t)I2S_DMA_BUF_SAMPLES)
                              ? (uint32_t)I2S_DMA_BUF_SAMPLES : pairs;
        size_t written = 0;
        i2s_write(I2S_PORT, silenceChunk, chunkPairs * 4u, &written, portMAX_DELAY);
        _samplesWritten += chunkPairs;
        pairs -= chunkPairs;
    }
}
