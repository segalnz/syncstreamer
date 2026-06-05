#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"

// ============================================================
// AudioOutput v2 — ESP-IDF I2S v5 driver for PCM5102A
//
// Clock runs CONTINUOUSLY from audio_out_init(). Never stop/restart it.
// XSMT is tied to 3.3V — mute/unmute handled by I2S zeros.
//
// audio_out_task() must be created on Core 1, priority 22.
// It calls resampler_get_frame() when LOCKED/RECOVERING,
// otherwise writes zero samples.
// ============================================================

// ── Hardware pins ─────────────────────────────────────────────
#define AO_PIN_BCLK   15
#define AO_PIN_LRCLK  16
#define AO_PIN_DOUT   17

// ── I2S config ────────────────────────────────────────────────
#define AO_SAMPLE_RATE   48000u
#define AO_DMA_BUFS      4
#define AO_DMA_FRAMES    512u   // frames (stereo pairs) per DMA buffer

// Shared I2S channel handle (used by audio_out_task internally).
extern i2s_chan_handle_t g_i2s_tx;

// Initialise I2S driver, configure channel, start clock.
// Returns false on error. Must be called before creating audio_out_task.
bool audio_out_init(void);

// Mute — I2S zeros handle silence.
void audio_out_mute(void);

// Unmute — I2S resumes audio.
void audio_out_unmute(void);

// FreeRTOS task body. Pin to Core 1, priority 22.
void audio_out_task(void* pvParam);
