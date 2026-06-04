#include "AudioOutput.h"
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include <Arduino.h>
#include <string.h>

// Forward declarations — defined in SyncController.cpp / Resampler.cpp
extern "C" {
    typedef enum { ST_IDLE, ST_ACQUIRING, ST_LOCKED, ST_RECOVERING, ST_REACQUIRING } client_state_t;
    extern volatile client_state_t g_state;
    extern volatile bool           g_muted;
    extern volatile uint64_t       g_first_present_us;
}
// Resampler forward — avoids circular include; Resampler.h included in SyncController
struct resampler_t;
extern resampler_t g_resampler;
extern bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r);

// ── Global I2S channel handle ─────────────────────────────────────────────────
i2s_chan_handle_t g_i2s_tx = nullptr;

// ── DMA output buffer (512 stereo frames = 2048 bytes) ───────────────────────
static int16_t s_dma_buf[AO_DMA_FRAMES * 2];

// ── Init ──────────────────────────────────────────────────────────────────────
bool audio_out_init(void)
{
    // XSMT low = muted while clock starts (prevents pop).
    pinMode(AO_PIN_XSMT, OUTPUT);
    digitalWrite(AO_PIN_XSMT, LOW);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;   // zero DMA on underrun
    chan_cfg.dma_desc_num  = AO_DMA_BUFS;
    chan_cfg.dma_frame_num = AO_DMA_FRAMES;

    esp_err_t err = i2s_new_channel(&chan_cfg, &g_i2s_tx, nullptr);
    if (err != ESP_OK) {
        log_e("AudioOutput: i2s_new_channel failed: %s", esp_err_to_name(err));
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AO_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                     I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)AO_PIN_BCLK,
            .ws   = (gpio_num_t)AO_PIN_LRCLK,
            .dout = (gpio_num_t)AO_PIN_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };

    err = i2s_channel_init_std_mode(g_i2s_tx, &std_cfg);
    if (err != ESP_OK) {
        log_e("AudioOutput: i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2s_channel_enable(g_i2s_tx);
    if (err != ESP_OK) {
        log_e("AudioOutput: i2s_channel_enable failed: %s", esp_err_to_name(err));
        return false;
    }

    // Clock is now running; safe to unmute.
    digitalWrite(AO_PIN_XSMT, HIGH);

    log_i("AudioOutput: I2S started — BCLK=%d LRCLK=%d DOUT=%d XSMT=%d",
          AO_PIN_BCLK, AO_PIN_LRCLK, AO_PIN_DOUT, AO_PIN_XSMT);
    return true;
}

// ── Mute / Unmute ─────────────────────────────────────────────────────────────
void audio_out_mute(void)
{
    digitalWrite(AO_PIN_XSMT, LOW);
    g_muted = true;
}

void audio_out_unmute(void)
{
    digitalWrite(AO_PIN_XSMT, HIGH);
    g_muted = false;
}

// ── audio_out_task ────────────────────────────────────────────────────────────
// Core 1, priority 22. Fills one DMA buffer per iteration from resampler or zeros.
// Presentation-time stall on first LOCKED entry per stream.
#define BUFFER_FILL_US      ((uint64_t)JB_STARTUP_FRAMES * 1000000ULL / JB_SAMPLE_RATE)
#define PLAYBACK_LATENCY_US 11000

void audio_out_task(void* pvParam)
{
    TickType_t xLastWake = xTaskGetTickCount();
    static bool s_started = false;

    for (;;) {
        bool play = (g_state == ST_ACQUIRING || g_state == ST_LOCKED || g_state == ST_RECOVERING) && !g_muted;

        // Presentation-time stall: block output until server's intended
        // present_us for the first buffered frame.
        if (!s_started && play && g_first_present_us != 0) {
            int64_t target = (int64_t)g_first_present_us
                           + (int64_t)BUFFER_FILL_US
                           - (int64_t)PLAYBACK_LATENCY_US;
            if (server_now_us() < target) {
                play = false;
            } else {
                s_started = true;
            }
        }
        if (g_state != ST_LOCKED && g_state != ST_RECOVERING)
            s_started = false;

        if (play) {
            for (uint32_t i = 0; i < AO_DMA_FRAMES; i++) {
                int16_t l = 0, r = 0;
                resampler_get_frame(&g_resampler, &l, &r);
                s_dma_buf[i * 2]     = l;
                s_dma_buf[i * 2 + 1] = r;
            }
        } else {
            memset(s_dma_buf, 0, sizeof(s_dma_buf));
        }

        size_t written = 0;
        i2s_channel_write(g_i2s_tx, s_dma_buf, sizeof(s_dma_buf), &written, portMAX_DELAY);
    }
}
