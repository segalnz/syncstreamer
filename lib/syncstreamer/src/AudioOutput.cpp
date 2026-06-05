#include "AudioOutput.h"
#include "SyncController.h"
#include "JitterBuffer.h"
#include "Resampler.h"
#include <Arduino.h>
#include <string.h>

// ── Global I2S channel handle ─────────────────────────────────────────────────
i2s_chan_handle_t g_i2s_tx = nullptr;

// ── DMA output buffer ─────────────────────────────────────────────────────────
static int16_t s_dma_buf[AO_DMA_FRAMES * 2];

// ── Init ──────────────────────────────────────────────────────────────────────
bool audio_out_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
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

    log_i("AudioOutput: I2S started — BCLK=%d LRCLK=%d DOUT=%d",
          AO_PIN_BCLK, AO_PIN_LRCLK, AO_PIN_DOUT);
    return true;
}

// ── Mute / Unmute (I2S zeros handle silence) ──────────────────────────────────
void audio_out_mute(void)   { g_muted = true; }
void audio_out_unmute(void) { g_muted = false; }

// ── audio_out_task ────────────────────────────────────────────────────────────
// Core 1, priority 22. Fills one DMA buffer per iteration from resampler or zeros.
void audio_out_task(void* pvParam)
{
    TickType_t xLastWake = xTaskGetTickCount();

    for (;;) {
        bool play = (g_state == ST_ACQUIRING || g_state == ST_LOCKED || g_state == ST_RECOVERING) && !g_muted;

        if (play) {
            for (uint32_t i = 0; i < AO_DMA_FRAMES; i++) {
                int16_t l = 0, r = 0;
                resampler_get_frame(&g_resampler, &l, &r);
                s_dma_buf[i * 2]     = (int16_t)(l * 0.89f);  // −1 dB headroom for DAC intersample overs
                s_dma_buf[i * 2 + 1] = (int16_t)(r * 0.89f);
            }
        } else {
            memset(s_dma_buf, 0, sizeof(s_dma_buf));
        }

        size_t written = 0;
        i2s_channel_write(g_i2s_tx, s_dma_buf, sizeof(s_dma_buf), &written, portMAX_DELAY);
    }
}
