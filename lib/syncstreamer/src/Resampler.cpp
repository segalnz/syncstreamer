#include "Resampler.h"
#include "JitterBuffer.h"
#include <string.h>
#include <math.h>

// g_rate_ppm defined in SyncController.cpp
extern volatile float g_rate_ppm;

#define CONCEAL_FADE_FRAMES 240   // ~5ms at 48 kHz — smooth fade in/out

volatile uint32_t g_dropout_frames = 0;
resampler_t g_resampler;

void resampler_init(resampler_t* rs)
{
    rs->phase             = 0.0f;
    rs->prev[0]           = 0;
    rs->prev[1]           = 0;
    rs->amplitude         = 1.0f;
    rs->rate_ppm_filtered = 0.0f;
    rs->conceal_phase     = 0;
    g_dropout_frames      = 0;
}

bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r)
{
    // Copy volatile g_rate_ppm once to avoid torn reads from sync_task preemption.
    float target_ppm = g_rate_ppm;
    // Slow IIR: alpha = 0.0007 → τ ≈ 30 ms at 48 kHz
    rs->rate_ppm_filtered += (target_ppm - rs->rate_ppm_filtered) * 0.0007f;
    rs->phase += 1.0f + rs->rate_ppm_filtered * 1e-6f;

    bool valid = true;

    // Consume as many input frames as phase has accumulated.
    while (rs->phase >= 1.0f) {
        rs->phase -= 1.0f;

        jb_frame_t* f = jb_peek();
        if (f) {
            rs->prev[0] = f->pcm[0];
            rs->prev[1] = f->pcm[1];
        } else {
            valid = false;
        }
        jb_advance();
    }

    // Determine if we have a next frame for interpolation.
    jb_frame_t* next = jb_peek();
    bool have_next = (next != nullptr);
    bool frame_valid = valid || !have_next;

    if (have_next && valid) {
        // Normal: interpolate between prev and next.
        float alpha  = rs->phase;
        *out_l = (int16_t)(rs->prev[0] + alpha * (next->pcm[0] - rs->prev[0]));
        *out_r = (int16_t)(rs->prev[1] + alpha * (next->pcm[1] - rs->prev[1]));
    } else {
        // No valid frame: zero-order hold of prev.
        *out_l = rs->prev[0];
        *out_r = rs->prev[1];
    }

    // ── Linear crossfade gap concealment ──────────────────────────
    // conceal_phase > 0: fade-out (counts 1..CONCEAL_FADE_FRAMES, amplitude → 0)
    // conceal_phase < 0: fade-in  (counts -1..-CONCEAL_FADE_FRAMES, amplitude → 1)
    // conceal_phase = 0: normal playback
    if (!frame_valid) {
        if (rs->conceal_phase <= 0) {
            rs->conceal_phase = 1;          // start fade-out
        } else {
            rs->conceal_phase++;            // continue fade-out
        }
        if (rs->conceal_phase >= CONCEAL_FADE_FRAMES) {
            rs->amplitude = 0.0f;
        } else {
            rs->amplitude = 1.0f - (float)rs->conceal_phase / (float)CONCEAL_FADE_FRAMES;
        }
        g_dropout_frames += 1;
    } else if (rs->conceal_phase > 0) {
        // Exiting concealment — start fade-in from current position.
        // Use the same number of frames for fade-in as were used for fade-out
        // (capped at CONCEAL_FADE_FRAMES).
        rs->conceal_phase = -rs->conceal_phase;   // negative = fading in
    } else if (rs->conceal_phase < 0) {
        rs->conceal_phase++;                      // count toward zero
        if (rs->conceal_phase >= 0) {
            rs->conceal_phase = 0;
            rs->amplitude     = 1.0f;
        } else {
            float t = (float)(-rs->conceal_phase) / (float)CONCEAL_FADE_FRAMES;
            if (t > 1.0f) t = 1.0f;
            rs->amplitude = 1.0f - t;             // ramp from 0 → 1
        }
    } else {
        rs->amplitude = 1.0f;
    }

    // Apply crossfade amplitude.
    int32_t l = (int32_t)(*out_l) * (int32_t)(rs->amplitude * 65536.0f + 0.5f) >> 16;
    int32_t r = (int32_t)(*out_r) * (int32_t)(rs->amplitude * 65536.0f + 0.5f) >> 16;
    if (l > 32767) l = 32767; if (l < -32768) l = -32768;
    if (r > 32767) r = 32767; if (r < -32768) r = -32768;
    *out_l = (int16_t)l;
    *out_r = (int16_t)r;

    return frame_valid;
}
