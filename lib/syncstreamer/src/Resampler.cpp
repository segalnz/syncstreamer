#include "Resampler.h"
#include "JitterBuffer.h"
#include <string.h>
#include <math.h>

// g_rate_ppm defined in SyncController.cpp
extern volatile float g_rate_ppm;

#define CONCEAL_FADE_FRAMES 240   // ~5ms at 48 kHz — smooth fade in/out

resampler_t g_resampler;

void resampler_init(resampler_t* rs)
{
    rs->phase      = 0.0f;
    rs->prev[0]    = 0;
    rs->prev[1]    = 0;
    rs->amplitude  = 1.0f;
}

bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r)
{
    static uint32_t s_conceal_count = 0;   // consecutive concealed frames
    static bool     s_concealing    = false;

    // Advance phase by one output frame worth.
    rs->phase += 1.0f + g_rate_ppm * 1e-6f;

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

    // ── Crossfade gap concealment ─────────────────────────────────
    if (!frame_valid) {
        // Entering or continuing concealment.
        if (!s_concealing) {
            s_concealing    = true;
            s_conceal_count = 0;
        }
        s_conceal_count++;
        // Fade out: linear ramp from current amplitude toward 0 over CONCEAL_FADE_FRAMES.
        if (s_conceal_count < CONCEAL_FADE_FRAMES) {
            float fade = 1.0f - (float)s_conceal_count / (float)CONCEAL_FADE_FRAMES;
            rs->amplitude = rs->amplitude * 0.9f + fade * 0.1f;  // smoothing
            if (rs->amplitude < 0.0f) rs->amplitude = 0.0f;
        } else {
            rs->amplitude = 0.0f;
        }
    } else if (s_concealing) {
        // Exiting concealment: fade back in.
        if (s_conceal_count > 0) {
            float fade_in = 1.0f - (float)s_conceal_count / (float)CONCEAL_FADE_FRAMES;
            rs->amplitude = rs->amplitude * 0.9f + (1.0f - fade_in) * 0.1f;
            if (rs->amplitude > 1.0f) rs->amplitude = 1.0f;
            s_conceal_count--;
        } else {
            rs->amplitude = 1.0f;
            s_concealing  = false;
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
