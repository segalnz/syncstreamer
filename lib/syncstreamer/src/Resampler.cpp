#include "Resampler.h"
#include "JitterBuffer.h"
#include <string.h>
#include <math.h>

// g_rate_ppm defined in SyncController.cpp
extern volatile float g_rate_ppm;

#define CONCEAL_FADE_FRAMES 240   // ~5 ms at 48 kHz — Hann window fade in/out

// Precomputed 256-entry raised-cosine LUT for gap concealment.
// Entry i gives the amplitude for t = i / 255, where t goes 0.0 → 1.0.
// amplitude = 0.5f * (1.0f + cosf(M_PI * t))
static const uint16_t s_cos_lut[256] = {
    65535, 65533, 65525, 65513, 65495, 65473, 65446, 65413,
    65376, 65334, 65287, 65235, 65178, 65116, 65049, 64977,
    64900, 64819, 64733, 64641, 64545, 64444, 64339, 64228,
    64113, 63993, 63868, 63739, 63605, 63466, 63322, 63174,
    63021, 62864, 62702, 62536, 62365, 62189, 62009, 61825,
    61636, 61443, 61245, 61044, 60837, 60627, 60412, 60194,
    59971, 59743, 59512, 59277, 59038, 58794, 58547, 58296,
    58041, 57782, 57520, 57253, 56983, 56709, 56432, 56151,
    55866, 55578, 55287, 54992, 54693, 54392, 54087, 53778,
    53467, 53153, 52835, 52514, 52191, 51864, 51535, 51202,
    50867, 50529, 50189, 49845, 49500, 49151, 48800, 48447,
    48091, 47733, 47373, 47011, 46646, 46279, 45911, 45540,
    45167, 44792, 44416, 44038, 43658, 43276, 42893, 42509,
    42122, 41735, 41346, 40956, 40564, 40171, 39778, 39383,
    38987, 38590, 38192, 37794, 37394, 36994, 36594, 36193,
    35791, 35389, 34986, 34583, 34180, 33777, 33373, 32969,
    32566, 32162, 31758, 31355, 30952, 30549, 30146, 29744,
    29342, 28941, 28541, 28141, 27741, 27343, 26945, 26548,
    26152, 25757, 25364, 24971, 24579, 24189, 23800, 23413,
    23026, 22642, 22259, 21877, 21497, 21119, 20743, 20368,
    19995, 19624, 19256, 18889, 18524, 18162, 17802, 17444,
    17088, 16735, 16384, 16035, 15690, 15346, 15006, 14668,
    14333, 14000, 13671, 13344, 13021, 12700, 12382, 12068,
    11757, 11448, 11143, 10842, 10543, 10248,  9957,  9669,
     9384,  9103,  8826,  8552,  8282,  8015,  7753,  7494,
     7239,  6988,  6741,  6497,  6258,  6023,  5792,  5564,
     5341,  5123,  4908,  4698,  4491,  4290,  4092,  3899,
     3710,  3526,  3346,  3170,  2999,  2833,  2671,  2514,
     2361,  2213,  2069,  1930,  1796,  1667,  1542,  1422,
     1307,  1196,  1091,   990,   894,   802,   716,   635,
      558,   486,   419,   357,   300,   248,   201,   159,
      122,    89,    62,    40,    22,    10,     2,     0
};

volatile uint32_t g_dropout_frames = 0;
resampler_t g_resampler;

void resampler_init(resampler_t* rs)
{
    rs->phase             = 0.0f;
    rs->prev[0]           = 0;
    rs->prev[1]           = 0;
    rs->amplitude         = 1.0f;
    rs->conceal_phase     = 0;
    g_dropout_frames      = 0;
}

bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r)
{
    // Copy volatile g_rate_ppm once to avoid torn reads from sync_task preemption.
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

    // ── Raised-cosine (Hann window) gap concealment ──────────────
    // Single counter: conceal_phase
    //   = 0          → normal playback
    //   > 0 & rising → fading out (frame loss)
    //   > 0 & falling → fading in  (valid frames returning)
    //   >= 2*CONCEAL_FADE_FRAMES → forced reset (safety cap)
    //
    // t = conceal_phase / CONCEAL_FADE_FRAMES, clamped to [0, 1]
    // amplitude = Hann(t)  (1 at t=0, 0 at t=1)
    if (!frame_valid) {
        if (rs->conceal_phase <= 0) {
            rs->conceal_phase = 1;          // start fade-out
        } else {
            rs->conceal_phase++;            // continue fade-out
        }
        if (rs->conceal_phase >= 2 * CONCEAL_FADE_FRAMES) {
            rs->amplitude = 0.0f;
        } else {
            uint32_t t_idx = (rs->conceal_phase * 255) / CONCEAL_FADE_FRAMES;
            if (t_idx > 255) t_idx = 255;
            rs->amplitude = (float)s_cos_lut[t_idx] / 65535.0f;
        }
        g_dropout_frames += 1;
    } else if (rs->conceal_phase > 0) {
        // Fading back in by counting conceal_phase back toward 0.
        // The peak phase reached during the loss determines the max fade depth.
        rs->conceal_phase--;
        if (rs->conceal_phase <= 0) {
            rs->conceal_phase = 0;
            rs->amplitude     = 1.0f;
        } else {
            uint32_t t_idx = (rs->conceal_phase * 255) / CONCEAL_FADE_FRAMES;
            if (t_idx > 255) t_idx = 255;
            rs->amplitude = (float)s_cos_lut[t_idx] / 65535.0f;
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
