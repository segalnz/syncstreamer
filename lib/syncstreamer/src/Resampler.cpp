#include "Resampler.h"
#include "JitterBuffer.h"
#include <string.h>

// g_rate_ppm defined in SyncController.cpp
extern volatile float g_rate_ppm;

// ── Crossfade constants ─────────────────────────────────────────────────────────
// 10 ms window at 48 kHz = 480 frames.
// A raised-cosine (Hann) envelope over this window gives a smooth amplitude
// transition with zero derivative at both ends — no audible corner clicks.
//
// For gaps shorter than 480 frames the cosine never reaches 0 before valid
// data returns, so the fade is naturally proportional to the gap duration.
// For gaps >= 480 frames the amplitude reaches 0 and stays there.
#define CONCEAL_FADE_FRAMES     480u    // ~10 ms at 48 kHz
#define CONCEAL_LUT_ENTRIES     256u    // LUT size for raised-cosine

// Precomputed raised-cosine LUT:  lut[i] = 0.5 * (1 + cos(pi * i / ENTRIES))
// for i = 0..ENTRIES-1.  Maps fade progress [0,1) → amplitude [1,0).
static const uint16_t s_cos_lut[CONCEAL_LUT_ENTRIES] = {
    65535, 65533, 65525, 65513, 65496, 65473, 65446, 65414,
    65377, 65335, 65289, 65237, 65180, 65119, 65053, 64981,
    64905, 64825, 64739, 64648, 64553, 64453, 64348, 64238,
    64124, 64005, 63881, 63753, 63620, 63482, 63339, 63192,
    63041, 62885, 62724, 62559, 62389, 62215, 62036, 61853,
    61666, 61474, 61278, 61078, 60873, 60664, 60451, 60234,
    60013, 59787, 59558, 59324, 59087, 58845, 58600, 58350,
    58097, 57840, 57579, 57315, 57047, 56775, 56499, 56220,
    55938, 55652, 55362, 55069, 54773, 54473, 54170, 53864,
    53555, 53243, 52927, 52609, 52287, 51963, 51635, 51305,
    50972, 50636, 50298, 49957, 49613, 49267, 48919, 48567,
    48214, 47858, 47500, 47140, 46777, 46413, 46046, 45678,
    45307, 44935, 44560, 44184, 43807, 43427, 43046, 42663,
    42279, 41894, 41507, 41119, 40729, 40339, 39947, 39554,
    39160, 38765, 38369, 37973, 37575, 37177, 36779, 36379,
    35979, 35579, 35178, 34777, 34375, 33974, 33572, 33170,
    32768, 32365, 31963, 31561, 31160, 30758, 30357, 29956,
    29556, 29156, 28756, 28358, 27960, 27562, 27166, 26770,
    26375, 25981, 25588, 25196, 24806, 24416, 24028, 23641,
    23256, 22872, 22489, 22108, 21728, 21351, 20975, 20600,
    20228, 19857, 19489, 19122, 18758, 18395, 18035, 17677,
    17321, 16968, 16616, 16268, 15922, 15578, 15237, 14899,
    14563, 14230, 13900, 13572, 13248, 12926, 12608, 12292,
    11980, 11671, 11365, 11062, 10762, 10466, 10173,  9883,
     9597,  9315,  9036,  8760,  8488,  8220,  7956,  7695,
     7438,  7185,  6935,  6690,  6448,  6211,  5977,  5748,
     5522,  5301,  5084,  4871,  4662,  4457,  4257,  4061,
     3869,  3682,  3499,  3320,  3146,  2976,  2811,  2650,
     2494,  2343,  2196,  2053,  1915,  1782,  1654,  1530,
     1411,  1297,  1187,  1082,   982,   887,   796,   710,
      630,   554,   482,   416,   355,   298,   246,   200,
      158,   121,    89,    62,    39,    22,    10,     2,
};

volatile uint32_t g_dropout_frames = 0;
resampler_t g_resampler;

// Concealment state (must be reset by resampler_init on stream restart).
static bool     s_concealing  = false;
static int32_t  s_fade_count  = 0;   // progress through current fade (0..CONCEAL_FADE_FRAMES)
static int32_t  s_fade_total  = 0;   // total fade length for this event (mirrors rs->fade_len)

void resampler_init(resampler_t* rs)
{
    rs->phase      = 0.0f;
    rs->prev[0]    = 0;
    rs->prev[1]    = 0;
    rs->amplitude  = 1.0f;
    rs->fade_len   = 0;
    s_concealing   = false;
    s_fade_count   = 0;
    s_fade_total   = 0;
    g_dropout_frames = 0;
}

bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r)
{

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

    // ── Raised-cosine gap concealment ─────────────────────────────
    // Uses a fixed 10 ms window (480 frames).  The cosine envelope has
    // zero derivative at both ends, eliminating the audible "corner" of
    // a linear ramp.  For gaps shorter than 480 frames the amplitude
    // never reaches 0 — the fade is naturally proportional to the gap.
    if (!frame_valid) {
        if (!s_concealing) {
            s_concealing  = true;
            s_fade_count  = 0;
            s_fade_total  = (int32_t)CONCEAL_FADE_FRAMES;
            rs->fade_len  = (int32_t)CONCEAL_FADE_FRAMES;
        }
        g_dropout_frames += 1;
        s_fade_count++;
        if (s_fade_count < s_fade_total) {
            uint32_t idx = (uint32_t)s_fade_count * CONCEAL_LUT_ENTRIES / (uint32_t)s_fade_total;
            if (idx >= CONCEAL_LUT_ENTRIES) idx = CONCEAL_LUT_ENTRIES - 1;
            rs->amplitude = (float)s_cos_lut[idx] / 65535.0f;
        } else {
            rs->amplitude = 0.0f;
        }
    } else if (s_concealing) {
        // Exiting concealment: fade back in symmetrically over s_fade_total frames.
        if (s_fade_count > 0) {
            s_fade_count--;
            uint32_t idx = (uint32_t)s_fade_count * CONCEAL_LUT_ENTRIES / (uint32_t)s_fade_total;
            if (idx >= CONCEAL_LUT_ENTRIES) idx = CONCEAL_LUT_ENTRIES - 1;
            rs->amplitude = (float)s_cos_lut[idx] / 65535.0f;
        } else {
            rs->amplitude  = 1.0f;
            rs->fade_len   = 0;
            s_concealing   = false;
        }
    } else {
        rs->amplitude = 1.0f;
    }

    // Apply crossfade amplitude via Q16 fixed-point multiply.
    int32_t l = (int32_t)(*out_l) * (int32_t)(rs->amplitude * 65536.0f + 0.5f) >> 16;
    int32_t r = (int32_t)(*out_r) * (int32_t)(rs->amplitude * 65536.0f + 0.5f) >> 16;
    if (l > 32767) l = 32767; if (l < -32768) l = -32768;
    if (r > 32767) r = 32767; if (r < -32768) r = -32768;
    *out_l = (int16_t)l;
    *out_r = (int16_t)r;

    return frame_valid;
}
