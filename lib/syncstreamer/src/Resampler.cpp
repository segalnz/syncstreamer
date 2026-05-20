#include "Resampler.h"
#include "JitterBuffer.h"
#include <string.h>

// g_rate_ppm defined in SyncController.cpp
extern volatile float g_rate_ppm;

resampler_t g_resampler;

void resampler_init(resampler_t* rs)
{
    rs->phase   = 0.0f;
    rs->prev[0] = 0;
    rs->prev[1] = 0;
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
            jb_advance();
        } else {
            // Gap concealment: hold prev, don't advance read head.
            valid = false;
        }
    }

    // Linear interpolation between prev and the next frame in the JB.
    jb_frame_t* next = jb_peek();
    if (next && valid) {
        float alpha  = rs->phase;  // 0..1: position between prev and next
        *out_l = (int16_t)(rs->prev[0] + alpha * (next->pcm[0] - rs->prev[0]));
        *out_r = (int16_t)(rs->prev[1] + alpha * (next->pcm[1] - rs->prev[1]));
    } else {
        // No next frame available yet — output prev (zero-order hold).
        *out_l = rs->prev[0];
        *out_r = rs->prev[1];
        valid  = (next != nullptr) ? valid : false;
    }

    return valid;
}
