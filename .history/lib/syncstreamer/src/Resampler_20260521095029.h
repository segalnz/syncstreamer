#pragma once
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// Resampler — fractional-rate linear interpolation
//
// Consumes frames from the JitterBuffer at a rate of
//   (1.0 + g_rate_ppm * 1e-6) per output frame.
//
// When phase crosses 1.0 the read head advances one JB frame.
// Interpolates linearly between prev and next.
//
// Gap concealment: if next frame is invalid, repeats prev.
// ============================================================

struct resampler_t {
    float   phase;       // 0.0 – <1.0  (fractional position between prev and next)
    int16_t prev[2];     // last consumed frame [L, R]
};

// Initialise / reset the resampler (zeroes phase and prev).
void resampler_init(resampler_t* rs);

// Produce one output stereo frame.
// Reads g_rate_ppm from SyncController to advance phase.
// Returns true if audio is valid; false if gap-concealing.
bool resampler_get_frame(resampler_t* rs, int16_t* out_l, int16_t* out_r);

// Shared instance — defined in Resampler.cpp, used by AudioOutput.cpp.
extern resampler_t g_resampler;
