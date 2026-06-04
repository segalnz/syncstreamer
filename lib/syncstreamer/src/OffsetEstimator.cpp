#include "OffsetEstimator.h"
#include <Arduino.h>
#include "esp_timer.h"
#include <limits.h>

static int64_t          s_ring[OFFSET_RING_SIZE];
static uint32_t         s_idx       = 0;
static bool             s_filled    = false;
static int64_t          s_baseline  = 0;

volatile int64_t g_offset_us = 0;

void offset_estimator_init(void)
{
    for (uint32_t i = 0; i < OFFSET_RING_SIZE; i++) {
        s_ring[i] = INT64_MAX;
    }
    s_idx    = 0;
    s_filled = false;
    g_offset_us = 0;
}

void offset_update(uint64_t present_us)
{
    // Sample = (server timestamp) - (local time) - playback lead
    // Minimum of these samples ≈ best-case offset (lowest network jitter).
    int64_t sample = (int64_t)present_us - esp_timer_get_time() - OFFSET_FUDGE_US;

    s_ring[s_idx & (OFFSET_RING_SIZE - 1)] = sample;
    s_idx++;
    if (s_idx >= OFFSET_RING_SIZE) s_filled = true;

    uint32_t count = s_filled ? OFFSET_RING_SIZE : s_idx;
    int64_t  best  = INT64_MAX;
    for (uint32_t i = 0; i < count; i++) {
        if (s_ring[i] < best) best = s_ring[i];
    }
    g_offset_us = best;

    // Capture baseline once the ring is fully seeded.
    if (s_baseline == 0 && s_filled) {
        s_baseline = g_offset_us;
    }
}

void offset_estimator_reset(void)
{
    for (uint32_t i = 0; i < OFFSET_RING_SIZE; i++) {
        s_ring[i] = INT64_MAX;
    }
    s_idx     = 0;
    s_filled  = false;
    s_baseline = 0;
    g_offset_us = 0;
}

int64_t offset_drift_us(void)
{
    if (s_baseline == 0) return 0;
    return g_offset_us - s_baseline;
}

int64_t server_now_us(void)
{
    return esp_timer_get_time() + g_offset_us;
}
