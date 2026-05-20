#include "OffsetEstimator.h"
#include <Arduino.h>
#include "esp_timer.h"
#include <limits.h>

static int64_t          s_ring[OFFSET_RING_SIZE];
static uint32_t         s_idx       = 0;
static bool             s_filled    = false;

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
}

int64_t server_now_us(void)
{
    return esp_timer_get_time() + g_offset_us;
}
