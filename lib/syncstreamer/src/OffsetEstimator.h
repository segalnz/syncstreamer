#pragma once
#include <stdint.h>

// ============================================================
// OffsetEstimator — server clock offset tracker
//
// Maintains a 32-entry rolling window of observed one-way
// latency samples, keyed on minimum latency (the "best" path
// measurement). g_offset_us is the inferred difference between
// server time and esp_timer_get_time().
//
// Call offset_update() for every received audio packet.
// Call server_now_us() anywhere to get estimated server time.
// ============================================================

#define OFFSET_RING_SIZE 32u
#define OFFSET_FUDGE_US  200000LL   // 200 ms playback lead time

// Initialise internal state. Must be called before any other function.
void offset_estimator_init(void);

// Update offset estimate with a new measurement from a received packet.
// present_us = SyncPacket.present_us (server timestamp of packet frame 0).
void offset_update(uint64_t present_us);

// Returns estimated current server time in microseconds.
int64_t server_now_us(void);

// Raw offset (server_us - local_us). Exposed for status display.
extern volatile int64_t g_offset_us;
