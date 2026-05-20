#include "JitterBuffer.h"
#include <string.h>
#include <Arduino.h>

// ── Globals ───────────────────────────────────────────────────────────────────
static jb_frame_t*       s_ring      = nullptr;
static SemaphoreHandle_t s_mutex     = nullptr;
static uint32_t          s_read_head = 0;   // next frame_seq to consume
static uint32_t          s_write_head = 0;  // one past highest frame_seq written

static int64_t           s_last_write_us = 0;

volatile uint32_t g_seq_gaps = 0;

// ── Init ──────────────────────────────────────────────────────────────────────
bool jb_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        log_e("JitterBuffer: mutex alloc failed");
        return false;
    }

    size_t bytes = JB_FRAMES * sizeof(jb_frame_t);
    s_ring = static_cast<jb_frame_t*>(
        heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!s_ring) {
        log_e("JitterBuffer: PSRAM alloc failed (%u KB)", (unsigned)(bytes / 1024));
        return false;
    }

    memset(s_ring, 0, bytes);
    log_i("JitterBuffer: %u frames, %u KB PSRAM", JB_FRAMES, (unsigned)(bytes / 1024));
    return true;
}

// ── Write ─────────────────────────────────────────────────────────────────────
void jb_write(uint32_t frame_seq, const int16_t pcm[2])
{
    uint32_t idx = frame_seq & JB_MASK;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    s_ring[idx].pcm[0] = pcm[0];
    s_ring[idx].pcm[1] = pcm[1];
    s_ring[idx].valid  = true;

    // Advance write head to one past the highest seq written.
    uint32_t next = frame_seq + 1u;
    if ((int32_t)(next - s_write_head) > 0) {
        s_write_head = next;
    }

    s_last_write_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

// ── Peek ──────────────────────────────────────────────────────────────────────
jb_frame_t* jb_peek(void)
{
    // Called from audio_out_task; no mutex (read_head only moves in same task).
    uint32_t idx = s_read_head & JB_MASK;
    if (!s_ring[idx].valid) return nullptr;
    return &s_ring[idx];
}

// ── Advance ───────────────────────────────────────────────────────────────────
void jb_advance(void)
{
    uint32_t idx = s_read_head & JB_MASK;
    s_ring[idx].valid = false;
    s_read_head++;
}

// ── Occupancy ─────────────────────────────────────────────────────────────────
uint32_t jb_occupancy_frames(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t occ = s_write_head - s_read_head;
    xSemaphoreGive(s_mutex);
    if (occ > JB_FRAMES) occ = JB_FRAMES;
    return occ;
}

uint32_t jb_occupancy_ms(void)
{
    return (jb_occupancy_frames() * 1000u) / JB_SAMPLE_RATE;
}

// ── Flush ─────────────────────────────────────────────────────────────────────
void jb_flush(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (uint32_t i = 0; i < JB_FRAMES; i++) {
        s_ring[i].valid = false;
    }
    s_read_head  = 0;
    s_write_head = 0;
    s_last_write_us = 0;
    xSemaphoreGive(s_mutex);
    log_i("JitterBuffer: flushed");
}

// ── Stall detection ───────────────────────────────────────────────────────────
bool jb_stalled(void)
{
    if (s_last_write_us == 0) return false;
    return (esp_timer_get_time() - s_last_write_us) > JB_STALL_US;
}
