#include "JitterBuffer.h"
#include <string.h>
#include <Arduino.h>

// ── Globals ───────────────────────────────────────────────────────────────────
static jb_frame_t*         s_ring      = nullptr;
static SemaphoreHandle_t   s_mutex     = nullptr;
static volatile uint32_t   s_read_head = 0;   // next frame_seq to consume (written by wifi_rx on Core 0, read by audio_out on Core 1)
static volatile uint32_t   s_write_head = 0;  // one past highest frame_seq written (written by wifi_rx on Core 0, read by audio_out on Core 1)

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
        heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    if (!s_ring) {
        log_e("JitterBuffer: alloc failed (%u KB)", (unsigned)(bytes / 1024));
        return false;
    }

    memset(s_ring, 0, bytes);
    log_i("JitterBuffer: %u frames, %u KB internal RAM", JB_FRAMES, (unsigned)(bytes / 1024));
    return true;
}

// ── Write ─────────────────────────────────────────────────────────────────────
void jb_write(uint32_t frame_seq, const int16_t pcm[2])
{
    uint32_t idx = frame_seq & JB_MASK;

    // ── Sliding-window guard ─────────────────────────────────────────
    // If the new frame's absolute sequence number has lapped the read
    // head by more than a full ring, advance the read head forward so
    // the write lands on a stale slot instead of live playback data.
    // This prevents wrap-corruption during sustained streams.
    uint32_t dist = frame_seq - s_read_head;
    if (dist >= JB_FRAMES) {
        s_read_head = frame_seq - (JB_FRAMES - 1);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Store PCM first, then release-barrier so valid=true is visible
    // after PCM writes on the other core.
    s_ring[idx].pcm[0] = pcm[0];
    s_ring[idx].pcm[1] = pcm[1];
    __sync_synchronize();
    s_ring[idx].valid  = true;

    // Write head is always one past the highest frame_seq written.
    s_write_head = frame_seq + 1u;

    s_last_write_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

void jb_write_packet(uint32_t base_frame, const int16_t pcm[512])
{
    // Sliding-window guard: check base frame against read head.
    uint32_t dist = base_frame - s_read_head;
    if (dist >= JB_FRAMES) {
        s_read_head = base_frame - (JB_FRAMES - 1);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    for (uint32_t i = 0; i < 256; i++) {
        uint32_t seq  = base_frame + i;
        uint32_t idx  = seq & JB_MASK;
        s_ring[idx].pcm[0] = pcm[i * 2];
        s_ring[idx].pcm[1] = pcm[i * 2 + 1];
        __sync_synchronize();
        s_ring[idx].valid  = true;

        // Write head is always one past the highest frame_seq written.
        s_write_head = seq + 1u;
    }

    s_last_write_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

// ── Peek ──────────────────────────────────────────────────────────────────────
jb_frame_t* jb_peek(void)
{
    uint32_t idx = s_read_head & JB_MASK;

    // Acquire barrier: valid read must happen-before PCM read.
    if (!s_ring[idx].valid) return nullptr;
    __sync_synchronize();

    // Re-check valid after acquiring — if the writer wrote PCM + set
    // valid on this slot between our check and the barrier, we might
    // see stale PCM.  Double-check catches that case.
    if (!s_ring[idx].valid) return nullptr;

    return &s_ring[idx];
}

// ── Advance ───────────────────────────────────────────────────────────────────
void jb_advance(void)
{
    uint32_t idx = s_read_head & JB_MASK;
    s_ring[idx].valid = false;
    __sync_synchronize();
    s_read_head = s_read_head + 1u;
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
// ── Sync to write head ────────────────────────────────────────────────────────
void jb_sync_read_head(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (uint32_t i = 0; i < JB_FRAMES; i++) {
        s_ring[i].valid = false;
    }
    s_read_head     = s_write_head;   // start empty from current stream position
    s_last_write_us = 0;              // reset stall timer
    xSemaphoreGive(s_mutex);
    log_i("JitterBuffer: synced to stream position (ready for fill)");
}
// ── Stall detection ───────────────────────────────────────────────────────────
bool jb_stalled(void)
{
    if (s_last_write_us == 0) return false;
    return (esp_timer_get_time() - s_last_write_us) > JB_STALL_US;
}
