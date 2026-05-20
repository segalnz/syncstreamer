#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "SyncPacket.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

// ============================================================
// JitterBuffer — thread-safe ring buffer of PCM audio blocks
//
// Capacity: JITTER_BUFFER_CAPACITY blocks.
// At 480 samples/block (10 ms @ 48 kHz) that gives ~1280 ms.
//
// PCM payload memory is pre-allocated in PSRAM at init().
// Metadata lives in internal RAM.
//
// Indexed by (sequence % JITTER_BUFFER_CAPACITY): O(1) push/pop.
// Late and duplicate packets are silently dropped.
//
// Call init() before use.
// ============================================================

static constexpr size_t JITTER_BUFFER_CAPACITY  = 256;   // blocks (~1280 ms)
static constexpr size_t JITTER_MAX_SAMPLES       = SYNC_PACKET_MAX_SAMPLES;
// bytes per slot: stereo 16-bit worst case
static constexpr size_t JITTER_PCM_SLOT_BYTES    = JITTER_MAX_SAMPLES * 2 * sizeof(int16_t);

struct PCMBlock {
    uint32_t stream_id;
    uint32_t sequence;
    int64_t  presentation_time_us;
    uint16_t sample_count;          // stereo pairs (or mono samples if MONO flag)
    uint16_t flags;
    int16_t* pcm_data;              // points into PSRAM slab; owned by JitterBuffer
    bool     occupied;
};

class JitterBuffer {
public:
    // Allocate PSRAM slab and init mutex. Returns false on alloc failure.
    bool init();

    // Copy a received packet into the buffer.
    // Returns false if the packet is late, duplicate, or the slot is still
    // occupied (overflow — caller should log and discard).
    bool push(const SyncPacketHeader& hdr, const int16_t* pcm, uint32_t pcmWordCount);

    // Retrieve the oldest pending block (lowest sequence not yet popped).
    // Fills *out and returns true if a block is ready; false if empty.
    // Caller MUST call release() when done with the PCM data.
    bool pop(PCMBlock& out);

    // Return the slot occupied by *block back to the pool after pop().
    void release(const PCMBlock& block);

    // Discard all blocks for a stream whose stream_id differs from current.
    // Call when a new stream begins.
    void flush(uint32_t new_stream_id);

    // Buffer fill expressed as microseconds of audio queued.
    int64_t getFillLevelMicros() const;

    // Peek at the presentation_time_us of the next block to be popped
    // without removing it. Returns false if the buffer is empty.
    bool peekNextPresentationTime(int64_t& out_time_us) const;

    // ── Stats (reset on flush) ────────────────────────────────
    uint32_t statPushOk;
    uint32_t statDropLate;
    uint32_t statDropDuplicate;
    uint32_t statDropOverflow;
    uint32_t statPopOk;
    uint32_t statUnderrun;

private:
    PCMBlock         _slots[JITTER_BUFFER_CAPACITY];
    int16_t*         _psramSlab  = nullptr;   // contiguous PSRAM allocation
    SemaphoreHandle_t _mutex      = nullptr;

    uint32_t _nextPopSeq   = 0;   // next sequence number expected by pop()
    uint32_t _currentStream = 0;
    bool     _streamActive  = false;

    void _resetStats();
};
