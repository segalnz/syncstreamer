#include "JitterBuffer.h"
#include <Arduino.h>
#include <string.h>

// ── Init ──────────────────────────────────────────────────────────────────────
bool JitterBuffer::init()
{
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        log_e("JitterBuffer: failed to create mutex");
        return false;
    }

    // Pre-allocate contiguous PSRAM slab: one PCM slot per ring entry.
    size_t slabBytes = JITTER_BUFFER_CAPACITY * JITTER_PCM_SLOT_BYTES;
    _psramSlab = static_cast<int16_t*>(
        heap_caps_malloc(slabBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!_psramSlab) {
        log_e("JitterBuffer: PSRAM alloc failed (%u bytes)", slabBytes);
        return false;
    }

    // Wire each slot's pcm_data pointer into the slab.
    for (size_t i = 0; i < JITTER_BUFFER_CAPACITY; i++) {
        _slots[i].pcm_data  = _psramSlab + i * JITTER_MAX_SAMPLES * 2;
        _slots[i].occupied  = false;
    }

    _resetStats();
    log_i("JitterBuffer: %u slots, %u KB PSRAM", JITTER_BUFFER_CAPACITY, slabBytes / 1024);
    return true;
}

// ── Push ─────────────────────────────────────────────────────────────────────
bool JitterBuffer::push(const SyncPacketHeader& hdr,
                         const int16_t* pcm, uint32_t pcmWordCount)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    // First packet: establish stream context.
    if (!_streamActive) {
        _currentStream = hdr.stream_id;
        _nextPopSeq    = hdr.sequence;
        _streamActive  = true;
    }

    // Stream-id change: flush and restart.
    if (hdr.stream_id != _currentStream) {
        xSemaphoreGive(_mutex);
        flush(hdr.stream_id);
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }

    // Drop if older than next expected pop (late packet).
    if ((int32_t)(hdr.sequence - _nextPopSeq) < 0) {
        statDropLate++;
        xSemaphoreGive(_mutex);
        return false;
    }

    size_t idx = hdr.sequence % JITTER_BUFFER_CAPACITY;

    // Slot already occupied — either duplicate or we're overflowing the window.
    if (_slots[idx].occupied) {
        if (_slots[idx].sequence == hdr.sequence) {
            statDropDuplicate++;
        } else {
            statDropOverflow++;
        }
        xSemaphoreGive(_mutex);
        return false;
    }

    // Copy PCM into pre-allocated PSRAM slot.
    uint32_t copyWords = (pcmWordCount < JITTER_MAX_SAMPLES * 2u)
                         ? pcmWordCount : JITTER_MAX_SAMPLES * 2u;
    memcpy(_slots[idx].pcm_data, pcm, copyWords * sizeof(int16_t));

    _slots[idx].stream_id            = hdr.stream_id;
    _slots[idx].sequence             = hdr.sequence;
    _slots[idx].presentation_time_us = hdr.presentation_time_us;
    _slots[idx].sample_count         = hdr.sample_count;
    _slots[idx].flags                = hdr.flags;
    _slots[idx].occupied             = true;

    statPushOk++;
    xSemaphoreGive(_mutex);
    return true;
}

// ── Pop ───────────────────────────────────────────────────────────────────────
bool JitterBuffer::pop(PCMBlock& out)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    size_t idx = _nextPopSeq % JITTER_BUFFER_CAPACITY;

    if (!_streamActive || !_slots[idx].occupied) {
        statUnderrun++;
        xSemaphoreGive(_mutex);
        return false;
    }

    out = _slots[idx];   // shallow copy — pcm_data still points into slab
    // Slot stays occupied until release() is called.
    _nextPopSeq++;
    statPopOk++;

    xSemaphoreGive(_mutex);
    return true;
}

// ── Release ───────────────────────────────────────────────────────────────────
void JitterBuffer::release(const PCMBlock& block)
{
    size_t idx = block.sequence % JITTER_BUFFER_CAPACITY;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _slots[idx].occupied = false;
    xSemaphoreGive(_mutex);
}

// ── Flush ─────────────────────────────────────────────────────────────────────
void JitterBuffer::flush(uint32_t new_stream_id)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    for (size_t i = 0; i < JITTER_BUFFER_CAPACITY; i++) {
        _slots[i].occupied = false;
    }
    _currentStream = new_stream_id;
    _nextPopSeq    = 0;
    _streamActive  = false;
    _resetStats();

    log_i("JitterBuffer: flushed, new stream_id=%u", new_stream_id);
    xSemaphoreGive(_mutex);
}

// ── Fill level ────────────────────────────────────────────────────────────────
int64_t JitterBuffer::getFillLevelMicros() const
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    uint32_t count = 0;
    for (size_t i = 0; i < JITTER_BUFFER_CAPACITY; i++) {
        if (_slots[i].occupied) count++;
    }

    xSemaphoreGive(_mutex);

    // Nominal block duration derived from SYNC_PACKET_MAX_SAMPLES (240 @ 48 kHz = 5 000 µs)
    return (int64_t)count * SYNC_PACKET_BLOCK_DURATION_US;
}

// ── Peek ──────────────────────────────────────────────────────────────────────
bool JitterBuffer::peekNextPresentationTime(int64_t& out_time_us) const
{
    xSemaphoreTake(_mutex, portMAX_DELAY);

    size_t idx = _nextPopSeq % JITTER_BUFFER_CAPACITY;
    bool ok = _streamActive && _slots[idx].occupied;
    if (ok) out_time_us = _slots[idx].presentation_time_us;

    xSemaphoreGive(_mutex);
    return ok;
}

// ── Private ───────────────────────────────────────────────────────────────────
void JitterBuffer::_resetStats()
{
    statPushOk        = 0;
    statDropLate      = 0;
    statDropDuplicate = 0;
    statDropOverflow  = 0;
    statPopOk         = 0;
    statUnderrun      = 0;
}
