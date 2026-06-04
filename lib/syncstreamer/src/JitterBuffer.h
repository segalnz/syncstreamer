#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

// ============================================================
// JitterBuffer v2 — sequence-indexed elastic audio buffer in PSRAM
//
// Ring of JB_FRAMES stereo frames indexed by (frame_seq & (JB_FRAMES-1)).
// frame_seq = packet_seq * SYNC_FRAMES_PER_PACKET + frame_offset
//
// Capacity: 4096 frames = ~85 ms @ 48 kHz  (must exceed JB_STARTUP_FRAMES)
// Target fill: JB_TARGET_MS = 30 ms
// Startup fill: JB_STARTUP_MS = 60 ms
// ============================================================

#define JB_FRAMES        4096u          // must be power of 2; > JB_STARTUP_FRAMES (2880)
#define JB_MASK          (JB_FRAMES - 1)
#define JB_SAMPLE_RATE   48000u
#define JB_TARGET_MS     30u
#define JB_STARTUP_MS    60u
#define JB_TARGET_FRAMES ((JB_SAMPLE_RATE * JB_TARGET_MS)  / 1000u)   // 1440
#define JB_STARTUP_FRAMES ((JB_SAMPLE_RATE * JB_STARTUP_MS) / 1000u)  // 2880
#define JB_STALL_US      500000LL       // 500 ms without a new frame = stalled

struct jb_frame_t {
    volatile int16_t pcm[2];   // [0]=L, [1]=R — volatile for cross-core access
    bool    valid;
};

// Must be called once before any other jb_ function.
// Allocates ring in PSRAM. Returns false on failure.
bool jb_init(void);

// Write one stereo frame at the given absolute frame sequence number.
void jb_write(uint32_t frame_seq, const int16_t pcm[2]);

// Write all 256 frames from a SyncPacket at once. Takes mutex once.
void jb_write_packet(uint32_t base_frame, const int16_t pcm[512]);

// Peek at the frame at read_head without advancing. Returns pointer into ring
// (valid until next jb_write to the same slot, i.e. after 4096 frames).
// Returns nullptr if the slot is not valid.
jb_frame_t* jb_peek(void);

// Advance read head by one frame (call after consuming jb_peek result).
void jb_advance(void);

// Frames currently between read_head and write_head (saturates to JB_FRAMES).
uint32_t jb_occupancy_frames(void);

// Occupancy in milliseconds.
uint32_t jb_occupancy_ms(void);

// Discard all frames and reset heads. Call on server-directed stream stop.
// Resets both read and write heads to zero (use before seq-0 restart).
void jb_flush(void);

// Sync read head to the current write head position, clearing all valid flags.
// Use when entering ACQUIRING mid-stream: the ring starts empty from "now"
// and fills at real-time rate — avoids the read head being stranded at 0
// while valid data sits millions of frames ahead.
void jb_sync_read_head(void);

// Returns true if no jb_write has occurred in the last JB_STALL_US microseconds.
bool jb_stalled(void);

// Sequence gap counter (incremented by NetworkReceiver on missing sequence).
extern volatile uint32_t g_seq_gaps;
