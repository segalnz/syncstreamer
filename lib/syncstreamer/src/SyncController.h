#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// SyncController v2 — digital PLL + client state machine
//
// sync_task() runs at 5 Hz on Core 1, priority 15.
// PLL: IIR-filtered buffer-occupancy error drives g_rate_ppm.
// State machine: IDLE → ACQUIRING → LOCKED ↔ RECOVERING → REACQUIRING → ACQUIRING
// ============================================================

typedef enum {
    ST_IDLE        = 0,
    ST_ACQUIRING   = 1,
    ST_LOCKED      = 2,
    ST_RECOVERING  = 3,
    ST_REACQUIRING = 4,
} client_state_t;

typedef enum {
    MODE_MUSIC = 0,
    MODE_TTS   = 1,
} sync_mode_t;

// ── Thresholds (frames, Music / TTS) ─────────────────────────
#define SC_LO_MUSIC      ((48000u * 30u) / 1000u)    // 1440 frames
#define SC_HI_MUSIC      ((48000u * 120u) / 1000u)   // 5760 frames
#define SC_CRIT_MUSIC    ((48000u * 10u) / 1000u)    // 480 frames

#define SC_LO_TTS        ((48000u * 20u) / 1000u)    // 960 frames
#define SC_HI_TTS        ((48000u * 150u) / 1000u)   // 7200 frames
#define SC_CRIT_TTS      ((48000u * 5u) / 1000u)     // 240 frames

#define SC_PPM_MAX_MUSIC  100.0f
#define SC_PPM_MAX_TTS    200.0f
#define SC_PPM_DELTA_MAX  10.0f   // max ppm change per 200 ms tick

// ── Globals ───────────────────────────────────────────────────
// Written by sync_task, read by audio_out_task / status server.
extern volatile client_state_t g_state;
extern volatile sync_mode_t    g_mode;
extern volatile float          g_rate_ppm;    // fractional resampler rate
extern volatile float          g_filtered_err; // for status display
extern volatile bool           g_ducked;
extern volatile bool           g_muted;
extern volatile bool           g_stream_active; // set by ctrl_rx on STREAM_START

// For ACQUIRING → LOCKED transition: present_us of first expected frame.
extern volatile uint64_t g_next_present_us;

// Initialise globals. Call before creating sync_task.
void sync_controller_init(void);

// FreeRTOS task body. Pin to Core 1, priority 15.
void sync_task(void* pvParam);

// Called from wifi_rx_task on per-packet TTS flag (seq high bit).
// Sets g_mode to MODE_TTS or MODE_MUSIC.
void sync_controller_set_tts(bool tts);

// Called from ctrl_rx_task on DUCK_START / DUCK_END control messages.
void on_duck_start(void);
void on_duck_end(void);
