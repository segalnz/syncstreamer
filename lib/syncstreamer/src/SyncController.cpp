#include "SyncController.h"
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include "AudioOutput.h"
#include "Resampler.h"
#include "NetworkReceiver.h"
#include <Arduino.h>
#include <math.h>

// ── Globals ───────────────────────────────────────────────────────────────────
volatile client_state_t g_state       = ST_IDLE;
volatile sync_mode_t    g_mode        = MODE_MUSIC;
volatile float          g_rate_ppm    = 0.0f;
volatile float          g_filtered_err = 0.0f;
volatile bool           g_ducked      = false;
volatile bool           g_muted       = false;
volatile bool           g_stream_active = false;
volatile uint64_t       g_next_present_us = 0;

// ── Init ──────────────────────────────────────────────────────────────────────
void sync_controller_init(void)
{
    g_state        = ST_IDLE;
    g_mode         = MODE_MUSIC;
    g_rate_ppm     = 0.0f;
    g_filtered_err = 0.0f;
    g_ducked       = false;
    g_muted        = false;
    g_stream_active = false;
    g_next_present_us = 0;
}

// ── TTS detection (called from wifi_rx_task per packet) ────────────────────────
void sync_controller_set_tts(bool tts)
{
    g_mode = tts ? MODE_TTS : MODE_MUSIC;
}

// ── Duck helpers (called from ctrl_rx_task) ───────────────────────────────────
void on_duck_start(void) { g_ducked = true;  log_i("SyncCtrl: duck start"); }
void on_duck_end(void)   { g_ducked = false; log_i("SyncCtrl: duck end");   }

// ── Feed-forward state (file-scope) ──────────────────────────────────────────
static int64_t  s_last_ff_offset = 0;
static int64_t  s_last_ff_time   = 0;
static float    s_ff_ppm         = 0.0f;

// Must be called whenever g_rate_ppm is zeroed.
static void ff_reset(void)
{
    s_last_ff_offset = 0;
    s_last_ff_time   = 0;
    s_ff_ppm         = 0.0f;
}

// ── PLL update (called each tick) ────────────────────────────────────────────
static void pll_update(void)
{
    float err = (float)jb_occupancy_frames() - (float)JB_TARGET_FRAMES;

    g_filtered_err = 0.85f * g_filtered_err + 0.15f * err;

    // Feed-forward term: instantaneous PPM from server clock offset drift.
    int64_t off    = g_offset_us;
    int64_t dt_us  = esp_timer_get_time() - s_last_ff_time;
    if (dt_us > 0 && dt_us < 500000) {
        float inst_ppm = (float)(off - s_last_ff_offset) * 1e6f / (float)dt_us;
        // Reject physically impossible values caused by offset-estimator step changes.
        if (inst_ppm > -500.0f && inst_ppm < 500.0f) {
            s_ff_ppm += 0.4f * (inst_ppm - s_ff_ppm);
        }
    }
    s_last_ff_offset = off;
    s_last_ff_time   = esp_timer_get_time();

    float fb_ppm = g_filtered_err * 0.1f;

    // Disable feed-forward when occupancy error exceeds ±50 % of target
    // (transient network jitter spikes are not permanently integrated).
    float raw_ppm = (err > -1440.0f && err < 1440.0f) ? (s_ff_ppm + fb_ppm) : fb_ppm;

    // Rate-of-change damping.
    float prev_ppm = g_rate_ppm;
    float delta    = raw_ppm - prev_ppm;
    if (delta >  SC_PPM_DELTA_MAX) delta =  SC_PPM_DELTA_MAX;
    if (delta < -SC_PPM_DELTA_MAX) delta = -SC_PPM_DELTA_MAX;

    float new_ppm = prev_ppm + delta;

    // Clamp by mode.
    float max_ppm = (g_mode == MODE_TTS) ? SC_PPM_MAX_TTS : SC_PPM_MAX_MUSIC;
    if (new_ppm >  max_ppm) new_ppm =  max_ppm;
    if (new_ppm < -max_ppm) new_ppm = -max_ppm;

    g_rate_ppm = new_ppm;
}

// ── State machine tick ────────────────────────────────────────────────────────
static void state_machine_tick(void)
{
    uint32_t occ    = jb_occupancy_frames();
    bool     stall  = jb_stalled();

    uint32_t lo   = (g_mode == MODE_TTS) ? SC_LO_TTS   : SC_LO_MUSIC;
    uint32_t hi   = (g_mode == MODE_TTS) ? SC_HI_TTS   : SC_HI_MUSIC;
    uint32_t crit = (g_mode == MODE_TTS) ? SC_CRIT_TTS : SC_CRIT_MUSIC;
    uint32_t startup = (g_mode == MODE_TTS) ? JB_STARTUP_FRAMES_TTS : JB_STARTUP_FRAMES;

    switch (g_state) {

    case ST_IDLE:
        // Transition on explicit STREAM_START OR on incoming audio data.
        // Per-packet TTS detection (high bit) ensures g_mode is correct
        // before the first audio frame reaches the JB.
        if (g_stream_active || occ > 0) {
            g_stream_active = false;
            network_receiver_stream_reset();
            // Sync read head to current write position so the ring starts
            // filling from "now" — avoids the read head being stranded at 0
            // while valid data is at frame positions in the millions.
            jb_sync_read_head();
            resampler_init(&g_resampler);
            g_filtered_err = 0.0f;
            g_rate_ppm = 0.0f;
            ff_reset();
            g_state = ST_ACQUIRING;
            log_i("SyncCtrl: IDLE → ACQUIRING");
        }
        break;

    case ST_ACQUIRING:
        if (stall) {
            g_state = ST_IDLE;
            audio_out_mute();
            log_w("SyncCtrl: ACQUIRING stalled → IDLE");
            break;
        }
        if (occ >= startup) {
            audio_out_unmute();
            g_state = ST_LOCKED;
            log_i("SyncCtrl: ACQUIRING → LOCKED (occ=%u frames)", occ);
        }
        break;

    case ST_LOCKED:
        pll_update();
        if (occ < crit || stall) {
            audio_out_mute();
            jb_sync_read_head();
            resampler_init(&g_resampler);
            g_filtered_err = 0.0f;
            g_rate_ppm = 0.0f;
            ff_reset();
            g_state = ST_REACQUIRING;
            log_w("SyncCtrl: LOCKED → REACQUIRING (occ=%u, stall=%d)", occ, stall);
        } else if (occ < lo || occ > hi) {
            g_state = ST_RECOVERING;
            log_w("SyncCtrl: LOCKED → RECOVERING (occ=%u)", occ);
        }
        break;

    case ST_RECOVERING:
        pll_update();
        if (occ < crit || stall) {
            audio_out_mute();
            jb_sync_read_head();
            resampler_init(&g_resampler);
            g_filtered_err = 0.0f;
            g_rate_ppm = 0.0f;
            ff_reset();
            g_state = ST_REACQUIRING;
            log_w("SyncCtrl: RECOVERING → REACQUIRING (occ=%u, stall=%d)", occ, stall);
        } else if (occ >= lo && occ <= hi) {
            g_state = ST_LOCKED;
            log_i("SyncCtrl: RECOVERING → LOCKED (occ=%u)", occ);
        }
        break;

    case ST_REACQUIRING:
        // Wait for enough frames to refill, then restart.
        if (occ >= startup) {
            audio_out_unmute();
            g_filtered_err = 0.0f;
            g_rate_ppm = 0.0f;
            ff_reset();
            g_state = ST_LOCKED;
            log_i("SyncCtrl: REACQUIRING → LOCKED (occ=%u frames)", occ);
        } else if (stall) {
            g_state = ST_IDLE;
            log_w("SyncCtrl: REACQUIRING stalled → IDLE");
        }
        break;
    }
}

// ── sync_task ─────────────────────────────────────────────────────────────────
void sync_task(void* pvParam)
{
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(200);  // 5 Hz

    for (;;) {
        vTaskDelayUntil(&xLastWake, xPeriod);
        state_machine_tick();
        log_d("SyncCtrl: state=%d occ=%u ms ppm=%.1f", (int)g_state, jb_occupancy_ms(), g_rate_ppm);
    }
}
