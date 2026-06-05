#include "NetworkReceiver.h"
#include "SyncPacket.h"
#include "JitterBuffer.h"
#include "OffsetEstimator.h"
#include "SyncController.h"
#include "AudioOutput.h"
#include "Resampler.h"
#include <Arduino.h>
#include <lwip/sockets.h>
#include <string.h>

// ── Stream sequence tracking (file-scope, reset across streams) ──
static uint32_t s_last_seq = 0;
static bool     s_first    = true;

// ── Packet statistics (exposed for dashboard / MQTT) ─────────────
volatile uint32_t g_rx_packets = 0;
volatile uint32_t g_rx_missed  = 0;

void network_receiver_stream_reset(void)
{
    s_last_seq  = 0;
    s_first     = true;
    g_seq_gaps  = 0;
    g_rx_packets = 0;
    g_rx_missed  = 0;
    offset_estimator_reset();
}

// ── wifi_rx_task — audio packets (port 5005) ──────────────────────────────────
void wifi_rx_task(void* pvParam)
{
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { log_e("wifi_rx: socket() failed"); vTaskDelete(nullptr); return; }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int rcvbuf = 131072;  // ~677ms of audio buffering
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(NET_AUDIO_PORT);
    if (lwip_bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log_e("wifi_rx: bind() failed"); lwip_close(sock); vTaskDelete(nullptr); return;
    }

    log_i("wifi_rx: listening UDP %u", NET_AUDIO_PORT);

    static SyncPacket pkt;

    for (;;) {
        int n = lwip_recv(sock, &pkt, sizeof(pkt), 0);
        if (n <= 0) continue;

        if ((size_t)n != SYNC_PACKET_SIZE) continue;

        // READY packet: sequence=0, present_us=0. Server sends this on the
        // audio port to signal the current stream is complete. Ordered with
        // audio on the same port — no race between control and data.
        if (pkt.magic == SYNC_MAGIC && pkt.sequence == 0 && pkt.present_us == 0) {
            network_receiver_stream_reset();
            g_state = ST_IDLE;
            jb_flush();
            audio_out_mute();
            continue;
        }

        if (pkt.magic != SYNC_MAGIC) continue;

        g_rx_packets += 1;

        // Strip TTS flag from sequence number high bit.
        uint32_t raw_seq = pkt.sequence;
        bool tts_pkt = (raw_seq & 0x80000000) != 0;
        uint32_t seq = raw_seq & 0x7FFFFFFF;

        sync_controller_set_tts(tts_pkt);

        // Track sequence gaps — count silently.
        if (!s_first) {
            uint32_t expected = s_last_seq + 1;
            if (seq != expected) {
                uint32_t gap = seq - expected;
                g_seq_gaps += gap;
                g_rx_missed += gap;
                if (gap >= 10) {
                    static uint32_t s_last_gap_log = 0;
                    uint32_t now = millis();
                    if (now - s_last_gap_log > 5000) {
                        s_last_gap_log = now;
                        log_w("wifi_rx: large seq gap %u→%u (%u lost)",
                              expected, seq, gap);
                    }
                }
            }
        }
        s_last_seq = seq;
        s_first    = false;

        // Update server clock offset estimator.
        offset_update(pkt.present_us);

        // Write 256 stereo frames into jitter buffer (one mutex lock per packet).
        uint32_t base_frame = seq * SYNC_FRAMES_PER_PACKET;
        jb_write_packet(base_frame, pkt.pcm);

        // Store presentation time for ACQUIRING decision.
        g_next_present_us = pkt.present_us;
    }
}

// ── ctrl_rx_task — control messages (port 5006) ───────────────────────────────
void ctrl_rx_task(void* pvParam)
{
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { log_e("ctrl_rx: socket() failed"); vTaskDelete(nullptr); return; }

    int reuse = 1;
    lwip_setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(NET_CTRL_PORT);
    if (lwip_bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log_e("ctrl_rx: bind() failed"); lwip_close(sock); vTaskDelete(nullptr); return;
    }

    log_i("ctrl_rx: listening UDP %u", NET_CTRL_PORT);

    char     buf[64];
    struct sockaddr_in sender{};
    socklen_t slen = sizeof(sender);

    for (;;) {
        int n = lwip_recvfrom(sock, buf, sizeof(buf) - 1, 0,
                              (struct sockaddr*)&sender, &slen);
        if (n <= 0) continue;
        buf[n] = '\0';

        if (strncmp(buf, "STREAM_START", 12) == 0) {
            log_i("ctrl_rx: STREAM_START");
            network_receiver_stream_reset();
            g_stream_active = true;
        } else if (strncmp(buf, "STREAM_STOP", 11) == 0) {
            log_i("ctrl_rx: STREAM_STOP");
            network_receiver_stream_reset();
            g_state = ST_IDLE;
            jb_flush();
            audio_out_mute();
        } else if (strncmp(buf, "PING", 4) == 0) {
            lwip_sendto(sock, "PONG", 4, 0,
                        (struct sockaddr*)&sender, slen);
        }
    }
}
