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

void network_receiver_stream_reset(void)
{
    s_last_seq = 0;
    s_first    = true;
    g_seq_gaps = 0;
}

// ── wifi_rx_task — audio packets (port 5005) ──────────────────────────────────
void wifi_rx_task(void* pvParam)
{
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { log_e("wifi_rx: socket() failed"); vTaskDelete(nullptr); return; }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int rcvbuf = 65536;
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

        if ((size_t)n != SYNC_PACKET_SIZE || pkt.magic != SYNC_MAGIC) continue;

        // Track sequence gaps — count silently; suppress per-gap log spam
        // which causes audio glitches by flooding the serial output at high rate.
        if (!s_first) {
            uint32_t expected = s_last_seq + 1;
            if (pkt.sequence != expected) {
                uint32_t gap = pkt.sequence - expected;
                g_seq_gaps += gap;
                // Only log large bursts of loss to avoid serial flooding.
                if (gap >= 10) {
                    log_w("wifi_rx: large seq gap %u→%u (%u lost)",
                          expected, pkt.sequence, gap);
                }
            }
        }
        s_last_seq = pkt.sequence;
        s_first    = false;

        // Update server clock offset estimator.
        offset_update(pkt.present_us);

        // Write 256 stereo frames into jitter buffer.
        uint32_t base_frame = pkt.sequence * SYNC_FRAMES_PER_PACKET;
        for (uint32_t i = 0; i < SYNC_FRAMES_PER_PACKET; i++) {
            int16_t frame[2] = { pkt.pcm[i * 2], pkt.pcm[i * 2 + 1] };
            jb_write(base_frame + i, frame);
        }

        // Store presentation time for ACQUIRING decision.
        g_next_present_us = pkt.present_us;
    }
}

// ── ctrl_rx_task — control messages (port 5006) ───────────────────────────────
void ctrl_rx_task(void* pvParam)
{
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { log_e("ctrl_rx: socket() failed"); vTaskDelete(nullptr); return; }

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
            g_stream_active = true;   // sync_task will notice and transition IDLE→ACQUIRING
        } else if (strncmp(buf, "STREAM_STOP", 11) == 0) {
            log_i("ctrl_rx: STREAM_STOP");
            network_receiver_stream_reset();
            g_state = ST_IDLE;
            jb_flush();
            audio_out_mute();
        } else if (strncmp(buf, "DUCK_START", 10) == 0) {
            on_duck_start();
        } else if (strncmp(buf, "DUCK_END", 8) == 0) {
            on_duck_end();
        } else if (strncmp(buf, "PING", 4) == 0) {
            lwip_sendto(sock, "PONG", 4, 0,
                        (struct sockaddr*)&sender, slen);
        }
    }
}
