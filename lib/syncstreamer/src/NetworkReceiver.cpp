#include "NetworkReceiver.h"
#include "SyncPacket.h"
#include <Arduino.h>
#include <lwip/sockets.h>
#include <string.h>

static constexpr size_t RX_BUF_SIZE = SYNC_PACKET_HEADER_SIZE
                                     + SYNC_PACKET_MAX_SAMPLES * 2 * sizeof(int16_t);

// ── Public ────────────────────────────────────────────────────────────────────
bool NetworkReceiver::begin(JitterBuffer* jbuf, uint16_t port)
{
    _jbuf    = jbuf;
    _port    = port;
    _running = false;
    statPacketsReceived = 0;
    statParseErrors     = 0;
    statPushFailed      = 0;

    // Open socket here so errors are reported synchronously.
    _sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (_sock < 0) {
        log_e("NetworkReceiver: socket() failed");
        return false;
    }

    // Receive timeout so the task loop can check _running.
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };  // 200 ms
    lwip_setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Increase socket receive buffer (helps on busy WiFi).
    int rcvbuf = 65536;
    lwip_setsockopt(_sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(_port);

    if (lwip_bind(_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        log_e("NetworkReceiver: bind() failed on port %u", _port);
        lwip_close(_sock);
        _sock = -1;
        return false;
    }

    _running = true;
    xTaskCreatePinnedToCore(_taskFunc, "net_rx", 4096, this, 5, &_task, 0);
    log_i("NetworkReceiver: listening on UDP port %u", _port);
    return true;
}

void NetworkReceiver::stop()
{
    _running = false;
    if (_sock >= 0) {
        lwip_close(_sock);
        _sock = -1;
    }
    if (_task) {
        vTaskDelay(pdMS_TO_TICKS(400));  // let task exit its loop
        _task = nullptr;
    }
}

// ── Task ──────────────────────────────────────────────────────────────────────
void NetworkReceiver::_taskFunc(void* arg)
{
    static_cast<NetworkReceiver*>(arg)->_run();
    vTaskDelete(nullptr);
}

void NetworkReceiver::_run()
{
    uint8_t rxBuf[RX_BUF_SIZE];

    while (_running) {
        struct sockaddr_in sender{};
        socklen_t senderLen = sizeof(sender);

        int n = lwip_recvfrom(_sock, rxBuf, sizeof(rxBuf), 0,
                               (struct sockaddr*)&sender, &senderLen);
        if (n <= 0) {
            // Timeout or transient error — loop back.
            continue;
        }

        statPacketsReceived++;

        // ── Parse header ──────────────────────────────────────
        SyncPacketHeader hdr;
        if (!syncPacketParseHeader(rxBuf, (size_t)n, hdr)) {
            statParseErrors++;
            log_w("NetworkReceiver: short packet (%d bytes)", n);
            continue;
        }

        // ── Validate header + payload length ──────────────────
        if (!syncPacketValidate(rxBuf, (size_t)n, hdr)) {
            statParseErrors++;
            log_w("NetworkReceiver: invalid packet seq=%u count=%u flags=0x%x",
                  hdr.sequence, hdr.sample_count, hdr.flags);
            continue;
        }

        // ── Push PCM into jitter buffer ───────────────────────
        const int16_t* pcm = reinterpret_cast<const int16_t*>(
                                 rxBuf + SYNC_PACKET_HEADER_SIZE);
        bool mono      = (hdr.flags & SYNC_PACKET_FLAGS_MONO) != 0;
        uint32_t words = hdr.sample_count * (mono ? 1u : 2u);

        if (!_jbuf->push(hdr, pcm, words)) {
            statPushFailed++;
            // JitterBuffer already logs drop reason at verbose level.
        }
    }
}
