#pragma once
#include <stdint.h>

// ============================================================
// SyncStreamer UDP Packet Format
// All multi-byte fields are network byte order (big-endian).
//
// Header: 20 bytes
//   stream_id            uint32  Stream identifier, shared across all clients
//   sequence             uint32  Monotonically increasing per stream
//   presentation_time_us int64   UNIX µs: when sample[0] should emerge from DAC
//   sample_count         uint16  Mono: mono samples; Stereo: stereo pairs
//   flags                uint16  See FLAGS_* below
//
// Payload: PCM samples
//   Stereo: sample_count * 2 * int16_t  (L, R interleaved)
//   Mono:   sample_count * 1 * int16_t
// ============================================================

#define SYNC_PACKET_FLAGS_MONO    (0x0001u)  // payload is mono (1 channel)
#define SYNC_PACKET_FLAGS_DUCKED  (0x0002u)  // server requests volume duck
// bits 2-15 reserved, must be zero

#define SYNC_PACKET_HEADER_SIZE   20u
#define SYNC_PACKET_MAX_SAMPLES   240u       // 5ms at 48kHz — keeps UDP payload ≤980 B (below 1472 B MTU)
#define SYNC_PACKET_BLOCK_DURATION_US  ((uint32_t)SYNC_PACKET_MAX_SAMPLES * 1000000u / 48000u)  // 5000 µs
#define SYNC_PACKET_MAX_UDP_SIZE  (SYNC_PACKET_HEADER_SIZE + SYNC_PACKET_MAX_SAMPLES * 2 * sizeof(int16_t))

#pragma pack(push, 1)
struct SyncPacketHeader {
    uint32_t stream_id;
    uint32_t sequence;
    int64_t  presentation_time_us;
    uint16_t sample_count;
    uint16_t flags;
};
#pragma pack(pop)

static_assert(sizeof(SyncPacketHeader) == SYNC_PACKET_HEADER_SIZE,
              "SyncPacketHeader size mismatch");

// Helper: deserialise a raw big-endian buffer into a host-endian header.
// Returns false if buf_len is too small.
inline bool syncPacketParseHeader(const uint8_t* buf, size_t buf_len,
                                   SyncPacketHeader& out)
{
    if (buf_len < SYNC_PACKET_HEADER_SIZE) return false;

    // Manual big-endian decode — avoids ntohl dependency issues across platforms
    out.stream_id = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
                  | ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];

    out.sequence  = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16)
                  | ((uint32_t)buf[6] <<  8) |  (uint32_t)buf[7];

    uint64_t hi   = ((uint64_t)buf[ 8] << 24) | ((uint64_t)buf[ 9] << 16)
                  | ((uint64_t)buf[10] <<  8) |  (uint64_t)buf[11];
    uint64_t lo   = ((uint64_t)buf[12] << 24) | ((uint64_t)buf[13] << 16)
                  | ((uint64_t)buf[14] <<  8) |  (uint64_t)buf[15];
    out.presentation_time_us = (int64_t)((hi << 32) | lo);

    out.sample_count = ((uint16_t)buf[16] << 8) | buf[17];
    out.flags        = ((uint16_t)buf[18] << 8) | buf[19];
    return true;
}

// Helper: validate a fully-received packet buffer (header + expected payload).
inline bool syncPacketValidate(const uint8_t* buf, size_t buf_len,
                                const SyncPacketHeader& hdr)
{
    bool mono       = (hdr.flags & SYNC_PACKET_FLAGS_MONO) != 0;
    size_t expected = SYNC_PACKET_HEADER_SIZE
                    + hdr.sample_count * (mono ? 1u : 2u) * sizeof(int16_t);
    return (buf_len == expected) && (hdr.sample_count > 0)
        && (hdr.sample_count <= SYNC_PACKET_MAX_SAMPLES);
}
