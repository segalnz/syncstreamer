#pragma once
#include <stdint.h>

// ============================================================
// SyncStreamer v2 UDP Audio Packet
//
// Header: 16 bytes (little-endian / native)
//   magic        uint32   0xEE15A3D1 — packet sentinel
//   sequence     uint32   Monotonically increasing per stream
//   present_us   uint64   Server µs timestamp: when frame[0] should emerge
//
// Payload: 256 stereo frames = 512 × int16_t  (L,R interleaved)
//
// Total UDP payload: 1040 bytes
// ============================================================

#define SYNC_MAGIC              0xEE15A3D1u
#define SYNC_FRAMES_PER_PACKET  256u
#define SYNC_PACKET_SIZE        (sizeof(SyncPacket))   // 1040 bytes

#pragma pack(push, 1)
struct SyncPacket {
    uint32_t magic;                            //  0
    uint32_t sequence;                         //  4
    uint64_t present_us;                       //  8
    int16_t  pcm[SYNC_FRAMES_PER_PACKET * 2]; // 16  (L0,R0, L1,R1, ...)
};
#pragma pack(pop)

static_assert(sizeof(SyncPacket) == 1040, "SyncPacket size mismatch");
