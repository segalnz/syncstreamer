# SyncStreamer

ESP32-S3 firmware for a LAN audio endpoint receiving uncompressed stereo PCM over Wi-Fi and playing it through a PCM5102A DAC. The system uses a jitter buffer, a software PLL, and a fractional resampler to control playback rate for music and text-to-speech (TTS) across multiple speakers.

This document specifies the current firmware, hardware connections, and server interface. Our design priorities are smooth playback, stable latency, and a lightweight client that leaves audio generation and mixing to the server. The inter-speaker synchronization targets are ±5 ms for music and ±20 ms for TTS; these remain development targets.

## Hardware and wiring

The supplied configuration targets `esp32-s3-devkitc-1`, with 8 MB flash and QSPI PSRAM settings intended for the N8R2 variant. Use a 2.4 GHz Wi-Fi network. A separate server must supply the PCM stream; the server is developed separately.

| ESP32-S3 connection | PCM5102A signal |
| --- | --- |
| GPIO15 | BCK / bit clock |
| GPIO16 | LRCK / word clock |
| GPIO17 | DIN / audio data |
| GND | Common ground |
| 3.3 V logic level | XSMT held high (unmuted) |

Power the DAC module according to its board requirements and connect its analog outputs to an amplifier or powered speakers. Muting uses zero-valued audio samples while the audio clock continues running. XSMT remains high; no GPIO mute connection is required.

The driver uses `I2S_STD_MSB_SLOT_DEFAULT_CONFIG` with 16-bit stereo slots and no MCLK output. Check the DAC module's format straps against this MSB-aligned configuration. Pin and DMA settings are in [AudioOutput.h](lib/syncstreamer/src/AudioOutput.h).

## Configuration and build

Use PlatformIO with the pioarduino platform configured in [platformio.ini](platformio.ini). The project selects both Arduino and ESP-IDF; [src/app_main.cpp](src/app_main.cpp) explicitly initializes Arduino and runs `setup()` and `loop()`.

1. Configure the definitions in `src/secrets.h` for your network:

   ```cpp
   #pragma once
   #define WIFI_SSID "your-network"
   #define WIFI_PASSWORD "your-password"
   #define SERVER_HOST "192.168.1.100"
   #define ANNOUNCE_PORT 5006
   ```

   `SERVER_HOST` must be a numeric IPv4 address. **`src/secrets.h` is currently tracked by Git.** Keep real credentials out of commits; the values above are placeholders.

2. Adjust the machine-specific settings in `platformio.ini`. In particular, replace `/mnt/Sam/Builds/syncstreamer` with a writable build directory such as `.pio/build`, and change `/dev/ttyACM0` if necessary. Match flash and PSRAM settings to your actual board.

3. Build, upload over USB, and open the serial monitor:

   ```sh
   pio run -e syncstreamer
   pio run -e syncstreamer -t upload
   pio device monitor -b 115200
   ```

The configured dependencies are ElegantOTA, AsyncTCP, and PubSubClient. HTTP support also requires ESPAsyncWebServer. If it is not resolved transitively, add `esp32async/ESPAsyncWebServer` to `lib_deps`.

The platform URL follows the moving `stable` release, so builds are not pinned to a specific toolchain. The partition table provides two 2 MB OTA application slots and a SPIFFS partition in 8 MB flash.

## Audio pipeline

```text
External PCM server
    -> UDP receiver (5005)
    -> sequence-indexed ring buffer
    -> fractional linear resampler
    -> continuous I2S DMA
    -> PCM5102A -> amplifier / powered speakers

Packet timestamps -> offset estimator --+
Buffer occupancy ----------------------+-> PLL -> resampler rate
```

| Setting | Value |
| --- | --- |
| Audio format | 48,000 Hz, signed 16-bit, interleaved stereo |
| Packet size | 256 stereo frames; 1,040 bytes including header |
| Ring capacity | 4,096 frames, approximately 85.3 ms, allocated in internal RAM |
| PLL target fill | 30 ms / 1,440 frames |
| Acquisition/reacquisition lock threshold | 60 ms / 2,880 frames |
| Stall threshold | 500 ms since the last buffer write, once a write has occurred |
| PLL update interval | 200 ms / 5 Hz |
| Maximum rate adjustment | ±100 ppm music; ±200 ppm TTS |
| Maximum rate change | 10 ppm per PLL update |
| DMA configuration | Four buffers of 512 stereo frames |
| Output gain | PCM multiplied by 0.89, approximately −1 dB |

The PLL combines filtered buffer occupancy error with a filtered estimate of timestamp-offset drift. The offset estimator takes the rolling minimum of 32 samples of `present_us - esp_timer_get_time() - 200000`. Clients do not consult NTP.

Acquisition is controlled by buffer occupancy. Presentation timestamps supply the offset estimator and PLL; they do not gate the start of playback. Audio output is enabled in `ACQUIRING`, `LOCKED`, and `RECOVERING` whenever the mute flag is clear. After a mute event, output resumes when the buffer reaches the 60 ms lock threshold. Startup latency is therefore state-dependent.

The state machine progresses through `IDLE`, `ACQUIRING`, `LOCKED`, `RECOVERING`, and `REACQUIRING`. It can enter acquisition on `STREAM_START` or incoming audio. Critical underrun or a stall while locked/recovering mutes output and resets the read position for refilling. `STREAM_STOP` immediately discards queued audio and mutes output.

Network and control tasks run on core 0 at priorities 18 and 16. Audio output and synchronization run on core 1 at priorities 22 and 15. An announcement task runs on core 0 every 30 seconds. Wi-Fi sleep is disabled, and the main loop attempts reconnection after a disconnect.

## Server protocol

The server sends one unicast PCM stream to each client. The packet definition is maintained in [SyncPacket.h](lib/syncstreamer/src/SyncPacket.h). Use the interface below when integrating a server; additional server design information is in the [v2 server specification](server_specification_v2.txt).

### Audio: UDP port 5005

Packets contain a packed 16-byte header followed by 1,024 bytes of PCM. All integer fields and samples use little-endian encoding.

| Byte offset | Type / size | Meaning |
| --- | --- | --- |
| 0 | `uint32_t` | Magic value `0xEE15A3D1` |
| 4 | `uint32_t` | Bit 31 selects TTS; bits 0–30 hold the packet sequence |
| 8 | `uint64_t` | `present_us`, server presentation timestamp in microseconds |
| 16 | 512 × `int16_t` | `L0, R0, L1, R1, …, L255, R255` |

The masked sequence maps each packet to 256 consecutive buffer frames. Set bit 31 for TTS packets and clear it for music. Each audio packet updates the client mode, overriding any mode selected through the web interface.

A full-size packet with valid magic, raw `sequence == 0`, and `present_us == 0` is a **READY/end-of-stream marker**: it resets receiver statistics, flushes the buffer, enters idle, and mutes. Reserve that combination for stream completion.

For steady playback, the server should pace packets at the audio rate: 187.5 packets per second, approximately 5.33 ms apart. Raw PCM traffic is 192,000 bytes/s per client, excluding headers. Use consistent presentation timestamps advancing with the sample timeline; the estimator currently subtracts a fixed 200 ms lead.

### Control and registration: UDP port 5006

| Message | Direction | Behavior |
| --- | --- | --- |
| `SYNC_HELLO` | Client → configured server | Sent after initial Wi-Fi connection and every 30 seconds; ten bytes including a trailing NUL |
| `STREAM_START` | Server → client | Resets receiver tracking and requests acquisition |
| `STREAM_STOP` | Server → client | Resets tracking, flushes buffered audio, enters idle, and mutes |
| `PING` | Server → client | Replies `PONG` to the sender's address and port |

Mixing and volume ducking are server responsibilities and must be applied to the outgoing PCM. `DUCK_START` and `DUCK_END` are reserved and ignored by this firmware. The transport uses a fixed PCM format without codec negotiation, retransmission, or stream IDs.

## Dashboard and OTA

After Wi-Fi connects, use the IP address printed on the serial monitor:

| Route | Function |
| --- | --- |
| `GET /` | Dashboard |
| `GET /api/status` | JSON state, buffer fill, rate correction, offset, packet/dropout counters, and Wi-Fi information |
| `POST /api/config` | Form fields `mode=music|tts` and `target_fill_ms=20..500` |
| `/update` | ElegantOTA upload handler registered during initialization |

The asynchronous HTTP server starts during initialization. MQTT and periodic ElegantOTA servicing are disabled in the supplied application to reduce competition with audio traffic. The OTA upload route remains registered, but the complete update/reboot workflow still requires validation. Use USB for initial installation and firmware updates in this configuration.

The PLL target is a compile-time setting, `JB_TARGET_FRAMES`. The dashboard target-fill field updates the displayed/status value only. Incoming audio packets determine music/TTS mode and take precedence over manual selection. Dashboard settings apply only until reboot.

When enabled through `status_server_loop()`, MQTT targets the broker defined in [StatusServer.h](lib/syncstreamer/src/StatusServer.h) and publishes to `speaker/<node-id>/status` at a nominal two-second interval. It skips publishing when buffer occupancy is low.

The application configures no HTTP/OTA authentication, and UDP input is unauthenticated. Deploy on a trusted LAN.

## Development status

This version provides PCM reception, playback-rate control, buffer recovery, and a local dashboard. Wake-word inference, microphone capture, speech recognition, and TTS generation are outside the supplied audio endpoint application.

Remaining development work includes:

- Measuring inter-speaker synchronization and long-duration playback stability.
- Completing MQTT and OTA integration under sustained audio load.
- Connecting runtime target-fill configuration to the PLL and tuning recovery thresholds for the 85 ms ring capacity.
- Improving handling of reordered, duplicate, and wrapped packet sequences, including loss-counter accuracy.
- Refining gap concealment and empty-buffer behavior. The resampler uses a 960-frame (20 ms) cosine-envelope setting; reliable fade-to-silence on underrun remains work in progress.
- Completing initialization error handling for buffer allocation and I2S startup.

Buffer occupancy represents the distance between the read and write positions and can include missing frames. Packet-loss and dropout counters are diagnostic indicators, not measurements of audible interruption duration.

## Specification history

The [v1 RTF specification](ESP32_AudioSync_FirmwareSpec_v1.rtf) records the original design. This README supersedes it for the current firmware configuration and protocol. The main revisions are the smaller internal-RAM buffer, 30 ms target and 60 ms lock threshold, occupancy-based acquisition, offset-drift feed-forward, revised DAC connections, and the TTS flag and READY marker in the audio protocol. MQTT servicing and advisory duck commands remain deferred.

## Source map

| File | Responsibility |
| --- | --- |
| [src/main.cpp](src/main.cpp) | Application setup, Wi-Fi, registration, and task creation |
| [JitterBuffer.cpp](lib/syncstreamer/src/JitterBuffer.cpp) | Sequence-indexed audio storage |
| [NetworkReceiver.cpp](lib/syncstreamer/src/NetworkReceiver.cpp) | Audio and control UDP parsing |
| [OffsetEstimator.cpp](lib/syncstreamer/src/OffsetEstimator.cpp) | Timestamp offset and drift estimation |
| [SyncController.cpp](lib/syncstreamer/src/SyncController.cpp) | PLL and playback state machine |
| [Resampler.cpp](lib/syncstreamer/src/Resampler.cpp) | Fractional reading and gap concealment |
| [AudioOutput.cpp](lib/syncstreamer/src/AudioOutput.cpp) | DAC interface and DMA output |
| [StatusServer.cpp](lib/syncstreamer/src/StatusServer.cpp) | HTTP, OTA registration, and MQTT implementation |
| [syncstreamer.h](lib/syncstreamer/src/syncstreamer.h) | Alternative library wrapper API; not used by the root `main.cpp` |

The original [instructions](instructions.txt), [server specification](server_specification.txt), and [v2 server specification](server_specification_v2.txt) are retained as design history. Use this README as the current application specification.
