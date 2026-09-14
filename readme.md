# SyncStreamer

ESP32-S3 firmware for a LAN audio endpoint receiving uncompressed stereo PCM over Wi-Fi and playing it through a PCM5102A DAC. The project aims to support synchronized music and text-to-speech (TTS) across multiple speakers, using a jitter buffer, a software PLL, and a fractional resampler.

This README describes the root PlatformIO application in [src/main.cpp](src/main.cpp) and the implementation in [lib/syncstreamer/src](lib/syncstreamer/src). It was checked against the code and the [draft firmware specification](ESP32_AudioSync_FirmwareSpec_v1.rtf). Synchronization accuracy and glitch-free playback are design goals, not verified performance guarantees.

## Hardware and wiring

The root configuration targets `esp32-s3-devkitc-1`, with 8 MB flash and QSPI PSRAM settings intended for the N8R2 variant. Use a 2.4 GHz Wi-Fi network. A separate server must supply the PCM stream; a server implementation is not included in the tracked project.

| ESP32-S3 connection | PCM5102A signal |
| --- | --- |
| GPIO15 | BCK / bit clock |
| GPIO16 | LRCK / word clock |
| GPIO17 | DIN / audio data |
| GND | Common ground |
| 3.3 V logic level | XSMT held high (unmuted) |

Power the DAC module according to its board requirements and connect its analog outputs to an amplifier or powered speakers. The current firmware does not drive GPIO10 for mute. It produces silence by writing zero samples while keeping the audio clock running.

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

   `SERVER_HOST` must be an IPv4 address: the root application's announcement code uses `inet_aton`, not hostname resolution. **`src/secrets.h` is currently tracked by Git.** Keep real credentials out of commits; the values above are placeholders.

2. Adjust the machine-specific settings in `platformio.ini`. In particular, replace `/mnt/Sam/Builds/syncstreamer` with a writable build directory such as `.pio/build`, and change `/dev/ttyACM0` if necessary. Match flash and PSRAM settings to your actual board.

3. Build, upload over USB, and open the serial monitor:

   ```sh
   pio run -e syncstreamer
   pio run -e syncstreamer -t upload
   pio device monitor -b 115200
   ```

The configured dependencies are ElegantOTA, AsyncTCP, and PubSubClient. The source also includes `ESPAsyncWebServer.h`; if dependency resolution does not supply it transitively, add `esp32async/ESPAsyncWebServer` to `lib_deps`.

The platform URL follows the moving `stable` release, so builds are not pinned to a specific toolchain. The partition table provides two 2 MB OTA application slots and a SPIFFS partition in 8 MB flash.

The working directory used for this documentation review also contained untracked CMake/component files. They are not included in this README-only change. A build from a clean clone has not been verified; local build success should not be taken as proof that all build inputs are committed.

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

| Setting | Current implementation |
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

Playback is **not scheduled against `present_us`** in the current state machine. The receiver records timestamps, but acquisition only checks buffer occupancy. The audio task can also play in `ACQUIRING` when not muted; startup behavior therefore depends on the existing mute state. Do not assume a fixed startup delay or a shared timestamp-aligned start.

The state machine progresses through `IDLE`, `ACQUIRING`, `LOCKED`, `RECOVERING`, and `REACQUIRING`. It can enter acquisition on `STREAM_START` or incoming audio. Critical underrun or a stall while locked/recovering mutes output and resets the read position for refilling. `STREAM_STOP` flushes immediately, rather than draining queued audio.

Network and control tasks run on core 0 at priorities 18 and 16. Audio output and synchronization run on core 1 at priorities 22 and 15. An announcement task runs on core 0 every 30 seconds. Wi-Fi sleep is disabled, and the main loop attempts reconnection after a disconnect.

## Server protocol

See [SyncPacket.h](lib/syncstreamer/src/SyncPacket.h) and [NetworkReceiver.cpp](lib/syncstreamer/src/NetworkReceiver.cpp) for the implemented wire format. The [v2 server specification](server_specification_v2.txt) provides design context; the details below take precedence where it differs from the receiver.

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

| Message | Direction | Current behavior |
| --- | --- | --- |
| `SYNC_HELLO` | Client → configured server | Sent after initial Wi-Fi connection and every 30 seconds; ten bytes including a trailing NUL |
| `STREAM_START` | Server → client | Resets receiver tracking and requests acquisition |
| `STREAM_STOP` | Server → client | Resets tracking, flushes buffered audio, enters idle, and mutes |
| `PING` | Server → client | Replies `PONG` to the sender's address and port |

The receiver does not currently handle `DUCK_START` or `DUCK_END`. Apply mixing and volume ducking to the PCM on the server. There is no codec negotiation, retransmission, or stream-ID field.

## Dashboard and OTA

After Wi-Fi connects, use the IP address printed on the serial monitor:

| Route | Function |
| --- | --- |
| `GET /` | Dashboard |
| `GET /api/status` | JSON state, buffer fill, rate correction, offset, packet/dropout counters, and Wi-Fi information |
| `POST /api/config` | Form fields `mode=music|tts` and `target_fill_ms=20..500` |
| `/update` | ElegantOTA upload handler registered during initialization |

The asynchronous HTTP server starts in `setup()`. However, the call to `status_server_loop()` is commented out in the root application's `loop()`. Consequently MQTT connection/publishing and `ElegantOTA.loop()` servicing are disabled. OTA completion/reboot behavior has not been verified in this configuration; use USB upload for initial setup.

The target-fill form currently changes only the displayed/status value. The PLL continues to use the compile-time `JB_TARGET_FRAMES` constant. Mode selection is temporary because incoming packets set it again. Settings are not persisted across reboot.

The MQTT implementation, if its servicing is restored, targets the broker defined in [StatusServer.h](lib/syncstreamer/src/StatusServer.h) and publishes to `speaker/<node-id>/status` at a nominal two-second interval. It skips publishing when buffer occupancy is low.

The application configures no HTTP/OTA authentication, and UDP input is unauthenticated. Deploy on a trusted LAN.

## Differences from the draft and remaining limitations

The [v1 RTF specification](ESP32_AudioSync_FirmwareSpec_v1.rtf) describes intended behavior. The following are material differences found in the root code:

| Draft design | Code reviewed |
| --- | --- |
| 8,192-frame PSRAM ring, 60 ms target, 200 ms startup | 4,096-frame internal-RAM ring, 30 ms target, 60 ms lock threshold |
| Wait for presentation time before playback | No presentation-time gate; acquisition can output audio when unmuted |
| Occupancy-only PLL | Occupancy feedback plus offset-drift feed-forward |
| Example DAC pins 12/13/11 and XSMT control | Pins 15/16/17; XSMT held high; software silence |
| Advisory duck commands | No handlers in the active control receiver |
| MQTT health publication every two seconds | Servicing disabled in the root main loop |
| Wake-word task on combined nodes | No wake-word task created by the root application |
| Music ±5 ms and TTS ±20 ms synchronization | Targets remain unverified by this documentation review |

The draft's 200 ms startup requirement also exceeds its own approximately 170 ms ring capacity. Its N8R2 PSRAM capacity and 5 GHz Wi-Fi assumptions should not be used to configure the target board.

Additional code limitations matter when interpreting diagnostics or testing playback:

- The 120/150 ms high-water thresholds exceed the current approximately 85 ms ring capacity, so those high-fill recovery conditions cannot be reached through the reported occupancy.
- Occupancy is calculated from read/write-head distance, not by counting valid samples. Sequence-gap accounting uses unsigned subtraction, so reordered or duplicate packets can produce misleadingly large loss counts. Reordering and sequence wrap need validation.
- The resampler contains a cosine-envelope concealment path with a 960-frame (20 ms) constant, despite older 5/10 ms descriptions. Its `frame_valid = valid || !have_next` expression can treat an empty buffer as valid; reliable fade-to-silence behavior must not be assumed.
- The root setup does not check the return values of jitter-buffer and I2S initialization. Inspect serial errors if startup fails.

## Source map

| File | Responsibility |
| --- | --- |
| [src/main.cpp](src/main.cpp) | Root application setup, Wi-Fi, registration, and task creation |
| [JitterBuffer.cpp](lib/syncstreamer/src/JitterBuffer.cpp) | Sequence-indexed audio storage |
| [NetworkReceiver.cpp](lib/syncstreamer/src/NetworkReceiver.cpp) | Audio and control UDP parsing |
| [OffsetEstimator.cpp](lib/syncstreamer/src/OffsetEstimator.cpp) | Timestamp offset and drift estimation |
| [SyncController.cpp](lib/syncstreamer/src/SyncController.cpp) | PLL and playback state machine |
| [Resampler.cpp](lib/syncstreamer/src/Resampler.cpp) | Fractional reading and gap concealment |
| [AudioOutput.cpp](lib/syncstreamer/src/AudioOutput.cpp) | DAC interface and DMA output |
| [StatusServer.cpp](lib/syncstreamer/src/StatusServer.cpp) | HTTP, OTA registration, and MQTT implementation |
| [syncstreamer.h](lib/syncstreamer/src/syncstreamer.h) | Alternative library wrapper API; not used by the root `main.cpp` |

The original [instructions](instructions.txt), [server specification](server_specification.txt), and [v2 server specification](server_specification_v2.txt) are retained as design history. Prefer the implementation and this README for current root-application behavior.
