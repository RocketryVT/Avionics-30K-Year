# Rocket Antenna Tracker

The tracker turns a rocket GPS fix and a ground-station reference into azimuth and elevation. Both active boards are **Pico 2 W**: one transmits live GPS or the existing simulated trajectory, and the ground Pico receives, calculates pointing, and controls both motors in the same `gs_motor` executable. The laptop provides operator commands and monitoring.

Run the shell commands below from the **`Avionics-30K-Year` repository root**, where `tracker/` and `motor/` are siblings. The older standalone tracker checkout does not contain this integrated build.

## Build and flash the firmware on macOS

There are two active firmware images:

| Board | Build directory | Target | Image |
| --- | --- | --- | --- |
| Pico 2 W transmitter | `build/radio-link-pico2-w-tx` | `radio_link` | `build/radio-link-pico2-w-tx/radio_link.uf2` |
| Ground/motor Pico 2 W | `build/pico-mac` | `gs_motor` | `build/pico-mac/gs_motor.uf2` |

Rebuild the configured images after changing source, source selection, or station/replay settings:

```bash
cmake --build build/radio-link-pico2-w-tx --target radio_link -j4
cmake --build build/pico-mac --target gs_motor -j4
```

For a new build directory, install CMake (3.18 or newer), Ninja, the Arm GNU toolchain (`arm-none-eabi-g++`), and the Pico SDK with its submodules. Configure once, substituting your SDK path if needed:

```bash
cmake -S tools/radio_link -B build/radio-link-pico2-w-tx -G Ninja \
    -DPICO_BOARD=pico2_w \
    -DPICO_SDK_PATH="$HOME/pico/pico-sdk" \
    -DRADIO_LINK_TRANSMITTER=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake -S motor/firmware -B build/pico-mac -G Ninja \
    -DPICO_BOARD=pico2_w \
    -DPICO_SDK_PATH="$HOME/pico/pico-sdk" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Then run the two rebuild commands above. CMake fetches RadioLib **7.8.1** and SDK build tools as needed. The existing builds reuse dependencies in **`build/radio-deps`**; keep that directory. To reuse that cache when recreating a build, add these options to its configure command:

```bash
-DFETCHCONTENT_SOURCE_DIR_RADIOLIB="$PWD/build/radio-deps/radiolib-src" \
-DPICOTOOL_FETCH_FROM_GIT_PATH="$PWD/build/radio-deps"
```

Hold **BOOTSEL** while connecting each board over USB and copy its matching UF2 onto the mounted drive. The drive disappears when the Pico reboots. Open a separate USB serial monitor for each board. **Both applications currently wait for a USB connection before starting**; unattended field startup is not implemented.

The old `build/radio-link` standalone receiver and `build/radio-link-pico-w` transmitter build folders have been removed. Reception now runs inside `gs_motor`; no separate receiver image is needed.

The banner includes a `[motor] replay:` line that says whether the simulated flight is ready to play. The old `GS_ENABLE_REPLAY` option is gone. A build folder configured with it drops it the next time CMake runs.

## Choose live GPS or radio replay

[radio_config.hpp](include/radio_config.hpp) holds the common radio wiring, carrier frequency, output power, transmit start interval, completion timeout, and source/station settings for **both images**. The defaults remain 915 MHz, 10 dBm, a 200 ms start interval, and a 250 ms transmit timeout. The HAL and its timing guard still use SPI0. The source currently selects:

```cpp
inline constexpr RadioPositionSource RADIO_POSITION_SOURCE =
    RadioPositionSource::Replay;
```

`Replay` uses the existing trajectory and the synthetic station in [replay_config.hpp](tests/bench_tests/replay/replay_config.hpp). The station is 250 ft (76.2 m) south of the simulated launch point; zero station altitude and zero geoid separation are bench assumptions.

For live reception, change the selection to `RadioPositionSource::Gps` and set `RADIO_LIVE_STATION` to a verified `GPSPosition`: latitude and longitude in decimal degrees, altitude in meters above the **WGS84 ellipsoid**. Its default is `std::nullopt`, which refuses radio arming with `ERR,<id>,station,unset or invalid`. The Florida replay station is never selected for live GPS mode.

Rebuild and flash **both boards** after changing the selection. The packet does not identify GPS versus replay mode, so the two flashed images must agree.

## Replay over the real radio

The complete path is transmitter replay → 22-byte RF69 packet → ground decoding/order checks → pointing → internal motor `TARGET`. The laptop does not stream coordinates or calculate pointing.

Start with motor power disconnected to check telemetry and loop timing. Before a powered test, confirm the current assembly, gearing, driver configuration, direction, and travel limits using the [motor guide](../motor/README.md).

1. Select `Replay`, build and flash both images, and open both serial monitors with **LF or CRLF** line endings. On the transmitter, wait for `Replay holding at t=0. START runs; STOP ends.` It repeatedly transmits the initial position while simulation time remains zero.
2. On the ground console, inspect the active configuration:

   ```text
   SETTINGS,1
   LIMITS,2
   GEAR,3
   ```

   Set the appropriate gear mode before declaring either axis's position. `ZERO` declares a known pose; it does not home or move an axis. Only if the current pose is correctly represented by azimuth zero and elevation zero should you use `ZERO,4,ALL,0`. Both axes must be zeroed and idle before a new tracking session can arm.
3. On the **ground** console, select radio targeting:

   ```text
   TRACK,5,ON
   ```

   The held initial position now feeds pointing and motor targets. In a powered test, preposition and let the mechanism settle before advancing the flight. In a Pico-only test, reported positions are pulse counts rather than physical motion.
4. On the **transmitter** console, send:

   ```text
   START
   ```

   `START` is uppercase and requires a completed held-position transmission. If it says `Waiting for first held transmission`, wait and send `START` again. This is a local transmit-completion check, not confirmation that the ground antenna has settled.
5. The transmitter interpolates the existing flight at the configured minimum start interval (200 ms by default). At the end of the roughly 156-second trajectory it prints `Replay finished; no new packets`. After 1500 ms without an accepted target, the ground watchdog disarms tracking and ramps the axes to rest. Reception returning does not re-arm tracking.

To stop early, send **`STOP` to the transmitter** to stop producing packets, or `TRACK,<id>,OFF`, `STOP,<id>`, or `ESTOP,<id>` to the ground console to stop motor tracking. Ground stop commands do not stop the transmitter's replay clock. The transmitter completes cleanup of any in-flight send.

To repeat a completed/stopped radio replay, disarm the ground station and wait for rest, then reboot the transmitter. Once it is running in hold mode again, send `TRACK,<id>,OFF` on the ground to reset packet ordering, then explicitly re-arm with `TRACK,<id>,ON`. `START` does not restart a finished replay.

### Target source ownership

| Ground command | Effect |
| --- | --- |
| `TRACK,<id>,ON` | Select radio targets and end local replay; a new session needs idle, zeroed axes and a configured station. |
| `TRACK,<id>,REPLAY` | Select the ground Pico's built-in replay. Radio packets may still be monitored but cannot steer. |
| USB `TARGET,<id>,<az_mdeg>,<el_mdeg>` | Take target ownership from radio/local replay; tracking must already be armed. |
| `TRACK,<id>,OFF` | Disarm, clear the pending radio position, and reset packet ordering. |

When local replay ends or tracking times out, ownership does not automatically switch to radio. Use `TRACK,<id>,ON` explicitly. STOP, ESTOP, zero checks, active settings, and travel-limit clamping remain in the motor controller.

### Read reception and timing

The ground console emits a best-effort summary once per second:

```text
RX,ON|OFF,ok=...,bad=...,seq=...,tgt=...,e=...,rx=...,gap=...
```

| Field | Meaning |
| --- | --- |
| `ON` / `OFF` | Reception service enabled/disabled, not the tracking arm state. |
| `ok` | Decoded positions that passed packet ordering checks. |
| `bad` | Read, length, decoding, or ordering rejections; not an over-air loss count. |
| `seq` | Last accepted position sequence. |
| `tgt` | Radio positions converted into accepted motor targets. Clamped targets count; this does not measure physical tracking. |
| `e` | Radio initialization/restart status; zero means no latched fault. |
| `rx` | Maximum `pollReceiver()` duration in microseconds since startup. |
| `gap` | Maximum motor-loop gap in microseconds since startup, including USB, pointing, replay, and radio work. |

While unarmed, `ok` can increase with `tgt` unchanged. Compare changes in `tgt` over time to assess accepted pointing-update rate; packet arrivals and physical antenna tracking are separate measurements. The preliminary aim is at least five pointing updates per second, which still needs bench measurement under load. `send_if_room()` may drop summaries when USB has no room, and these timing maxima do not establish a hard real-time pulse guarantee.

## Run the replay

This section is the **ground Pico's built-in local replay**, which needs no transmitting board. For the RF path, use the procedure above and `TRACK ON` instead.

### From the window

Run `py -m gs.gui` from `motor/laptop`, choose mode **3 track** and **the Pico's built-in flight**. Confirm gearing and both zero declarations before pressing **ARM tracking**. The target/actual readout fills in from the `TRK` stream. **disarm**, **STOP**, and **E-STOP** all end it early. When the flight ends, the window notes that the Pico stopped tracking.

Mode 3 runs in `GEARED` mode, so the angles count the final gear pairs. The numbers below were taken in `DIRECT` mode from a serial monitor, as described next.

### From a serial monitor

For the **Pico-only bench test**, use `DIRECT` mode when the final drive is absent. In the serial monitor, set the line ending to **LF** or **CRLF** and send these one line at a time:

```text
SETTINGS,1
LIMITS,2
GEAR,3
```

`SETTINGS` and `LIMITS` show active limits and controller settings. Select the correct gear mode, then declare the known pose using `ZERO`; `ZERO,4,ALL,0` is appropriate only when the current reference is actually azimuth zero and elevation zero. Once both axes are zeroed and idle, start local replay:

```text
TRACK,5,REPLAY
```

An accepted `TRACK REPLAY` arms tracking and immediately starts the local flight clock at zero; it has no prepositioning hold. Its reply is `TRK,5,ON,...`. The Pico interpolates the trajectory and submits a new position every **200 ms (5 Hz)**. Keep the serial monitor open to capture the `TRK` lines. To stop early, send `TRACK,6,OFF`. `STOP`, `ESTOP`, an accepted `TRACK ON`, or a `TARGET` from the laptop also ends it. Stop and wait for rest before zeroing or starting a new session.

`TRACK ON` selects radio targeting, using the GPS/replay selection compiled into both boards; it never starts the built-in local replay. From the `py -m gs.motor` prompt, `track replay` does the same as `TRACK,<id>,REPLAY`.

The current trajectory runs for about **156 seconds**. Once it runs out of positions, the target watchdog disarms tracking after 1,500 ms without an accepted target. A normal ending shows the first message below, then `TRK,0,OFF,...`, the second message, and then `IDLE` status:

```text
[motor] replay finished - tracking stops in 1500 ms
[motor] no target for 1500 ms - tracking stopped
```

For a test with motors connected, first verify the physical gear ratios, direction, and travel limits using the [motor setup and command guide](../motor/README.md). `DIRECT` counts gearbox-shaft angles; `GEARED` accounts for the configured final drive. Set the gear mode before zeroing. `ZERO` declares the current angle and does not move or home an axis. Use a known physical reference: azimuth zero faces true north and elevation zero is level.

### Read the tracking results

```text
TRK,id,ON|OFF,az_target,az_position,az_velocity,el_target,el_position,el_velocity
```

Divide angles by 1,000 to get degrees and velocities by 1,000 to get degrees per second. The position is inferred from the Pico's step count. A target report is normally emitted every 100 ms while tracking, separately from the 200 ms replay updates.

For each axis, `abs(target - position)` is the gap to the latest commanded target. The largest gap seen in the log is a sampled peak. The final gap is the last reported difference. With a 0.5° deadband, the controller can settle with a small offset; larger gaps can occur while catching up to a new target.

The Pico-only runs used the same simulated station, `DIRECT` mode, and 0.5° deadband. Both completed and disarmed automatically:

| Largest sampled gap | 2 Hz (500 ms) | 5 Hz (200 ms) |
| --- | ---: | ---: |
| Azimuth | 0.540° | 0.517° |
| Elevation | 4.162° | 1.642° |

These are gaps to the latest motor target. Checking against the continuous trajectory requires matching positions to simulation time. Checking physical tracking requires measuring the geared mechanism with motors connected.

### Change the replay data or settings

[replay_config.hpp](tests/bench_tests/replay/replay_config.hpp) holds the update period, station position, and geoid separation. The current **synthetic** station is at `28.6093124565, -80.6`, 250 ft (76.2 m) due south of the CSV launch point. Its altitude and the geoid separation are both set to `0.0` m for the bench test. Those are test assumptions; use a verified position and altitude reference for deployment. Both optional reference values must be set for replay to submit positions.

The checked-in [replay_data.hpp](tests/bench_tests/replay/replay_data.hpp) contains 889 samples from `gps_sim_data.csv`, so the CSV is only needed to regenerate the table. The input columns are time in seconds, altitude above sea level in feet, latitude in decimal degrees north, and longitude in decimal degrees east. The converter checks the header and increasing timestamps, and converts feet to meters. Replay adds the signed geoid separation to obtain ellipsoidal altitude.

To regenerate the table from the original CSV in Downloads:

```bash
python3 tracker/tests/bench_tests/replay/convert_csv.py \
    "$HOME/Downloads/gps_sim_data.csv" \
    tracker/tests/bench_tests/replay/replay_data.hpp
```

Both local and radio replay reuse this table and `replayPositionAt()`. Rebuild and flash both boards after changing shared configuration or the table. The existing dataset is sufficient for these tests; regeneration is only needed when intentionally changing the data.

## Build and test on the Mac

Use CMake and a C++17 compiler, such as Clang from the macOS Command Line Tools. The desktop build does not require the Pico SDK.

```bash
cmake -S tracker -B tracker/build \
    -DTRACKER_BUILD_DESKTOP=ON -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build tracker/build -j4
ctest --test-dir tracker/build --output-on-failure
```

The eight suites cover coordinate conversion, pointing, target angular velocity, the motor adapter, GPS parsing, sentence framing, replay interpolation, and the position-packet codec. The old interactive `tracker` executable and its `main.cpp` have been removed; the desktop build provides libraries and tests. To run just the replay suite after a build:

```bash
ctest --test-dir tracker/build -R replay_position --output-on-failure
```

Pointing uses WGS84 → ECEF → ENU. Azimuth is clockwise from true north and elevation is relative to the horizon. Invalid inputs and undefined directions, including the provisional 5 m proximity/horizontal-displacement thresholds, produce no motor target. Successful targets are absolute millidegree commands and remain subject to active motor travel limits.

## GPS receiver bench test

The [GPS capture firmware](../tools/gps_capture/main.cpp) tests the FLYWOO GOKU GM10 Nano V3.1 receiver separately. The current firmware receives GPS TX on **GP1 (physical pin 2), UART0, 115200 baud, 8N1**. Connect a common ground. The capture only reads from the receiver, so GPS RX is not used.

```bash
cmake -S tools/gps_capture -B build/gps-capture -G Ninja \
    -DPICO_BOARD=pico2_w -DPICO_SDK_PATH="$HOME/pico/pico-sdk"
cmake --build build/gps-capture --target gps_capture -j4
```

Flash `build/gps-capture/gps_capture.uf2` and open the USB serial monitor. Once valid GGA fixes arrive, it prints latitude, longitude, and ellipsoidal altitude. The sentence collector frames NMEA text; the GGA parser validates the checksum and fix, converts coordinates to decimal degrees, and adds geoid separation to the reported sea-level altitude. Flash `gs_motor.uf2` again to return to the motor firmware.

## Radio implementation and packet policy

[radio_link.cpp](src/radio_link.cpp) owns the shared radio hardware and bounded receive service; [radio_link.hpp](include/radio_link.hpp) exposes the interface. The modules are **RFM69HCW using FSK**, configured at **915 MHz and 10 dBm on the HCW high-power path** through RadioLib 7.8.1. That release's configuration frequency is in MHz.

Both boards use SPI0 with MISO GP16, CS GP17, SCK GP18, MOSI GP19, DIO0 GP20, and RESET GP21. GPS TX connects to UART0 RX on GP1 (physical pin 2), at 115200 baud, 8N1, without hardware flow control. Radio power is 3V3 OUT with common ground. GPIO numbers differ from physical header numbers.

Preserve `RadioPicoHal`'s SPI timing guard: wait until SPI0 is no longer busy before releasing CS, hold for 2 us before raising CS, and delay 2 us after either CS transition. It fixed FIFO corruption on the bench. The default SPI clock is 500 kHz; synchronous extraction/restart work still needs motor-loop timing measurement. Initialization or receive-restart failure disables reception and leaves the motor console running.

The encoder/decoder in [position_packet.cpp](src/position_packet.cpp) are implemented and tested. The payload is exactly 22 bytes, serialized field by field:

| Bytes | Field | Encoding |
| --- | --- | --- |
| 0 | Version | `1` |
| 1 | Flags | Valid `0x01`; optional barometric altitude `0x02` |
| 2–5 | Sequence | Unsigned 32-bit, big-endian |
| 6–9 | Timestamp | Unsigned 32-bit transmitter uptime in milliseconds when the fix was accepted |
| 10–13 | Latitude | Signed 32-bit decimal degrees × 10⁷, big-endian |
| 14–17 | Longitude | Signed 32-bit decimal degrees × 10⁷, big-endian |
| 18–21 | Altitude | Signed 32-bit ellipsoidal meters × 100, big-endian |

The valid flag is required; unknown flags are rejected. The altitude flag does not change units or reference, and no barometer is integrated. Raw C++ struct memory is never transmitted. GGA fixes use ellipsoidal height = MSL altitude + signed geoid separation; simulation samples use the same conversion through `replayPositionAt()`.

The ground station requires both sequence and fix-acceptance timestamp to advance. It accepts sequence gaps, rejects duplicates/backward values, and uses unsigned modular comparisons for normal counter wrap; jumps of half the 32-bit range or more are rejected. Each pending position is consumed once and discarded if it waits more than **500 ms after decoding** before target submission. Only an accepted motor target refreshes the **1500 ms watchdog**. Malformed packets, ordering failures, and undefined pointing do not keep tracking alive.

The 500 ms check measures receiver-side pending time, not GPS fix age or radio latency. Packet timestamps are transmitter uptime, not GPS UTC, and the board clocks are not synchronized. Do not subtract them to claim absolute age. Version 1 has no boot/session identifier: after a known transmitter reboot, explicitly reset ordering with ground `TRACK OFF` before re-arming. Silence alone never resets ordering or re-arms tracking.

`sent=1` in transmitter diagnostics confirms local transmission completion, not a receiver acknowledgment. Software builds and desktop tests pass; physical tracking, range, end-to-end latency, and the reliable accepted-target rate still require measurement on the assembled system.

## Quick troubleshooting

- **`SDK location was not specified`**: configure with `-DPICO_SDK_PATH` pointing to the installed SDK.
- **Serial commands get no response**: open the correct board's USB monitor and enable LF or CRLF. Both applications wait for USB at startup. Transmitter replay commands are uppercase `START` and `STOP`; ground commands include their numeric ID.
- **`RX,ON` but no movement or `tgt` stays unchanged**: reception can run while tracking is off. Check the `TRK` arm state, selected target owner, and station/pointing validity. `ok` counts positions, not motor targets or measured movement.
- **Tracking stops after about 1.5 seconds**: the watchdog saw no accepted target. For radio replay use ground `TRACK ON` and the transmitting board; for local replay use ground `TRACK REPLAY`. Check reception, ordering, source ownership, and valid pointing geometry.
- **`START` is ignored or refused**: send it to the transmitter with a newline. Wait for one successful held-position transmission. A finished/stopped replay requires a transmitter reboot; reset ground ordering with `TRACK OFF` afterward.
- **Packets are rejected after a transmitter reboot**: disarm, wait for rest, and send `TRACK OFF` once the rebooted transmitter is running, then explicitly re-arm. Sequence/timestamp rollback is not an automatic session reset.
- **`RX,OFF` with nonzero `e`**: radio initialization or receive restart failed. Check radio wiring and restart the board after fixing the cause; firmware does not repeatedly initialize the radio in the motion loop.
- **`ERR,<id>,station,unset or invalid`**: live GPS mode needs a verified finite station position in `radio_config.hpp`. Do not use the synthetic replay station for deployment.
- **`ERR,<id>,replay`**: `replay_config.hpp` is missing the station position or the geoid separation. Set both and rebuild.
- **The editor flags includes or `std::optional`**: check that you opened `Avionics-30K-Year`, that the editor uses C++17, and that the compile database matches the build (`tracker/build/compile_commands.json` for the desktop, `build/pico-mac/compile_commands.json` for the motor firmware). Reload the VS Code window if the diagnostics are stale.
