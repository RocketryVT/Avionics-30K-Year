# Rocket Antenna Tracker

The tracker turns a rocket GPS fix and a ground-station reference into azimuth and elevation. The ground-station Pico 2 W runs the pointing calculation and motor controller in the same `gs_motor` firmware. The laptop configures the bench test and records its output.

Run the shell commands below from the **`Avionics-30K-Year` repository root**, where `tracker/` and `motor/` are siblings. The older standalone tracker checkout does not contain this integrated build.

## Build and flash the firmware on macOS

There is one firmware, `gs_motor`. The simulated flight is always built in and only runs when you send `TRACK,<id>,REPLAY`, so the same image works for the bench test and for normal motor use.

If `build/pico-mac` is already configured, this is the rebuild command after changing source or replay settings:

```bash
cmake --build build/pico-mac --target gs_motor -j4
```

For a new build directory, install CMake (3.18 or newer), Ninja, the Arm GNU toolchain (`arm-none-eabi-g++`), and the Pico SDK with its submodules. Configure once, substituting your SDK path if needed:

```bash
cmake -S motor/firmware -B build/pico-mac -G Ninja \
    -DPICO_BOARD=pico2_w \
    -DPICO_SDK_PATH="$HOME/pico/pico-sdk" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/pico-mac --target gs_motor -j4
```

The image is **`build/pico-mac/gs_motor.uf2`**. Hold **BOOTSEL** while connecting the Pico 2 W over USB, then copy that file to the mounted drive. The drive disappears when the Pico reboots. Open the Pico's USB serial port in VS Code's Serial Monitor to see the startup banner.

The banner includes a `[motor] replay:` line that says whether the simulated flight is ready to play. The old `GS_ENABLE_REPLAY` option is gone. A build folder configured with it drops it the next time CMake runs.

## Run the replay

### From the window

Run `py -m gs.gui` from `motor/laptop`. Zero both axes, switch to mode **3 track**, choose **the Pico's built-in flight**, and press **ARM tracking**. The target/actual readout fills in from the `TRK` stream. **disarm**, **STOP**, and **E-STOP** all end it early. When the flight ends, the window notes that the Pico stopped tracking.

Mode 3 runs in `GEARED` mode, so the angles count the final gear pairs. The numbers below were taken in `DIRECT` mode from a serial monitor, as described next.

### From a serial monitor

For the **Pico-only bench test**, leave `DIRECT` gear mode active. In the serial monitor, set the line ending to **LF** or **CRLF** and send these one line at a time:

```text
SETTINGS,1
LIMITS,2
ZERO,3,ALL,0
```

`SETTINGS` and `LIMITS` show the active gear mode, travel limits, and controller settings. After `ACK,3,ZERO`, start the replay:

```text
TRACK,4,REPLAY
```

An accepted `TRACK REPLAY` arms tracking and starts the replay at simulation time zero. Its reply is the same `TRK,4,ON,...` that `TRACK ON` gives. The Pico interpolates the trajectory and submits a new position every **200 ms (5 Hz)**. Keep the serial monitor open to capture the `TRK` lines. To stop early, send `TRACK,5,OFF`. `STOP`, `ESTOP`, `TRACK ON`, or a `TARGET` from the laptop also ends it. If replay is already running, stop it and wait for the axes to come to rest before zeroing and starting again.

`TRACK ON` never starts the replay. It arms tracking for targets sent by the laptop, so arming for a real launch cannot play the simulated flight. From the `py -m gs.motor` prompt, `track replay` does the same as `TRACK,<id>,REPLAY`.

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

Rebuild and flash after changing the configuration or table. If you replace the trajectory, update the dataset-specific expectations in [test_replay_position.cpp](tests/cases/test_replay_position.cpp).

## Build and test on the Mac

Use CMake and a C++17 compiler, such as Clang from the macOS Command Line Tools. The desktop build does not require the Pico SDK.

```bash
cmake -S tracker -B tracker/build \
    -DTRACKER_BUILD_DESKTOP=ON -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build tracker/build -j4
ctest --test-dir tracker/build --output-on-failure
```

The seven suites cover coordinate conversion, pointing, target angular velocity, the motor adapter, GPS parsing, sentence framing, and replay interpolation. To run just the replay suite after a build:

```bash
ctest --test-dir tracker/build -R replay_position --output-on-failure
```

To try the interactive pointing calculator:

```bash
./tracker/build/tracker
```

Enter the station coordinates first, then the rocket coordinates. Latitude and longitude are decimal degrees; altitudes are meters above the WGS84 ellipsoid. The result is azimuth clockwise from true north and elevation relative to the horizon. The calculation uses WGS84 → ECEF → ENU and rejects invalid inputs and undefined pointing directions, including nearly coincident or nearly vertical positions.

## GPS receiver bench test

The [GPS capture firmware](../tools/gps_capture/main.cpp) tests the FLYWOO GOKU GM10 Nano V3.1 receiver separately. The current firmware receives GPS TX on **GP1 (physical pin 2), UART0, 115200 baud, 8N1**. Connect a common ground. The capture only reads from the receiver, so GPS RX is not used.

```bash
cmake -S tools/gps_capture -B build/gps-capture -G Ninja \
    -DPICO_BOARD=pico2_w -DPICO_SDK_PATH="$HOME/pico/pico-sdk"
cmake --build build/gps-capture --target gps_capture -j4
```

Flash `build/gps-capture/gps_capture.uf2` and open the USB serial monitor. Once valid GGA fixes arrive, it prints latitude, longitude, and ellipsoidal altitude. The sentence collector frames NMEA text; the GGA parser validates the checksum and fix, converts coordinates to decimal degrees, and adds geoid separation to the reported sea-level altitude. Flash `gs_motor.uf2` again to return to the motor firmware.

## Radio integration status

The live GPS parser and the replay-to-pointing-to-motor path have been tested separately. The next piece is the radio payload. [position_packet.hpp](include/position_packet.hpp) defines a 22-byte format with a version, flags, sequence number, transmitter timestamp, coordinates, and ellipsoidal altitude. [position_packet.cpp](src/position_packet.cpp) has packing helpers; the encoder and decoder still need implementation and tests.

Finish the codec on the desktop, then send and receive known packets between the two radio-equipped boards. The pictured [RFM69HCW modules](https://secure.hoperf.com/modules/rf_transceiver/RFM69HCW.html) support FSK. Confirm the two modules' bands and matching radio settings before testing over the air. On the ground Pico, pass validated decoded positions to the same pointing and motor path used by replay. Use the receiver's clock to check time since the last usable packet.

## Quick troubleshooting

- **`SDK location was not specified`**: configure with `-DPICO_SDK_PATH` pointing to the installed SDK.
- **Serial commands get no response**: enable LF or CRLF. Send a blank line to flush buffered text, then resend the full command. `ZERO` alone is not the raw serial command; use `ZERO,3,ALL,0` for the Pico-only test.
- **Tracking arms and stops after about 1.5 seconds**: check that you sent `TRACK,<id>,REPLAY` and not `TRACK,<id>,ON`, which waits for targets from the laptop. If it still stops, check that the geometry produces valid pointing angles.
- **`ERR,<id>,replay`**: `replay_config.hpp` is missing the station position or the geoid separation. Set both and rebuild.
- **The editor flags includes or `std::optional`**: check that you opened `Avionics-30K-Year`, that the editor uses C++17, and that the compile database matches the build (`tracker/build/compile_commands.json` for the desktop, `build/pico-mac/compile_commands.json` for the motor firmware). Reload the VS Code window if the diagnostics are stale.
