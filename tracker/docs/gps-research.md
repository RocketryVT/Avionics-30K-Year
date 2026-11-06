# GPS Hardware, UART, and Parsing Integration Guide

**Project:** Rocket Antenna Tracker  
**Receiver under investigation:** FLYWOO GOKU GM10 Nano V3.1  
**Software:** C++17-compatible parser and Raspberry Pi Pico SDK firmware  
**Document status:** Research and implementation plan; **hardware wiring and output format NOT YET verified on the actual unit**  
**Last reviewed:** 2026-10-18

> **Before powering or wiring hardware:** Confirm the *exact* board, the markings/order of the GPS pins, the GPS TX logic-high voltage, the power supply/current budget, and the selected Pico UART pins. The wiring below is an **example for a standard Pico/Pico 2 breakout**, not a verified schematic for a custom ground-station PCB.

## 1. Purpose and scope

This document covers the steps from receiving satellite-navigation data to obtaining a validated `GPSPosition` for the existing antenna pointing algorithm:

```text
Satellites → FLYWOO GNSS module → UART bytes on Pico
                               → message framing
                               → NMEA/other protocol parser
                               → validated GPSPosition
                               → calculatePointingAngles(rocket, station)
```

The GNSS module itself processes satellite radio signals and estimates location. **We do not decode satellite signals.** We receive the module's *output messages* and parse them.

The parser should be hardware-independent and testable on a Mac. Pico-specific code handles UART and delivers data bytes to the parser. If the GPS is attached to the **rocket**, parsing normally occurs on the rocket before a compact position packet is sent over LoRa. The ground station then decodes LoRa telemetry rather than the original NMEA sentence. **Confirm which processor actually has the FLYWOO module before integrating.**

## 2. Hardware: confirmed manufacturer specifications

The [FLYWOO product page][flywoo] for the **GOKU GM10 Nano V3.1** lists:

| Item | Manufacturer's published specification | Integration implication |
|---|---|---|
| GNSS chipset | M10050 | Identify matching vendor/protocol documentation if available |
| Supported constellations | GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS; default enabled group listed as GPS/Galileo/BeiDou | Satellite constellations are **not** the UART message format |
| Electrical supply (VCC) | 3.3–5 V DC | Pick and verify a suitable regulated supply |
| Serial interface | RX, TX; lists TTL or RS-232 with *default TTL level* | Measure/confirm **actual pin voltage levels** before Pico hookup |
| Supported baud rates | 4,800–921,600 bps | Start by testing 115,200 bps |
| Default baud rate | **115,200 bps** | Both UART ends must use matching settings |
| Output/update rate | 1–10 Hz, **default 10 Hz** | Position updates may come every ~100 ms; multiple sentences may be output per epoch |
| Listed data/protocol compatibility | `BF4.3.X(UBLOX)/BF4.2.X(NMEA)` | This is **not proof** that the live serial stream is UBX; capture bytes |
| Operating limit: altitude | 50,000 m maximum | Check launch trajectory envelope against limit |
| Operating limit: velocity | 500 m/s maximum | Check launch trajectory envelope against limit |
| Operating limit: acceleration | Less than 4 g | Rocket boost may exceed this: do not assume fixes are valid in all flight phases |
| Temperature range | -40 °C to +85 °C operating | Verify against environment |
| Position-fix indicator | Yellow PPS LED: off before fix, blinking after fix | Helpful indication, **not** a substitute for software validity checks |
| Connector signals | VCC, GND, TX, RX; 4-pad interface | Confirm the order from the **specific physical board** |

**Important uncertainty:** The manufacturer lists a supply range, but does **not clearly establish that TX outputs at 3.3 V under every supply condition** on the page reviewed. Pico GPIOs must not be exposed to 5 V or true ±RS-232 signaling. The label "TTL" alone does not prove safe voltage compatibility. Use a schematic, datasheet, or meter/oscilloscope/logic analyzer to verify; add a level shifter if required. Manufacturer power consumption/current was not established from the reviewed page, so verify the Pico 3.3 V rail can supply the module and other peripherals.

### Questions to resolve before hardware hookup

- [ ] Exactly which Pico/PCB is used (Pico, Pico 2, or custom team board)?
- [ ] Is this GPS mounted on the rocket, ground station, or both?
- [ ] What is the **physical pad order / cable pin numbering** on this specific GM10 unit?
- [ ] What is the **TX idle-high voltage** at the proposed VCC?
- [ ] Can the chosen supply reliably power the GPS (including peak current)?
- [ ] Which MCU UART, GPIOs, and ground connection are free on the actual PCB?
- [ ] Which protocol and messages does the GPS **actually output** at power-up?
- [ ] Are baud rate and update frequency still set to manufacturer defaults?

## 3. Connecting GPS to a Pico: wiring plan

### 3.1 Standard Pico / Pico 2 **example**, not a final team PCB pinout

For a *standard 40-pin Pico-series board* using **UART0** on GP0/GP1:

| GPS module signal | Pico signal | Standard Pico physical pin | Notes |
|---|---|---:|---|
| **TX** (data **out** of GPS) | **GP1 / UART0 RX** | 2 | Main required data path; confirm GPS TX logic voltage safe for 3.3 V GPIO |
| **RX** (data **into** GPS) | **GP0 / UART0 TX** | 1 | Optional for receiver configuration; not needed for receive-only capture |
| **GND** | **GND** | 3 (example) | Common electrical reference required |
| **VCC** | **3V3(OUT)**, **only if verified suitable** | 36 | Supply GPS with verified voltage/current budget; alternatively regulated external supply with common GND |

**Do not connect GPS VCC to Pico GPIO; do not connect GPS TX to 5 V-tolerant assumptions.** Keep USB serial/debug independent from this hardware UART (disable UART `stdio` if it uses the same port). On **custom ground station boards**, GP0/GP1 may not be available, the physical connector may differ, or power may be provided elsewhere. Confirm board schematics before using this table.

### 3.2 Pre-power checklist

1. Disconnect all power and inspect GPS pad silkscreen / cable labels against the product's pinout image.
2. Confirm GPS VCC and GND with a continuity or wiring check; ensure no swapped conductors or shorts.
3. Check planned VCC, logic TX voltage, and current limits. If output is 5 V or RS-232, use appropriate conversion hardware.
4. Confirm Pico GPIO pin selection and UART multiplexing, avoiding other peripherals and debug UART conflicts.
5. Connect shared GND, power, and GPS TX → Pico RX; add Pico TX → GPS RX only if configuration commands are needed.
6. Power the system; check supply voltage, GPS LED, and startup current/temperature. Turn off immediately if abnormal.
7. Place the antenna with a clear sky view to acquire a fix (initial fix can take time); continue validating fix quality in software.

### 3.3 UART settings

Start with the device's published default and verify by reading the actual stream:

- Baud: **115200**
- Bits: **8 data, no parity, 1 stop bit** (`8N1`; confirm actual unit)
- Hardware flow control: **off** (four-wire module lacks RTS/CTS)
- Both devices: share GND and use matching baud/framing
- The GPS TX → Pico RX path transmits **bytes**, not complete C++ objects

At 115200 baud using 8N1 (~10 serial bits per byte), the link can carry roughly 11,520 bytes/s at most. **10 Hz does not mean 10 sentences/s**: each position update may contain several sentences. Avoid blocking logging or expensive work in the receive loop.

## 4. Pico SDK: first UART data capture

Read the official [Pico UART example][pico-uart] and [Pico SDK UART API][pico-uart-api]. **Only build this on a configured Pico SDK project, not with host `clang++`.** The following is a *bring-up example* for text NMEA; it assumes a **standard Pico breakout**, verified safe electrical connection, **UART0 on GP1 (RX)**, and USB `stdio` for logging:

```cpp
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/uart.h"

namespace {
constexpr uint GPS_RX_PIN = 1;   // GP1: UART0 RX, standard Pico example
constexpr uint GPS_BAUD = 115200;
constexpr size_t LINE_CAPACITY = 256;
}

int main() {
    stdio_init_all();             // Route debug output to USB using CMake settings
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart0, GPS_BAUD);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart0, false, false);

    char line[LINE_CAPACITY] = {};
    size_t length = 0;
    bool discarding = false;

    while (true) {
        while (uart_is_readable(uart0)) {
            char c = static_cast<char>(uart_getc(uart0));
            if (c == '\n') {
                if (!discarding && length != 0) {
                    line[length] = '\0';
                    printf("GPS: %s\n", line);  // Debug only, may block/lose bytes
                }
                length = 0;
                discarding = false;
            } else if (c != '\r' && !discarding) {
                if (length + 1 < LINE_CAPACITY) {
                    line[length++] = c;
                } else {
                    length = 0;
                    discarding = true;      // Ignore oversized line until newline
                }
            }
        }
        tight_loop_contents();
    }
}
```

Example *target* CMake settings (insert into the Pico project's CMake configuration; `gps_capture` is the example executable target):

```cmake
add_executable(gps_capture firmware/gps_capture.cpp)
target_link_libraries(gps_capture PRIVATE pico_stdlib hardware_uart)
pico_enable_stdio_usb(gps_capture 1)
pico_enable_stdio_uart(gps_capture 0)  # Don't use UART0 for debug prints
pico_add_extra_outputs(gps_capture)
```

**Limits of this bring-up sketch:** This prints whole text lines for inspection only, assumes NMEA-like ASCII and newline delimiters, and may drop serial bytes if USB output stalls. It is **not production receive code**. For regular operation, buffer in a bounded UART receive queue/ring buffer (or IRQ/DMA path), parse outside interrupts, avoid per-byte USB printing, and instrument buffer overflows. If bytes are binary rather than `$...` ASCII, **capture raw bytes/hex** and identify the protocol instead of trying to force the line parser.

The Pico SDK also has an [official `gps_uart` example in `pico-examples`][pico-examples]; consult it when implementing a real receiver.

## 5. Identify the GPS message protocol **before coding the parser**

1. Power the verified module and capture the **raw byte stream** at the actual UART baud.
2. If readable ASCII sentences begin with `$GP...`, `$GN...`, or similar and end in CR/LF, treat them as **NMEA 0183** candidates.
3. If you observe binary packets beginning with bytes `B5 62`, investigate **UBX** framing and its exact message definitions. Do not assume the module implements u-blox commands merely because the FLYWOO page mentions Betaflight "UBLOX" support.
4. Mixed output is possible. Log the observed sentence/message IDs and the message rates.
5. Save several **complete raw messages**, including before and after a position fix, in `tests/data/` with a README recording firmware settings and capture date.
6. Implement only the confirmed messages required by this project; label unsupported protocols clearly.

For UBX-specific framing, refer to the [u-blox UBX protocol reference][ubx-spec] **only after confirming the actual device uses a compatible binary protocol**. A UBX-like packet starts with `0xB5 0x62` and includes class, ID, little-endian length, payload and two checksum bytes; do not mix this decoder with an NMEA parser.

## 6. NMEA: what the parser must understand

### 6.1 Sentence shape and checksum

Typical ASCII sentence:

```text
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n
```

- `$` begins a sentence; `GP` or `GN` is a **talker ID**; `GGA` is the **sentence type**. Do not assume every valid GGA begins with `$GPGGA`: `$GNGGA` is also used.
- Commas separate positional fields; **empty fields are significant**. Never discard empty fields while splitting.
- `*47` contains an example hexadecimal checksum. Calculate **XOR of all bytes between `$` and `*`**, excluding both markers; compare with the two hex digits (case-insensitive).
- The normal wire terminator is `\r\n`. The framing layer should accept an entire sentence across any number of UART reads.
- Require a bounded maximum sentence length; do not overflow a fixed array on malformed input.

### 6.2 GGA fields (GPS fix and altitude)

For a comma-separated GGA message, the following indexes assume **field 0 is `GPGGA` / `GNGGA`, after removing `$` and the `*hh` suffix**:

| Field index | Sample | Meaning | Use |
|---:|---|---|---|
| 0 | GPGGA | Talker + GGA message ID | Validate sentence type |
| 1 | 123519 | UTC time `hhmmss[.sss]` | Optional metadata / fix association (no date in GGA) |
| 2 | 4807.038 | Latitude `ddmm.mmmm` | Convert to decimal degrees |
| 3 | N | Latitude hemisphere N/S | Apply sign |
| 4 | 01131.000 | Longitude `dddmm.mmmm` | Convert to decimal degrees |
| 5 | E | Longitude hemisphere E/W | Apply sign |
| 6 | 1 | Fix quality (`0` means invalid) | Reject invalid fixes |
| 7 | 08 | Satellites used | Record for diagnostics |
| 8 | 0.9 | HDOP (dimensionless) | Optional quality indicator |
| 9 | 545.4 | Altitude above mean sea level | **Not** ellipsoidal altitude |
| 10 | M | Altitude units | Require meters if assumed |
| 11 | 46.9 | Geoid separation / undulation (meters) | Needed to derive ellipsoidal height |
| 12 | M | Geoid separation units | Require meters if assumed |
| 13 | empty | Age of differential GPS data | Not needed for initial position |
| 14 | empty | Differential reference station ID | Not needed for initial position |

A simple implementation can accept known valid fixes (for example quality `1` or `2`) and explicitly configure which other quality types are acceptable. Different receivers define enhanced quality codes differently. Reject `0` and, unless there is a reason to support them, simulated/manual/estimated fixes.

**Never accept altitude from GGA without also checking that the geoid separation is present and both unit fields are `M`, if the parser promises WGS84 ellipsoidal height.** If the field is unavailable, report incomplete altitude rather than silently equating MSL height to ellipsoidal height.

### 6.3 Latitude and longitude conversion

NMEA stores coordinates in degrees and **decimal minutes**, not decimal degrees:

```text
Latitude:  4807.038,N  → 48° + (7.038 / 60)°  = +48.117300°
Longitude: 01131.000,E → 11° + (31.000 / 60)° = +11.5166667°
```

Formula for nonnegative raw `ddmm.mmmm` or `dddmm.mmmm`:

```text
degrees = floor(raw / 100)
minutes = raw - degrees * 100
decimal_degrees = degrees + minutes / 60
if hemisphere == S or W: decimal_degrees = -decimal_degrees
```

Use **2 degree digits for latitude and 3 for longitude** when validating textual layout; verify minutes are `[0,60)` and final latitude `[-90,90]`, longitude `[-180,180]`. Reject unexpected hemisphere letters, malformed numeric fields, and non-finite values. Leading zeros are significant to the *text format*, not to the numeric decimal-degree result.

### 6.4 Altitude conversion for the existing ECEF math

`GPSPosition::altitude` is documented as **meters above the WGS84 ellipsoid**. GGA typically supplies orthometric (mean-sea-level) altitude `H` and geoid separation `N`:

```text
h_ellipsoid = H_MSL + N_geoid
Example: 545.4 m + 46.9 m = 592.3 m
```

Do **not** mix a rocket altitude above the ellipsoid with a station altitude above mean sea level. Use the same reference for both positions before invoking `gpsToECEF()`. Check the actual receiver's definition of geoid separation and output datum when implementing this conversion.

### 6.5 RMC, if needed later

RMC commonly contains UTC time, validity status (`A` = data valid, `V` = warning/void), latitude/longitude, speed over ground (knots), course, and date. It normally **does not provide altitude**, so it is not a substitute for GGA when constructing the full `GPSPosition`. If combining RMC and GGA, associate sentences from the same fix epoch and handle missing or delayed messages.

### 6.6 Example messages and expected results

These are **synthetic / standard reference examples**, with verified NMEA XOR checksums, **not captures from the FLYWOO module**:

```text
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
$GNGGA,123520,3713.8000,N,08025.2000,W,1,10,0.8,600.0,M,-30.0,M,,*6B
$GNGGA,123521,3713.8000,N,08025.2000,W,0,00,99.9,600.0,M,-30.0,M,,*5B
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
```

| Sentence | Expected interpretation |
|---|---|
| GPGGA `*47` | Valid fix; latitude `+48.117300`, longitude `+11.5166667`, ellipsoidal height `592.3 m` |
| GNGGA `*6B` | Valid fix; latitude `+37.230000`, longitude `-80.420000`, ellipsoidal height `570.0 m` |
| GNGGA `*5B` | **Reject** as a usable position: fix quality `0`, even though checksum is valid |
| GPRMC `*6A` | Optional RMC test, not enough to provide altitude |

A parser should also reject a one-character modification of a sentence without a recomputed checksum.

## 7. Recommended software design

Keep the responsibilities separate:

```text
include/gps_position.hpp          → shared coordinate struct
include/gps_parser.hpp            → parser API (no Pico SDK)
src/gps_parser.cpp                → checksum, fields, numeric and fix validation
include/gps_stream.hpp            → bounded byte-to-message framing API (optional)
src/gps_stream.cpp                → framing / re-synchronization (optional)
firmware/...                      → Pico UART reception and timeouts
include/coordinate_utils.hpp      → ECEF/ENU conversion (existing)
include/pointing.hpp              → azimuth/elevation (existing)
tests/test_gps_parser.cpp         → desktop parser tests
tests/data/                      → recorded and synthetic fixtures
```

Minimal shared structure already in the project:

```cpp
struct GPSPosition {
    double latitude;   // decimal degrees
    double longitude;  // decimal degrees
    double altitude;   // meters above WGS84 ellipsoid
};
```

Example **proposed API** for the first GGA-only version (not implemented by this document):

```cpp
#include <optional>
#include <string_view>
#include "gps_position.hpp"

// Accept one complete GGA sentence with its checksum.
// Return std::nullopt if malformed, checksum-invalid, no fix,
// or lacking the required height-reference fields.
std::optional<GPSPosition> parseGGA(std::string_view sentence);
```

For production telemetry, prefer also returning/recording `fix_quality`, UTC time, satellite count, HDOP and a **monotonic receiver timestamp** (timestamp captured by the Pico on reception) to detect stale fixes. `std::nullopt` alone does not distinguish different failure reasons; a richer status result may be useful for logging and debugging.

### Parser steps in order

1. **Frame:** collect bounded bytes until a full recognized message arrives; reset safely on overflow and re-sync on next start marker.
2. **Validate envelope:** correct `$...*hh` layout, checksum, sentence type and sufficient comma-delimited fields (including empties).
3. **Validate fix:** known acceptable fix quality; don't use invalid, simulated or stale positions as tracking inputs.
4. **Parse fields:** strict numeric parsing, hemisphere flags, degree/minute bounds, altitude/geoid fields and units.
5. **Convert:** decimal degrees and ellipsoidal height in meters; reject NaN/infinity.
6. **Publish:** one validated fix plus timestamp/quality metadata. Do not mix fields from different fix epochs.
7. **Expire:** mark a fix stale after a **configurable, tested timeout**, independent of whether a new bad sentence arrived. No new motor command from stale position.

Use a **bounded buffer**, not unlimited string growth, on the microcontroller. A preliminary 256-byte line limit is a pragmatic starting *implementation choice* for text messages, **not a confirmed maximum for this receiver**. Instrument oversized messages and adjust based on captured data. Keep slow printing, allocations and heavier parsing out of time-critical UART interrupt handlers.

## 8. Test and bring-up plan

### Desktop parser tests (before connecting hardware)

- [ ] Accept both `$GPGGA` and `$GNGGA` with valid checksums.
- [ ] Parse known N/S and E/W coordinate cases, including negative longitude.
- [ ] Check degrees/minutes, final coordinate bounds, and minutes `<60`.
- [ ] Check altitude `H + N` including negative geoid separation.
- [ ] Reject fix quality `0` and other disallowed qualities.
- [ ] Reject checksum mismatch, truncated message, missing `$`/`*`, missing required fields and invalid units.
- [ ] Reject NaN/infinity, extra unexpected numeric characters, invalid hemisphere letters.
- [ ] Stream a sentence one byte at a time, in randomly sized chunks, and with noise between sentences.
- [ ] Exercise bounded buffer overflow, truncation, resynchronization, and multiple consecutive messages.
- [ ] Test stale-fix timeout and that no stale position triggers a new pointing command.

### Hardware validation tests

- [ ] Record actual VCC and UART TX levels and confirm Pico compatibility.
- [ ] Capture raw data at 115200 8N1 (or determine the actual configured speed).
- [ ] Confirm the output protocol, message types, and real update/sentence frequencies.
- [ ] Obtain a real outdoor fix; record samples before and after acquisition.
- [ ] Confirm checksum, fix quality, coordinate plausibility, and altitude/geoid field availability.
- [ ] Count framing errors, invalid fixes, and UART buffer overflows over an extended run.
- [ ] Confirm Pico-received fixes match desktop parsing of the same raw samples.
- [ ] Test timeout/invalid-data behavior with the GPS disconnected or reception obstructed.
- [ ] Verify the code's placement: rocket-side GPS parsing versus ground-station LoRa decoding.
- [ ] Check operational limitations against the actual rocket mission; plan behavior when GNSS may fail during acceleration.

**Definition of done:** A team member can reproduce the wiring, collect raw GPS output, understand its protocol, run parser unit tests, and obtain a validated, properly referenced `GPSPosition` on the target MCU. Physical wiring and runtime tests must be signed off after the actual hardware is identified.

## 9. Troubleshooting quick reference

| Symptom | Common causes / next checks |
|---|---|
| No bytes | Wrong TX→RX mapping, no shared GND, no module power, wrong UART/pin mux, wrong UART peripheral |
| Garbage bytes | Baud mismatch, incorrect framing/logic levels, real RS-232 vs TTL, signal integrity |
| Bytes but no `$...` text | Binary protocol or wrong baud; capture in hex, do not force NMEA parsing |
| Sentences but no fix | Poor sky view, initial acquisition, invalid GGA fix quality; wait/test outdoors |
| Valid checksum but null position | No fix, unsupported sentence, missing geoid separation, invalid fields |
| Latitude/longitude incorrect | NMEA `ddmm.mmmm` conversion, `N/S/E/W` sign, wrong index, lost empty fields |
| Altitude differs from maps | MSL vs ellipsoid, different vertical datums, live GPS noise |
| Intermittent corrupt sentences | UART overruns from blocking USB logs, insufficient buffer, power/signal problems |
| Angles wrong despite GPS fix | Coordinate reference mismatch, station/rocket positions swapped, old telemetry |

## 10. External technical references

1. **FLYWOO**, [GOKU GM10 Nano V3.1 GPS manufacturer product/specifications][flywoo]. Hardware ratings, pins, baud/update defaults and listed protocol compatibility.
2. **Raspberry Pi**, [Pico-series boards, pinouts and supply pins][pico-board]. Verify actual physical pinout before wiring.
3. **Raspberry Pi**, [Official Pico UART advanced example][pico-uart]. UART0, GP0/GP1, baud, 8N1, GPIO setup, hardware flow control.
4. **Raspberry Pi**, [Pico SDK hardware APIs][pico-uart-api]. `uart_init`, `uart_set_format`, `uart_is_readable`, `uart_getc`, GPIO configuration.
5. **Raspberry Pi**, [pico-examples list][pico-examples]. Contains `gps_uart` reference project.
6. **Trimble**, [NMEA GGA message field documentation][nmea-gga]. Field positions, quality and height conventions.
7. **gpsd**, [NMEA driver and field reference][gpsd-nmea]. GGA parsing and degree/minute interpretation.
8. **u-blox**, [UBX packet structure][ubx-spec]. *Only applicable if a compatible UBX output is confirmed*.

[flywoo]: https://flywoo.net/products/goku-gm10-nano-v3.1-gps
[pico-board]: https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html
[pico-uart]: https://github.com/raspberrypi/pico-examples/blob/master/uart/uart_advanced/uart_advanced.c
[pico-uart-api]: https://www.raspberrypi.com/documentation/pico-sdk/hardware.html
[pico-examples]: https://github.com/raspberrypi/pico-examples
[nmea-gga]: https://help.fieldsystems.trimble.com/r580/nmea0183-messages-gga.htm
[gpsd-nmea]: https://github.com/andersson/gpsd/blob/master/driver_nmea0183.c
[ubx-spec]: https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
