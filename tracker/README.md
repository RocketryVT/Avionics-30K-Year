# Rocket Antenna Tracker

GPS-based antenna pointing system written in C++.

## Hardware
- Raspberry Pi Pico
- FLYWOO GOKU GM10 Nano V3.1 GPS
- LoRa communication module (TBD)
- Azimuth and elevation motors (TBD)

## Project Goals
1. Research GPS hardware and data protocols
2. Implement pointing mathematics
3. Validate azimuth and elevation calculations
4. Integrate GPS and LoRa communication
5. Implement motor control

## Development
The pointing algorithm is developed and tested
on a Mac before deployment to the Pico.

## Building and Running

### Prerequisites

- C++17 or newer
- Clang C++ compiler (included with macOS Command Line Tools)

### Interactive Pointing Demonstration

The antenna tracker includes a command-line demonstration that calculates the azimuth and elevation required to point an antenna from a ground station toward a rocket.

The demonstration accepts two sets of GPS coordinates:

- Ground station latitude, longitude, and altitude
- Rocket latitude, longitude, and altitude

Latitude and longitude must be provided in decimal degrees. Altitudes must be in meters above the WGS84 ellipsoid.

#### Build

From the repository root:

```bash
mkdir -p build
clang++ -std=c++17 -Wall -Wextra -Iinclude \
    src/main.cpp src/pointing.cpp src/coordinate_utils.cpp \
    -o build/tracker
```

#### Run

```bash
./build/tracker
```

Enter the requested GPS coordinates when prompted.

The program displays the calculated azimuth and elevation in degrees.

#### Input Validation

The program rejects:

- Latitude outside [-90°, 90°]
- Longitude outside [-180°, 180°]
- Non-numeric or non-finite input
- Undefined pointing directions, including coincident or nearly vertically aligned positions under the current provisional tolerances

No physical GPS, LoRa, or motor hardware is required to run the demonstration.

See the **Unit Testing** section for automated testing instructions.

### Project Structure

```text
rocket-antenna-tracker/
├── include/
│   ├── gps_position.hpp
│   └── pointing.hpp
├── src/
│   ├── main.cpp
│   └── pointing.cpp
├── build/
└── README.md
```

The `include/` directory contains header files, while `src/` contains C++ implementations. The `build/` directory contains generated executables and is excluded from Git using `.gitignore`.

## Unit Testing

The project includes automated unit tests to verify the accuracy and reliability of the antenna pointing algorithm.

Tests are written in C++17 and can be executed locally without requiring Raspberry Pi Pico hardware or external testing frameworks.

### Test Coverage

The unit tests validate the following components:

- **GPS to ECEF Conversion:** Tests coordinate transformations at different latitudes, longitudes, and altitudes using WGS84 reference values.
- **ECEF to ENU Conversion:** Verifies relative East, North, and Up displacement calculations.
- **Pointing Angle Calculations:** Tests azimuth and elevation for known geographic positions and directions.
- **Input Validation:** Verifies that invalid GPS coordinates, non-finite values, and undefined pointing directions return `std::nullopt`.

Floating-point results are compared against expected values using specified numerical tolerances.

### Building the Tests

From the project's root directory, compile the test executable:

```bash
mkdir -p build

clang++ -std=c++17 -Wall -Wextra -Iinclude \
    tests/test_pointing.cpp \
    src/coordinate_utils.cpp \
    src/pointing.cpp \
    -o build/test_pointing
```

### Running the Tests

Execute the compiled test program:

```bash
./build/test_pointing
```

Each test reports whether it passed or failed. The program returns an exit code of `0` when all tests pass and a nonzero exit code when any test fails.

### Build and Run Tests Together

```bash
clang++ -std=c++17 -Wall -Wextra -Iinclude \
    tests/test_pointing.cpp src/coordinate_utils.cpp src/pointing.cpp \
    -o build/test_pointing && ./build/test_pointing
```

### Notes

- Tests are located in `tests/test_pointing.cpp`.
- Tests use independently established reference coordinates and expected values.
- The pointing algorithm currently uses provisional 5-meter tolerances for coincident and nearly vertical positions.
- Unit tests should be rerun whenever changes are made to coordinate conversions, pointing calculations, or input validation.