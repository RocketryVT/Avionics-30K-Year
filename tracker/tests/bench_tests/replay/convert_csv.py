import csv
import math
import sys
from pathlib import Path

if len(sys.argv) != 3:
    raise SystemExit("Usage: convert_csv.py INPUT.csv OUTPUT.hpp")

input_path = Path(sys.argv[1])
output_path = Path(sys.argv[2])

lines = input_path.read_text(encoding="utf-8-sig").splitlines()

# Check the column order and units before converting.
expected_header = (
    "# Time (s),Altitude above sea level (ft),"
    "Latitude (° N),Longitude (° E)"
)

if expected_header not in lines:
    raise ValueError("Unexpected CSV columns or units")

samples = []

# Skip blank lines and simulation comments/events.
data_lines = (
    line for line in lines
    if line.strip() and not line.lstrip().startswith("#")
)

for row in csv.reader(data_lines):
    if len(row) != 4:
        raise ValueError("Each sample must have four fields")

    time_s, altitude_ft, lat, lon = map(float, row)

    if not all(math.isfinite(v) for v in (time_s, altitude_ft, lat, lon)):
        raise ValueError("Non-finite sample")

    if time_s < 0 or (samples and time_s <= samples[-1][0]):
        raise ValueError("Timestamps must increase strictly")

    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError("Invalid latitude or longitude")

    altitude_m = altitude_ft * 0.3048
    samples.append((time_s, lat, lon, altitude_m))

if not samples:
    raise ValueError("No samples found")

header = """#pragma once

#include <array>

/** @brief One simulated position; altitude remains above mean sea level. */
struct ReplaySample
{
    double timeSeconds;       ///< Seconds since the simulation began.
    double latitude;          ///< Decimal degrees, north positive.
    double longitude;         ///< Decimal degrees, east positive.
    double altitudeMslMeters; ///< Meters above mean sea level.
};
"""

header += (
    f"\ninline constexpr std::array<ReplaySample, {len(samples)}> "
    "REPLAY_SAMPLES = {{\n"
)

for sample in samples:
    values = ", ".join(format(value, ".17g") for value in sample)
    header += "    {" + values + "},\n"

header += "}};\n"

output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(header, encoding="utf-8")

print(f"Wrote {len(samples)} samples to {output_path}")