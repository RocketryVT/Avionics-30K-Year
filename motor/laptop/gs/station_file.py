r"""Where the ground station is, kept on the laptop.

The ground Pico points the radio from a ground-station position. One is
built into its firmware (RADIO_LIVE_STATION in radio_config.hpp), but moving
it that way costs a rebuild and a reflash. So the window keeps the station
here instead, in `station.json` beside the `gs` package, and sends it with
STATION every time it connects to the Pico and every time it arms the radio.
The Pico only holds it until it restarts; sending it again on arming is what
makes a restart in the middle of a session harmless.

What is typed is what a GPS or a map gives:

    latitude, longitude   decimal degrees, north and east positive. A pair
                          pasted from a map ("37.19503, -80.39448") into the
                          latitude box is split into both.
    MSL                   height above mean sea level, metres
    geoid                 the geoid separation, metres: how far sea level
                          sits above the WGS84 ellipsoid. Negative across
                          North America - about -33 m around Blacksburg. A
                          GPS's GGA sentence carries it, next to MSL.

The Pico wants the station's height above the ellipsoid, because that is
what the rocket's GPS altitude is turned into before pointing:

    ellipsoid height = MSL + geoid

Getting the geoid wrong by its whole size (33 m) tilts the antenna by about
2 degrees at a kilometre out, so it is asked for rather than assumed.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

from gs import protocol as p

# motor/laptop/station.json: beside the package rather than wherever the
# window happened to be started from, so the station is the same one however
# it is opened. Ignored by git - it belongs to a site, not to the code.
DEFAULT_PATH = Path(__file__).resolve().parent.parent / "station.json"

# The geoid sits within about -107 m and +86 m of the ellipsoid everywhere on
# Earth. Anything well past that is the ellipsoid height or MSL typed into
# the wrong box.
GEOID_MAX_M = 120.0


@dataclass(frozen=True)
class SavedStation:
    """The station as typed: degrees, and metres above sea level plus the
    geoid separation. Rounded to what the wire carries when made by
    `parse`, so what is saved is exactly what the Pico is sent."""

    lat_deg: float
    lon_deg: float
    msl_m: float
    geoid_m: float

    @property
    def alt_m(self) -> float:
        """Height above the WGS84 ellipsoid: what STATION sends."""
        return round(self.msl_m + self.geoid_m, 3)

    def wire(self) -> tuple[int, int, int]:
        """As the Pico would report it: e7 degrees and millimetres."""
        return (p.deg_to_e7(self.lat_deg), p.deg_to_e7(self.lon_deg),
                p.deg_to_mdeg(self.alt_m))

    def matches(self, station: p.Station | None) -> bool:
        """Is this the station the Pico is using?"""
        return (station is not None and station.usable and
                (station.lat_e7, station.lon_e7, station.alt_mm)
                == self.wire())

    def where(self) -> str:
        """The position the way the Pico's own report shows it."""
        lat, lon, alt = self.wire()
        return (f"{p.format_e7(lat)}, {p.format_e7(lon)}, "
                f"{p.format_mdeg(alt)} m")


def _number(text: str, what: str) -> float:
    word = str(text).strip().rstrip("°").strip()
    if not word:
        raise ValueError(f"type the {what}")
    try:
        value = float(word)
    except ValueError:
        raise ValueError(f"{what} must be a number, got {text!r}") from None
    if not math.isfinite(value):
        raise ValueError(f"{what} must be a number, got {text!r}")
    return value


def parse(lat: str, lon: str, msl: str, geoid: str) -> SavedStation:
    """The four boxes as a station, or ValueError saying what is wrong.

    A "lat, lon" pair in the latitude box is taken as both, whatever the
    longitude box holds - that is what pasting from a map gives.
    """
    if "," in lat:
        first, _, second = lat.partition(",")
        lat, lon = first, second
    lat_deg = _number(lat, "latitude")
    lon_deg = _number(lon, "longitude")
    msl_m = _number(msl, "height above sea level (MSL)")
    geoid_m = _number(geoid, "geoid separation")

    if abs(lat_deg) > p.STATION_LAT_MAX_DEG:
        raise ValueError(f"latitude must be -90..90, got {lat_deg:g}")
    if abs(lon_deg) > p.STATION_LON_MAX_DEG:
        raise ValueError(f"longitude must be -180..180, got {lon_deg:g}")
    if abs(geoid_m) > GEOID_MAX_M:
        raise ValueError(
            f"a geoid separation of {geoid_m:g} m is not on Earth - it is "
            "within about 110 m of zero everywhere (about -33 around "
            "Blacksburg). Is that the height in the wrong box?")
    station = SavedStation(round(lat_deg, 7), round(lon_deg, 7),
                           round(msl_m, 3), round(geoid_m, 3))
    if not p.STATION_ALT_MIN_M <= station.alt_m <= p.STATION_ALT_MAX_M:
        raise ValueError(
            f"MSL plus geoid comes to {station.alt_m:g} m, outside "
            f"{p.STATION_ALT_MIN_M:g}..{p.STATION_ALT_MAX_M:g} m - is the "
            "height in feet or millimetres?")
    return station


def load(path: os.PathLike | str = DEFAULT_PATH) -> SavedStation | None:
    """The saved station, or None if there is none yet.

    A file that is there but cannot be read raises ValueError, rather than
    quietly coming back as no station: the operator should hear that what
    they saved is gone.
    """
    try:
        text = Path(path).read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise ValueError(f"could not read {path}: {exc}") from None
    try:
        data = json.loads(text)
        return parse(str(data["lat_deg"]), str(data["lon_deg"]),
                     str(data["msl_m"]), str(data["geoid_m"]))
    except (ValueError, KeyError, TypeError) as exc:
        raise ValueError(f"{path} is not a station: {exc}") from None


def save(station: SavedStation,
         path: os.PathLike | str = DEFAULT_PATH) -> None:
    """Keep it for next time. Written whole and then swapped in, so a
    crash part way leaves the old file rather than half a new one."""
    path = Path(path)
    data = {
        "lat_deg": station.lat_deg,
        "lon_deg": station.lon_deg,
        "msl_m": station.msl_m,
        "geoid_m": station.geoid_m,
        # For whoever opens the file: the number the Pico is sent.
        "ellipsoid_m": station.alt_m,
    }
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)
