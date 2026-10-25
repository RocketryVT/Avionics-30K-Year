r"""The flight log: one row per target sent, written as it happens.

Not the same shape as `results_file.py`, which is why it is its own module.
That one is a bench table - one row per move, with a column waiting for a
protractor reading that arrives seconds later, so the whole file is rewritten
whenever anything changes. This is a time series. Nothing is ever filled in
afterwards, so rows are appended and the file is only ever opened for writing
once.

What each row holds, and why there are three angles rather than one:

    source_*    what the source said to point at
    target_*    what the Pico says it is pointing at

They differ in two cases. A compass bearing past 180 is sent as a negative
azimuth - 270 goes out as -90 - because the Pico's azimuth runs -180 to +180.
And a target outside an axis's travel gets held at the limit - the antenna is
doing the best it can, and the two columns say so where one column would just
quietly show a lie.

    actual_*    where the Pico has got to
    error_*     target minus actual, which is what the chase loop is closing
    rate_*      how fast each axis is turning

`armed` is on every row because the Pico's watchdog can drop out of tracking
on its own, and the row where that happens is the one worth finding.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass
from datetime import datetime

from gs import protocol as p

# Where `--log` with no file name puts its file, matching results_file.py.
LOG_DIR = "logs"

COLUMNS = [
    "t_s",
    "when",
    "pico",
    "gear",
    "source",
    "armed",
    "source_az",
    "source_el",
    "target_az",
    "target_el",
    "actual_az",
    "actual_el",
    "error_az",
    "error_el",
    "rate_az",
    "rate_el",
    "fix_age_s",
    "note",
]


def default_path(when: datetime | None = None) -> str:
    """logs/track_<date>_<time>.csv, making the folder if need be."""
    stamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S")
    os.makedirs(LOG_DIR, exist_ok=True)
    return os.path.join(LOG_DIR, f"track_{stamp}.csv")


@dataclass
class FlightLog:
    """A CSV opened once and appended to.

    Used as a context manager, or closed by hand. Every number is written
    through `format_mdeg`, so the text is exactly what the Pico used rather
    than a float's idea of it - the same rule results_file.py follows.
    """

    path: str
    pico: str = ""
    gear: str = ""
    source: str = ""

    def __post_init__(self) -> None:
        self._handle = None
        self._writer = None
        self._rows = 0

    def open(self) -> FlightLog:
        folder = os.path.dirname(self.path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        self._handle = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=COLUMNS)
        self._writer.writeheader()
        self._handle.flush()
        return self

    def __enter__(self) -> FlightLog:
        return self.open() if self._handle is None else self

    def __exit__(self, *_exc) -> None:
        self.close()

    @property
    def rows(self) -> int:
        return self._rows

    def add(self, t_s: float, fix, trk: p.Trk | None,
            note: str = "") -> None:
        """One row. `fix` is a targets.Fix or None, `trk` the last report.

        Flushed every time. A flight is a minute or two and the interesting
        rows are the ones just before something went wrong, which is exactly
        when a buffer would still be holding them.
        """
        if self._writer is None:
            self.open()

        row = {name: "" for name in COLUMNS}
        row.update({
            "t_s": f"{t_s:.3f}",
            "when": datetime.now().isoformat(timespec="milliseconds"),
            "pico": self.pico,
            "gear": self.gear,
            "source": self.source,
            "note": note,
        })

        if fix is not None:
            row["source_az"] = p.format_mdeg(p.deg_to_mdeg(fix.az_deg))
            row["source_el"] = p.format_mdeg(p.deg_to_mdeg(fix.el_deg))
            row["fix_age_s"] = f"{max(0.0, t_s - fix.at):.3f}"

        if trk is not None:
            row.update({
                "armed": "on" if trk.armed else "off",
                "target_az": p.format_mdeg(trk.az_target_mdeg),
                "target_el": p.format_mdeg(trk.el_target_mdeg),
                "actual_az": p.format_mdeg(trk.az_pos_mdeg),
                "actual_el": p.format_mdeg(trk.el_pos_mdeg),
                "error_az": p.format_mdeg(trk.az_error_mdeg),
                "error_el": p.format_mdeg(trk.el_error_mdeg),
                "rate_az": p.format_mdeg(trk.az_mdps),
                "rate_el": p.format_mdeg(trk.el_mdps),
            })

        self._writer.writerow(row)
        self._handle.flush()
        self._rows += 1

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
            self._writer = None
