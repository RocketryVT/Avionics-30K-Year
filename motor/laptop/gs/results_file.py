r"""The results file: one CSV row per move, written as the test happens.

The deliverable from a bench test is a table: what was commanded, what the
Pico predicted, what it reported, and what the protractor actually read.
The laptop knows the first three the moment a move finishes. The fourth is
a person reading a scale, which arrives some seconds later and has to find
its way onto the right row.

So a row is written as soon as the move ends, with the protractor column
empty, and `set_true()` fills it in afterwards. At the prompt that is the
`true` command.

Why the whole file is rewritten every time
------------------------------------------

Appending would leave a reading that arrives after its row was written with
nowhere to go. A test run is a few hundred rows at most, so the file is
rebuilt from the rows held in memory whenever anything changes. It goes to
a temporary file first and is then moved into place, so an interrupted
write cannot leave half a table behind.

The error column
----------------

`error_deg` is the protractor reading minus the angle the Pico predicted,
not minus the angle that was asked for. Those differ by the rounding to
whole steps: asking for 10 degrees really commands 9.990, and measuring the
mechanism against a number it was never given would fold a known rounding
into what is meant to be a measurement of the gearing.
"""

from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass, field
from datetime import datetime

from gs import protocol as p

# Where `--log` with no file name puts its file, relative to the folder the
# program is run from.
LOG_DIR = "logs"

# How hard to try when moving the finished file into place. See write().
REPLACE_TRIES = 5
REPLACE_WAIT_S = 0.1

# The columns, in the order they appear in the file. Changing this changes
# every file written afterwards, so leave it alone unless whatever reads the
# files is changing too.
COLUMNS = [
    "row",
    "when",
    "pico",
    "id",
    "axis",
    "speed_dps",
    "commanded_deg",
    "from_deg",
    "predicted_deg",
    "true_deg",
    "error_deg",
    "steps",
    "steps_done",
    "shape",
    "predicted_s",
    "measured_s",
    "result",
    "note",
]


def default_path(when: datetime | None = None) -> str:
    """logs/motor_test_<date>_<time>.csv, making the folder if need be."""
    stamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S")
    os.makedirs(LOG_DIR, exist_ok=True)
    return os.path.join(LOG_DIR, f"motor_test_{stamp}.csv")


@dataclass
class Row:
    """One move, and the reading that belongs to it."""

    row: int
    when: str
    pico: str
    id: int
    axis: str
    speed_mdps: int
    commanded_mdeg: int | None
    from_mdeg: int
    predicted_mdeg: int
    steps: int
    steps_done: int
    shape: str
    predicted_us: int
    measured_us: int
    result: str
    true_mdeg: int | None = None
    note: str = ""

    @property
    def error_mdeg(self) -> int | None:
        """How far the antenna really ended up from where it was predicted."""
        if self.true_mdeg is None:
            return None
        return self.true_mdeg - self.predicted_mdeg

    def as_record(self) -> dict:
        """The row as the strings that go in the file.

        Angles are formatted from the whole millidegrees rather than from a
        float, so the text is exactly the number the Pico used.
        """
        return {
            "row": self.row,
            "when": self.when,
            "pico": self.pico,
            "id": self.id,
            "axis": self.axis,
            "speed_dps": p.format_mdeg(self.speed_mdps),
            "commanded_deg": _mdeg_or_blank(self.commanded_mdeg),
            "from_deg": p.format_mdeg(self.from_mdeg),
            "predicted_deg": p.format_mdeg(self.predicted_mdeg),
            "true_deg": _mdeg_or_blank(self.true_mdeg),
            "error_deg": _mdeg_or_blank(self.error_mdeg),
            "steps": self.steps,
            "steps_done": self.steps_done,
            "shape": self.shape,
            "predicted_s": f"{self.predicted_us / 1e6:.3f}",
            "measured_s": f"{self.measured_us / 1e6:.3f}",
            "result": self.result,
            "note": self.note,
        }

    def summary(self) -> str:
        """One line, for showing at the prompt."""
        true = (
            "   -   " if self.true_mdeg is None
            else f"{p.format_mdeg(self.true_mdeg):>8}"
        )
        error = (
            "" if self.error_mdeg is None
            else f"  err {p.format_mdeg(self.error_mdeg)}"
        )
        return (
            f"  {self.row:>3}  {self.axis}  "
            f"{p.format_mdeg(self.from_mdeg):>9} ->"
            f"{p.format_mdeg(self.predicted_mdeg):>9} deg"
            f"  true {true}{error}  {self.result}"
        )


def _mdeg_or_blank(mdeg: int | None) -> str:
    return "" if mdeg is None else p.format_mdeg(mdeg)


class ResultsError(Exception):
    """Something was asked of the results file that it cannot do."""


@dataclass
class ResultsFile:
    """Every move of a session, and the CSV file they are written to."""

    path: str
    pico: str = ""
    rows: list[Row] = field(default_factory=list)

    def add(self, outcome, commanded_deg: float | None = None,
            note: str = "") -> Row:
        """Record a finished move. Writes the file before returning."""
        plan, done = outcome.plan, outcome.done
        row = Row(
            row=len(self.rows) + 1,
            when=datetime.now().isoformat(timespec="seconds"),
            pico=self.pico,
            id=done.id,
            axis=done.axis,
            speed_mdps=plan.speed_mdps,
            commanded_mdeg=(
                None if commanded_deg is None else p.deg_to_mdeg(commanded_deg)
            ),
            from_mdeg=plan.from_mdeg,
            predicted_mdeg=done.to_mdeg,
            steps=plan.steps,
            steps_done=done.steps_done,
            shape="TRI" if plan.triangular else "TRAP",
            predicted_us=plan.est_us,
            measured_us=done.elapsed_us,
            # A stopped move did not go where it was predicted to, so the row
            # is kept but marked - leaving it out would lose the reason a
            # reading is missing.
            result="OK" if done.ok else "ABORT",
            note=note,
        )
        self.rows.append(row)
        self.write()
        return row

    def set_true(self, deg: float, row: int | None = None) -> Row:
        """Fill in what the protractor said. Defaults to the last move."""
        target = self._find(row)
        target.true_mdeg = p.deg_to_mdeg(deg)
        self.write()
        return target

    def annotate(self, note: str, row: int | None = None) -> Row:
        """Put a note against a row. Defaults to the last one."""
        target = self._find(row)
        target.note = (target.note + " " + note).strip()
        self.write()
        return target

    def _find(self, row: int | None) -> Row:
        """The row with this number, or the last row if it is None."""
        if not self.rows:
            raise ResultsError("nothing has been moved yet, so there is no row")
        if row is None:
            return self.rows[-1]
        for r in self.rows:
            if r.row == row:
                return r
        raise ResultsError(
            f"there is no row {row}. Rows go 1..{len(self.rows)}"
        )

    def missing_readings(self) -> list[Row]:
        """Finished moves with no protractor reading yet.

        Worth saying out loud before anyone packs up: a blank here is a
        measurement nobody can go back and take afterwards.
        """
        return [
            r for r in self.rows
            if r.true_mdeg is None and r.result == "OK"
        ]

    def write(self) -> None:
        """Rewrite the whole file, through a temporary, in one go."""
        temporary = self.path + ".part"
        with open(temporary, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row.as_record())

        # On Windows the move fails if anything else has the file open for
        # even a moment - a virus scanner, a sync client, or a spreadsheet
        # left open on the last run. Those all let go quickly, so it is worth
        # a few tries before giving up and losing the row just measured.
        for attempt in range(REPLACE_TRIES):
            try:
                os.replace(temporary, self.path)
                return
            except PermissionError:
                if attempt == REPLACE_TRIES - 1:
                    raise
                time.sleep(REPLACE_WAIT_S)
