r"""The startup guide: what to do, in order, and what the window should show.

The window opens it from the "startup guide" button on the connection line,
in a window of its own beside the main one (see `App._open_guide` in
gui.py). Not a panel inside the main window: that is already as tall as a
laptop screen, and the guide is read at startup and then closed.

Only words here, no widgets, so it can be read and checked without a
display.

Every grey line (`expect`) and every "if instead" line is something the
window or the Pico really prints, with example numbers filled in - "heard
5.0/s" says more to somebody at a launch site than a format string. Change a
message in gui.py, gui_worker.py, radio.py or the firmware, and change it
here too.
"""

from __future__ import annotations

from dataclasses import dataclass

from gs.protocol import TRACK_TIMEOUT_S


@dataclass(frozen=True)
class Step:
    """One thing to do, and how to tell it worked."""

    title: str
    # Where on screen the expected lines appear.
    look: str
    # What to do. Blank when the step is only looking.
    do: str = ""
    # What should appear, line for line.
    expect: tuple[str, ...] = ()
    # Said after the expected lines: what they mean, or what varies.
    note: str = ""
    # What might appear instead, and what to do about it.
    otherwise: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class Part:
    title: str
    intro: str
    steps: tuple[Step, ...]


TITLE = "Startup guide"

INTRO = ("Do each step, then check the window shows what is in the grey "
         "box. Steps 1 to 3 only check the radio, so motor power can stay "
         "off for them. Esc is the ramped STOP in this window too.")

CHECK_RADIO = Part(
    title="Check the radio is receiving",
    intro="",
    steps=(
        Step(
            title="Plug the ground Pico into this laptop",
            do="Use a data cable, not a charge-only one. The window finds "
               "the Pico by itself.",
            look="the connection line, at the top",
            expect=("COM5  gs_motor <version>  up 12s  heartbeat 3s ago",),
            note="Your port, version and times will differ. The heartbeat "
                 "says \"-\" for the first few seconds, then never more "
                 "than about 5 s ago.",
            otherwise=(
                ("not connected: No Pico found",
                 "Check the Pico is plugged in with a data cable. The "
                 "window keeps looking by itself."),
                ("not connected: More than one Pico found",
                 "Unplug the other Pico. The window tries again by "
                 "itself."),
            ),
        ),
        Step(
            title="Check the radio line",
            look="the radio line, just under the connection line",
            expect=("radio  no packets yet",),
            note="In amber. The radio is on and listening, and nothing is "
                 "sending yet. For the first second or two it says "
                 "\"waiting for the first radio report\". If the "
                 "transmitter is already on it goes straight to "
                 "\"receiving\" - see step 3.",
            otherwise=(
                ("radio  RADIO OFF - the radio chip did not answer - check "
                 "its wiring, 3V3 and ground",
                 "Fix the wiring, then unplug and replug the Pico. The "
                 "radio only starts when the Pico does."),
                ("radio  no radio reports - this firmware may not have the "
                 "receiver built in",
                 "Flash gs_motor built from the lora-integration branch."),
            ),
        ),
        Step(
            title="Turn on the transmitter",
            do="Power it on. It needs no laptop. In GPS mode it sends once "
               "the GPS has a fix. In replay mode it sends the starting "
               "position at once, and START in its serial monitor sets the "
               "flight going.",
            look="the radio line",
            expect=("radio  receiving   heard 5.0/s, last 0 s ago  "
                    "rejected 0/5 s (0)  steering 0.0/s  seq 1234",),
            note="In green, about 5 heard a second, and seq climbing. That "
                 "is the radio check done.",
            otherwise=(
                ("radio  no packets yet   (and it stays)",
                 "Is the transmitter sending? Its serial monitor prints "
                 "TX seq= lines. Were both boards flashed from the same "
                 "radio_config.hpp?"),
                ("radio  no packets for 12 s   (and rejected climbing)",
                 "The transmitter restarted, so the ground throws its "
                 "packets away as old. Unplug and replug the ground Pico."),
            ),
        ),
    ),
)

TRACK_ROCKET = Part(
    title="Then, to point the antenna at the rocket",
    intro="Motor power on from here.",
    steps=(
        Step(
            title="Zero both axes",
            do="With the antenna pointing at true north and level, leave 0 "
               "in the \"is now\" box and press \"zero both\". Pointing "
               "somewhere else you know? Type that angle in the box and "
               "press \"az\" or \"el\" for each axis. Once zeroed, the move "
               "row under the position turns it anywhere.",
            look="the position panel",
            expect=("zeroed",),
            note="Zeroing tells the Pico where the antenna points now. It "
                 "does not move anything.",
            otherwise=(
                ("NOT ZEROED",
                 "Press \"zero both\". Nothing moves until both axes are "
                 "zeroed."),
                ("GEAR OFF - angles at the gearbox shaft, not the antenna",
                 "Press \"ARM tracking\" and accept the switch to gear on, "
                 "then zero again."),
            ),
        ),
        Step(
            title="Arm tracking on the rocket",
            do="In the track panel choose \"the rocket (radio)\", press "
               "\"ARM tracking\" and confirm.",
            look="the status line at the bottom, and the note beside the "
                 "buttons",
            expect=("TRACKING - the Pico is following the radio",
                    "following the radio - 5.0 targets/s"),
            otherwise=(
                ("the ground station's position is not set in "
                 "radio_config.hpp",
                 "Set RADIO_LIVE_STATION in radio_config.hpp and reflash "
                 "the ground Pico."),
                ("following the radio - packets but NO targets (station?)",
                 "Positions arrive but give no pointing. Check "
                 "RADIO_LIVE_STATION - a rocket within 5 m of it gives "
                 "none."),
            ),
        ),
        Step(
            title="If tracking stops by itself",
            do="The Pico stops when no position arrives for "
               f"{TRACK_TIMEOUT_S} s, and never starts again by itself. "
               "When the radio line says \"receiving\" again, press "
               "\"re-arm radio\".",
            look="the note beside the buttons",
            expect=(f"STOPPED - radio quiet {TRACK_TIMEOUT_S} s. Re-arm when "
                    "packets arrive",),
            note="STOP (Esc) and E-STOP work at any time.",
        ),
    ),
)

PARTS = (CHECK_RADIO, TRACK_ROCKET)


def steps() -> list[Step]:
    """Every step in order, as the guide numbers them."""
    return [step for part in PARTS for step in part.steps]
