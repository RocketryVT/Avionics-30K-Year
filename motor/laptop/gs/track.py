r"""Rehearse or run a flight from a terminal: `py -m gs.track`.

    py -m gs.track --sim                 a synthetic 30,000 ft flight
    py -m gs.track --sim --log           ...writing a flight log
    py -m gs.track --replay flight.csv   a recorded one again
    py -m gs.track --manual 45 60        hold one direction and watch it
    py -m gs.track --sim --dry-run       no Pico, just the angles

The window is the better place to watch this happen; this exists so a flight
can be rehearsed, logged and argued about without one, and so the whole chain
can be exercised over a serial port with nothing bolted to the motors.

**It arms tracking, which means the motors move on their own.** The axes must
be zeroed first - do that at the `motor>` prompt, then close the prompt, since
only one program can have the port open - and the Pico refuses otherwise.
Ctrl-C disarms and the Pico's watchdog covers anything worse.
"""

from __future__ import annotations

import argparse
import sys

from gs import flight_log
from gs import session as sn
from gs import targets as tg
from gs import tracking


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="py -m gs.track",
        description="Send a stream of pointing targets to the motor Pico.",
    )
    where = parser.add_mutually_exclusive_group(required=True)
    where.add_argument("--sim", action="store_true",
                       help="a synthetic flight, about 30,000 ft from 300 m "
                            "downrange")
    where.add_argument("--replay", metavar="FILE",
                       help="replay t_s,az_deg,el_deg from a CSV")
    where.add_argument("--manual", nargs=2, type=float,
                       metavar=("AZ", "EL"),
                       help="hold one direction")

    parser.add_argument("--port", help="serial port, such as COM5")
    parser.add_argument("--hz", type=float, default=tracking.DEFAULT_HZ,
                        help=f"targets per second (default "
                             f"{tracking.DEFAULT_HZ:g})")
    parser.add_argument("--seconds", type=float,
                        help="stop after this long. Default: the whole flight "
                             "for --sim and --replay, forever for --manual")
    parser.add_argument("--log", nargs="?", const="", metavar="FILE",
                        help="write a flight log. No name for a dated one in "
                             "logs/")
    parser.add_argument("--dry-run", action="store_true",
                        help="do not open a port; just print the angles the "
                             "source produces")
    return parser


def make_source(args) -> tg.TargetSource:
    if args.replay:
        return tg.SimSource.from_csv(args.replay)
    if args.manual:
        return tg.ManualSource(args.manual[0], args.manual[1])
    return tg.SimSource.rocket(hz=args.hz)


def dry_run(source: tg.TargetSource, seconds: float, hz: float) -> int:
    """Print what the source would ask for. No Pico, no motors."""
    print(f"source: {source.name}, {hz:g} Hz, {seconds:g} s   (no Pico)")
    print(f"{'t_s':>7}  {'az':>8}  {'el':>7}  {'d_el/dt':>8}")
    step = 1.0 / hz
    t = 0.0
    last = None
    peak = 0.0
    while t <= seconds + 1e-9:
        fix = source.latest(t)
        rate = 0.0
        if fix is not None and last is not None:
            rate = (fix.el_deg - last.el_deg) / step
            peak = max(peak, abs(rate))
        if fix is not None:
            print(f"{t:7.2f}  {fix.az_deg:8.2f}  {fix.el_deg:7.2f}  "
                  f"{rate:8.2f}")
            last = fix
        t += step
    print(f"\nfastest elevation sweep: {peak:.1f} deg/s")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    source = make_source(args)

    seconds = args.seconds
    if seconds is None:
        seconds = (None if args.manual
                   else getattr(source, "duration", None))

    if args.dry_run:
        return dry_run(source, seconds if seconds else 30.0, args.hz)

    try:
        session = sn.Session.open(args.port)
    except Exception as exc:  # usually a PortError, message already fit
        print(exc)
        return 2

    log = None
    try:
        session.drain_until_quiet()
        who = ""
        try:
            pong = session.ping()
            who = f"{pong.target} {pong.version}"
            print(f"connected: {who}")
        except sn.SessionError as exc:
            print(f"warning: no answer to PING ({exc})")

        gear = ""
        try:
            got = session.gear()
            gear = "on" if got.geared else "off"
            print(f"gear {gear}: {got.describe()}")
        except sn.SessionError as exc:
            print(f"warning: could not read the gear mode ({exc})")

        if args.log is not None:
            path = args.log or flight_log.default_path()
            log = flight_log.FlightLog(path=path, pico=who, gear=gear,
                                       source=source.name).open()
            print(f"logging to {path}")

        print(f"source: {source.name}, {args.hz:g} targets/sec"
              + (f", {seconds:g} s" if seconds else ""))
        print("arming - the motors will move. Ctrl-C to stop.\n")

        def show(sample: tracking.Sample) -> None:
            print(sample.describe())

        try:
            sent = tracking.run(session, source, hz=args.hz, seconds=seconds,
                                log=log, on_sample=show)
        except sn.Refused as refused:
            print(f"refused: {refused}")
            print("Zero both axes at the motor> prompt first.")
            return 1

        print(f"\ndisarmed. {sent} targets sent.")
        if log is not None:
            print(f"flight log: {log.path} ({log.rows} rows)")
    finally:
        if log is not None:
            log.close()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
