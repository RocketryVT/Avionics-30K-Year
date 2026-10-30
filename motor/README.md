motor actuation

The part of the rocket-tracking ground station that points the antenna. A
Raspberry Pi Pico 2 W drives the antenna's two stepper motors, azimuth
(`az`) and elevation (`el`), and a program on the laptop tells it what to do
over USB.

```
laptop                                 Pico                        drivers + motors
py -m gs.motor  --- text commands -->  gs_motor  --- step + dir -->  CL57TE x2
                <-- text replies ----
```

## What's in here

| Folder | What's in it |
| --- | --- |
| `firmware/src/` | The program that runs on the Pico (C++) |
| `firmware/uf2/` | The built program, ready to copy onto the Pico (made by the build, not kept in git) |
| `laptop/gs/` | The program you run on the laptop (Python) |
| `docs/` | Setting up a new machine, and the motor driver's manual |
| `logs/` | Results files written by `--log` (not kept in git) |

### The Pico program, `firmware/src/`

| File | What it does |
| --- | --- |
| `main.cpp` | Starts up, then runs one loop: read commands, step the motors, report back |
| `motor_config.hpp` | **The numbers to edit**: gear pairs, steps per degree, travel limits, speeds, acceleration |
| `pins.hpp` | Which Pico pin goes to which driver wire |
| `line_reader` | Collects bytes from USB into one command line |
| `command_parser` | Turns a line of text into a command |
| `command_rules` | Decides whether a command is allowed, and what to answer |
| `replies` | Builds the lines sent back to the laptop |
| `motion_profile` | Turns angles into steps, and works out the speed ramp |
| `stepper` | Sends the pulses for one axis |
| `settings` | What `set` may change, and the record that `save` keeps |
| `settings_flash` | Reads and writes that record in the Pico's flash |

### The laptop program, `laptop/gs/`

| File | What it does |
| --- | --- |
| `motor.py` | The `motor>` prompt: the program you run |
| `session.py` | Talks to the Pico: one command at a time, matching each reply to its command |
| `protocol.py` | The exact text of every command and reply |
| `ports.py` | Finds the Pico's serial port |
| `scripted_runs.py` | The `sweep` and `random` runs |
| `results_file.py` | The CSV results file |
| `targets.py` | Where a pointing direction comes from: typed, simulated, or live |
| `command_help.py` | What each command does, and when to reach for it |
| `tracking.py` | The loop that keeps sending targets |
| `track.py` | `py -m gs.track`: rehearse or run a flight |
| `flight_log.py` | The time-series CSV a tracking run writes |
| `gui.py`, `gui_worker.py` | The window, and the thread that talks to the Pico for it |

## Quick start

1. **Install the laptop tools** (Windows, once per machine, from this folder):
   `py -m pip install -e laptop`
2. **Build the Pico program** (WSL, from this folder):
   `cmake -S firmware -B ~/build/gs -G Ninja` once, then `ninja -C ~/build/gs`
3. **Flash it:** hold BOOTSEL, plug the Pico in, and drag
   `firmware/uf2/gs_motor.uf2` onto the drive that appears.
4. **Drive the motors:** `py -m gs.motor --log`, then at the prompt:

   ```
   motor> zero
   motor> move az 5 45
   motor> move az 10 -45 el 5 20
   ```

New machine? Do `docs/SETUP.md` first.

## Commands

### At the `motor>` prompt

| Command | What it does |
| --- | --- |
| `move <az\|el> <deg/s> <deg>` | Turn one axis. `move az 5 45` turns azimuth 45° at 5°/s |
| `move az <deg/s> <deg> el <deg/s> <deg>` | Turn both at once, each at its own speed. Either axis may come first |
| `goto <az\|el> <deg/s> <deg>` | Turn **to** an angle rather than **by** one. `goto az 5 45` points at 45 whatever it started from |
| `goto az <deg/s> <deg> el <deg/s> <deg>` | Point both at once |
| `stop` | Slow to a stop |
| `estop` | Stop at once. Loses the zero, so re-zero afterwards |
| Ctrl-C during a move | Same as `stop`, then waits for the report. Twice stops waiting |
| `zero [az\|el\|all] [deg]` | Say where the axes are now. Nothing moves. `zero` alone means both at 0 |
| `pos` | Where it thinks it is, in degrees and steps |
| `ping` | Is it there, which program, how long it has been up |
| `limits` | The travel limits and speed range in force |
| `settings` | Acceleration and travel for each axis, and whether they are saved |
| `set <az\|el> accel <deg/s/s>` | Change an axis's acceleration (10-360). Lasts until a restart unless saved |
| `set <az\|el> <min\|max> <deg>` | Narrow an axis's travel. It can't go past what `motor_config.hpp` allows |
| `set <az\|el> deadband <deg>` | How close to a target counts as being there. Tracking only |
| `save` | Keep the settings, so a restart comes back with them |
| `defaults` | Back to the numbers in `motor_config.hpp` (then `save` to keep that) |
| `gear [on\|off]` | Whether the final gear pair is counted. See **Gear modes** |
| `track [on\|off]` | Arm or disarm chasing a target. See **Tracking** |
| `pin <gpio> <0\|1>` | Hold a pin high or low, for a multimeter (GP4-7, GP12-15). A step pin (GP4, GP6) costs that axis its zero |
| `sweep [axis=az] [angle=45] [speeds=1,5,90] [pause]` | One angle at each speed in turn |
| `random [axis=az] [count=10] [speed=5] [seed=N] [pause]` | Random angles inside the limits |
| `true [row] <deg>` | Record what the protractor reads, for the last row or the one given |
| `note [row] <text>` | Write a note against a row |
| `rows [n]` | Show the last n rows of the results table |
| `verbose [on\|off]` | Show or hide the heartbeat |
| `help` | List these |
| `quit` | Leave (`exit` works too) |

`move` is relative: `move az 5 -45` turns back 45°. `goto` is absolute,
measured from the zero: `goto az 5 45` points at 45° wherever it started, and
the Pico works out how far that is, because the step count is the only record
of where the axis is and it lives on that side. Asking `goto` for the angle it
is already at is refused as zero steps.

Every move is refused until the axes have been zeroed, because there are no
limit switches and the soft limits are measured from the zero.

Acceleration and travel can be tuned at the prompt with no reflash. Saved
settings live on the Pico and survive restarts and reflashes. The exception
is when you change `motor_config.hpp` and reflash: then the saved settings
are ignored and the new built-in numbers are used.

### Gear modes

Each motor has a 10:1 planetary gearbox, and after that a final gear pair to
the antenna. `gear` says whether that pair is in the drive train, and so
whether it should be in the maths.

| | `gear off` (direct) | `gear on` (geared) |
| --- | --- | --- |
| What is bolted up | Each motor on its own | The belt to the centre piece |
| An angle means | At the gearbox output shaft | At the antenna |
| Azimuth | 44.444 steps/deg | 101.587 steps/deg (32:14 pair) |
| Elevation | 44.444 steps/deg | 63.492 steps/deg (20:14 pair) |
| Travel | az -180 to +180, el -25 to 360 | az -180 to +180, el -25 to 180 |
| Speed ceiling | 181 deg/s both | az 79.187, el 126.700 deg/s |
| Acceleration | 135 deg/s/s both | az 59.062, el 94.500 deg/s/s |

The speeds look slower in geared mode because they are measured at the
antenna. The motor is asked for exactly the same thing in both: the pair
changes what the antenna does, not what the motor does. That is deliberate -
181 deg/s of antenna through azimuth's pair would be about 690 rpm at the
motor, past where its torque halves, and a stepper that stalls there loses
position with nothing able to notice.

**Run the wrong mode and every move is out by the pair's ratio** - 1.43x on
elevation, 2.29x on azimuth. This is not hypothetical: on 18 Oct 2026, with
the pair in the maths and not in the machine, 180 commanded on elevation
turned about 270.

Switching costs both axes their zero, because the drive train has physically
changed and the step count no longer describes a known angle. Re-zero before
moving. Asking for the mode already in force changes nothing and costs no
zero.

The mode is kept by `save` along with the other settings, so a restart comes
back in it. A Pico with nothing saved comes up in direct mode. The mode is
printed in the startup banner, by `gear`, and by `settings`.

Before the first geared move, check `motor_config.hpp`: nobody has counted
azimuth's 32 teeth on this machine, and elevation's -25 to 180 travel has not
been measured on the assembled antenna. Those two numbers are the only thing
between a mistyped command and a hard stop.

### Tracking

Modes 1 and 2 above move the antenna by a command at a time. Tracking is the
third way: the laptop keeps saying *point here*, and the Pico keeps closing
the gap on its own.

| | |
| --- | --- |
| `track on` | Arm it. Needs both axes idle and zeroed |
| `track off` | Disarm, ramping to a stop. Never refused |
| `TARGET` | Where to point, absolute, both axes. Sent by the tracking program, not typed |

**There is no plan and no end.** An ordinary move works out a whole trapezium
up front and follows it to the last step, which is why a target that changes
has nowhere to go. Tracking carries its speed instead, and once per loop asks
one question: how far off am I, and how fast should I be going now? Two
things cap the answer - the axis's speed ceiling, and how fast it could still
brake from. A target that moves is not a special case, it is just the next
answer.

Three things about it are worth knowing before you arm it:

- **A target outside the travel limits is held at the limit, not refused.**
  A move is checked whole and refused whole, because it is a finite thing
  somebody typed. A target arrives several times a second from a rocket that
  does not know where the gimbal can point, so clamping keeps the antenna
  pointing as close as it can and lets it pick the rocket up again by itself.

- **If targets stop arriving, it gives up.** No target for 1.5 seconds and
  both axes ramp to rest and stay there until something arms tracking again.
  That is what a radio dropout or a closed laptop looks like from the Pico's
  side, and it is what pays for clamping rather than refusing.

- **It lags a moving target**, by `rate² / (2 × accel)`. About 3.4° at 20°/s
  on azimuth, 13.6° at 40°/s. That is the braking cap needing an error to ask
  for any speed at all. Nothing against a ~50° beam, but it grows with the
  square of the rate.

**Zero azimuth facing north.** Directions arrive as compass bearings, 0 to
360. The Pico's azimuth runs -180 to +180 with no wrap-around, so the laptop
sends a bearing past 180 as a negative angle - 270 goes out as -90. That only
points the right way if azimuth's zero faces north. It also makes due south
the seam: a rocket drifting across south goes from +179.9 to -179.9, and the
antenna turns a full circle the other way to follow it, because it will never
wind past ±180.

While tracking, the Pico sends a `TRK` line a few times a second with the
target, the position and the rate for each axis. It is the only report that
comes out while an axis is turning - `pos` is refused then, and the heartbeat
stops - and it is sent only if there is room in the USB buffer, never waited
on. A dropped telemetry line costs a moment of staleness; waiting would cost
a stalled step loop.

`stop` and `estop` end tracking. While it is armed, every other command
except `track` is refused as busy - `track off` first, then zero, move or
change gear.

Starting tracking also requires both axes to be idle: `track on` returns
`busy` during a planned move or while braking after tracking stops. Wait for
the move to finish or the axes to come to rest before arming again. Status
queries and `track off` remain available, and `track on` while already armed
leaves the current tracking session running.

#### Sending the targets

Something has to supply directions. In the window that is mode 3; from a
terminal it is `py -m gs.track`, which arms tracking itself, sends the
targets, and disarms when it finishes. Close the `motor>` prompt first - only
one program can have the port open. (`track on` at the prompt only arms it,
which is mostly useful for seeing the 1.5 second give-up happen.)

| | |
| --- | --- |
| `py -m gs.track --sim` | A synthetic flight: 30,000 ft from 300 m downrange |
| `py -m gs.track --sim --dry-run` | The angles only. No port, no motors |
| `py -m gs.track --sim --log` | ...writing a flight log to `logs/` |
| `py -m gs.track --replay f.csv` | A recorded flight again |
| `py -m gs.track --manual 45 60` | Hold one direction and watch it settle |

Start with `--dry-run`. It prints what the source would ask for and the
fastest elevation sweep in it, which with the defaults is about 30°/s in the
first couple of seconds — close to the pad means a little altitude is a lot
of angle. That is the number to compare against an axis's ceiling before
anything is bolted up.

`--manual` is the one to use first on real hardware: it takes the rocket, the
radio and the geometry out of the picture, so what is left under test is the
chase loop.

Three sources, one interface, in `targets.py`. The third is `LiveSource`,
which is where the lat/lon/alt-to-az/el module plugs in when it is ready —
hand it a callable returning `(az_deg, el_deg, fix_time)` and it works. It
wants **absolute** angles, not changes; `DeltaSource` is there if that cannot
be arranged, and its docstring says why it is the worse option.

A flight log is a time series rather than a table — one row per target sent,
appended as it happens. It records the direction asked for *and* the one the
Pico is using, which differ when a target is out of reach and gets held at a
travel limit.

### In a terminal (Windows, from this folder)

| Command | What it does |
| --- | --- |
| `py -m gs.motor` | Start the prompt, finding the Pico by itself |
| `py -m gs.motor --port COM5` | ...on a port you name |
| `py -m gs.motor --log` | ...writing results to a dated file in `logs/` |
| `py -m gs.motor --log bench.csv` | ...writing results to a file you name |
| `py -m gs.motor --verbose` | ...showing the heartbeat |
| `py -m gs.motor -c "pos"` | Run one prompt command and exit. Repeat `-c` for more |
| `py -m gs.motor --list` | List serial ports, marking the Pico, and exit |
| `py -m gs.gui` | The window: all of the above, with the state on screen |
| `py -m gs.track --sim` | Send a stream of pointing targets. See **Tracking** |

### In WSL (from this folder)

| Command | What it does |
| --- | --- |
| `cmake -S firmware -B ~/build/gs -G Ninja` | Set up the Pico build. Once, and again after adding a file |
| `ninja -C ~/build/gs` | Build the Pico program into `firmware/uf2/gs_motor.uf2` |
