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
| `motor_config.hpp` | **The numbers to edit**: steps per degree, travel limits, speeds, acceleration |
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
| `save` | Keep the settings, so a restart comes back with them |
| `defaults` | Back to the numbers in `motor_config.hpp` (then `save` to keep that) |
| `pin <gpio> <0\|1>` | Hold a pin high or low, for a multimeter (GP4-7, GP12-15). A step pin (GP4, GP6) costs that axis its zero |
| `sweep [axis=az] [angle=45] [speeds=1,5,90] [pause]` | One angle at each speed in turn |
| `random [axis=az] [count=10] [speed=5] [seed=N] [pause]` | Random angles inside the limits |
| `true [row] <deg>` | Record what the protractor reads, for the last row or the one given |
| `note [row] <text>` | Write a note against a row |
| `rows [n]` | Show the last n rows of the results table |
| `verbose [on\|off]` | Show or hide the heartbeat |
| `help` | List these |
| `quit` | Leave (`exit` works too) |

Angles are relative: `move az 5 -45` turns back 45°. Every move is refused
until the axes have been zeroed, because there are no limit switches and the
soft limits are measured from the zero.

Acceleration and travel can be tuned at the prompt with no reflash. Saved
settings live on the Pico and survive restarts and reflashes. The exception
is when you change `motor_config.hpp` and reflash: then the saved settings
are ignored and the new built-in numbers are used.

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

### In WSL (from this folder)

| Command | What it does |
| --- | --- |
| `cmake -S firmware -B ~/build/gs -G Ninja` | Set up the Pico build. Once, and again after adding a file |
| `ninja -C ~/build/gs` | Build the Pico program into `firmware/uf2/gs_motor.uf2` |
