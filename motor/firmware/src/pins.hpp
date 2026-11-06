// Which Pico pin goes to which motor driver wire.
//
// Two numbers describe the same hole on the board, and mixing them up is the
// most common wiring mistake:
//
//   GPIO number     what the code uses              (GP4)
//   physical pin    what you count on the board     (pin 6)
//
// The constants below are GPIO numbers. The physical pin is in the comment so
// you can count it out on the board while wiring.
//
// Two StepperOnline CL57TE drivers, one per axis. The gs_pcb_v1 board calls
// their headers "Servo 1" (top, azimuth) and "Servo 2" (bottom, elevation).
// Each driver runs in CW/CCW (double pulse) mode, which is DIP switch SW7 ON,
// and takes two signals:
//
//   CW  - one pulse per step one way         (the terminal marked PUL)
//   CCW - one pulse per step the other way   (the terminal marked DIR)
//
//   driver        Pico 2 W    physical pin   wire
//   ------------  ----------  -------------  -----
//   Servo 1 PUL-  GP4         6              red
//   Servo 1 DIR-  GP5         7              red
//   Servo 2 PUL-  GP6         9              blue
//   Servo 2 DIR-  GP7         10             blue
//   all PUL+/DIR+ +5V, top two pins of each screw terminal, green
//
// The manual gives each terminal both names, PUL-(CW-) and DIR-(CCW-), so the
// wiring is the same in either mode. Only SW7 and this program change. With
// SW7 OFF the driver expects PUL/DIR - DIR held at a level rather than pulsed
// - and this program cannot drive that: every move one way would go nowhere.
//
// ENA and ALM are not connected. The Pico cannot disable a driver and cannot
// tell when one faults - watch the red LED on the driver instead.
#pragma once

#include <stdint.h>

constexpr uint8_t PIN_AZ_CW  = 4;  // physical pin 6   - azimuth, PUL terminal
constexpr uint8_t PIN_AZ_CCW = 5;  // physical pin 7   - azimuth, DIR terminal
constexpr uint8_t PIN_EL_CW  = 6;  // physical pin 9   - elevation, PUL terminal
constexpr uint8_t PIN_EL_CCW = 7;  // physical pin 10  - elevation, DIR terminal

// ============================================================================
// The signals are active low
// ============================================================================
// The drivers' PUL+ and DIR+ are tied to +5V on the board, so the Pico sinks
// current through each driver's optocoupler rather than driving the line
// high. "On" means pulling the pin LOW.
//
// Never switch these pins to inputs or open-drain to "fix" a signal problem.
// Pico pins are not 5V tolerant, and a released pin lets the 5V rail push
// current back into it through the optocoupler.
//
// Both lines of an axis work the same way: HIGH at rest, LOW for one step.
// The driver acts on the falling edge.
constexpr bool LINE_IDLE  = true;   // HIGH between steps
constexpr bool LINE_PULSE = false;  // LOW makes one step

// ============================================================================
// Why CW/CCW rather than PUL/DIR
// ============================================================================
// Nothing ever has to HOLD a level. In PUL/DIR mode the DIR line did, and the
// HIGH this board gives it is not a level the driver can be relied on to
// read: a 3.3 V Pico high leaves 1.7 V across an input whose manual counts
// 0-0.5 V as off and 4.5 V or more as on (its numbers for ENA, the one input
// it gives them for). Every direction fault seen at the bench was on that
// line - a reversal that set off the old way first, a first move after
// power-up that went whichever way it pleased, an axis that turned one way
// for every command.
//
// A pulse line gets away with the same weak HIGH, because the driver only
// acts on its edge into a solid LOW. In this mode both lines are pulse lines,
// and there is no direction to set up, or wait for, before a step.

// ============================================================================
// Which line turns an axis which way
// ============================================================================
// A positive step goes out on the CCW line, the DIR terminal, before the
// axis's invert_dir (motor_config.hpp) is applied.
//
// That is chosen to keep the direction the bench verified in PUL/DIR mode,
// where positive was DIR held LOW - the DIR input switched on. The bet is
// that the driver reads "the DIR input is on" as the same turn in both
// modes, so the pulses for that turn now go in on DIR as well. The manual
// does not say, so check it before anything else: `move az 5 10`, then
// `move el 5 10`.
//
//   both axes turn the wrong way   the bet is wrong - swap ccw and cw in the
//                                  return line below, rather than inverting
//                                  all four specs
//   one axis turns the wrong way   that axis's invert_dir
constexpr unsigned step_line(unsigned cw, unsigned ccw, bool positive,
                             bool invert_dir)
{
    return (positive != invert_dir) ? ccw : cw;
}

// ============================================================================
// Which pins the PIN command may set
// ============================================================================
// PIN holds one pin high or low, for checking the wiring with a multimeter.
// Two groups are allowed:
//
//   GP4-GP7    the motor lines. A step pulse is 20 us long and no meter will
//              ever see one, so holding a line steady is the only way to
//              measure it. Every one of them is a step line: driving one LOW
//              makes the driver take a step the firmware did not count, so
//              that axis loses its zero and must be zeroed again. A line
//              left LOW is put back HIGH by the next move or TRACK ON.
//
//   GP12-GP15  spare. Nothing is meant to be on the other end on this board,
//              so a wrong level can do no harm.
//
// Everything else is refused.
constexpr uint8_t SPARE_PIN_MIN = 12;
constexpr uint8_t SPARE_PIN_MAX = 15;

constexpr bool pin_is_spare(uint8_t gpio)
{
    return gpio >= SPARE_PIN_MIN && gpio <= SPARE_PIN_MAX;
}

constexpr bool pin_is_motor(uint8_t gpio)
{
    return gpio == PIN_AZ_CW || gpio == PIN_AZ_CCW || gpio == PIN_EL_CW ||
           gpio == PIN_EL_CCW;
}

constexpr bool pin_may_be_set(uint8_t gpio)
{
    return pin_is_spare(gpio) || pin_is_motor(gpio);
}
