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
// Each driver takes two signals:
//
//   PUL - one pulse per step
//   DIR - which way to turn
//
//   driver        Pico 2 W    physical pin   wire
//   ------------  ----------  -------------  -----
//   Servo 1 PUL-  GP4         6              red
//   Servo 1 DIR-  GP5         7              red
//   Servo 2 PUL-  GP6         9              blue
//   Servo 2 DIR-  GP7         10             blue
//   all PUL+/DIR+ +5V, top two pins of each screw terminal, green
//
// ENA and ALM are not connected. The Pico cannot disable a driver and cannot
// tell when one faults - watch the red LED on the driver instead.
#pragma once

#include <stdint.h>

constexpr uint8_t PIN_AZ_PUL = 4;  // physical pin 6   - azimuth step
constexpr uint8_t PIN_AZ_DIR = 5;  // physical pin 7   - azimuth direction
constexpr uint8_t PIN_EL_PUL = 6;  // physical pin 9   - elevation step
constexpr uint8_t PIN_EL_DIR = 7;  // physical pin 10  - elevation direction

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
constexpr bool PUL_IDLE  = true;   // HIGH between steps
constexpr bool PUL_PULSE = false;  // LOW makes one step

// The DIR level that turns an axis the positive way, before that axis's
// invert_dir (motor_config.hpp) is applied.
constexpr bool DIR_POSITIVE = false;  // LOW

// The DIR level for a move one way or the other. Two things combine here,
// and getting either backwards sends the antenna the wrong way: the signal
// is active low, and invert_dir flips an axis whose gearing turns it round.
constexpr bool dir_level(bool positive, bool invert_dir)
{
    return (positive != invert_dir) ? DIR_POSITIVE : !DIR_POSITIVE;
}

// ============================================================================
// Which pins the PIN command may set
// ============================================================================
// PIN holds one pin high or low, for checking the wiring with a multimeter.
// Two groups are allowed:
//
//   GP4-GP7    the motor lines. A step pulse is 5 us long and no meter will
//              ever see one, so holding a line steady is the only way to
//              measure it. Driving a PUL line low makes the driver take a
//              step the firmware did not count, so that axis loses its zero
//              and must be zeroed again. A DIR line moves nothing - it is set
//              again at the start of every move - so it costs nothing.
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
    return gpio == PIN_AZ_PUL || gpio == PIN_AZ_DIR || gpio == PIN_EL_PUL ||
           gpio == PIN_EL_DIR;
}

constexpr bool pin_may_be_set(uint8_t gpio)
{
    return pin_is_spare(gpio) || pin_is_motor(gpio);
}
