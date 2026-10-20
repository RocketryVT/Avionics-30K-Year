// The numbers that describe the two motor axes: how many steps make a degree,
// how far each axis may travel, how fast it may go and how quickly it may
// change speed. This is the file to edit when the machine changes.
//
// Two integer units are used throughout:
//
//   mdeg  millidegrees          45 degrees  -> 45000
//   mdps  millidegrees/second    5 deg/sec  ->  5000
//
// Integers rather than floats because these values cross the USB link as
// text, and whole numbers survive that exactly. The laptop converts to and
// from degrees for the operator.
//
// Every angle here is at the OUTPUT SHAFT OF THE 10:1 PLANETARY GEARBOX - not
// at the motor, and (for now) not at the antenna. See "Steps per degree".
//
// Acceleration and travel can also be changed at the motor> prompt, with
// `set` and `save`, without reflashing (see settings.hpp). The numbers here
// are then the BUILT-IN ones: what the Pico starts with until settings are
// saved, and the outer bounds any setting must stay inside. Change one here
// and reflash, and saved settings are ignored - the code wins when the code
// changes.
#pragma once

#include <stdint.h>

// ============================================================================
// Per-axis description
// ============================================================================
struct AxisSpec {
    // Steps per output degree, kept as an exact fraction steps_num/steps_den
    // rather than a decimal. 44.444... never ends, and rounding it here would
    // make the error grow over a long move.
    int32_t steps_num;
    int32_t steps_den;

    // How far this axis may travel, in millidegrees. There are no limit
    // switches on this machine, so these two numbers are the only thing
    // stopping a move from driving the gimbal into a hard stop.
    int32_t min_mdeg;
    int32_t max_mdeg;

    // Set true if a positive command turns the axis the wrong way. That
    // depends on the driver's SW5 switch and on the gear mesh, so it cannot
    // be known until it is tried.
    bool invert_dir;

    // The speed a move begins and ends at. See V_START_MDPS.
    int32_t v_start_mdps;

    // How quickly the axis may change speed, in millideg/sec/sec.
    int32_t accel_mdps2;
};

// Moves start and end at 5 deg/sec rather than at a dead stop.
//
// A stepper does not need to ease away from rest at these speeds: 5 deg/sec
// is about 8 rpm at the motor, behind the 10:1 gearbox. Starting here buys
// two things:
//
//   1. The bench procedure runs at 5 deg/sec, so those moves have no ramp at
//      all. They run at one steady speed and take exactly angle/speed
//      seconds, which keeps the results easy to check by hand.
//   2. Starting from a true zero makes the first step interval enormous
//      (tens of milliseconds), which looks like the program has hung.
constexpr int32_t V_START_MDPS = 5000;  // 5 deg/sec

// ----------------------------------------------------------------------------
// Steps per degree - both axes
// ----------------------------------------------------------------------------
// Where 400/9 comes from:
//
//   driver pulses per motor turn          1600        (CL57TE DIP switches)
//   gearbox                              10 : 1       (planetary)
//
//   steps per gearbox output turn   = 1600 * 10 = 16000
//   steps per gearbox output degree = 16000/360 = 400/9 = 44.444...
//
// Both axes have the same motor, driver setting and gearbox, so both get the
// same fraction. Every multiple of 9 degrees is a whole number of steps:
// 45 degrees is exactly 2000.
//
// THE 1600 IS A SWITCH SETTING, AND NOTHING IN SOFTWARE CAN CHECK IT. It is
// DIP switches 1-4 on each driver, which the CL57T manual reads as:
//
//   1600 pulses/turn (microstep 8)   SW1 on   SW2 off  SW3 on   SW4 on
//    800 pulses/turn (microstep 4)   SW1 off  SW2 on   SW3 on   SW4 on
//
// The drivers on this machine were checked at 1600 on 17 Oct 2026. If one is
// ever swapped or reset, check this first: get it wrong and every angle is
// out by a clean factor of two, while the Pico reports the angle it meant to
// turn.
//
// For later - the full antenna. After each gearbox there is a final gear
// pair, and once the whole antenna is tested the numbers become:
//
//   azimuth    32 : 14 pair -> 10 * 32/14 = 160/7 : 1 (22.857 : 1)
//              1600 * 160/7 / 360 = 6400/63 = 101.587 steps per degree
//   elevation  20 : 14 pair -> 10 * 20/14 = 100/7 : 1 (14.286 : 1)
//              1600 * 100/7 / 360 = 4000/63 =  63.492 steps per degree
//
// Count the teeth before switching to these - nobody has yet. Using them
// while the gear pair is not in the drive train makes every move overshoot by
// the pair's ratio (1.43x on elevation, seen at the bench on 18 Oct 2026). The
// travel limits will need changing then too, since they will be antenna
// angles rather than gearbox angles.

// ----------------------------------------------------------------------------
// Azimuth (the board calls it Servo 1)
// ----------------------------------------------------------------------------
// Travel is one full turn of the gearbox shaft.
//
// Acceleration is 135 deg/sec/sec, verified at the bench on 18 Oct 2026. It
// reaches the 181 deg/sec ceiling in 1.3 seconds, over about 121 degrees, so
// a move needs about 242 degrees to get up to full speed and back down;
// shorter moves peak lower. Raise it one step at a time - `set az accel 150`
// at the prompt, no reflash - watching the driver's alarm light and the
// return-to-zero reading.
constexpr AxisSpec AZ_SPEC = {
    400,     // steps_num
    9,       // steps_den     -> 44.444 steps per degree
    0,       // min_mdeg      0 degrees
    360000,  // max_mdeg      360 degrees
    false,   // invert_dir    unverified
    V_START_MDPS,
    135000,  // accel_mdps2   135 deg/sec/sec (aiming for 180)
};

// ----------------------------------------------------------------------------
// Elevation (the board calls it Servo 2)
// ----------------------------------------------------------------------------
// Travel is 0 to 360 degrees of the gearbox shaft, verified safe at the bench
// on 18 Oct 2026 with the axis tested on its own. On the full antenna the
// mechanism only swings 0 to 180, so cut it back then: `set el max 180` and
// `save` at the prompt, or here.
//
// Acceleration is the same 135 deg/sec/sec as azimuth. Recheck it once the
// axis is lifting the antenna's weight.
constexpr AxisSpec EL_SPEC = {
    400,     // steps_num
    9,       // steps_den     -> 44.444 steps per degree
    0,       // min_mdeg      0 degrees
    360000,  // max_mdeg      360 degrees (the antenna itself swings 0-180)
    false,   // invert_dir    unverified - expect this one to need flipping
    V_START_MDPS,
    135000,  // accel_mdps2   135 deg/sec/sec
};

// ============================================================================
// How far acceleration may be set at the prompt
// ============================================================================
// Travel set at the prompt can only narrow the limits above, never widen
// them. Acceleration is different: the point is to try higher values without
// reflashing, so it gets bounds of its own.
//
// These guard against typing mistakes; they are not a promise the motors can
// do it. 360 is twice the 180 aimed for, so a slipped digit (1800) is refused
// while every value worth trying is allowed. 10 keeps a move from being so
// gentle it seems to hang.
constexpr int32_t ACCEL_MIN_MDPS2 = 10000;   //  10 deg/sec/sec
constexpr int32_t ACCEL_MAX_MDPS2 = 360000;  // 360 deg/sec/sec

// ============================================================================
// Speed range
// ============================================================================
// The floor exists so a typo like "0" cannot start a move that never ends.
//
// The ceiling comes from the motor's torque curve (23HS40-5004D-E1000 on a
// CL57T at 36 V): 181 deg/sec is about 300 rpm at the motor, still on the
// flat part of the curve with roughly 2.0 N.m of pull-out torque, against
// 2.3 N.m at low speed. Past about 600 rpm (360 deg/sec) the torque halves.
// Below 36 V the curve falls off sooner, so measure the supply before
// raising this.
constexpr int32_t SPEED_MIN_MDPS = 100;     //   0.1 deg/sec
constexpr int32_t SPEED_MAX_MDPS = 181000;  // 181   deg/sec

// Sanity rails on the gap between two steps. Nothing should ever reach these:
// 181 deg/sec is one step every 124 us, and 0.1 deg/sec one every 225 ms. If
// a computed interval hits a rail the maths is wrong, and failing toward
// "too slow" is the safe direction.
constexpr uint32_t MIN_INTERVAL_US = 50;      // 20 kHz, far above anything used
constexpr uint32_t MAX_INTERVAL_US = 500000;  // half a second between steps

// ============================================================================
// Pulse timing
// ============================================================================
// The CL57TE manual's minimums are a 1 us pulse and 5 us of DIR setup. These
// are what last year's ground station ran, which is far more generous. There
// is no reason to shave them: even at full speed one step is still 124 us
// from the next.
constexpr uint32_t PULSE_LOW_US     = 5;    // how long PUL is held LOW
constexpr uint32_t DIR_SETUP_US     = 100;  // settle time after changing DIR
constexpr uint32_t REVERSE_PAUSE_MS = 1;    // extra pause before reversing
