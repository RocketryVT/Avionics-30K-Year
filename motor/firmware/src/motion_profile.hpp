// The motion maths: turning an angle into a number of steps and back, and
// working out the gap before each step so the axis speeds up and slows down
// smoothly.
//
// This is the maths that decides whether the antenna ends up where the
// operator asked, so it is kept small enough to check by hand. It touches no
// hardware; stepper.cpp does the pulsing.
#pragma once

#include <stdint.h>

#include "motor_config.hpp"

// How many steps to move this axis by the given angle.
//
// Rounds to the nearest whole step, away from zero on a tie, so that +X and -X
// always give the same number of steps in opposite directions.
int32_t mdeg_to_steps(const AxisSpec &axis, int32_t mdeg);

// The angle those steps actually represent.
//
// This is NOT the inverse of mdeg_to_steps. A step is a fixed size and most
// angles do not land on one, so asking for 10.000 deg really moves 9.990 deg.
// That difference is the honest answer and belongs in the results, which is
// why it is reported rather than hidden.
int32_t steps_to_mdeg(const AxisSpec &axis, int32_t steps);

// ============================================================================
// Working out whether a move is allowed
// ============================================================================

// Why a move was refused, or Ok if it was not.
enum class PlanResult {
    Ok,
    BadSpeed,    // speed outside SPEED_MIN_MDPS..axis.speed_max_mdps
    OutOfRange,  // the move would finish outside the axis's soft limits
    NoMovement,  // the angle is so small it rounds to zero steps
};

// What a move will do. Filled in even when the move is refused, so an error
// message can say where the move would have ended up.
struct Move {
    int32_t steps;       // signed - how many steps, and which way
    int32_t from_mdeg;   // where the axis is now
    int32_t to_mdeg;     // where it will really stop, after rounding to steps
    int32_t speed_mdps;  // as commanded

    // --- the speed plan, worked out by plan_move ---------------------------
    //
    // The axis speeds up, holds a steady speed, then slows down again by the
    // same amount. Speeds here are in steps per second, because that is what
    // decides the gap between one pulse and the next.
    uint32_t n_steps;      // how many steps in total (never negative)
    uint32_t n_ramp;       // steps spent speeding up, and again slowing down
    float v_start_sps;     // speed at the first and last step
    float v_cruise_sps;    // speed in the middle
    float accel_sps2;      // how fast the speed changes
    bool triangular;       // true if the move ends before reaching full speed
    uint32_t est_us;       // roughly how long the whole move will take
};

// Check a command and work out the move it describes.
//
// pos_steps is where the axis is now, counted in steps rather than degrees.
// Steps are what the machine actually does, so counting them is exact; storing
// an angle instead would re-round on every move and drift.
//
// delta_mdeg is relative and signed, because that is how the bench procedure
// is written ("+45, then -45") and what the operator types.
//
// Nothing here moves a motor. It only decides whether the move is allowed,
// and works out the speed plan if it is.
PlanResult plan_move(const AxisSpec &axis, int32_t pos_steps,
                     int32_t delta_mdeg, int32_t speed_mdps, Move *out);

// The same, but the angle is where to END UP rather than how far to turn.
//
// Both go through the same checks and fill in the same Move, so an absolute
// move and a relative one of the same length behave identically; only the
// arithmetic at the front differs.
//
// The subtraction belongs here rather than on the laptop because the step
// count is the only record of where the axis is, and it lives on this side.
// A laptop working out "target minus where I last heard it was" would be
// using a copy that is up to half a second old.
//
// Rounding happens once, in steps, for the same reason plan_move rounds
// once: going via a delta in millidegrees would round on the way in and
// again on the way out, and could land a step away from the angle asked for.
PlanResult plan_move_to(const AxisSpec &axis, int32_t pos_steps,
                        int32_t target_mdeg, int32_t speed_mdps, Move *out);

// How long to wait before taking step number i, counting from 0.
//
// This is the one function the motor loop calls over and over, so it answers
// from the plan alone and keeps no state of its own.
uint32_t interval_us(const Move &move, uint32_t i);

// ============================================================================
// Chasing a target that keeps moving
// ============================================================================
// Everything above plans a whole move up front and then follows it:
// plan_move decides how far, interval_us answers "how long until step i" from
// the plan and the step number alone. That is exact and easy to check, and it is also
// why a target that changes has nowhere to go - there is no longer a plan to
// be part way through.
//
// Chasing works the other way round. Nothing is planned. Velocity becomes
// something the axis CARRIES, and once per loop it is asked a single
// question: given how far off I am and how fast I am going, how fast should I
// be going now? A target that moves is not a special case - it is just the
// next answer.
//
// Two caps decide it:
//
//   the axis ceiling          speed_max_mdps, as for any move
//   what can still be braked  sqrt(2 * accel * error)
//
// The second is the one that matters. Go faster than that and the axis
// physically cannot slow down before the target, so it overshoots, comes
// back, overshoots the other way, and hunts for ever. That one square root is
// what makes this stable.
//
// The two hand over to each other on their own: far from the target the
// ceiling binds and the axis cruises, close to it the braking cap binds and
// the axis slows. The handover happens exactly where a trapezium would start
// braking, so for a target that is NOT moving this reproduces the shape
// plan_move would have planned - without planning it.
//
// What it costs: a steady lag behind a target that keeps moving
// ------------------------------------------------------------------
// The braking cap is the only thing asking the axis to move, so some error
// has to remain for it to move at all. To hold a steady rate u the loop
// settles at the error where the cap equals u:
//
//     sqrt(2 * accel * lag) = u      ->      lag = u * u / (2 * accel)
//
// With the built-in accelerations that comes out at:
//
//     target rate      geared az lag       geared el lag
//      20 deg/s          3.4 deg             2.1 deg
//      40 deg/s         13.6 deg             8.5 deg
//
// Against a half-power beam of about +/-25 degrees the slower end is
// nothing. It grows with the SQUARE of the rate though, so the fast part of
// a boost is where to look. The fix, when it is needed, is to feed the
// target's own rate forward into the commanded velocity, so the loop no
// longer needs an error to sustain motion - but that needs the target's rate,
// which means the target stream, so it is not here yet.
//
// Note this is lag behind a MOVING target, not error against a still one. A
// target that stops is arrived at exactly, inside the deadband.
//
// Hardware-free, like the rest of this file, so the whole loop can be run
// thousands of times at a desk and checked.

// The velocity this axis should be turning at, in steps per second, signed.
//
//   error_steps  target minus where the axis is, in steps. Signed.
//   v_sps        what it is turning at now. Signed. Zero when stopped.
//   dt_s         seconds since this was last asked. Clamped to
//                CHASE_MAX_DT_S, so a loop that was held up resumes gently
//                instead of jumping.
//
// Inside the axis's deadband the answer is zero: the point of the deadband is
// that a fix noisy by less than that must not set the motors chattering.
//
// Leaving a standstill the answer jumps straight to v_start_mdps rather than
// creeping up from nothing, for the reason given there - from a true zero the
// first step interval is enormous and looks like a hang. That is a kick at
// departure only, not a floor: on the way in, the braking cap is allowed all
// the way down to zero, which is what lets it settle instead of dithering.
float chase_velocity(const AxisSpec &axis, int32_t error_steps, float v_sps,
                     float dt_s);

// How long to wait before the next step at this velocity.
//
// The sign is ignored - direction is the caller's business. A velocity of
// zero, or one so small it would mean waiting longer than MAX_INTERVAL_US,
// comes back as MAX_INTERVAL_US rather than as infinity.
uint32_t interval_us_at(float v_sps);
