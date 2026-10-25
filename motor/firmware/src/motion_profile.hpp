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

// How long to wait before taking step number i, counting from 0.
//
// This is the one function the motor loop calls over and over, so it answers
// from the plan alone and keeps no state of its own.
uint32_t interval_us(const Move &move, uint32_t i);
