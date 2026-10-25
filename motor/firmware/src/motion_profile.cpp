#include "motion_profile.hpp"

#include <math.h>

// Convert a speed or an acceleration from "per degree" units into "per step"
// units. Everything about timing is easier once it is counted in steps.
static float per_step(const AxisSpec &axis, int32_t milli_per_sec)
{
    const float steps_per_deg =
        static_cast<float>(axis.steps_num) / static_cast<float>(axis.steps_den);
    return static_cast<float>(milli_per_sec) * steps_per_deg / 1000.0f;
}

// Divide a by b and round to the nearest whole number, going away from zero
// when the answer lands exactly halfway.
//
// C++ integer division truncates toward zero, so 2285.7 would become 2285 and
// the antenna would stop a third of a step short on every single move. Adding
// half the divisor before dividing fixes that.
//
// b is always positive here. a can be negative, and the sign is handled by
// doing the work on the magnitude and putting the sign back afterwards -
// otherwise rounding would be lopsided and +45 and -45 would disagree.
static int64_t div_round(int64_t a, int64_t b)
{
    if (a >= 0) {
        return (a + b / 2) / b;
    }
    return -((-a + b / 2) / b);
}

int32_t mdeg_to_steps(const AxisSpec &axis, int32_t mdeg)
{
    // steps = mdeg/1000 * steps_num/steps_den, rearranged to divide only once.
    //
    // int64 because the top can get too big for int32. In direct mode a full
    // turn is 360000 * 400 = 144 million, which fits, but in geared mode
    // azimuth is 360000 * 12800 = 4.6 billion - well past the int32 ceiling
    // of 2.15 billion, where it would silently come out negative. The gear
    // switch is flipped at the prompt, so this is reached at run time, not
    // by editing the code.
    const int64_t top    = static_cast<int64_t>(mdeg) * axis.steps_num;
    const int64_t bottom = static_cast<int64_t>(axis.steps_den) * 1000;
    return static_cast<int32_t>(div_round(top, bottom));
}

int32_t steps_to_mdeg(const AxisSpec &axis, int32_t steps)
{
    const int64_t top    = static_cast<int64_t>(steps) * axis.steps_den * 1000;
    const int64_t bottom = axis.steps_num;
    return static_cast<int32_t>(div_round(top, bottom));
}

// Work out the speed-up / hold / slow-down plan for a move whose step count
// and commanded speed are already known.
//
// The shape is a trapezium: speed rises, flattens, falls. If the move is too
// short to reach the commanded speed before it has to start slowing down, the
// flat part disappears and the shape becomes a triangle.
static void plan_speed(const AxisSpec &axis, Move *m)
{
    m->n_steps    = static_cast<uint32_t>(m->steps < 0 ? -m->steps : m->steps);
    m->accel_sps2 = per_step(axis, axis.accel_mdps2);

    const float v_cmd = per_step(axis, m->speed_mdps);
    float v0          = per_step(axis, axis.v_start_mdps);

    // A command slower than the normal starting speed just runs at that
    // slower speed the whole way. Starting faster than the operator asked for
    // would be surprising and wrong.
    if (v_cmd < v0) {
        v0 = v_cmd;
    }

    m->v_start_sps  = v0;
    m->v_cruise_sps = v_cmd;

    // How many steps it takes to get from v0 up to the commanded speed, from
    // v^2 = v0^2 + 2*a*distance rearranged for distance.
    uint32_t n_ramp = 0;
    if (v_cmd > v0 && m->accel_sps2 > 0.0f) {
        const float d = (v_cmd * v_cmd - v0 * v0) / (2.0f * m->accel_sps2);
        n_ramp        = static_cast<uint32_t>(ceilf(d));
    }

    // Speeding up and slowing down both need that many steps. If there is not
    // room for both, split the move down the middle and accept whatever speed
    // it reaches - this is the triangular case.
    m->triangular = (2 * n_ramp > m->n_steps);
    if (m->triangular) {
        n_ramp          = m->n_steps / 2;
        m->v_cruise_sps = sqrtf(v0 * v0 + 2.0f * m->accel_sps2 *
                                              static_cast<float>(n_ramp));
    }
    m->n_ramp = n_ramp;

    // Add up every interval rather than using a closed-form time. It costs a
    // few microseconds here, happens once per move, and cannot disagree with
    // what the motor loop will actually do.
    uint64_t total = 0;
    for (uint32_t i = 0; i < m->n_steps; i++) {
        total += interval_us(*m, i);
    }
    m->est_us = (total > 0xFFFFFFFFu) ? 0xFFFFFFFFu
                                      : static_cast<uint32_t>(total);
}

// The body shared by plan_move and plan_move_to. Both work out how many
// steps to turn and then everything else is identical, so the checks and the
// speed plan live here once.
static PlanResult plan_from_steps(const AxisSpec &axis, int32_t pos_steps,
                                  int32_t delta_steps, int32_t speed_mdps,
                                  Move *out)
{
    const int32_t target_steps = pos_steps + delta_steps;

    // Fill this in before any check can fail, so an error message can report
    // where the move would have ended up.
    out->steps      = delta_steps;
    out->from_mdeg  = steps_to_mdeg(axis, pos_steps);
    out->to_mdeg    = steps_to_mdeg(axis, target_steps);
    out->speed_mdps = speed_mdps;

    // The ceiling is the axis's own, not a global: geared mode scales it
    // down with the gearing, so the motor is never commanded past the speed
    // its torque curve was read at.
    if (speed_mdps < SPEED_MIN_MDPS || speed_mdps > axis.speed_max_mdps) {
        return PlanResult::BadSpeed;
    }

    // A tiny angle can round to no steps at all. Say so rather than reporting
    // a successful move that does nothing - otherwise the operator writes a
    // number in the data table for a move that never happened.
    if (delta_steps == 0) {
        return PlanResult::NoMovement;
    }

    // Compare in steps, not degrees.
    //
    // The limits are written in degrees, but the machine only moves in whole
    // steps, and converting the result back to degrees rounds a second time.
    // For example, at geared azimuth's 101.587 steps per degree, 36571 steps
    // reads back as 359.996 and one more as 360.006 - so a 360.000 limit
    // checked in degrees would refuse the step nearest to it.
    // Converting the limit into steps once removes the second rounding and
    // lets a move land exactly on the boundary.
    const int32_t min_steps = mdeg_to_steps(axis, axis.min_mdeg);
    const int32_t max_steps = mdeg_to_steps(axis, axis.max_mdeg);

    if (target_steps < min_steps || target_steps > max_steps) {
        return PlanResult::OutOfRange;
    }

    plan_speed(axis, out);
    return PlanResult::Ok;
}

PlanResult plan_move(const AxisSpec &axis, int32_t pos_steps,
                     int32_t delta_mdeg, int32_t speed_mdps, Move *out)
{
    return plan_from_steps(axis, pos_steps, mdeg_to_steps(axis, delta_mdeg),
                           speed_mdps, out);
}

PlanResult plan_move_to(const AxisSpec &axis, int32_t pos_steps,
                        int32_t target_mdeg, int32_t speed_mdps, Move *out)
{
    // Subtracted in steps, not in millidegrees, so the angle is rounded
    // exactly once. The axis ends up on the nearest whole step to the angle
    // asked for, and PLAN reports which one that is.
    const int32_t target_steps = mdeg_to_steps(axis, target_mdeg);
    return plan_from_steps(axis, pos_steps, target_steps - pos_steps,
                           speed_mdps, out);
}

// Clamp a to the range lo..hi.
static float clamped(float a, float lo, float hi)
{
    if (a < lo) {
        return lo;
    }
    return (a > hi) ? hi : a;
}

float chase_velocity(const AxisSpec &axis, int32_t error_steps, float v_sps,
                     float dt_s)
{
    const float accel   = per_step(axis, axis.accel_mdps2);
    const float v_max   = per_step(axis, axis.speed_max_mdps);
    const float v_start = per_step(axis, axis.v_start_mdps);
    const int32_t dead  = mdeg_to_steps(axis, axis.deadband_mdeg);

    const float dt = clamped(dt_s, 0.0f, CHASE_MAX_DT_S);

    // Worked on the magnitude with the sign put back at the end, so that an
    // error of +N and one of -N are answered with the same speed in opposite
    // directions rather than two subtly different ones.
    const int32_t away = (error_steps < 0) ? -error_steps : error_steps;

    float want = 0.0f;
    if (away > dead) {
        // How fast this axis could be going and still come to rest by the
        // time it arrives. Measured to the MIDDLE of the deadband, not its
        // near edge: braking from sqrt(2*a*d) covers exactly d, so aiming at
        // the middle means arriving there already stopped. The deadband's job
        // is only to stop it setting off again, not to leave room to brake
        // in.
        const float brakeable =
            sqrtf(2.0f * accel * static_cast<float>(away));
        want = (brakeable < v_max) ? brakeable : v_max;
        if (error_steps < 0) {
            want = -want;
        }
    }

    // Leaving a standstill: go straight to the starting speed rather than
    // creeping up from nothing. Only ever applied on the way out - never when
    // the braking cap is what is holding the speed down - so it cannot stop
    // the axis settling.
    const float stopped = 0.5f;  // half a step per second is standing still
    if (v_sps > -stopped && v_sps < stopped && want != 0.0f) {
        const float kick = (want < 0.0f) ? -v_start : v_start;
        // Unless the target is so close that the starting speed would carry
        // it past: then the braking cap wins and it creeps in.
        if ((want < 0.0f ? -want : want) >= v_start) {
            return kick;
        }
        return want;
    }

    // Change speed by no more than the axis is allowed to.
    const float most = accel * dt;
    return clamped(want, v_sps - most, v_sps + most);
}

uint32_t interval_us_at(float v_sps)
{
    const float speed = (v_sps < 0.0f) ? -v_sps : v_sps;
    if (speed < 0.001f) {
        return MAX_INTERVAL_US;  // standing still: nothing is due
    }

    const float us = 1000000.0f / speed;
    if (us <= static_cast<float>(MIN_INTERVAL_US)) {
        return MIN_INTERVAL_US;
    }
    if (us >= static_cast<float>(MAX_INTERVAL_US)) {
        return MAX_INTERVAL_US;
    }
    return static_cast<uint32_t>(us);
}

uint32_t interval_us(const Move &move, uint32_t i)
{
    float v;

    if (move.n_steps == 0) {
        v = move.v_start_sps;
    } else if (i < move.n_ramp) {
        // Speeding up. v^2 = v0^2 + 2*a*distance, the standard constant
        // acceleration result, with distance counted in steps.
        v = sqrtf(move.v_start_sps * move.v_start_sps +
                  2.0f * move.accel_sps2 * static_cast<float>(i));
    } else if (i + move.n_ramp < move.n_steps) {
        // Holding the commanded speed.
        v = move.v_cruise_sps;
    } else {
        // Slowing down. Mirror of the ramp up: work out how many steps are
        // left and use the same curve, so the move is symmetric.
        const uint32_t left = (i < move.n_steps) ? (move.n_steps - 1 - i) : 0;
        v = sqrtf(move.v_start_sps * move.v_start_sps +
                  2.0f * move.accel_sps2 * static_cast<float>(left));
    }

    if (v < 0.001f) {
        return MAX_INTERVAL_US;  // cannot happen; avoids dividing by zero
    }

    const float us = 1000000.0f / v;

    if (us <= static_cast<float>(MIN_INTERVAL_US)) {
        return MIN_INTERVAL_US;
    }
    if (us >= static_cast<float>(MAX_INTERVAL_US)) {
        return MAX_INTERVAL_US;
    }
    return static_cast<uint32_t>(us);
}
