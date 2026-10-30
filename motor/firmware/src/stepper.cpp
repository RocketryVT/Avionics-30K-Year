#include "stepper.hpp"

#include "hardware/gpio.h"

#include "pins.hpp"

void Stepper::init()
{
    // Set the level before switching the pin to an output. Doing it the other
    // way round makes the pin briefly drive low, and a low on PUL is a step -
    // so the motor would twitch every time the Pico restarted.
    gpio_init(pin_pul_);
    gpio_put(pin_pul_, PUL_IDLE);
    gpio_set_dir(pin_pul_, GPIO_OUT);

    dir_positive_ = true;
    gpio_init(pin_dir_);
    gpio_put(pin_dir_, dir_level(dir_positive_, spec_.invert_dir));
    gpio_set_dir(pin_dir_, GPIO_OUT);
}

bool Stepper::pins_ok() const
{
    return gpio_get(pin_pul_) == PUL_IDLE &&
           gpio_get(pin_dir_) == dir_level(dir_positive_, spec_.invert_dir);
}

void Stepper::set_direction(bool positive)
{
    // Check the pad rather than only the remembered sign: an idle PIN
    // command may have written a different level, or a replacement spec may
    // have changed invert_dir. This readback does not verify the voltage or
    // current at the driver's input.
    const bool level     = dir_level(positive, spec_.invert_dir);
    const bool reversing = (level != gpio_get(pin_dir_));

    dir_positive_ = positive;
    gpio_put(pin_dir_, level);

    // The driver wants the direction settled well before the next pulse, and
    // a reversal is where that is tightest, so it gets an extra pause.
    if (reversing) {
        sleep_ms(REVERSE_PAUSE_MS);
    }
    busy_wait_us(DIR_SETUP_US);
}

void Stepper::step()
{
    gpio_put(pin_pul_, PUL_PULSE);
    busy_wait_us(PULSE_LOW_US);
    gpio_put(pin_pul_, PUL_IDLE);

    pos_steps_ += dir_positive_ ? 1 : -1;
}

void Stepper::aim(const Move &m)
{
    set_direction(m.steps >= 0);
}

void Stepper::begin(const Move &m, uint16_t id)
{
    plan_    = m;
    index_   = 0;
    aborted_ = false;
    move_id_ = id;
    mode_    = (m.n_steps > 0) ? Mode::Planned : Mode::Idle;

    // The first step is due immediately. aim() has already waited out the
    // setup time, so the driver is ready.
    next_step_time_ = get_absolute_time();
    first_pulse_    = next_step_time_;
    last_pulse_     = next_step_time_;
}

bool Stepper::tick(absolute_time_t now)
{
    if (mode_ == Mode::Chasing || mode_ == Mode::Braking) {
        return chase_tick(now, mode_ == Mode::Braking);
    }
    if (mode_ != Mode::Planned) {
        return false;
    }

    // Not due yet. This is the usual answer - the loop runs far more often
    // than steps are needed.
    if (absolute_time_diff_us(now, next_step_time_) > 0) {
        return true;
    }

    if (index_ == 0) {
        first_pulse_ = now;
    }
    step();
    last_pulse_ = now;
    index_++;

    if (index_ >= plan_.n_steps) {
        mode_ = Mode::Idle;
        return false;
    }

    const uint32_t gap = interval_us(plan_, index_);

    // The next deadline is measured from the last deadline, not from now, so
    // small delays do not add up over thousands of steps.
    absolute_time_t next = delayed_by_us(next_step_time_, gap);

    // But if something held the loop up for longer than a whole step, that
    // would fire a burst of pulses back to back trying to catch up - which is
    // exactly how a stepper loses its position. Better to give up the lost
    // time and carry on at the right rate.
    if (absolute_time_diff_us(next, now) > static_cast<int64_t>(gap)) {
        next = delayed_by_us(now, gap);
    }
    next_step_time_ = next;
    return true;
}

// ----------------------------------------------------------------------------
// Chasing
// ----------------------------------------------------------------------------

void Stepper::arm_chase()
{
    // PIN and spec changes are allowed while idle and can leave DIR out of
    // sync with the remembered sign. Reassert it using the current spec and
    // wait for setup before tracking can emit a pulse. If the first target
    // asks for the other direction, chase_tick() will set that up as usual.
    // Neither PIN nor spec changes are allowed once tracking is armed.
    set_direction(dir_positive_);

    // The target starts where the axis is, so arming on its own never moves
    // anything. Something has to send a target first.
    target_steps_ = pos_steps_;
    v_sps_        = 0.0f;
    index_        = 0;
    aborted_      = false;
    move_id_      = 0;
    last_chase_   = get_absolute_time();
    next_step_time_ = last_chase_;
    first_pulse_  = last_chase_;
    last_pulse_   = last_chase_;
    mode_         = Mode::Chasing;
}

void Stepper::disarm_chase()
{
    // Ramped down rather than cut off, so an axis at speed is not jerked to
    // a halt. Braking chases an error of zero, which is the same code.
    if (mode_ == Mode::Chasing) {
        mode_ = Mode::Braking;
    }
}

void Stepper::set_target_steps(int32_t steps)
{
    target_steps_ = steps;
}

int32_t Stepper::velocity_mdps() const
{
    // steps/sec back into millidegrees/sec. Rounded, and the sign kept.
    const float per_deg = static_cast<float>(spec_.steps_num) /
                          static_cast<float>(spec_.steps_den);
    if (per_deg <= 0.0f) {
        return 0;
    }
    const float mdps = v_sps_ * 1000.0f / per_deg;
    return static_cast<int32_t>(mdps < 0.0f ? mdps - 0.5f : mdps + 0.5f);
}

float Stepper::since_last_chase(absolute_time_t now)
{
    const int64_t gap = absolute_time_diff_us(last_chase_, now);
    last_chase_       = now;
    if (gap <= 0) {
        return 0.0f;
    }
    // chase_velocity clamps this to CHASE_MAX_DT_S, so a loop held up by a
    // flash write resumes gently instead of jumping.
    return static_cast<float>(gap) / 1000000.0f;
}

bool Stepper::chase_tick(absolute_time_t now, bool braking)
{
    const float dt = since_last_chase(now);
    const int32_t error = braking ? 0 : (target_steps_ - pos_steps_);

    v_sps_ = chase_velocity(spec_, error, v_sps_, dt);

    if (v_sps_ == 0.0f) {
        if (braking) {
            mode_ = Mode::Idle;  // come to rest; disarming is complete
        }
        // Still armed, just inside the deadband with nothing to do. The next
        // step is due immediately once a target moves, rather than after a
        // stale deadline.
        next_step_time_ = now;
        return false;
    }

    if (absolute_time_diff_us(now, next_step_time_) > 0) {
        return true;  // not due yet
    }

    // The driver needs the direction pin settled before a pulse, and
    // set_direction() blocks to arrange that. It is only ever reached here at
    // a near standstill: the velocity ramp passes through zero before it
    // changes sign, which the host tests check on every axis in both gear
    // modes.
    const bool positive = (v_sps_ > 0.0f);
    if (positive != dir_positive_) {
        set_direction(positive);
        now = get_absolute_time();
    }

    step();
    last_pulse_ = now;

    const uint32_t gap = interval_us_at(v_sps_);
    absolute_time_t next = delayed_by_us(next_step_time_, gap);

    // Same guard as a planned move: a loop that was held up must not fire a
    // burst of pulses trying to catch up, which is how a stepper loses its
    // position.
    if (absolute_time_diff_us(next, now) > static_cast<int64_t>(gap)) {
        next = delayed_by_us(now, gap);
    }
    next_step_time_ = next;
    return true;
}

// ----------------------------------------------------------------------------
// Stopping
// ----------------------------------------------------------------------------

void Stepper::request_stop()
{
    // While chasing there is no plan to shorten: stop means stop tracking
    // and come to rest.
    if (mode_ == Mode::Chasing) {
        disarm_chase();
        aborted_ = true;
        return;
    }
    if (mode_ != Mode::Planned) {
        return;
    }

    // Stop by shortening the move rather than with a separate braking mode.
    // interval_us() already slows down over the last n_ramp steps, so ending
    // the move that many steps from here gives a proper ramp down from
    // whatever speed it is doing now.
    //
    // While still speeding up, the distance needed to slow down is the
    // distance already covered, so it is index_ rather than n_ramp.
    const uint32_t braking   = (index_ < plan_.n_ramp) ? index_ : plan_.n_ramp;
    const uint32_t shortened = index_ + braking + 1;

    if (shortened < plan_.n_steps) {
        plan_.n_steps = shortened;
    }
    aborted_ = true;
}

void Stepper::estop()
{
    if (mode_ == Mode::Idle) {
        return;
    }
    mode_    = Mode::Idle;
    v_sps_   = 0.0f;
    aborted_ = true;

    // Pulses stopped mid-move with no ramp down. A closed-loop driver will
    // settle somewhere near, but not exactly where our count says, and there
    // is no feedback to find out. Refusing further moves until the operator
    // re-zeros is the honest response.
    zeroed_ = false;
}

int32_t Stepper::steps_done() const
{
    const int32_t n = static_cast<int32_t>(index_);
    return (plan_.steps < 0) ? -n : n;
}

uint32_t Stepper::elapsed_us() const
{
    const int64_t us = absolute_time_diff_us(first_pulse_, last_pulse_);
    return (us < 0) ? 0 : static_cast<uint32_t>(us);
}
