#include "stepper.hpp"

#include "hardware/gpio.h"

#include "pins.hpp"

// Claim one line as an output, at rest.
//
// The level is set before the pin is switched to an output. Doing it the
// other way round makes the pin briefly drive low, and a low on either line
// is a step - so the motor would twitch every time the Pico restarted.
static void park(uint pin)
{
    gpio_init(pin);
    gpio_put(pin, LINE_IDLE);
    gpio_set_dir(pin, GPIO_OUT);
}

void Stepper::init()
{
    park(pin_cw_);
    park(pin_ccw_);
    dir_positive_ = true;
}

bool Stepper::pins_ok() const
{
    return gpio_get(pin_cw_) == LINE_IDLE && gpio_get(pin_ccw_) == LINE_IDLE;
}

void Stepper::release_lines()
{
    // Read from the pads, which is where PIN writes. PIN is only accepted
    // while the axes are idle, so a line found LOW here was left that way by
    // hand. This readback does not verify the voltage or current at the
    // driver's input.
    if (pins_ok()) {
        return;
    }
    gpio_put(pin_cw_, LINE_IDLE);
    gpio_put(pin_ccw_, LINE_IDLE);

    // Going back HIGH is not a step - the driver acts on the falling edge -
    // but the input takes time to let go, and a pulse sent before it has
    // would be lost. Nothing is turning yet, so the wait costs nothing.
    sleep_ms(RELEASE_PAUSE_MS);
}

void Stepper::step()
{
    const uint line =
        step_line(pin_cw_, pin_ccw_, dir_positive_, spec_.invert_dir);
    gpio_put(line, LINE_PULSE);
    busy_wait_us(PULSE_LOW_US);
    gpio_put(line, LINE_IDLE);

    pos_steps_ += dir_positive_ ? 1 : -1;
}

void Stepper::aim(const Move &m)
{
    dir_positive_ = (m.steps >= 0);
    release_lines();
}

void Stepper::begin(const Move &m, uint16_t id)
{
    plan_    = m;
    index_   = 0;
    aborted_ = false;
    move_id_ = id;
    mode_    = (m.n_steps > 0) ? Mode::Planned : Mode::Idle;

    // The first step is due immediately. aim() has already put both lines
    // at rest, so the driver is ready.
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
    // PIN is allowed while idle and can leave a line LOW. Put both back
    // before tracking can emit a pulse. PIN is refused once tracking is
    // armed, so nothing can disturb them again until it is off.
    release_lines();

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

    // Reversing is only a matter of which line the next pulse goes out on:
    // there is no direction pin to set and nothing to wait for, so the other
    // axis is never held up by it. It happens at a near standstill anyway,
    // because the velocity ramp passes through zero before it changes sign.
    dir_positive_ = (v_sps_ > 0.0f);

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
