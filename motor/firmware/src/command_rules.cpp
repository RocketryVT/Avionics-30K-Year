#include "command_rules.hpp"

#include "pins.hpp"
#include "settings.hpp"

namespace {

// Each reply is written straight into the Decision's next free line, then
// counted by say(). A format_* function returns 0 when it wrote nothing
// usable, and then nothing is counted.
char *line_buf(Decision &d)
{
    return d.lines[d.line_count];
}

void say(Decision &d, size_t written)
{
    if (written > 0) {
        d.line_count++;
    }
}

Decision refuse(const Command &c, ErrCode code, const char *detail)
{
    Decision d;
    say(d, format_err(line_buf(d), MAX_REPLY_LEN, c.id, code, detail));
    return d;
}

// The settings in force on an axis, which moves are judged against.
const AxisSpec &spec_for(AxisId axis, const MachineState &s)
{
    return (axis == AxisId::El) ? s.el_spec : s.az_spec;
}

// What motor_config.hpp says for this axis in this gear mode, which
// settings are bounded by. Both the travel limits and the acceleration
// bounds differ between the modes, so the mode has to come in here.
AxisSpec built_in_for(AxisId axis, GearMode gear)
{
    return (axis == AxisId::El) ? el_built_in(gear) : az_built_in(gear);
}

bool axis_zeroed(AxisId axis, const MachineState &s)
{
    return (axis == AxisId::El) ? s.el_zeroed : s.az_zeroed;
}

int32_t axis_pos(AxisId axis, const MachineState &s)
{
    return (axis == AxisId::El) ? s.el_pos_steps : s.az_pos_steps;
}

// ---------------------------------------------------------------------------
// MOVE
// ---------------------------------------------------------------------------

// One axis's share of a MOVE.
struct Half {
    bool wanted  = false;
    int32_t mdps = 0;
    int32_t mdeg = 0;
};

// The command's share for one axis, wherever in the line it was written.
Half half_for(const Command &c, AxisId axis)
{
    Half h;
    if (c.axis == axis) {
        h = {true, c.mdps, c.mdeg};
    } else if (c.axis2 == axis) {
        h = {true, c.mdps2, c.mdeg2};
    }
    return h;
}

// Check one axis's share and plan it, exactly as a one-axis MOVE is checked.
// Fills in *m and returns true if it may go; otherwise writes the refusal
// into d and returns false.
bool check_half(const Command &c, AxisId axis, const Half &h,
                const MachineState &s, Move *m, Decision &d)
{
    // Without a zero the soft limits are measured from an unknown origin,
    // which is no protection at all. This is the main thing standing between
    // a mistyped command and the gimbal's hard stops.
    if (!axis_zeroed(axis, s)) {
        say(d, format_err(line_buf(d), MAX_REPLY_LEN, c.id, ErrCode::NoZero,
                          axis_name(axis)));
        return false;
    }

    const AxisSpec &spec = spec_for(axis, s);
    switch (plan_move(spec, axis_pos(axis, s), h.mdeg, h.mdps, m)) {
    case PlanResult::BadSpeed:
        say(d, format_err(line_buf(d), MAX_REPLY_LEN, c.id, ErrCode::BadSpeed,
                          axis_name(axis)));
        return false;

    case PlanResult::OutOfRange:
        // Refused whole, never shortened. A clamped move would put a
        // different angle on the gimbal than the one written down.
        say(d, format_err_limit(line_buf(d), MAX_REPLY_LEN, c.id, axis,
                                m->to_mdeg, spec.min_mdeg, spec.max_mdeg));
        return false;

    case PlanResult::NoMovement:
        say(d, format_err(line_buf(d), MAX_REPLY_LEN, c.id, ErrCode::ZeroSteps,
                          axis_name(axis)));
        return false;

    case PlanResult::Ok:
        break;
    }
    return true;
}

Decision do_move(const Command &c, const MachineState &s)
{
    if (c.axis != AxisId::Az && c.axis != AxisId::El) {
        return refuse(c, ErrCode::BadAxis, "");
    }

    // Looked up by axis rather than taken in the order written, so the checks
    // and the PLAN lines always go azimuth first.
    const Half az = half_for(c, AxisId::Az);
    const Half el = half_for(c, AxisId::El);

    // Both halves are checked before either starts. If one half is refused
    // the other does not move on its own: the operator asked for both, and
    // half of it is a different move.
    Decision d;
    Move az_move{};
    Move el_move{};
    if (az.wanted && !check_half(c, AxisId::Az, az, s, &az_move, d)) {
        return d;
    }
    if (el.wanted && !check_half(c, AxisId::El, el, s, &el_move, d)) {
        return d;
    }

    d.act     = Act::StartMove;
    d.id      = c.id;
    d.move_az = az.wanted;
    d.move_el = el.wanted;
    d.az_move = az_move;
    d.el_move = el_move;

    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, Verb::Move));

    // PLAN carries the end angle after rounding to whole steps, which is not
    // simply start plus commanded: 10 degrees is 444 steps, or 9.990 degrees.
    // That is the angle a protractor reading is compared against.
    if (az.wanted) {
        say(d, format_plan(line_buf(d), MAX_REPLY_LEN, c.id, AxisId::Az,
                           az_move));
    }
    if (el.wanted) {
        say(d, format_plan(line_buf(d), MAX_REPLY_LEN, c.id, AxisId::El,
                           el_move));
    }
    return d;
}

// While an axis is turning, only the two stop commands are acted on. They are
// answered by the DONE that follows rather than by an ACK, so that nothing is
// printed between the first pulse and the last.
Decision do_while_moving(const Command &c)
{
    Decision d;
    if (c.verb == Verb::Stop) {
        d.act = Act::RequestStop;
    } else if (c.verb == Verb::Estop) {
        d.act = Act::Estop;
    } else {
        say(d, format_err(line_buf(d), MAX_REPLY_LEN, c.id, ErrCode::Busy, ""));
    }
    return d;
}

// STOP or ESTOP with nothing moving: nothing to stop, but still answered so
// the laptop is never left waiting.
Decision do_stop_when_idle(const Command &c)
{
    Decision d;
    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, c.verb));
    return d;
}

// ---------------------------------------------------------------------------
// ZERO, POS, PIN, LIMITS, PING
// ---------------------------------------------------------------------------

Decision do_zero(const Command &c, const MachineState &s)
{
    const int32_t mdeg = c.has_mdeg ? c.mdeg : 0;

    Decision d;
    d.act = Act::SetZero;

    if (c.axis == AxisId::All) {
        d.zero_az  = true;
        d.zero_el  = true;
        d.az_steps = mdeg_to_steps(s.az_spec, mdeg);
        d.el_steps = mdeg_to_steps(s.el_spec, mdeg);
    } else if (c.axis == AxisId::Az) {
        d.zero_az  = true;
        d.az_steps = mdeg_to_steps(s.az_spec, mdeg);
    } else if (c.axis == AxisId::El) {
        d.zero_el  = true;
        d.el_steps = mdeg_to_steps(s.el_spec, mdeg);
    } else {
        return refuse(c, ErrCode::BadAxis, "");
    }

    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, Verb::Zero));
    return d;
}

Decision do_pin(const Command &c)
{
    if (!pin_may_be_set(c.gpio)) {
        // An ERR detail keeps 16 characters, so the hint has to fit in that.
        return refuse(c, ErrCode::BadGpio, "GP4-7 or GP12-15");
    }

    Decision d;
    d.act   = Act::SetPin;
    d.gpio  = c.gpio;
    d.level = c.level;

    // Driving a step line makes the driver take a step this firmware did not
    // count, so afterwards the position it reports is one step away from the
    // truth. Giving up the zero is the honest response, the same as after an
    // emergency stop.
    d.clear_az_zero = (c.gpio == PIN_AZ_PUL);
    d.clear_el_zero = (c.gpio == PIN_EL_PUL);

    say(d, format_pin(line_buf(d), MAX_REPLY_LEN, c.id, c.gpio, c.level));
    return d;
}

Decision do_pos(const Command &c, const MachineState &s)
{
    Decision d;
    say(d, format_pos(line_buf(d), MAX_REPLY_LEN, c.id,
                      steps_to_mdeg(s.az_spec, s.az_pos_steps),
                      steps_to_mdeg(s.el_spec, s.el_pos_steps), s.az_pos_steps,
                      s.el_pos_steps, s.az_zeroed && s.el_zeroed));
    return d;
}

Decision do_limits(const Command &c, const MachineState &s)
{
    Decision d;
    say(d, format_lim(line_buf(d), MAX_REPLY_LEN, c.id, s.az_spec, s.el_spec));
    return d;
}

Decision do_ping(const Command &c, const MachineState &s)
{
    Decision d;
    say(d, format_pong(line_buf(d), MAX_REPLY_LEN, c.id, s.target, s.version,
                       s.uptime_ms));
    return d;
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

Decision do_set(const Command &c, const MachineState &s)
{
    if (c.axis != AxisId::Az && c.axis != AxisId::El) {
        return refuse(c, ErrCode::BadAxis, "");
    }

    // Judged against the axis as it is now, not as built: MIN may not pass
    // the current MAX, and MAX may not pass the current MIN.
    AxisSpec spec = spec_for(c.axis, s);
    const SettingRange r =
        allowed_range(c.setting, built_in_for(c.axis, s.gear), spec);
    if (c.value < r.lo || c.value > r.hi) {
        // Refused whole, never clamped: the operator would otherwise be
        // running with a number they did not type.
        Decision d;
        say(d, format_err_range(line_buf(d), MAX_REPLY_LEN, c.id, c.axis,
                                c.value, r.lo, r.hi));
        return d;
    }
    put_setting(c.setting, c.value, &spec);

    Decision d;
    d.act     = Act::SetSettings;
    d.gear    = s.gear;
    d.az_spec = (c.axis == AxisId::Az) ? spec : s.az_spec;
    d.el_spec = (c.axis == AxisId::El) ? spec : s.el_spec;
    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, Verb::Set));
    return d;
}

Decision do_save(const Command &c)
{
    // The ACK is only sent once the flash write has been checked, which is
    // the caller's job: see the SAVE note at the top of the header.
    Decision d;
    d.act = Act::SaveSettings;
    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, Verb::Save));
    return d;
}

Decision do_defaults(const Command &c, const MachineState &s)
{
    // Back to motor_config.hpp, for the mode in force. Like SET, this lasts
    // until a restart unless it is saved.
    //
    // The gear mode is deliberately left alone. It says which gearing is
    // bolted to the machine, which is not something `defaults` can know.
    Decision d;
    d.act     = Act::SetSettings;
    d.gear    = s.gear;
    d.az_spec = az_built_in(s.gear);
    d.el_spec = el_built_in(s.gear);
    say(d, format_ack(line_buf(d), MAX_REPLY_LEN, c.id, Verb::Defaults));
    return d;
}

Decision do_settings(const Command &c, const MachineState &s)
{
    const bool built_in = same_settings(s.az_spec, az_built_in(s.gear)) &&
                          same_settings(s.el_spec, el_built_in(s.gear));

    Decision d;
    say(d, format_cfg(line_buf(d), MAX_REPLY_LEN, c.id, s.gear, s.az_spec,
                      s.el_spec, !s.settings_unsaved, built_in));
    return d;
}

// ---------------------------------------------------------------------------
// GEAR
// ---------------------------------------------------------------------------

Decision do_gear(const Command &c, const MachineState &s)
{
    const GearMode wanted =
        c.has_gear ? (c.gear_on ? GearMode::Geared : GearMode::Direct) : s.gear;

    Decision d;

    // Asking which mode, or asking for the one already in force, changes
    // nothing. The second case matters: `gear on` twice must not cost the
    // zero the first one was followed by.
    if (wanted != s.gear) {
        d.act     = Act::SetGear;
        d.gear    = wanted;
        d.az_spec = az_built_in(wanted);
        d.el_spec = el_built_in(wanted);

        // The belt has just gone on or come off, so the step count no longer
        // describes a known angle and the travel limits have changed what
        // they mean. Giving up both zeros is the honest response, the same
        // as after an emergency stop.
        d.clear_az_zero = true;
        d.clear_el_zero = true;
    }

    say(d, format_gear(line_buf(d), MAX_REPLY_LEN, c.id, wanted,
                       (wanted == s.gear) ? s.az_spec : d.az_spec,
                       (wanted == s.gear) ? s.el_spec : d.el_spec));
    return d;
}

// What a line the parser could not read is refused with.
ErrCode err_for(ParseError e)
{
    switch (e) {
    case ParseError::UnknownVerb: return ErrCode::BadVerb;
    case ParseError::BadAxis:     return ErrCode::BadAxis;
    case ParseError::BadGpio:     return ErrCode::BadGpio;
    case ParseError::BadSetting:  return ErrCode::BadSetting;
    case ParseError::Syntax:
    case ParseError::None:        break;
    }
    return ErrCode::Syntax;
}

}  // namespace

Decision decide(const Command &c, const MachineState &s)
{
    if (c.err != ParseError::None) {
        return refuse(c, err_for(c.err), "");
    }
    if (s.busy) {
        return do_while_moving(c);
    }

    switch (c.verb) {
    case Verb::Move:     return do_move(c, s);
    case Verb::Stop:     return do_stop_when_idle(c);
    case Verb::Estop:    return do_stop_when_idle(c);
    case Verb::Zero:     return do_zero(c, s);
    case Verb::Pos:      return do_pos(c, s);
    case Verb::Pin:      return do_pin(c);
    case Verb::Limits:   return do_limits(c, s);
    case Verb::Ping:     return do_ping(c, s);
    case Verb::Set:      return do_set(c, s);
    case Verb::Save:     return do_save(c);
    case Verb::Defaults: return do_defaults(c, s);
    case Verb::Settings: return do_settings(c, s);
    case Verb::Gear:     return do_gear(c, s);
    case Verb::None:     break;
    }
    return Decision{};
}
