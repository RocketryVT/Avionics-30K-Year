// gs_motor: the program on the motor Pico.
//
// Reads commands from the laptop over USB, moves the two axes, and reports
// what happened. The whole program is one loop:
//
//     forever:
//         let each axis send a pulse if one is due
//         report any axis that has just finished
//         read any bytes that arrived, and act on a complete line
//         send a heartbeat if it is time and nothing is moving
//
// No interrupts, no second core. The loop runs far more often than steps are
// needed - the fastest step rate used here is one every 124 us - so an axis
// is never left waiting on it, and the whole program reads top to bottom.
//
// How a command is handled:
//
//     line_reader     bytes from USB  ->  one line of text
//     command_parser  line of text    ->  Command
//     command_rules   Command         ->  Decision (lines to send, one action)
//     main.cpp        sends the lines, then carries out the action
//
// ONE RULE MATTERS MORE THAN THE REST: nothing is printed between the first
// and last pulse of a move. Writing to USB blocks when the laptop stops
// reading, and a block in the middle of a move would stall the step loop with
// the motor part way round. So the heartbeat is held back while moving, DONE
// is sent after the last pulse, and a command that arrives mid-move is
// answered with a single short line.
//
// TRACKING IS THE ONE EXCEPTION, and it has to be, because while tracking an
// axis is nearly always turning - so that rule would mean no telemetry at
// all, ever. The TRK stream goes out through send_if_room(), which asks the
// USB layer whether a whole line will fit and drops the line if it will not.
// It never waits. A dropped telemetry line costs a moment of staleness on
// screen; a wait would cost a stalled step loop, which is worse. Only the
// tracking reports are sent this way - everything else still goes out at a
// moment when nothing is turning.
//
// Deliberately absent:
//   no ENA, no ALM     not wired on this board; the Pico cannot disable a
//                      driver and cannot tell when one faults
//   no limit switches  none fitted; the soft limits are all there is
//   no encoder         position here is what we believe, not what happened
#include <stdio.h>
#include <cmath>
#include <optional>

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "command_parser.hpp"
#include "command_rules.hpp"
#include "line_reader.hpp"
#include "motor_config.hpp"
#include "pins.hpp"
#include "replies.hpp"
#include "settings.hpp"
#include "settings_flash.hpp"
#include "stepper.hpp"
#include "pointing.hpp"
#include "motor_interface.hpp"
#include "radio_link.hpp"
#include "radio_tracking_config.hpp"

#include "replay_config.hpp"
#include "replay_data.hpp"
#include "replay_position.hpp"

constexpr uint32_t HEARTBEAT_PERIOD_MS = 5000;

// How often the TRK stream goes out while tracking. Fast enough to watch the
// error settle, slow enough that it is a rounding error against the step
// loop, which runs tens of thousands of times a second.
constexpr uint32_t TRACK_REPORT_MS = 100;

// Which gearing the maths counts: the gearbox alone, or the gearbox and the
// final pair. Direct until a saved record says otherwise, because a wrong
// guess that way undershoots instead of driving into a hard stop.
static GearMode g_gear = GearMode::Direct;

// The settings in force - acceleration and travel, which can be changed at
// the prompt. They start as the built-in ones for g_gear and at startup are
// replaced by any good ones saved in flash. The steppers read their fixed
// numbers (steps per degree, direction) from here too, so each axis has one
// description, and switching gear mode rewrites it in place.
static AxisSpec g_az_spec = AZ_SPEC;
static AxisSpec g_el_spec = EL_SPEC;

// What a restart would come back with, so the laptop can be told whether the
// settings in force have been saved. The gear mode is saved with them.
static GearMode g_boot_gear = GearMode::Direct;
static AxisSpec g_boot_az = AZ_SPEC;
static AxisSpec g_boot_el = EL_SPEC;

// True once TRACK ON was accepted, and when the last target arrived. If
// targets stop coming, the watchdog in the main loop brings the axes to rest:
// without it a radio dropout leaves the antenna driving at a target that is
// minutes stale.
static bool g_tracking = false;
static absolute_time_t g_last_target = {};

// True while the built-in simulated flight is feeding the targets. Only
// TRACK REPLAY sets it; TRACK ON/OFF, STOP, ESTOP, a gear change, the
// watchdog, a TARGET from the laptop and the end of the data all clear it.
static bool g_replay_on = false;
static absolute_time_t g_replay_start = {};
static absolute_time_t g_next_replay_update = {};
static uint16_t g_replay_command_id = 1;

// The replay is pointed at from this station position, so without one it
// cannot run. Both are set in replay_config.hpp.
static constexpr bool REPLAY_READY =
    REPLAY_STATION.has_value() && REPLAY_GEOID_SEPARATION_M.has_value();

static constexpr std::optional<GPSPosition> RADIO_STATION =
    RADIO_POSITION_SOURCE == RadioPositionSource::Replay
        ? REPLAY_STATION : RADIO_LIVE_STATION;

// Radio ownership is separate from arming and the local replay state.
static bool g_radio_targets = false;
static bool g_radio_have_order = false;
static uint32_t g_radio_last_sequence = 0;
static uint32_t g_radio_last_timestamp = 0;
static uint16_t g_radio_command_id = 1;
static uint32_t g_radio_target_count = 0;

struct ReceivedRadioPosition
{
    GPSPosition position;
    absolute_time_t received_at;
};

static std::optional<ReceivedRadioPosition> g_radio_pending;

static Stepper g_az(g_az_spec, PIN_AZ_CW, PIN_AZ_CCW);
static Stepper g_el(g_el_spec, PIN_EL_CW, PIN_EL_CCW);
static LineReader g_reader;

// One buffer, reused. Every reply is built into it whole and then sent with a
// single fputs, so two messages can never interleave.
static char g_out[MAX_REPLY_LEN];

static void send(size_t n)
{
    if (n > 0)
    {
        fputs(g_out, stdout);
    }
}

// Send only if the whole line will fit in the USB buffer right now.
//
// TinyUSB is asked how much room there is, and the write still goes out
// through stdio as every other reply does - so this is a gate, not a second
// way of writing. If there is not room the line is dropped and never
// mentioned again, which is the right trade while an axis is turning: a
// stale reading on screen beats a stalled step loop.
static void send_if_room(size_t n)
{
    if (n == 0)
    {
        return;
    }
    if (!tud_cdc_connected() || tud_cdc_write_available() < n)
    {
        return;
    }
    fputs(g_out, stdout);
}

// ----------------------------------------------------------------------------
// Startup
// ----------------------------------------------------------------------------

// Wait for the laptop to open the port, so the banner is not printed into a
// port nobody is reading.
static void wait_for_usb_forever()
{
    while (!stdio_usb_connected())
    {
        sleep_ms(50);
    }
    sleep_ms(200); // let the laptop finish opening before the first line
}

// Millidegrees as degrees with three decimals, for the banner.
static void print_mdeg(int32_t mdeg)
{
    const long whole = mdeg / 1000;
    long frac = mdeg % 1000;
    if (frac < 0)
    {
        frac = -frac;
    }
    if (mdeg < 0 && whole == 0)
    {
        printf("-0.%03ld", frac);
    }
    else
    {
        printf("%ld.%03ld", whole, frac);
    }
}

static void print_steps_per_deg(const AxisSpec &axis)
{
    const int32_t milli = mdeg_to_steps(axis, 1000000);
    printf("%ld.%03ld", static_cast<long>(milli / 1000),
           static_cast<long>(milli % 1000));
}

// Never die quietly, and never carry on regardless.
static void fatal_forever(const char *reason)
{
    for (;;)
    {
        printf("[fatal] %s\n", reason);
        sleep_ms(2000);
    }
}

// What the next startup would load: the saved settings if they are good for
// this build, otherwise the built-in ones.
static LoadResult read_boot_settings()
{
    // Seeded with direct mode and its numbers, so a record that is blank,
    // corrupt or stale leaves the machine counting the gearbox alone.
    g_boot_gear = GearMode::Direct;
    g_boot_az = AZ_SPEC;
    g_boot_el = EL_SPEC;
    return load_settings(&g_boot_gear, &g_boot_az, &g_boot_el);
}

static const char *settings_note(LoadResult r)
{
    switch (r)
    {
    case LoadResult::Ok:
        return "saved ones in use";
    case LoadResult::Blank:
        return "built in";
    case LoadResult::Stale:
        return "built in - saved ones ignored, motor_config.hpp has changed";
    case LoadResult::Corrupt:
        return "built in - saved ones could not be read";
    }
    return "built in";
}

// Spelled out rather than just "on" or "off": getting this wrong puts every
// move out by the gear pair's ratio, and this line is the operator's one
// chance to notice before they type a move.
static const char *gear_note(GearMode g)
{
    return (g == GearMode::Geared)
               ? "GEARED - final pair counted, angles at the ANTENNA"
               : "direct - gearbox only, angles at the GEARBOX OUTPUT SHAFT";
}

static void print_banner(LoadResult loaded)
{
    printf("[motor] pins AZ CW=GP%u CCW=GP%u  EL CW=GP%u CCW=GP%u  startup levels OK\n",
           PIN_AZ_CW, PIN_AZ_CCW, PIN_EL_CW, PIN_EL_CCW);
    // Nothing here can read the switch, so this line is the operator's one
    // chance to notice a driver set the other way.
    printf("[motor] CW/CCW pulse mode - SW7 must be ON on both drivers\n");
    printf("[motor] active low: both lines rest HIGH and are pulled LOW to step\n");

    printf("[motor] gear %s\n", gear_note(g_gear));

    printf("[motor] ready az=");
    print_steps_per_deg(g_az_spec);
    printf(" st/deg el=");
    print_steps_per_deg(g_el_spec);
    printf(" st/deg\n");

    printf("[motor] limits az ");
    print_mdeg(g_az_spec.min_mdeg);
    printf("..");
    print_mdeg(g_az_spec.max_mdeg);
    printf("  el ");
    print_mdeg(g_el_spec.min_mdeg);
    printf("..");
    print_mdeg(g_el_spec.max_mdeg);
    printf("\n");

    printf("[motor] accel az ");
    print_mdeg(g_az_spec.accel_mdps2);
    printf("  el ");
    print_mdeg(g_el_spec.accel_mdps2);
    printf(" deg/s/s\n");

    printf("[motor] settings: %s\n", settings_note(loaded));

    printf("[motor] deadband az ");
    print_mdeg(g_az_spec.deadband_mdeg);
    printf("  el ");
    print_mdeg(g_el_spec.deadband_mdeg);
    printf(" deg (tracking only)\n");

    printf("[motor] tracking %s\n", g_tracking ? "ARMED" : "off");

    if (REPLAY_READY)
    {
        printf("[motor] replay: %lu s simulated flight built in - TRACK REPLAY plays it\n",
               static_cast<unsigned long>(REPLAY_SAMPLES.back().timeSeconds));
    }
    else
    {
        printf("[motor] replay: no station set in replay_config.hpp - TRACK REPLAY refused\n");
    }

    printf("[motor] NOT ZEROED - send ZERO before any MOVE\n");
}

// ----------------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------------

// The machine as it is right now, for decide() to judge.
//
// Taken fresh for every command, so that two commands arriving in one USB
// packet are each judged against the machine as it is when that command is
// handled - a MOVE that the first one started makes the second one busy.
static MachineState snapshot()
{
    MachineState s;
    s.busy = g_az.busy() || g_el.busy();
    s.az_zeroed = g_az.zeroed();
    s.el_zeroed = g_el.zeroed();
    s.az_pos_steps = g_az.pos_steps();
    s.el_pos_steps = g_el.pos_steps();
    s.gear = g_gear;
    s.tracking = g_tracking;
    s.replaying = g_replay_on;
    s.replay_ready = REPLAY_READY;
    s.az_target_steps = g_az.target_steps();
    s.el_target_steps = g_el.target_steps();
    s.az_mdps = g_az.velocity_mdps();
    s.el_mdps = g_el.velocity_mdps();
    s.az_spec = g_az_spec;
    s.el_spec = g_el_spec;
    s.settings_unsaved = g_gear != g_boot_gear ||
                         !same_settings(g_az_spec, g_boot_az) ||
                         !same_settings(g_el_spec, g_boot_el);
    s.uptime_ms = to_ms_since_boot(get_absolute_time());
    s.target = GS_TARGET_NAME;
    s.version = GS_VERSION;
    return s;
}

static void set_test_pin(const Decision &d)
{
    // A spare pin has never been set up, so it needs claiming first.
    //
    // A motor pin has: Stepper::init() already made it an output at its idle
    // level, and gpio_init() would make it an input again for a moment on the
    // way past - letting it float, which pins.hpp warns against. So the level
    // is written straight to it instead.
    if (pin_is_spare(d.gpio))
    {
        gpio_init(d.gpio);
        gpio_set_dir(d.gpio, GPIO_OUT);
    }
    gpio_put(d.gpio, d.level != 0);
}

// Carry out what decide() asked for. Nothing here chooses anything.
static void apply(const Decision &d)
{
    switch (d.act)
    {
    case Act::Nothing:
        break;
    case Act::StartMove:
        // Every axis about to move is aimed before any of them begins. An
        // aim may have to wait for a line the PIN command left LOW, so doing
        // them all first means both axes of a two-axis move set off together.
        if (d.move_az)
        {
            g_az.aim(d.az_move);
        }
        if (d.move_el)
        {
            g_el.aim(d.el_move);
        }
        if (d.move_az)
        {
            g_az.begin(d.az_move, d.id);
        }
        if (d.move_el)
        {
            g_el.begin(d.el_move, d.id);
        }
        break;
    case Act::RequestStop:
        // A stop while tracking means stop tracking: the axes ramp to rest
        // and no further target will move them until TRACK ON again.
        g_tracking = false;
        g_az.request_stop();
        g_el.request_stop();
        break;
    case Act::Estop:
        g_tracking = false;
        g_az.estop();
        g_el.estop();
        break;
    case Act::SetZero:
        if (d.zero_az)
        {
            g_az.set_pos_steps(d.az_steps);
        }
        if (d.zero_el)
        {
            g_el.set_pos_steps(d.el_steps);
        }
        break;
    case Act::SetPin:
        set_test_pin(d);
        break;
    case Act::SetSettings:
        g_az_spec = d.az_spec;
        g_el_spec = d.el_spec;
        break;
    case Act::SaveSettings:
        break; // already done in handle(), before the reply
    case Act::SetTrack:
        // Arm only on the way in: switching between ON and REPLAY while
        // already armed changes where targets come from, and re-arming
        // would throw away the chase in progress.
        if (d.track_on && !g_tracking)
        {
            g_az.arm_chase();
            g_el.arm_chase();
            // Counted from here, so arming does not immediately look like a
            // dropout to the watchdog.
            g_last_target = get_absolute_time();
        }
        else if (!d.track_on)
        {
            g_az.disarm_chase();
            g_el.disarm_chase();
        }
        g_tracking = d.track_on;

        // REPLAY starts the simulated flight from time zero; anything else
        // ends it.
        g_replay_on = d.track_replay;
        if (g_replay_on)
        {
            g_replay_start = get_absolute_time();
            g_next_replay_update = g_replay_start;
            g_replay_command_id = 1;
        }
        break;
    case Act::SetTarget:
        g_az.set_target_steps(d.az_steps);
        g_el.set_target_steps(d.el_steps);
        g_last_target = get_absolute_time();
        break;
    case Act::SetGear:
        // Both specs are replaced whole, not just their settable numbers:
        // the mode decides steps per degree and every speed number too.
        g_gear = d.gear;
        g_az_spec = d.az_spec;
        g_el_spec = d.el_spec;
        // A degree means something different now, so any target in hand is
        // meaningless and the zeros are gone. A gear change is refused while
        // tracking, so this is only a backstop.
        g_tracking = false;
        g_az.disarm_chase();
        g_el.disarm_chase();
        break;
    }

    // An axis stops knowing where it is when a step goes out uncounted (a
    // step pin driven by hand) or when the drive train changes under it (a
    // gear mode switch). Either way it has to be re-zeroed before it may
    // move again.
    if (d.clear_az_zero)
    {
        g_az.clear_zero();
    }
    if (d.clear_el_zero)
    {
        g_el.clear_zero();
    }
}

static bool radioStationReady()
{
    if (!RADIO_STATION)
        return false;

    const auto &s = *RADIO_STATION;
    return std::isfinite(s.latitude) && s.latitude >= -90.0 &&
           s.latitude <= 90.0 && std::isfinite(s.longitude) &&
           s.longitude >= -180.0 && s.longitude <= 180.0 &&
           std::isfinite(s.altitude);
}

// Compare transmitter counters only; the two Pico clocks are not synchronized.
static bool acceptRadioOrder(const PositionPacket &packet)
{
    if (g_radio_have_order)
    {
        const uint32_t sequence_delta =
            packet.sequence - g_radio_last_sequence;
        const uint32_t timestamp_delta =
            packet.timestamp - g_radio_last_timestamp;

        // Modular comparison allows ordinary counter wrap.
        if (sequence_delta == 0 || sequence_delta >= 0x80000000u ||
            timestamp_delta == 0 || timestamp_delta >= 0x80000000u)
            return false;
    }

    g_radio_have_order = true;
    g_radio_last_sequence = packet.sequence;
    g_radio_last_timestamp = packet.timestamp;
    return true;
}

static void handle(const Command &c)
{
    if (c.err == ParseError::None && c.verb == Verb::Track &&
        c.has_track && c.track_on && !c.track_replay &&
        !radioStationReady())
    {
        send_if_room(static_cast<size_t>(snprintf(
            g_out, sizeof(g_out),
            "ERR,%u,station,unset or invalid\n",
            static_cast<unsigned>(c.id))));
        return;
    }


    const Decision d = decide(c, snapshot());

    // A save is the one action whose answer depends on how it went, so it is
    // done before the reply rather than after. It is checked by reading flash
    // back, the same way the next startup will, so the ACK means a restart
    // will come back with these settings.
    if (d.act == Act::SaveSettings)
    {
        save_settings(g_gear, g_az_spec, g_el_spec);
        read_boot_settings();
        if (g_boot_gear != g_gear || !same_settings(g_boot_az, g_az_spec) ||
            !same_settings(g_boot_el, g_el_spec))
        {
            send(format_err(g_out, sizeof(g_out), c.id, ErrCode::Flash, ""));
            return;
        }
    }

    // Lines first, then the action. A move's ACK and PLANs have to be on the
    // wire before its first pulse.
    for (size_t i = 0; i < d.line_count; i++)
    {
        fputs(d.lines[i], stdout);
    }
    apply(d);

    // Explicit TRACK modes choose the owner; a status query leaves it unchanged.
    if (c.err == ParseError::None && c.verb == Verb::Track && c.has_track)
    {
        if (!c.track_on)
        {
            g_radio_targets = false;
            g_radio_have_order = false; // Explicit OFF starts a new RF session.
            g_radio_pending.reset();
        }
        else if (g_tracking && g_replay_on == c.track_replay)
        {
            g_radio_targets = !c.track_replay;
            g_radio_pending.reset();
        }
    }
}

// Read whatever has arrived without waiting for more.
static void poll_input()
{
    for (;;)
    {
        const int ch = getchar_timeout_us(0);
        if (ch < 0)
        {
            return;
        }

        const LineReader::Result r = g_reader.feed(static_cast<char>(ch));
        if (r == LineReader::Result::Line)
        {
            const Command c = parse_command(g_reader.line());
            if (c.err == ParseError::None && c.verb == Verb::Target)
            {
                g_radio_targets = false;
                g_radio_pending.reset();
            }

            // A target from the laptop means the laptop is steering, so the
            // simulated flight stops rather than fight it for the axes.
            if (c.verb == Verb::Target && g_replay_on)
            {
                g_replay_on = false;
                send_if_room(static_cast<size_t>(snprintf(
                    g_out, sizeof(g_out),
                    "[motor] replay stopped - the laptop sent a target\n")));
            }
            handle(c);
        }
        else if (r == LineReader::Result::TooLong)
        {
            send(format_err(g_out, sizeof(g_out), 0, ErrCode::TooLong, ""));
        }
    }
}

// Route radio and local replay through the same motor command rules.
static bool submitRocketPosition(const GPSPosition &rocket,
                                 const GPSPosition &station,
                                 uint16_t commandId)
{
    if (!g_tracking)
        return false;

    const auto angles = calculatePointingAngles(rocket, station);
    if (!angles)
        return false;

    const auto command = makeMotorTarget(*angles, commandId);
    if (!command)
        return false;

    handle(*command);
    return true;
}

static void pollRadioTarget(absolute_time_t now)
{
    if (!g_radio_pending)
        return;

    const auto received = *g_radio_pending;
    g_radio_pending.reset(); // Each received position is consumed once.

    // This measures receiver queue time, not the original GPS fix's age.
    const int64_t wait_us =
        absolute_time_diff_us(received.received_at, now);

    if (!g_tracking || !g_radio_targets || g_replay_on ||
        !RADIO_STATION || wait_us < 0 || wait_us > 500'000)
        return;

    if (submitRocketPosition(received.position, *RADIO_STATION,
                             g_radio_command_id++))
        ++g_radio_target_count;
}

// Feed the next simulated position, while TRACK REPLAY is running.
static void pollReplay(absolute_time_t now)
{
    if (!g_replay_on)
    {
        return;
    }

    // Whatever disarmed tracking - STOP, ESTOP, a gear change, the
    // watchdog - ends the replay with it.
    if (!g_tracking || !REPLAY_READY)
    {
        g_replay_on = false;
        return;
    }

    // Return immediately if the next update is still in the future.
    if (absolute_time_diff_us(now, g_next_replay_update) > 0)
    {
        return;
    }

    g_next_replay_update = delayed_by_ms(now, REPLAY_UPDATE_PERIOD_MS);

    const double elapsedSeconds =
        absolute_time_diff_us(g_replay_start, now) / 1'000'000.0;

    const auto rocket =
        replayPositionAt(elapsedSeconds, *REPLAY_GEOID_SEPARATION_M);

    // When the data ends, stop sending targets. The tracking watchdog then
    // brings the axes to rest, exactly as it would after a radio dropout.
    if (!rocket)
    {
        g_replay_on = false;
        send_if_room(static_cast<size_t>(snprintf(
            g_out, sizeof(g_out),
            "[motor] replay finished - tracking stops in %lu ms\n",
            static_cast<unsigned long>(TRACK_TIMEOUT_MS))));
        return;
    }

    submitRocketPosition(*rocket, *REPLAY_STATION, g_replay_command_id++);
}

// What the TRK stream says. Built from the steppers rather than from a
// snapshot, because it is sent from the loop rather than in answer to a
// command.
static TrackReport track_report()
{
    TrackReport r{};
    r.armed = g_tracking;
    r.az_target_mdeg = steps_to_mdeg(g_az_spec, g_az.target_steps());
    r.az_pos_mdeg = g_az.pos_mdeg();
    r.az_mdps = g_az.velocity_mdps();
    r.el_target_mdeg = steps_to_mdeg(g_el_spec, g_el.target_steps());
    r.el_pos_mdeg = g_el.pos_mdeg();
    r.el_mdps = g_el.velocity_mdps();
    return r;
}

// Called once each time an axis finishes, to report what it did.
static void report_done(Stepper &s, AxisId axis)
{
    send(format_done(g_out, sizeof(g_out), s.move_id(), axis, !s.aborted(),
                     s.steps_done(), s.plan().from_mdeg, s.pos_mdeg(),
                     s.elapsed_us()));
}

int main()
{
    // These are bounded, read-only flash operations. Then park both axes'
    // lines before stdio initialization or any wait for a host, so they are
    // never left floating. An overlapping settings sector is never read; the
    // built-in specs are used and the error is reported once USB is
    // available.
    const bool settings_free = settings_sector_is_free();
    const LoadResult loaded = settings_free ? read_boot_settings()
                                            : LoadResult::Blank;
    g_gear = g_boot_gear;
    g_az_spec = g_boot_az;
    g_el_spec = g_boot_el;

    g_az.init();
    g_el.init();

    stdio_init_all();
    wait_for_usb_forever();

    printf("=== gs_pico_link %s %s ===\n", GS_TARGET_NAME, GS_VERSION);

    if (!g_az.pins_ok())
    {
        fatal_forever("azimuth pins did not read back their startup levels");
    }
    if (!g_el.pins_ok())
    {
        fatal_forever("elevation pins did not read back their startup levels");
    }

    if (!settings_free)
    {
        fatal_forever("the program has grown into the settings sector");
    }

    print_banner(loaded);

    // A failed radio stays disabled; the motor console still runs.
    int radio_error = initializeRadio(RadioMode::Receiver);
    bool radio_enabled = radio_error == 0;

    printf("[motor] radio %s, code=%d\n",
           radio_enabled ? "listening" : "disabled",
           radio_error);

    uint32_t rx_packets = 0;
    uint32_t rx_rejected = 0;
    uint32_t rx_last_sequence = 0;
    // Cumulative timing maxima in microseconds since startup.
    uint32_t max_rx_us = 0;
    uint32_t max_loop_us = 0;
    uint32_t previous_loop_us = time_us_32();

    absolute_time_t next_radio_report =
        delayed_by_ms(get_absolute_time(), 1000);

    absolute_time_t next_heartbeat =
        delayed_by_ms(get_absolute_time(), HEARTBEAT_PERIOD_MS);
    absolute_time_t next_track_report = next_heartbeat;

    for (;;)
    {
        const absolute_time_t now = get_absolute_time();

        // Measure the full loop, including USB and replay work.
        const uint32_t loop_us = time_us_32();
        const uint32_t gap_us = loop_us - previous_loop_us;
        previous_loop_us = loop_us;

        if (gap_us > max_loop_us)
            max_loop_us = gap_us;

        const bool az_moving = g_az.tick(now);
        const bool el_moving = g_el.tick(now);
        const bool moving = az_moving || el_moving;

        // Report completed plans before accepting another command. TRACK ON
        // or a new MOVE may replace a stepper's move ID, so accepting one
        // between the last pulse and DONE would erase the old report. The
        // pending ID also catches a one-step move, whose first tick finishes
        // it without ever returning true for moving.
        if (!moving)
        {
            if (!g_az.moving() && g_az.move_id() != 0)
            {
                report_done(g_az, AxisId::Az);
                g_az.clear_move_id();
            }
            if (!g_el.moving() && g_el.move_id() != 0)
            {
                report_done(g_el, AxisId::El);
                g_el.clear_move_id();
            }
        }
        poll_input();

        // The watchdog. Targets arriving is the only sign the other end is
        // still there, so if they stop, the axes come to rest and stay
        // there until something arms tracking again. This is what a radio
        // dropout or a closed laptop looks like from here.
        if (g_tracking &&
            absolute_time_diff_us(g_last_target, get_absolute_time()) >
                static_cast<int64_t>(TRACK_TIMEOUT_MS) * 1000)
        {
            g_tracking = false;
            g_az.disarm_chase();
            g_el.disarm_chase();
            // The axes are still slowing down, so these go out the same
            // never-wait way as the stream. The laptop may well be gone -
            // that is usually why the targets stopped.
            send_if_room(format_trk(g_out, sizeof(g_out), 0, track_report()));
            send_if_room(static_cast<size_t>(snprintf(
                g_out, sizeof(g_out),
                "[motor] no target for %lu ms - tracking stopped\n",
                static_cast<unsigned long>(TRACK_TIMEOUT_MS))));
        }

        // Apply last iteration's radio position only after the watchdog check.
        pollRadioTarget(get_absolute_time());
        pollReplay(now);

        // Store ordered positions; target submission happens next iteration.
        if (radio_enabled)
        {
            const uint32_t started_us = time_us_32();
            const auto result = pollReceiver();
            const uint32_t elapsed_us = time_us_32() - started_us;

            if (elapsed_us > max_rx_us)
                max_rx_us = elapsed_us;

            switch (result.status)
            {
            case RadioReceiveStatus::Idle:
                break;

            case RadioReceiveStatus::Packet:
                if (!acceptRadioOrder(*result.packet))
                {
                    ++rx_rejected;
                    break;
                }

                ++rx_packets;
                rx_last_sequence = result.packet->sequence;
                g_radio_pending = ReceivedRadioPosition{
                    result.packet->position, get_absolute_time()};
                break;

            case RadioReceiveStatus::ReadRejected:
            case RadioReceiveStatus::DecodeRejected:
                ++rx_rejected;
                break;

            case RadioReceiveStatus::Fault:
                radio_error = result.radioState;
                radio_enabled = false;
                g_radio_pending.reset();
                break;
            }
        }

        // One best-effort summary per second; send_if_room() may drop it.
        if (absolute_time_diff_us(now, next_radio_report) <= 0)
        {
            next_radio_report = delayed_by_ms(now, 1000);

            const int n = snprintf(
                g_out, sizeof(g_out),
                "RX,%s,ok=%lu,bad=%lu,seq=%lu,tgt=%lu,e=%d,rx=%lu,gap=%lu\n",
                radio_enabled ? "ON" : "OFF",
                static_cast<unsigned long>(rx_packets),
                static_cast<unsigned long>(rx_rejected),
                static_cast<unsigned long>(rx_last_sequence),
                static_cast<unsigned long>(g_radio_target_count),
                radio_error,
                static_cast<unsigned long>(max_rx_us),
                static_cast<unsigned long>(max_loop_us));

            if (n > 0 && static_cast<size_t>(n) < sizeof(g_out))
                send_if_room(static_cast<size_t>(n));
        }

        // The tracking stream. Sent through send_if_room() because an axis
        // is turning - see the rule at the top of this file.
        if (g_tracking && absolute_time_diff_us(now, next_track_report) <= 0)
        {
            next_track_report = delayed_by_ms(now, TRACK_REPORT_MS);
            send_if_room(format_trk(g_out, sizeof(g_out), 0, track_report()));
        }

        // The heartbeat stays quiet while moving - see the rule at the top of
        // this file - and while someone is part way through typing a line,
        // where it would scramble the screen.
        //
        // Either way the deadline is pushed along rather than left in the
        // past, so the heartbeat resumes a full period after things go quiet
        // instead of firing the instant a move ends or a line is finished.
        if (moving || g_tracking || g_reader.partial())
        {
            next_heartbeat = delayed_by_ms(now, HEARTBEAT_PERIOD_MS);
        }
        else if (absolute_time_diff_us(now, next_heartbeat) <= 0)
        {
            next_heartbeat = delayed_by_ms(now, HEARTBEAT_PERIOD_MS);
            send(format_mstat(g_out, sizeof(g_out),
                              to_ms_since_boot(now) / 1000, g_az.pos_mdeg(),
                              g_el.pos_mdeg(), false));
        }
    }
}
