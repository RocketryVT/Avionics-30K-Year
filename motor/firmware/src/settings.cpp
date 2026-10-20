#include "settings.hpp"

namespace {

// "GSMS" - gs_pico_link motor settings - as the first four bytes in flash.
// Erased flash reads 0xFF, so a blank sector can never be mistaken for it.
constexpr uint32_t MAGIC = 0x534D5347u;

// Bumped if the layout below ever changes. A record of another version is
// not read at all: its bytes would mean different things.
constexpr uint32_t VERSION = 1;

constexpr Setting EVERY_SETTING[] = {Setting::Accel, Setting::Min,
                                     Setting::Max};

// Little-endian by hand rather than copying a struct, so the layout does not
// depend on what padding the compiler chooses.
void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

uint32_t get_u32(const uint8_t *p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// The settable numbers of both axes, in the order the record stores them.
void flatten(const AxisSpec &az, const AxisSpec &el, int32_t out[6])
{
    for (size_t i = 0; i < 3; i++) {
        out[i]     = setting_value(EVERY_SETTING[i], az);
        out[i + 3] = setting_value(EVERY_SETTING[i], el);
    }
}

// The standard CRC-32, the one zip files use. It catches a sector that was
// half written when the power went, which a magic number alone would not.
uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
        }
    }
    return ~crc;
}

// Is every setting in spec one this build would accept?
bool fits(const AxisSpec &built_in, const AxisSpec &spec)
{
    for (Setting s : EVERY_SETTING) {
        const SettingRange r = allowed_range(s, built_in, spec);
        const int32_t v      = setting_value(s, spec);
        if (v < r.lo || v > r.hi) {
            return false;
        }
    }
    return true;
}

}  // namespace

SettingRange allowed_range(Setting setting, const AxisSpec &built_in,
                           const AxisSpec &current)
{
    switch (setting) {
    case Setting::Accel:
        return {ACCEL_MIN_MDPS2, ACCEL_MAX_MDPS2};
    case Setting::Min:
        return {built_in.min_mdeg, current.max_mdeg};
    case Setting::Max:
        return {current.min_mdeg, built_in.max_mdeg};
    case Setting::None:
        break;
    }
    return {1, 0};  // nothing fits
}

int32_t setting_value(Setting setting, const AxisSpec &spec)
{
    switch (setting) {
    case Setting::Accel: return spec.accel_mdps2;
    case Setting::Min:   return spec.min_mdeg;
    case Setting::Max:   return spec.max_mdeg;
    case Setting::None:  break;
    }
    return 0;
}

void put_setting(Setting setting, int32_t value, AxisSpec *spec)
{
    switch (setting) {
    case Setting::Accel: spec->accel_mdps2 = value; break;
    case Setting::Min:   spec->min_mdeg = value; break;
    case Setting::Max:   spec->max_mdeg = value; break;
    case Setting::None:  break;
    }
}

bool same_settings(const AxisSpec &a, const AxisSpec &b)
{
    for (Setting s : EVERY_SETTING) {
        if (setting_value(s, a) != setting_value(s, b)) {
            return false;
        }
    }
    return true;
}

size_t encode_settings(const AxisSpec &az_built_in,
                       const AxisSpec &el_built_in, const AxisSpec &az,
                       const AxisSpec &el, uint8_t *out, size_t n)
{
    if (n < SETTINGS_RECORD_LEN) {
        return 0;
    }

    int32_t built_in[6];
    int32_t now[6];
    flatten(az_built_in, el_built_in, built_in);
    flatten(az, el, now);

    put_u32(out, MAGIC);
    put_u32(out + 4, VERSION);
    for (size_t i = 0; i < 6; i++) {
        put_u32(out + 8 + 4 * i, static_cast<uint32_t>(built_in[i]));
        put_u32(out + 32 + 4 * i, static_cast<uint32_t>(now[i]));
    }
    put_u32(out + 56, crc32(out, 56));
    return SETTINGS_RECORD_LEN;
}

LoadResult decode_settings(const uint8_t *in, size_t n,
                           const AxisSpec &az_built_in,
                           const AxisSpec &el_built_in, AxisSpec *az,
                           AxisSpec *el)
{
    if (n < SETTINGS_RECORD_LEN) {
        return LoadResult::Corrupt;
    }

    bool erased = true;
    for (size_t i = 0; i < SETTINGS_RECORD_LEN; i++) {
        erased = erased && (in[i] == 0xFF);
    }
    if (erased) {
        return LoadResult::Blank;
    }

    if (get_u32(in) != MAGIC || get_u32(in + 4) != VERSION ||
        get_u32(in + 56) != crc32(in, 56)) {
        return LoadResult::Corrupt;
    }

    // Saved against the numbers motor_config.hpp had then. If those have
    // changed, the code has been edited and reflashed on purpose, and the
    // new numbers win.
    int32_t built_in[6];
    flatten(az_built_in, el_built_in, built_in);
    for (size_t i = 0; i < 6; i++) {
        if (static_cast<int32_t>(get_u32(in + 8 + 4 * i)) != built_in[i]) {
            return LoadResult::Stale;
        }
    }

    AxisSpec new_az = az_built_in;
    AxisSpec new_el = el_built_in;
    for (size_t i = 0; i < 3; i++) {
        put_setting(EVERY_SETTING[i],
                    static_cast<int32_t>(get_u32(in + 32 + 4 * i)), &new_az);
        put_setting(EVERY_SETTING[i],
                    static_cast<int32_t>(get_u32(in + 44 + 4 * i)), &new_el);
    }

    // The bounds on acceleration are not part of the built-in snapshot, so
    // a build that tightened them could otherwise load a value it refuses.
    if (!fits(az_built_in, new_az) || !fits(el_built_in, new_el)) {
        return LoadResult::Stale;
    }

    for (Setting s : EVERY_SETTING) {
        put_setting(s, setting_value(s, new_az), az);
        put_setting(s, setting_value(s, new_el), el);
    }
    return LoadResult::Ok;
}
