#include "DIWUMexParams.hpp"
#include "DIWPyCompat.hpp"

#include "../Config.hpp"
#include "../PrintConfig.hpp"

#include <cmath>

namespace Slic3r {
namespace DIW {

namespace {

// Scalar or first element of a vector option; def when the key is missing.
double num(const ConfigBase &c, const char *key, double def)
{
    const ConfigOption *o = c.option(key);
    if (o == nullptr)
        return def;
    if (const auto *v = dynamic_cast<const ConfigOptionFloats *>(o))
        return v->values.empty() ? def : v->values.front();
    if (const auto *v = dynamic_cast<const ConfigOptionInts *>(o))
        return v->values.empty() ? def : double(v->values.front());
    return o->getFloat();
}

int integer(const ConfigBase &c, const char *key, int def)
{
    const ConfigOption *o = c.option(key);
    return o == nullptr ? def : o->getInt();
}

// Python: re.search(r"P(\d+)", s, re.IGNORECASE) -> int(group(1)), or nothing.
std::optional<int> uv_pin_of(const std::string &s)
{
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] != 'P' && s[i] != 'p')
            continue;
        size_t j = i + 1;
        while (j < s.size() && s[j] >= '0' && s[j] <= '9') ++j;
        if (j > i + 1)
            return std::stoi(s.substr(i + 1, j - i - 1));
    }
    return std::nullopt;
}

} // namespace

std::pair<UMexParams, std::vector<std::string>> read_umex_params(const ConfigBase &c, const std::string &uv_on_cmd)
{
    // v2 key <- fork key. Speeds are mm/s in the fork and mm/min in v2.
    UMexParams p;
    std::vector<std::string> warns;

    const double comp_t = num(c, "diw_compaction_time", 3.);
    if (comp_t > 8.0)
        throw DIWConfigError("compaction_duration = " + py_repr(comp_t) +
                             " s exceeds the 8.0 s limit (requirements 4.4: Brazil-nut segregation)");
    if (comp_t <= 0.0)
        throw DIWConfigError("compaction_duration must be > 0");
    if (comp_t < 1.0)
        warns.push_back("compaction_duration = " + py_repr(comp_t) + " s is below the documented 1.0 s lower bound");
    p.comp_t = comp_t;

    p.s_hatch  = num(c, "diw_powder_hatch_spacing", 1.2);
    p.m_dot    = num(c, "diw_powder_mass_flow", 6.24);
    p.rho_tap  = num(c, "diw_powder_tap_density", 1.95);
    p.eta      = num(c, "diw_powder_fill_multiplier", 1.);
    p.f_max    = num(c, "diw_powder_speed_max", 50.) * 60.;
    p.d_powder = num(c, "diw_powder_nozzle_diameter", 0.8);
    const std::pair<const char *, double> positive[] = {
        { "powder_hatch_spacing", p.s_hatch }, { "powder_mass_flow_rate", p.m_dot }, { "powder_tap_density", p.rho_tap },
        { "powder_fill_multiplier", p.eta },   { "powder_f_max", p.f_max },          { "powder_nozzle_dia", p.d_powder } };
    for (const auto &[key, v] : positive)
        if (v <= 0.0)
            throw DIWConfigError(std::string(key) + " must be > 0");

    p.lag_on      = num(c, "diw_powder_lag_on", 0.15);
    p.lag_off     = num(c, "diw_powder_lag_off", 0.15);
    p.delta       = num(c, "diw_shell_powder_clearance", 0.85);
    p.comp_settle = num(c, "diw_compaction_settle_time", 0.5);
    p.travel_hop  = num(c, "diw_powder_travel_hop", 2.);
    p.comp_hop    = num(c, "diw_compaction_z_hop", 2.);
    const std::pair<const char *, double> non_negative[] = {
        { "powder_lag_on", p.lag_on },                { "powder_lag_off", p.lag_off },           { "shell_powder_clearance", p.delta },
        { "compaction_settle_dwell", p.comp_settle }, { "powder_travel_hop_mm", p.travel_hop }, { "compaction_z_hop_mm", p.comp_hop } };
    for (const auto &[key, v] : non_negative)
        if (v < 0.0)
            throw DIWConfigError(std::string(key) + " must be >= 0");

    switch (integer(c, "diw_powder_overflow_policy", diwpoIncreaseSpacing)) {
    case diwpoClamp: p.policy = "clamp"; break;
    case diwpoError: p.policy = "error"; break;
    default:         p.policy = "increase_spacing"; break;
    }

    p.comp_interval = integer(c, "diw_compaction_interval", 1);
    if (p.comp_interval < 1)
        throw DIWConfigError("compaction_interval must be >= 1");

    p.voltage = num(c, "diw_powder_motor_voltage", 3.65);
    if (! (3.65 - 1e-9 <= p.voltage && p.voltage <= 12.0 + 1e-9))
        throw DIWConfigError("powder_motor_voltage must be within 3.65-12.0 V");
    if (std::abs(p.voltage - 3.65) > 1e-9)
        warns.push_back("powder_motor_voltage = " + py_repr(p.voltage) +
                        " V but powder_mass_flow_rate is only valid at its calibration voltage; no discharge curve is "
                        "available, re-calibrate m_dot");

    // Digital outputs must be distinct (Python dict repr of {name: pin}).
    p.powder_pin = integer(c, "diw_powder_pin", 5);
    p.comp_pin   = integer(c, "diw_compaction_pin", 6);
    const std::optional<int> uv_pin = uv_pin_of(uv_on_cmd);
    if (p.powder_pin == p.comp_pin || (uv_pin && (*uv_pin == p.powder_pin || *uv_pin == p.comp_pin))) {
        std::string m = "{'powder_pin': " + std::to_string(p.powder_pin) + ", 'compaction_pin': " + std::to_string(p.comp_pin);
        if (uv_pin)
            m += ", 'uv_on_cmd pin': " + std::to_string(*uv_pin);
        throw DIWConfigError("digital output pins must be distinct: " + m + "}");
    }

    if (p.delta < p.d_powder / 2.0 + 0.35 - 1e-9)
        warns.push_back("shell_powder_clearance = " + py_repr(p.delta) + " mm is below d_powder/2 + c_safety,min = " +
                        py_fixed(2, p.d_powder / 2.0 + 0.35) + " mm even for a zero-dispersion jet");

    p.off_x   = num(c, "diw_powder_offset_x", 0.);
    p.off_y   = num(c, "diw_powder_offset_y", 0.);
    p.off_z   = num(c, "diw_powder_offset_z", 0.);
    p.z_clear = num(c, "diw_powder_z_clearance", 0.);
    if (p.z_clear - p.off_z < -1e-9)
        warns.push_back("powder_offset_z = " + py_repr(p.off_z) + " mm with powder_z_clearance_mm = " + py_repr(p.z_clear) +
                        " mm puts the PASTE nozzle tip " + py_fixed(3, p.off_z - p.z_clear) +
                        " mm BELOW the layer top during dispensing; check the sign convention of powder_offset_z");

    p.rho_app        = num(c, "diw_powder_apparent_density", 1.60);
    p.point_len      = num(c, "diw_powder_point_dwell_length", 5.);
    p.hatch_angle    = num(c, "diw_powder_hatch_angle", 0.);
    p.travel_feed    = 3000.;   // v2 umex_travel_feed_mm_min; no fork setting yet
    p.stage1_dwell   = num(c, "diw_auger_stop_dwell", 0.2);
    p.wiper_x0       = num(c, "diw_wiper_x_start", 13.);
    p.wiper_x1       = num(c, "diw_wiper_x_end", 48.);
    p.wiper_y        = num(c, "diw_wiper_y", 198.);
    p.wiper_z        = num(c, "diw_wiper_z", 0.5);
    p.wiper_feed     = num(c, "diw_wiper_speed", 50.) * 60.;
    p.wiper_hop      = num(c, "diw_wiper_z_hop", 3.);
    p.wiper_zmin     = num(c, "diw_wiper_z_min", 5.);
    p.width_fallback = num(c, "nozzle_diameter", 1.4);
    p.paste_min_area = num(c, "diw_powder_min_area", 2.);
    return { std::move(p), std::move(warns) };
}

} // namespace DIW
} // namespace Slic3r
