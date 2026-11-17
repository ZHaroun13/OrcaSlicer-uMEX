#ifndef slic3r_GCode_DIWUMexParams_hpp_
#define slic3r_GCode_DIWUMexParams_hpp_

// u-MEX powder parameters: port of UMexParams / UMexParams.from_config of the v2 post-processor
// (diw_engine.py 2.0.0, lines 438-569), reading the fork's diw_* settings. The checks, their order
// and their messages match v2 character for character, so v2 stays the golden reference.

#include "../Exception.hpp"

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

class ConfigBase;

// A u-MEX setting is out of range (v2: UMexConfigError). Stops the export.
// (Exception.hpp undefines its SLIC3R_DERIVE_EXCEPTION helper, hence the plain class.)
class DIWConfigError : public RuntimeError { public: using RuntimeError::RuntimeError; };

namespace DIW {

struct UMexParams
{
    double d_powder = 0, delta = 0, m_dot = 0, rho_tap = 0, rho_app = 0, eta = 0, s_hatch = 0;
    double lag_on = 0, lag_off = 0, f_max = 0;   // f_max in mm/min
    std::string policy;                           // "increase_spacing" | "clamp" | "error"
    double point_len = 0, hatch_angle = 0;
    int    powder_pin = 0, comp_pin = 0;
    double comp_t = 0, comp_settle = 0, comp_hop = 0;
    int    comp_interval = 1;
    double off_x = 0, off_y = 0, off_z = 0, z_clear = 0, travel_hop = 0;
    double travel_feed = 0;                       // mm/min
    std::optional<double> z_feed;                 // mm/min; empty = first Z-only feed of the file
    double stage1_dwell = 0;
    double wiper_x0 = 0, wiper_x1 = 0, wiper_y = 0, wiper_z = 0, wiper_feed = 0, wiper_hop = 0, wiper_zmin = 0;
    double voltage = 0, width_fallback = 0;
    std::optional<int> n_bottom, n_top;
    double paste_min_area = 0;
    // soft_limit_x_min, x_max, y_min, y_max, z_min, z_max (not configurable yet: all empty, as in v2's defaults)
    std::array<std::optional<double>, 6> limits;
};

// Builds and validates the parameters. Returns (params, warnings); throws DIWConfigError.
// uv_on_cmd is the UV-on command, whose P<n> pin must differ from the powder and compaction pins.
std::pair<UMexParams, std::vector<std::string>> read_umex_params(const ConfigBase &config, const std::string &uv_on_cmd);

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWUMexParams_hpp_
