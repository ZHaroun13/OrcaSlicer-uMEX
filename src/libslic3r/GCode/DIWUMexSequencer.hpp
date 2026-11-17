#ifndef slic3r_GCode_DIWUMexSequencer_hpp_
#define slic3r_GCode_DIWUMexSequencer_hpp_

// u-MEX core-layer sequencing (POWDER_PLAN.md architecture C, increment 3a): port of the layer machinery of
// the v2 post-processor (diw_engine.py 2.0.0): scan_gcode / LineInfo, split_layers, parse_core_layer,
// resolve_powder_feed and the _UMexMixin flow (_process_lines_umex, _convert_core_layer, _paste_travel,
// _feed_paste_line, _mv, _apply_limits, _warn_agg / _flush_agg).
//
// The powder plan of a core layer is not computed here: it comes from a ";DIW:" record block
// (diw/POWDER_RECORDS.md, GCode/DIWPowderBlock). A layer is converted iff it carries a valid block; v2's
// classify_layers stays in v2, because in this architecture the slicer decides which layers are core.
// Each core layer is re-emitted as S1 paste walls + auger stop dwell, S2 nozzle wipe, S3 UV cure,
// S4 powder (increment 3b: feeder I/O, lag compensation, early stop, point dwell, nozzle offset, Z handling;
// a test-only stub reduces it to its header line), S5 compaction, S6 layer advance.
//
// The paste translator (GCode/DIWTranslator.cpp, port of DIWPostProcessor) derives from UMexSequencer as
// v2's DIWPostProcessor derives from _UMexMixin, so both share one state and one process_line.

#include "DIWPowderBlock.hpp"
#include "DIWUMexParams.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {
namespace DIW {

// ------------------------------------------------------------------------------------------------
// Whole-file scan (v2 scan_gcode / LineInfo): the fields the sequencer needs.
// ------------------------------------------------------------------------------------------------

struct LineInfo
{
    std::string cmd;                    // "G1", "M83", ... ("" for none)
    bool   is_motion  = false;          // G0..G3
    bool   is_arc     = false;          // G2 / G3
    bool   has_e_word = false;
    double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
    double de = 0;                      // resolved per-move dE (M82/M83/G92)
    double e_after = 0;                 // absolute E after the line
    bool   rel_after = false;           // M83 state after the line
    bool   relxyz = false;              // G91 state after the line
    double feed = 1200.;                // modal F after the line
    std::string ftype;                  // normalised ;TYPE: in force
    std::optional<double> width;        // ;WIDTH: in force
    std::optional<double> height_here;  // ;HEIGHT: value of THIS line's comment (split_layers)

    bool has_xy() const;
    bool is_extruding_move() const;     // straight G0/G1 that deposits paste while moving in XY
};

std::vector<LineInfo> scan_gcode(const std::vector<std::string> &lines);

// v2 _RE_LAYER_START.match: ^\s*;\s*(?:LAYER_CHANGE\b|LAYER\s*:\s*\d+), case-insensitive.
bool is_layer_start(const std::string &line);

// A printed layer: raw line range [start, end) (v2 LayerChunk without the classification).
struct LayerChunk
{
    int    index = 0;                   // 1-based printed-layer number k
    size_t start = 0, end = 0;
    std::set<std::string> types;
    std::optional<double> z, height;
    int    n_extruding = 0;
    bool   has_arc = false, has_relxyz = false;
};

// v2 split_layers: the part before "; CONFIG_BLOCK_START" split at ;LAYER_CHANGE / ;LAYER:<n>.
std::vector<LayerChunk> split_layers(const std::vector<std::string> &lines, const std::vector<LineInfo> &infos,
                                     size_t *cfg_start = nullptr);

// One continuous extrusion path of a single role (v2 PathRec).
struct PathRec
{
    std::string ptype;
    std::string kind;                   // "wall" | "powder" | "paste"
    std::vector<std::pair<double, double>> points;
    std::vector<double> seg_de, seg_feed;
    std::vector<std::string> seg_raw;   // empty for a synthetic (fallback) path
    double width = 0, z = 0;
    std::vector<size_t> pre_lines;
    std::optional<double> travel_feed;
};

struct LayerModel
{
    std::vector<size_t>  header;
    std::vector<PathRec> paths;
    std::vector<size_t>  tail;
    std::optional<double> z, z_feed;
};

LayerModel parse_core_layer(const std::vector<std::string> &lines, const std::vector<LineInfo> &infos,
                            const LayerChunk &chunk, double width_fallback);

// v2 resolve_powder_feed. Throws DIWConfigError for policy "error" (v2 message).
struct PowderFeed
{
    double F = 0, s_eff = 0, f_calc = 0;
    bool   overflow = false;
};
PowderFeed resolve_powder_feed(double m_dot, double rho_tap, double s_mm, double h_mm, double eta, double f_max,
                               const std::string &policy);

// ------------------------------------------------------------------------------------------------
// Translator settings and state (v2 DIWPostProcessor attributes), shared by the paste translator
// (DIWTranslator.cpp) and the sequencer.
// ------------------------------------------------------------------------------------------------

struct TranslatorParams
{
    double df                = 1.75;
    double volumetric_ratio  = 0.;
    double b_multiplier      = 1.;
    bool   b_drivetrain      = false;
    double lead_b            = 4.;
    double i_gear_b          = 7.5;
    int    a_mode            = 0;     // DIWAAxisMode (PrintConfig.hpp); diwaFlowScaled
    double k_screw           = 3.48;
    bool   enable_retraction = true;
    double max_retract_de    = 5.;
    double max_retract_f     = 300.;  // mm/min
    double post_dwell        = 0.;
    bool   enable_uv         = true;
    std::string uv_on        = "M62 P3 Q1";
    std::string uv_off       = "M62 P3 Q0";
    double uv_offset_y       = 112.538;
    double uv_end_dwell      = 0.;
    double uv_layer_dwell    = 5.;
    // u-MEX powder mode (diw_powder_infill).
    bool   umex              = false;
    UMexParams               U;
    std::vector<std::string> config_warnings;
};

// Where a ";DIW:" block sat in the input: its first line in the translator's line numbering, i.e. the
// number of G-code lines (after record removal and universal-newline splitting) that precede it.
struct PlacedBlock
{
    PowderLayerRec rec;
    size_t         pos = 0;
};

class UMexSequencer
{
public:
    using Lines = std::vector<std::string>;

    UMexSequencer(const TranslatorParams &p, bool stub_powder_stage)
        : P(p), warnings(p.config_warnings), m_stub_powder_stage(stub_powder_stage) {}
    virtual ~UMexSequencer() = default;

protected:
    const TranslatorParams &P;

    // ---- paste translator state (v2 DIWPostProcessor) ----
    double cur_x = 0., cur_y = 0., cur_z = 0., cur_f = 1200.;
    double last_e = 0.;
    bool   relative_e = false;
    double cumulative_b = 0., cumulative_a = 0.;
    bool   is_extruding = false;
    bool   uv_active = false;
    int    layer_count = 0;
    bool   extruded_on_layer = false;
    bool   cured_for_layer_change = false;
    bool   in_custom_start = false, startup_uv_check_done = false, in_initial_load = false, in_intro_line = false;
    bool   suppress_uv_curing = false;
    double nozzle_target_x = 0., nozzle_target_y = 0.;
    bool   final_curing_done = false;
    bool   in_config_block = false;
    long   line_idx = -1, final_stop_object_idx = -1;

    // ---- u-MEX state (v2 _UMexMixin) ----
    std::vector<std::string> warnings;
    bool   shutdown_done = false;

    void warn(const std::string &msg);

    // Implemented by the paste translator.
    virtual bool  speed_mode() const = 0;
    virtual Lines process_line(const std::string &raw_line) = 0;
    virtual Lines curing_sequence(double return_x, double return_y) = 0;

    // v2 _process_lines_umex: the whole body in powder mode. blocks: the accepted ";DIW:" blocks with their
    // positions (checked here against the layers and the settings; DIWConfigError on a bad one).
    Lines process_lines_umex(const std::vector<std::string> &raw_lines, const std::vector<PlacedBlock> &blocks);

private:
    struct AggRec
    {
        std::function<std::string(int n, const std::string &layers, double vmin, double vmax)> fmt;
        std::vector<int>    layers;
        std::vector<double> vals;
    };
    struct LimitHit
    {
        double count = 0., max_excess = 0., clamped = 0.;
    };

    bool                  m_stub_powder_stage;
    std::optional<double> m_file_z_feed;
    std::vector<std::pair<std::string, LimitHit>> m_limit_hits;   // insertion order, as v2's dict
    std::vector<std::pair<std::string, AggRec>>   m_agg;
    int                   m_converted_layers = 0;

    void   warn_agg(const std::string &key, std::function<std::string(int, const std::string &, double, double)> fmt,
                    int layer, double val = 0.0);
    void   flush_agg();
    std::optional<std::string> a_off_line() const;
    double z_feed() const;
    void   apply_limits(std::optional<double> &x, std::optional<double> &y, std::optional<double> &z, bool clamp,
                        const std::string &what);
    void   mv(Lines &out, const char *cmd, std::optional<double> x, std::optional<double> y, std::optional<double> z,
              std::optional<double> f, bool a_off, const char *check, bool clamp = true);

    std::optional<Lines> convert_core_layer(const std::vector<std::string> &raw_lines, const std::vector<LineInfo> &infos,
                                            const LayerChunk &chunk, double prev_z, bool last_in_stack,
                                            const PlacedBlock &block);
    void   paste_travel(double x, double y, double z, double feed, std::optional<double> z_feed_layer, Lines &out);
    void   feed_paste_line(const std::string &raw, double de, double feed, Lines &out);
    // Stage 4 (v2 _emit_powder, _powder_transit, _pmove, _powder_xy, _z_disp, _z_safe).
    void   emit_powder(const PowderLayerRec &plan, double z, double F, int k, Lines &out);
    std::pair<double, double> powder_xy(const RecPoint &p) const;
    double z_disp(double z_layer) const;
    double z_safe(double z_layer) const;
    void   powder_transit(const RecPoint &pt, double z_layer, Lines &out);
    void   pmove(Lines &out, const RecPoint &p, double F);
};

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWUMexSequencer_hpp_
