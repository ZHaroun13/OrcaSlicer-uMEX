#ifndef slic3r_GCode_DIWPowderShadow_hpp_
#define slic3r_GCode_DIWPowderShadow_hpp_

// u-MEX native powder planner, shadow mode (diw/PLANNER_DESIGN.md sections 1, 6, 8 and 9).
//
// The binder is a streaming filter over the final G-code text (GCode::GCodeOutputStream::write). It splits
// the bytes into lines, runs the translator's own GCodeScanner / ChunkTracker over them, holds the lines of the
// current printed layer (chunk) and, when the chunk closes, decides whether the layer is a core layer (gates
// 0-11 of section 6.3). For a core layer it plans the powder (DIWPowderPlanner) and inserts the ";DIW:" record
// block right after the chunk's start-marker line. Nothing else in the text changes: the ";DIWP <token> <id> m=<0|1>"
// placeholder lines written by the capture are always removed.
//
// Capture (generator thread, GCode::process_layer) stores the native geometry of a layer with put_native()
// and writes placeholder(id, m) into the G-code. In the offline replay (diw_translate --plan-records,
// section 9) there is no capture: with ShadowOptions::text_only, a chunk without placeholders is planned from
// its text alone (rings from v2's snap + linemerge of the text outer / overhang walls, no native band), and a
// chunk with placeholders uses the native data reloaded from an ORCA_DIW_SHADOW_DUMP directory when present.

#include "../Point.hpp"
#include "../Polygon.hpp"

#include "DIWPowderPlanner.hpp"
#include "DIWTranslator.hpp"   // DIW::ExtrusionBBox
#include "DIWUMexSequencer.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace Slic3r {

class DynamicPrintConfig;
class Layer;
class Print;
class PrintObject;
class SupportLayer;
struct PrintInstance;

namespace DIW {

// One GCode::LayerToPrint of the layer being generated (GCode::process_layer), as the capture needs it.
struct CaptureLayer
{
    const Layer        *object_layer    = nullptr;
    const SupportLayer *support_layer   = nullptr;
    const PrintObject  *original_object = nullptr;
};

// The G-code frame of the capture (section 5.3).
struct ShadowFrame
{
    Vec2d writer_xy_offset { 0., 0. };   // GCode::m_diw_xy_offset (double)
    Vec2d extruder_offset { 0., 0. };    // extruder_offset[0] (the shadow refuses differing offsets)
    bool  high_precision = false;        // 4 XY decimals instead of 3
    // GCode::text_grid_nm: an object point (scaled) and the instance origin (mm) -> the printed point on the nm grid.
    // When empty, the capture computes the same transform from the three fields above.
    std::function<Point(const Point &, const Vec2d &)> to_grid;
};

// The native capture of one layer (section 5), G-code frame, text grid in nm.
struct NativeLayer
{
    bool                             native_core = false;
    std::vector<Polygons>            rings;        // closed outer loops, per instance
    std::vector<WallPiece>           pieces;       // wall pieces of the band (and of the frame self-check)
    std::vector<std::vector<Vec2d>>  open_outer;   // open outer / overhang walls (mm, text grid): P1b
    std::string                      error;        // non-empty: the capture failed with this message
};

struct ShadowOptions
{
    // PRODUCT_DESIGN.md 2. shadow: increment 4 exactly (gate 3 agreement, no suppression, warnings only). product:
    // the native classification decides, a native core layer without a valid block is a failure (failed()), and the
    // sparse infill of every st=ok core chunk is suppressed (section 4). The struct default is shadow (the T8 unit
    // streams keep their increment-4 meaning); from_env() defaults to product.
    enum class Mode { shadow, product };
    Mode           mode        = Mode::shadow;
    // Product mode: suppress the sparse infill of st=ok core chunks (ORCA_DIW_TEST_NO_SUPPRESS=1 clears it).
    bool           suppress    = true;
    // Test only: a deliberately wrong edit (ORCA_DIW_TEST_SUPPRESS_MUTATE), and the release of the edit without
    // V0-V3 (ORCA_DIW_TEST_SKIP_VERIFY=1).
    SuppressMutate mutate      = SuppressMutate::none;
    bool           skip_verify = false;
    // Offline replay: a chunk without placeholders is planned from its text (text-only adapter).
    bool        text_only = false;
    // Keep the ";DIWP" lines in the output (ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS=1; replay debugging only).
    bool        keep_placeholders = false;
    // Write k<k>.json per block into this directory (ORCA_DIW_SHADOW_DUMP).
    std::string dump_dir;
    // Test-only fault injection (ORCA_DIW_SHADOW_FAULT): "capture", "plan", "binder", "suppress" (the suppression
    // throws: a counted fallback, product mode) or "cancel" (CanceledException from the first core chunk's finalize).
    std::string fault;
    // Decimals of the XY axes in the G-code (3, or 4 with high-precision G-code): frame self-check tolerance.
    int         xy_decimals = 3;
    // Called once per finalized core chunk; throws CanceledException to cancel the export.
    std::function<void()> throw_if_canceled;
    // The per-export token of the placeholder lines (";DIWP <token> <id> m=<0|1>"). Empty: the shadow draws a
    // random one. Only lines with this token are placeholders; any other ";DIWP" text (custom G-code) is ordinary
    // text and passes through unchanged.
    std::string placeholder_token;

    // From the environment: mode (ORCA_DIW_POWDER_MODE: unset or "product" -> product, "shadow" -> shadow),
    // suppress (ORCA_DIW_TEST_NO_SUPPRESS), mutate, skip_verify, keep_placeholders, dump_dir and fault. Throws
    // std::invalid_argument for an unknown ORCA_DIW_POWDER_MODE or ORCA_DIW_TEST_SUPPRESS_MUTATE value.
    static ShadowOptions from_env();
    bool product() const { return mode == Mode::product; }
};
const char *to_string(ShadowOptions::Mode m);

// Why a u-MEX powder job cannot be planned on this print, or "" (PRODUCT_DESIGN.md 5.1 / 6.3): BBL printer G-code,
// spiral vase, G-code axis remap, belt printer, or extruders with differing extruder_offset values. The kill switch
// is not a reason here (PowderShadow::create checks it first). Shared by PowderShadow::create and Print::validate.
std::string powder_refusal_reason(const DynamicPrintConfig &full, bool bbl_printer, bool spiral_vase, bool axis_remap);

class PowderShadow
{
public:
    using Sink = std::function<void(std::string_view)>;

    // P: the translator's own parameters (diw_read_translator_params of the full print config); P.umex must be set.
    // P must outlive the shadow.
    PowderShadow(const TranslatorParams &P, ShadowOptions opts);
    // The same, owning its parameters.
    PowderShadow(std::unique_ptr<TranslatorParams> P, ShadowOptions opts);
    ~PowderShadow();

    // Section 3: the shadow of a G-code export, or nullptr. *reason is left empty when the job is not a u-MEX
    // powder job (or its settings are invalid: the translator reports that), else it says why the shadow is
    // refused (kill switch ORCA_DIW_NO_NATIVE_PLAN=1, BBL G-code, spiral vase, axis remap, belt printer, differing
    // extruder offsets). Options from the environment (ShadowOptions::from_env); never throws.
    static std::shared_ptr<PowderShadow> create(const Print &print, const DynamicPrintConfig &full, bool spiral_vase,
                                                bool axis_remap, std::string *reason);

    // ---- capture (generator thread, GCode::process_layer; section 5) ----
    // Classifies the layer from the in-memory roles, stores its walls and rings when it is a native core layer and
    // returns its placeholder line ";DIWP <token> <id> m=<0|1>". Exceptions (other than CanceledException) are stored as the
    // capture's error; fault hook "capture".
    std::string capture(const std::vector<CaptureLayer> &layers, const std::vector<const PrintInstance *> *ordering,
                        size_t single_object_instance_idx, const ShadowFrame &frame,
                        const std::function<bool(const ExtrusionPath &)> &path_uses_arcs);

    // ---- capture side (any thread) ----
    // A new placeholder id (1, 2, ...).
    uint64_t           next_id();
    // ";DIWP <token> <id> m=<0|1>\n" with this shadow's token
    std::string        placeholder(uint64_t id, bool m) const { return placeholder_line(m_token, id, m); }
    static std::string placeholder_line(const std::string &token, uint64_t id, bool m);
    const std::string &token() const { return m_token; }
    // Stores the native capture of placeholder id (consumed by the binder when the placeholder's chunk closes).
    void               put_native(uint64_t id, NativeLayer layer);
    // Reloads the native capture of every k<k>.json of a dump directory (offline replay). Returns the count.
    size_t             load_dump_dir(const std::string &dir);

    // ---- output side (one thread, in file order) ----
    void feed(std::string_view bytes, const Sink &sink);
    void feed(const char *bytes, const Sink &sink) { feed(std::string_view(bytes ? bytes : ""), sink); }
    // Releases every held line; the open chunk can no longer get a block (reason "sync").
    void sync(const Sink &sink);
    // End of the text: the held chunk is final, a partial last line is released.
    void finish(const Sink &sink);

    // Aggregated warnings (section 8), for active_step_add_warning. Product mode (PRODUCT_DESIGN.md 3.2 / 5.1): only
    // text_only, native_only_nosparse and final; the failures are in failure_message().
    std::vector<std::string> warnings() const;
    // One log line with the counters (PRODUCT_DESIGN.md 5.2).
    std::string              summary() const;
    // Product mode: one log line per suppression fallback reason (k list and the first detail). Empty in shadow mode.
    std::vector<std::string> fallback_log() const;
    // Product mode (PRODUCT_DESIGN.md 5.1): a core layer would be printed without powder (a fatal reason was
    // recorded, the binder was disabled, or the export has no block at all). Always false in shadow mode, and
    // never true because of a suppression fallback. Final after finish().
    bool                     failed() const;
    // The export-stopping message of 5.1 ("" when not failed()).
    std::string              failure_message() const;
    const ShadowOptions     &options() const { return m_opts; }

    struct Stats
    {
        int blocks = 0;                          // blocks written (carriers included)
        int carriers = 0;                        // st=open carriers for h <= 0 and policy=error
        int max_held_chunks = 0;                 // never more than 1 (T10)
        bool disabled = false;
        double capture_ms = 0.;                  // time spent in capture() (T10)
        double binder_ms = 0.;                   // time spent in feed / sync / finish, planner included (T10)
        std::map<std::string, std::vector<int>> reasons;   // gate reason -> k list
        // PRODUCT_DESIGN.md 3.4 / 4 / 5.2
        std::vector<int> orphan;                 // st=ok chunks with a powder-kind point outside S (shadow: counted only)
        int    native_only = 0;                  // product: native core layers whose text has a skin type (3.2 (a))
        int    suppressed = 0;                   // product: st=ok chunks released edited
        std::map<std::string, std::vector<int>> fallbacks;   // product: suppression fallback reason -> k list
        int    removed_moves = 0, removed_other = 0, restore_lines = 0;
        double removed_de = 0.;                  // sum of the removed moves' dE (G8)
        bool   no_core = false;                  // product: the export ended without any block
    };
    const Stats &stats() const { return m_stats; }

private:
    struct Placeholder { size_t held_idx; uint64_t id; bool m; };

    std::unique_ptr<TranslatorParams> m_own_params;
    const TranslatorParams &P;
    ShadowOptions           m_opts;
    uint32_t                m_fp = 0;
    std::string             m_token;   // placeholder token (ShadowOptions::placeholder_token or random)
    int                     m_single_bead = 0;   // walls refused as rings by bead_bounds_slice (capture, m_mutex)

    // capture map
    mutable std::mutex                          m_mutex;
    std::unordered_map<uint64_t, NativeLayer>   m_native;
    uint64_t                                    m_next_id = 0;

    // stream state
    std::string               m_partial;
    std::string               m_out;          // released text not yet given to the sink
    GCodeScanner              m_scanner;
    ChunkTracker              m_tracker;
    size_t                    m_line_no = 0;  // index of the next complete line
    bool                      m_passthrough = false;
    bool                      m_finished = false;
    bool                      m_unbindable = false;   // the open chunk was released by sync()
    std::vector<std::string>  m_held;
    std::vector<LineInfo>     m_held_infos;
    std::vector<Placeholder>  m_ph;
    int                       m_held_chunk = -1;      // chunk index of the held lines (T10 counter)
    int                       m_held_nchunks = 0;
    double                    m_prev_z = 0.0;
    // The scanner state and the extruded-bbox state (translator rule) before the held chunk's line 0 (4.4).
    ScanState                 m_chunk_before;
    ExtrusionBBox             m_bbox;                 // fed with every line before the config block, in file order
    ExtrusionBBox             m_chunk_bbox_before;
    bool                      m_cancel_fired = false; // fault "cancel" fires once

    Stats                     m_stats;
    std::map<std::string, std::string> m_first_what;   // reason -> first message
    std::map<std::string, std::string> m_fallback_what; // fallback reason -> first detail

    void handle_line(std::string &&line, bool &consumed);
    void release(std::string_view line);                // drops ";DIWP" lines unless kept
    void release_held();
    void close_chunk(const LayerChunk &chunk, bool final);
    std::optional<std::string> finalize(const LayerChunk &chunk, bool final);   // the block, if any
    void count(const std::string &reason, int k, const std::string &what = std::string());
    void disable(const std::string &why);
    void flush(const Sink &sink);
    std::optional<NativeLayer> take_native(uint64_t id);
};

// Review fix (single-bead walls): a closed wall loop, or an open outer wall, bounds the cavity only when the layer's
// slice lies beyond the bead on one side of its centreline. A single-bead wall (an Arachne odd centreline loop, a
// classic thin wall) has nothing beyond the bead on either side: the area it encloses is empty (the bore of a thin
// tube), not solid, so it is no ring. Paths in the slices' frame (scaled object coordinates), with their bead widths
// (scaled). Up to 64 segment midpoints are probed at 0.75 bead widths on both sides of the centreline; the wall is a
// boundary when at least a quarter of them have the slice on exactly one side. No sample, or no slice: true.
struct BeadPath
{
    const Points *pts;
    double        width;   // scaled
};
bool bead_bounds_slice(const std::vector<BeadPath> &paths, const ExPolygons &slices);

// The ";DIWP <token> <id> m=<0|1>" placeholder of this token in a line (trailing "\r\n" / "\n" allowed). A line
// with another token, or without one, is not a placeholder.
bool parse_placeholder(std::string_view line, const std::string &token, uint64_t &id, bool &m);
bool is_placeholder_line(std::string_view line, const std::string &token);   // parse_placeholder succeeds

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPowderShadow_hpp_
