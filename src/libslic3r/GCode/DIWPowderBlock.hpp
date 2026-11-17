#ifndef slic3r_GCode_DIWPowderBlock_hpp_
#define slic3r_GCode_DIWPowderBlock_hpp_

// ";DIW:" powder records, format v1 (diw/POWDER_RECORDS.md). One block carries one core layer's
// powder plan (v2: PowderPlan of plan_powder_layer) from the planner to the translator's sequencer.
// The Python reference is diw/tests/v2_blocks.py; both must produce byte-identical blocks and dumps.
//
// Reader rules (parse_records): POWDER_RECORDS.md section 4 is authoritative, error reason strings
// included ("bad line", "unknown tag", "unknown key", "key order", "missing key", "bad number",
// "unsupported version", "count mismatch", "crc mismatch", "nested begin", "end without begin",
// "unterminated block", "duplicate k"). The reader mirrors diw/tests/v2_blocks.py parse_records step for
// step, so both produce byte-identical dumps (diw/tests/compare_records.py checks this).
// The reader does not check placement against layer markers, k against the layer number, or the
// fingerprint against the settings: that is the translator's job (POWDER_PLAN increment 3a), using
// PowderLayerRec::begin_line / end_line.

#include "DIWUMexParams.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Slic3r {
namespace DIW {

// v2 diw_engine.py constants that enter the plan fingerprint (§5).
constexpr double POWDER_GUARD_MM = 0.001;   // extra inward offset so 3-decimal rounding never violates delta
constexpr int    UMEX_QUAD_SEGS  = 32;      // shapely buffer arc resolution

constexpr int    RECORD_FORMAT_VERSION = 1;

using RecPoint = std::pair<double, double>;

// v2 PowderRun. point: kind "point" (one point held for t_feed seconds); otherwise kind "line"
// (t_feed unused, 0.0).
struct PowderRunRec
{
    bool                  point  = false;
    std::vector<RecPoint> pts;
    double                t_feed = 0.0;
};

// v2 PathRec of plan.fallback_paths (the fields the sequencer needs).
struct FallbackPathRec
{
    std::string           ptype;
    std::vector<RecPoint> pts;
    std::vector<double>   de, feed;      // one per segment: pts.size() - 1
    double                width = 0.0;
    std::optional<double> travel_feed;
    double                z = 0.0;
};

struct PowderLayerRec
{
    enum class Status { ok, open, small };

    int      k  = 0;                       // 1-based printed-layer number
    Status   status = Status::ok;
    double   z = 0.0, h = 0.0, s = 0.0, F = 0.0, fu = 0.0;
    uint32_t fp = 0;                       // plan fingerprint
    double   ca = 0.0, oa = 0.0, ra = 0.0, hl = 0.0;
    int      nc = 0;
    std::vector<PowderRunRec>    runs;
    std::vector<FallbackPathRec> fallbacks;

    // Set by parse_records only (1-based line numbers of BEGIN and END); ignored by write_block.
    int begin_line = 0, end_line = 0;
};

const char *status_name(PowderLayerRec::Status st);

// CRC-32 IEEE 802.3 / zlib (reflected, poly 0xEDB88320, init and xorout 0xFFFFFFFF).
uint32_t crc32_ieee(std::string_view data);
uint32_t crc32_ieee_update(uint32_t crc, std::string_view data);   // crc: a previous result (0 to start)
// FNV-1a 32-bit (offset 0x811c9dc5, prime 0x01000193).
uint32_t fnv1a32(std::string_view data);

// "v=1;delta=...;quad_segs=32;" (§5) and its FNV-1a hash.
std::string fingerprint_string(const UMexParams &U);
uint32_t    plan_fingerprint(const UMexParams &U);

// 8 lower-case hex digits.
std::string hex8(uint32_t v);

// ptype <-> ty token (§3): [a-z0-9.-] kept, ' ' -> '_', any other byte -> %XX (upper-case hex), then
// every "layer" -> "la%79er". encode_ptype throws std::invalid_argument unless ptype is printable ASCII.
std::string encode_ptype(std::string_view ptype);
std::optional<std::string> decode_ptype(std::string_view token);   // empty if not canonical

// The canonical bytes of one block, every line ending in '\n'. Throws std::invalid_argument for a
// record the reader would reject (non-finite number, k < 1, wrong point counts, RUN/FB in an open or
// small block, st=ok without runs, non-zero STAT for st=open, de/feed sizes not pts - 1).
std::string write_block(const PowderLayerRec &rec);

struct ParsedRecords
{
    std::vector<PowderLayerRec>              blocks;   // accepted blocks, file order
    std::vector<std::pair<int, std::string>> errors;   // (1-based line, reason), detection order
};

ParsedRecords parse_records(std::string_view gcode);

// Canonical text dump (§6).
std::string dump_records(const ParsedRecords &recs);

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPowderBlock_hpp_
