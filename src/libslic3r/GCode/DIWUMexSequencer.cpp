#include "DIWUMexSequencer.hpp"
#include "DIWPyCompat.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <string>

// Port of the u-MEX layer machinery of diw_engine.py 2.0.0 (see the header). Function by function, the
// structure and the order of operations follow the Python code, so the two can be compared side by side;
// every number in a generated line or a warning is formatted as the Python f-string does (py_fixed /
// py_repr), and floating-point expressions keep Python's operation order.

namespace Slic3r {
namespace DIW {

namespace {

constexpr double EPS_E = 1e-7;   // v2 _EPS_E

const std::set<std::string> SPARSE_TYPES = { "sparse infill", "internal infill" };

// Position after an optional "N<digits><spaces>" line number at the start of s (for ^(?:N\d+\s+)?).
// Uppercase 'N' only: v2's _RE_CMD has no IGNORECASE, so in "n7 G1 ..." the group does not match.
size_t skip_line_number(const std::string &s)
{
    if (! s.empty() && s[0] == 'N') {
        size_t k = 1;
        while (k < s.size() && is_digit(s[k])) ++k;
        if (k > 1 && space_at(s, k)) {
            for (size_t n; k < s.size() && (n = space_len_at(s, k)) > 0;) k += n;
            return k;
        }
    }
    return 0;
}

// v2 _cmd_of: _RE_CMD = ^(?:N\d+\s+)?([GMgm])0*(\d+), result f"{letter.upper()}{int(digits)}".
std::string cmd_of(const std::string &code)
{
    for (size_t k : { skip_line_number(code), size_t(0) }) {
        if (k >= code.size())
            continue;
        const char c = code[k];
        if (c != 'G' && c != 'g' && c != 'M' && c != 'm')
            continue;
        size_t j = k + 1;
        const size_t d0 = j;
        while (j < code.size() && is_digit(code[j])) ++j;
        if (j == d0)
            continue;
        size_t z = d0;
        while (z + 1 < j && code[z] == '0') ++z;   // int(): leading zeros dropped, "0" kept
        return std::string(1, char(c == 'g' ? 'G' : c == 'm' ? 'M' : c)) + code.substr(z, j - z);
    }
    return {};
}

// Skips \s* at s[i].
size_t skip_ws(const std::string &s, size_t i)
{
    for (size_t n; i < s.size() && (n = space_len_at(s, i)) > 0;) i += n;
    return i;
}

// ^\s*<KEY>\s*:\s*  (KEY upper case, matched case-insensitively). Returns the position after it or npos.
size_t match_key(const std::string &s, const char *key)
{
    size_t i = skip_ws(s, 0);
    for (const char *k = key; *k; ++k, ++i)
        if (i >= s.size() || lower(s[i]) != lower(*k))
            return std::string::npos;
    i = skip_ws(s, i);
    if (i >= s.size() || s[i] != ':')
        return std::string::npos;
    return skip_ws(s, i + 1);
}

// _RE_WIDTH / _RE_HEIGHT: ^\s*KEY\s*:\s*([-+]?[0-9]*\.?[0-9]+)
std::optional<double> match_key_number(const std::string &comment, const char *key)
{
    const size_t i = match_key(comment, key);
    if (i == std::string::npos)
        return std::nullopt;
    const size_t e = match_number(comment, i);
    if (e == std::string::npos)
        return std::nullopt;
    return parse_number(comment, i, e);
}

// _norm_type(m.group(1)) for _RE_TYPE = ^\s*TYPE\s*:\s*(.*?)\s*$ : re.sub(r"[\s_]+", " ", raw.strip().lower())
std::optional<std::string> match_type(const std::string &comment)
{
    const size_t i = match_key(comment, "TYPE");
    if (i == std::string::npos)
        return std::nullopt;
    const std::string raw = to_lower(strip(comment.substr(i)));
    // length of the [\s_] character at raw[q], 0 if none
    auto sep_len = [&raw](size_t q) -> size_t {
        const size_t n = space_len_at(raw, q);
        return n > 0 ? n : (raw[q] == '_' ? 1 : 0);
    };
    std::string r;
    for (size_t k = 0; k < raw.size();) {
        if (sep_len(k) > 0) {
            r += ' ';
            for (size_t n; k < raw.size() && (n = sep_len(k)) > 0;) k += n;
        } else
            r += raw[k++];
    }
    return r;
}

std::string path_kind(const std::string &ptype)
{
    if (SPARSE_TYPES.count(ptype)) return "powder";
    if (ptype.find("wall") != std::string::npos) return "wall";
    return "paste";
}

} // namespace

// Python truthiness of an Optional[float]: None and 0.0 are false.
bool truthy(const std::optional<double> &v) { return v.has_value() && *v != 0.0; }

// ------------------------------------------------------------------------------------------------
// scan_gcode / LineInfo
// ------------------------------------------------------------------------------------------------

bool LineInfo::has_xy() const { return py_hypot(x1 - x0, y1 - y0) > 1e-9; }

bool LineInfo::is_extruding_move() const { return is_motion && ! is_arc && de > EPS_E && has_xy(); }

LineInfo GCodeScanner::step(const std::string &raw)
{
    std::string s = raw;
    while (! s.empty() && (s.back() == '\r' || s.back() == '\n'))
        s.pop_back();
    const size_t ci = s.find(';');
    const std::string code    = strip(ci != std::string::npos ? s.substr(0, ci) : s);
    const std::string comment = ci != std::string::npos ? s.substr(ci + 1) : std::string();
    LineInfo li;
    if (! comment.empty()) {
        if (std::optional<std::string> t = match_type(comment))
            ftype = *t;
        else if (std::optional<double> w = match_key_number(comment, "WIDTH"))
            width = *w;
        else if (std::optional<double> h = match_key_number(comment, "HEIGHT"))
            li.height_here = *h;
    }
    li.cmd = cmd_of(code);
    li.x0 = x; li.y0 = y; li.z0 = z;
    const std::string &cmd = li.cmd;
    if (cmd == "G0" || cmd == "G1" || cmd == "G2" || cmd == "G3") {
        li.is_motion = true;
        li.is_arc    = cmd == "G2" || cmd == "G3";
        const WordMatch mx = find_word(code, "X"), my = find_word(code, "Y"), mz = find_word(code, "Z");
        if (mx) x = absxyz ? mx.value : x + mx.value;
        if (my) y = absxyz ? my.value : y + my.value;
        if (mz) z = absxyz ? mz.value : z + mz.value;
        if (const WordMatch me = find_word(code, "E")) {
            li.has_e_word = true;
            if (rel)
                li.de = me.value;
            else {
                li.de = me.value - e;
                e     = me.value;
            }
        }
        if (const WordMatch mf = find_word(code, "F"))
            feed = mf.value;
    } else if (cmd == "G92") {
        if (const WordMatch me = find_word(code, "E")) {
            li.has_e_word = true;
            e = me.value;
        }
        if (const WordMatch m = find_word(code, "X")) x = m.value;
        if (const WordMatch m = find_word(code, "Y")) y = m.value;
        if (const WordMatch m = find_word(code, "Z")) z = m.value;
    } else if (cmd == "M82")
        rel = false;
    else if (cmd == "M83")
        rel = true;
    else if (cmd == "G90")
        absxyz = true;
    else if (cmd == "G91")
        absxyz = false;
    else if (cmd == "G28")
        x = y = z = 0.;
    li.x1 = x; li.y1 = y; li.z1 = z;
    li.e_after   = e;
    li.rel_after = rel;
    li.relxyz    = ! absxyz;
    li.feed      = feed;
    li.ftype     = ftype;
    li.width     = width;
    return li;
}

std::vector<LineInfo> scan_gcode(const std::vector<std::string> &lines)
{
    std::vector<LineInfo> infos;
    infos.reserve(lines.size());
    GCodeScanner scanner;
    for (const std::string &raw : lines)
        infos.push_back(scanner.step(raw));
    return infos;
}

// ------------------------------------------------------------------------------------------------
// split_layers
// ------------------------------------------------------------------------------------------------

bool is_layer_start(const std::string &line)
{
    size_t i = skip_ws(line, 0);
    if (i >= line.size() || line[i] != ';')
        return false;
    i = skip_ws(line, i + 1);
    const std::string rest = to_lower(line.substr(i));
    if (rest.compare(0, 12, "layer_change") == 0 && ! word_at(rest, 12))
        return true;
    if (rest.compare(0, 5, "layer") != 0)
        return false;
    size_t q = skip_ws(rest, 5);
    if (q >= rest.size() || rest[q] != ':')
        return false;
    q = skip_ws(rest, q + 1);
    return q < rest.size() && is_digit(rest[q]);
}

std::vector<LayerChunk> split_layers(const std::vector<std::string> &lines, const std::vector<LineInfo> &infos, size_t *cfg_start_out)
{
    size_t cfg_start = lines.size();
    for (size_t i = 0; i < lines.size(); ++i)
        if (lines[i].find("; CONFIG_BLOCK_START") != std::string::npos) {
            cfg_start = i;
            break;
        }
    if (cfg_start_out)
        *cfg_start_out = cfg_start;
    std::vector<LayerChunk> chunks;
    ChunkTracker tracker;
    for (size_t i = 0; i < cfg_start; ++i)
        if (tracker.step(i, lines[i], infos[i]) == ChunkTracker::opened)
            if (std::optional<LayerChunk> c = tracker.take_closed())
                chunks.push_back(std::move(*c));
    if (std::optional<LayerChunk> c = tracker.finish(cfg_start))
        chunks.push_back(std::move(*c));
    return chunks;
}

ChunkTracker::Ev ChunkTracker::step(size_t i, const std::string &line, const LineInfo &li)
{
    Ev ev = none;
    if (is_layer_start(line) && (! m_cur || m_cur->n_extruding > 0)) {
        if (m_cur) {
            m_cur->end = i;
            m_closed.push_back(std::move(*m_cur));
        }
        m_cur.emplace();
        m_cur->index = ++m_n_opened;
        m_cur->start = i;
        ev = opened;
    }
    if (! m_cur)
        return ev;
    LayerChunk *cur = &*m_cur;
    cur->end = i + 1;
    if (li.relxyz)
        cur->has_relxyz = true;
    if (li.is_motion && li.is_arc && li.de > EPS_E)
        cur->has_arc = true;
    if (li.height_here && ! cur->height)
        cur->height = li.height_here;
    if (li.is_extruding_move()) {
        ++cur->n_extruding;
        cur->types.insert(li.ftype);
        if (! cur->z)
            cur->z = li.z1;
    }
    return ev;
}

std::optional<LayerChunk> ChunkTracker::take_closed()
{
    if (m_closed.empty())
        return std::nullopt;
    LayerChunk c = std::move(m_closed.front());
    m_closed.erase(m_closed.begin());
    return c;
}

std::optional<LayerChunk> ChunkTracker::finish(size_t end)
{
    if (! m_cur)
        return std::nullopt;
    LayerChunk c = std::move(*m_cur);
    m_cur.reset();
    c.end = end;
    return c;
}

// ------------------------------------------------------------------------------------------------
// parse_core_layer
// ------------------------------------------------------------------------------------------------

LayerModel parse_core_layer(const std::vector<std::string> &lines, const std::vector<LineInfo> &infos, const LayerChunk &chunk,
                            double width_fallback)
{
    LayerModel m;
    std::vector<size_t> pending;
    PathRec *cur = nullptr;
    bool seen = false;
    std::optional<double> trav_f;
    for (size_t i = chunk.start; i < chunk.end; ++i) {
        const LineInfo &li = infos[i];
        if (li.is_motion) {
            if (li.is_extruding_move()) {
                const bool contiguous = cur != nullptr && cur->ptype == li.ftype &&
                                        py_hypot(li.x0 - cur->points.back().first, li.y0 - cur->points.back().second) < 1e-6;
                if (! contiguous) {
                    PathRec p;
                    p.ptype = li.ftype;
                    p.kind  = path_kind(li.ftype);
                    p.points.emplace_back(li.x0, li.y0);
                    p.width       = truthy(li.width) ? *li.width : width_fallback;
                    p.z           = li.z1;
                    p.pre_lines   = std::move(pending);
                    p.travel_feed = trav_f;
                    pending.clear();
                    m.paths.push_back(std::move(p));
                    cur  = &m.paths.back();
                    seen = true;
                }
                cur->points.emplace_back(li.x1, li.y1);
                cur->seg_de.push_back(li.de);
                cur->seg_feed.push_back(li.feed);
                cur->seg_raw.push_back(lines[i]);
                cur->seg_width.push_back(truthy(li.width) ? *li.width : width_fallback);
                if (! m.z)
                    m.z = li.z1;
            } else {
                cur = nullptr;
                if (li.has_xy())
                    trav_f = li.feed;
                else if (std::abs(li.z1 - li.z0) > 1e-9)
                    m.z_feed = li.feed;
            }
        } else {
            if (! seen)
                m.header.push_back(i);
            else if (li.cmd == "M82" || li.cmd == "M83" || li.cmd == "G90" || li.cmd == "G91")
                continue;
            else if (li.cmd == "G92" && li.has_e_word)
                continue;
            else
                pending.push_back(i);
        }
    }
    m.tail = std::move(pending);
    return m;
}

// ------------------------------------------------------------------------------------------------
// resolve_powder_feed
// ------------------------------------------------------------------------------------------------

PowderFeed resolve_powder_feed(double m_dot, double rho_tap, double s_mm, double h_mm, double eta, double f_max, const std::string &policy)
{
    // compute_f_powder
    const double denom = rho_tap * s_mm * h_mm * eta;
    if (denom <= 0.0)
        throw DIWConfigError("rho_tap, hatch spacing, layer height and fill multiplier must be > 0");
    const double f_calc = 1000.0 * m_dot / denom;
    if (f_calc <= f_max)
        return { f_calc, s_mm, f_calc, false };
    if (policy == "error")
        throw DIWConfigError("F_powder = " + py_fixed(1, f_calc) + " mm/min exceeds powder_f_max = " + py_fixed(1, f_max) +
                             " mm/min (m_dot=" + py_repr(m_dot) + ", rho_tap=" + py_repr(rho_tap) + ", s=" + py_repr(s_mm) +
                             ", h=" + py_repr(h_mm) + ", eta=" + py_repr(eta) + ")");
    if (policy == "clamp")
        return { f_max, s_mm, f_calc, true };
    const double s_eff = 1000.0 * m_dot / (rho_tap * h_mm * eta * f_max);
    return { f_max, s_eff, f_calc, true };
}

// ------------------------------------------------------------------------------------------------
// UMexSequencer (v2 _UMexMixin)
// ------------------------------------------------------------------------------------------------

void UMexSequencer::warn(const std::string &msg)
{
    if (std::find(warnings.begin(), warnings.end(), msg) == warnings.end())
        warnings.push_back(msg);
}

void UMexSequencer::warn_agg(const std::string &key, std::function<std::string(int, const std::string &, double, double)> fmt, int layer, double val)
{
    auto it = std::find_if(m_agg.begin(), m_agg.end(), [&key](const auto &kv) { return kv.first == key; });
    if (it == m_agg.end()) {   // setdefault: the first format is kept
        m_agg.emplace_back(key, AggRec{ std::move(fmt), {}, {} });
        it = std::prev(m_agg.end());
    }
    it->second.layers.push_back(layer);
    it->second.vals.push_back(val);
}

void UMexSequencer::flush_agg()
{
    for (const auto &[key, rec] : m_agg) {
        const auto [lmin, lmax] = std::minmax_element(rec.layers.begin(), rec.layers.end());
        const auto [vmin, vmax] = std::minmax_element(rec.vals.begin(), rec.vals.end());
        warn(rec.fmt(int(rec.layers.size()), std::to_string(*lmin) + "-" + std::to_string(*lmax), *vmin, *vmax));
    }
    m_agg.clear();
}

std::optional<std::string> UMexSequencer::a_off_line() const
{
    if (speed_mode())
        return std::string("G1 A0.000\n");
    return std::nullopt;
}

double UMexSequencer::z_feed() const
{
    if (P.U.z_feed) return *P.U.z_feed;
    if (m_file_z_feed) return *m_file_z_feed;
    return P.U.travel_feed;
}

void UMexSequencer::apply_limits(std::optional<double> &x, std::optional<double> &y, std::optional<double> &z, bool clamp, const std::string &what)
{
    // Checks (and optionally clamps) a point against the configured soft limits. limits: x_min, x_max,
    // y_min, y_max, z_min, z_max. None of them is configurable in the fork yet, so this never fires.
    std::optional<double> *vals[3] = { &x, &y, &z };
    for (int i = 0; i < 3; ++i) {
        std::optional<double> &v = *vals[i];
        if (! v)
            continue;
        const std::optional<double> &lo = P.U.limits[2 * i], &hi = P.U.limits[2 * i + 1];
        double exc, nv;
        if (lo && *v < *lo) {
            exc = *lo - *v;
            nv  = *lo;
        } else if (hi && *v > *hi) {
            exc = *v - *hi;
            nv  = *hi;
        } else
            continue;
        auto it = std::find_if(m_limit_hits.begin(), m_limit_hits.end(), [&what](const auto &kv) { return kv.first == what; });
        if (it == m_limit_hits.end()) {
            m_limit_hits.emplace_back(what, LimitHit{ 0., 0., clamp ? 1. : 0. });
            it = std::prev(m_limit_hits.end());
        }
        it->second.count += 1;
        it->second.max_excess = std::max(it->second.max_excess, exc);
        if (clamp)
            v = nv;
    }
}

void UMexSequencer::mv(Lines &out, const char *cmd, std::optional<double> x, std::optional<double> y, std::optional<double> z,
                       std::optional<double> f, bool a_off, const char *check, bool clamp)
{
    // Emits one directly generated motion line and updates the position trackers.
    if (check != nullptr)
        apply_limits(x, y, z, clamp, check);
    std::string line = cmd;
    if (x) { line += " X" + py_fixed(3, *x); cur_x = *x; }
    if (y) { line += " Y" + py_fixed(3, *y); cur_y = *y; }
    if (z) { line += " Z" + py_fixed(3, *z); cur_z = *z; }
    if (f) line += " F" + py_fixed(1, *f);
    if (a_off && speed_mode())
        line += " A0.000";
    out.push_back(line + "\n");
}

UMexSequencer::Lines UMexSequencer::process_lines_umex(const std::vector<std::string> &raw_lines, const std::vector<PlacedBlock> &blocks)
{
    const UMexParams &U = P.U;
    const std::vector<LineInfo> infos = scan_gcode(raw_lines);
    const std::vector<LayerChunk> chunks = split_layers(raw_lines, infos);
    for (const LineInfo &li : infos)
        if (li.is_motion && ! li.has_xy() && std::abs(li.z1 - li.z0) > 1e-9) {
            m_file_z_feed = li.feed;
            break;
        }
    const bool all_unset = std::all_of(U.limits.begin(), U.limits.end(), [](const auto &v) { return ! v; });
    const bool any_unset = std::any_of(U.limits.begin(), U.limits.end(), [](const auto &v) { return ! v; });
    if (all_unset)
        warn("machine soft limits (soft_limit_x/y/z_min/max) are NOT configured: powder, wiper and "
             "Z-hop moves are neither range-checked nor clamped.");
    else if (any_unset)
        warn("only some soft limits are configured; unset axes/sides are not checked.");
    if (chunks.empty())
        warn("no ;LAYER_CHANGE / ;LAYER: markers found: nothing converted.");
    if (! all_unset) {
        std::optional<double> tx = nozzle_target_x, ty = nozzle_target_y, tz;
        apply_limits(tx, ty, tz, true, "uv_target");
        nozzle_target_x = *tx;
        nozzle_target_y = *ty;
    }

    // Block -> layer. Architecture C: the slicer decides the core layers, so a layer is converted iff it
    // carries a valid block (v2 classify_layers is not ported). Placement, k and the fingerprint are checked
    // here for every block before anything is emitted; z / h / F / s / fu when the layer is reached.
    const uint32_t fp = plan_fingerprint(U);
    std::vector<const PlacedBlock *> by_chunk(chunks.size(), nullptr);
    for (const PlacedBlock &b : blocks) {
        const PowderLayerRec &r = b.rec;
        const std::string where = "u-MEX powder records: the ;DIW: block k=" + std::to_string(r.k) + " (line " +
                                  std::to_string(r.begin_line) + " of the G-code)";
        // A block at the boundary of two layers precedes the next layer's start marker, so it is in the earlier one.
        size_t ci = 0;
        while (ci < chunks.size() && ! (chunks[ci].start < b.pos && b.pos <= chunks[ci].end)) ++ci;
        if (ci == chunks.size())
            throw DIWConfigError(where + " is not inside a printed layer (it must follow a ;LAYER_CHANGE marker and "
                                         "come before the config block)");
        const LayerChunk &c = chunks[ci];
        if (r.k != c.index)
            throw DIWConfigError(where + " sits in printed layer " + std::to_string(c.index) + "; k must equal the number of the "
                                         "layer it is placed in");
        for (size_t i = c.start; i < c.end && i < b.pos; ++i)
            if (infos[i].is_motion)
                throw DIWConfigError(where + " comes after the first motion line of its layer; it must sit between the layer's "
                                             "start marker and its first motion line");
        if (r.fp != fp)
            throw DIWConfigError(where + " has fingerprint " + hex8(r.fp) + " but the current settings give " + hex8(fp) +
                                 "; the plan was made with different settings, so it is rejected");
        if (c.has_arc || c.has_relxyz)
            throw DIWConfigError(where + " is in a layer with arcs (G2/G3) or relative XYZ (G91), which is never converted");
        if (c.index == int(chunks.size()))
            throw DIWConfigError(where + " is in the final printed layer, which is never converted");
        by_chunk[ci] = &b;
    }

    Lines out;
    auto feed = [&](size_t i) {
        const std::string &line = raw_lines[i];
        if (line.rfind("; Post-Processed by DIW Post-Processor", 0) == 0)
            return;
        line_idx = long(i);
        const Lines l = process_line(line);
        out.insert(out.end(), l.begin(), l.end());
    };

    const size_t first_start = chunks.empty() ? raw_lines.size() : chunks.front().start;
    for (size_t i = 0; i < first_start; ++i)
        feed(i);
    double prev_z = 0.0;
    for (size_t ci = 0; ci < chunks.size(); ++ci) {
        const LayerChunk &c = chunks[ci];
        bool converted = false;
        if (by_chunk[ci] != nullptr) {
            // v2: last_in_stack = not (next chunk is core); here "core" = "carries a block".
            const bool last_in_stack = ! (ci + 1 < chunks.size() && by_chunk[ci + 1] != nullptr);
            if (std::optional<Lines> res = convert_core_layer(raw_lines, infos, c, prev_z, last_in_stack, *by_chunk[ci])) {
                out.insert(out.end(), res->begin(), res->end());
                converted = true;
            }
        }
        if (! converted)
            for (size_t i = c.start; i < c.end; ++i)
                feed(i);
        if (c.z)
            prev_z = *c.z;
    }
    const size_t last_end = chunks.empty() ? first_start : chunks.back().end;
    for (size_t i = last_end; i < raw_lines.size(); ++i)
        feed(i);
    flush_agg();
    for (const auto &[what, rec] : m_limit_hits) {
        const char *verb = rec.clamped != 0. ? "clamped" : "outside limits (not clamped)";
        warn("soft limit: " + std::to_string((long long) rec.count) + " " + what + " coordinate(s) " + verb + "; max excess " +
             py_fixed(3, rec.max_excess) + " mm. A clamped powder path no longer matches the slicer geometry.");
    }
    return out;
}

std::optional<UMexSequencer::Lines> UMexSequencer::convert_core_layer(const std::vector<std::string> &raw_lines,
                                                                       const std::vector<LineInfo> &infos, const LayerChunk &chunk,
                                                                       double prev_z, bool last_in_stack, const PlacedBlock &block)
{
    // Re-emits one core layer in the 6-stage order; returns nullopt to leave it untouched.
    const UMexParams     &U   = P.U;
    const PowderLayerRec &rec = block.rec;
    const int k = chunk.index;
    const std::string where = "u-MEX powder records: the ;DIW: block k=" + std::to_string(rec.k) + " (line " +
                              std::to_string(rec.begin_line) + " of the G-code)";
    const LayerModel model = parse_core_layer(raw_lines, infos, chunk, U.width_fallback);
    if (! model.z)   // v2 returns None here (no extrusion in the layer), so it never plans such a layer
        throw DIWConfigError(where + " is in a layer without extrusion moves");
    const double z = *model.z;
    const double h = truthy(chunk.height) ? *chunk.height : z - prev_z;
    if (rec.z != z || rec.h != h)
        throw DIWConfigError(where + " was planned for z=" + py_repr(rec.z) + " h=" + py_repr(rec.h) + ", but the layer has z=" +
                             py_repr(z) + " h=" + py_repr(h));
    if (h <= 0.0) {
        warn("layer " + std::to_string(k) + ": non-positive layer height; left as paste.");
        return std::nullopt;
    }
    const PowderFeed pf = resolve_powder_feed(U.m_dot, U.rho_tap, U.s_hatch, h, U.eta, U.f_max, U.policy);
    if (pf.overflow) {
        if (U.policy == "clamp")
            warn("F_powder = " + py_fixed(1, pf.f_calc) + " mm/min exceeds powder_f_max = " + py_fixed(1, U.f_max) + ": clamped; "
                 "powder is over-delivered by " + py_fixed(1, 100.0 * (pf.f_calc / U.f_max - 1.0)) + " percent at "
                 "hatch spacing " + py_repr(U.s_hatch) + " mm (layer height " + py_fixed(3, h) + " mm).");
        else
            warn("F_powder = " + py_fixed(1, pf.f_calc) + " mm/min exceeds powder_f_max = " + py_fixed(1, U.f_max) + ": feed clamped and "
                 "hatch spacing increased " + py_repr(U.s_hatch) + " -> " + py_fixed(4, pf.s_eff) + " mm to keep the delivered mass exact "
                 "(layer height " + py_fixed(3, h) + " mm). Lowering the motor voltage is not possible: m_dot is "
                 "specified at the 3.65 V minimum.");
    }
    if (rec.F != pf.F || rec.s != pf.s_eff || rec.fu != pf.f_calc)
        throw DIWConfigError(where + " has F=" + py_repr(rec.F) + " s=" + py_repr(rec.s) + " fu=" + py_repr(rec.fu) +
                             ", but the current settings give F=" + py_repr(pf.F) + " s=" + py_repr(pf.s_eff) + " fu=" +
                             py_repr(pf.f_calc) + " for layer height " + py_repr(h) + "; the block is rejected");
    const double F = pf.F;
    if (rec.status == PowderLayerRec::Status::open) {
        warn("layer " + std::to_string(k) + ": wall loops could not be closed into a cavity; layer left as paste.");
        return std::nullopt;
    }
    if (rec.status == PowderLayerRec::Status::small) {
        warn_agg("revert_all", [](int n, const std::string &layers, double, double) {
            return std::to_string(n) + " core layer(s) (" + layers + ") have no cavity region of inscribed radius >= "
                   "shell_powder_clearance; the slicer's paste infill is kept unchanged there.";
        }, k);
        return std::nullopt;   // cavity too small everywhere: whole layer reverts to the slicer's paste infill
    }

    Lines out;
    auto run = [&](size_t i) {
        line_idx = long(i);
        const Lines l = process_line(raw_lines[i]);
        out.insert(out.end(), l.begin(), l.end());
    };
    const std::optional<std::string> a_line = a_off_line();
    // ---- Stage 1: paste walls (and any other paste feature) -------------------------
    const std::string s1 = "; --- uMEX L" + std::to_string(k) + " S1 paste contour z=" + py_fixed(3, z) + " ---\n";
    for (size_t n_hdr = 0; n_hdr < model.header.size(); ++n_hdr) {
        run(model.header[n_hdr]);
        if (n_hdr == 0)   // after the layer marker, so a legacy inter-layer UV macro of layer k-1 stays above it
            out.push_back(s1);
    }
    if (model.header.empty())
        out.push_back(s1);
    std::vector<PathRec> paste_paths;
    for (const PathRec &p : model.paths)
        if (p.kind == "wall" || p.kind == "paste")
            paste_paths.push_back(p);
    for (const FallbackPathRec &f : rec.fallbacks) {   // v2 _synthetic_path: raw text regenerated below
        PathRec p;
        p.ptype       = f.ptype;
        p.kind        = "paste";
        p.points      = f.pts;
        p.seg_de      = f.de;
        p.seg_feed    = f.feed;
        p.width       = f.width;
        p.z           = f.z;
        p.travel_feed = f.travel_feed;
        paste_paths.push_back(std::move(p));
    }
    for (const PathRec &p : paste_paths) {
        for (size_t i : p.pre_lines)
            run(i);
        paste_travel(p.points[0].first, p.points[0].second, z, truthy(p.travel_feed) ? *p.travel_feed : U.travel_feed, model.z_feed, out);
        for (size_t j = 0; j < p.seg_de.size(); ++j) {
            const double de = p.seg_de[j], fd = p.seg_feed[j];
            if (! p.seg_raw.empty())
                feed_paste_line(p.seg_raw[j], de, fd, out);
            else
                feed_paste_line("G1 X" + py_fixed(3, p.points[j + 1].first) + " Y" + py_fixed(3, p.points[j + 1].second) + " E1 F" +
                                    py_fixed(1, fd),
                                de, fd, out);
        }
    }
    for (size_t i : model.tail)
        run(i);
    const double dwell = P.post_dwell > 0.0 ? P.post_dwell : U.stage1_dwell;
    out.push_back("; --- uMEX L" + std::to_string(k) + " S1 end: auger stop, pressure-relief dwell ---\n");
    out.push_back(a_line ? *a_line : std::string("; (A held: cumulative position mode, no A0.000)\n"));
    out.push_back("G04 P" + py_fixed(3, dwell) + "\n");
    is_extruding = false;
    // ---- Stage 2: nozzle wipe -------------------------------------------------------
    const double z_safe = std::max(cur_z + U.wiper_hop, U.wiper_zmin);
    const double zf     = z_feed();
    out.push_back("; --- uMEX L" + std::to_string(k) + " S2 nozzle wipe ---\n");
    mv(out, "G0", std::nullopt, std::nullopt, z_safe, zf, true, "wiper", false);
    mv(out, "G0", U.wiper_x0, U.wiper_y, std::nullopt, U.travel_feed, true, "wiper", false);
    mv(out, "G0", std::nullopt, std::nullopt, U.wiper_z, zf, true, "wiper", false);
    mv(out, "G1", U.wiper_x1, std::nullopt, std::nullopt, U.wiper_feed, true, "wiper", false);
    mv(out, "G0", std::nullopt, std::nullopt, z_safe, zf, true, "wiper", false);
    // ---- Stage 3: UV curing (existing as-built macro, UV centre + offset) -----------
    out.push_back("; --- uMEX L" + std::to_string(k) + " S3 UV cure at bounding-box centre + offset ---\n");
    if (P.enable_uv && P.uv_layer_dwell > 0) {
        Lines seq = curing_sequence(cur_x, cur_y);
        seq.pop_back();                                   // drop the macro's return move
        const auto xy = std::find_if(seq.begin(), seq.end(), [](const std::string &l) { return l.rfind("G0 X", 0) == 0; });
        seq.insert(xy + 1, "G0 Z" + py_fixed(3, z) + " F" + py_fixed(1, zf) + "\n");   // cure at layer height, as the as-built macro does
        out.insert(out.end(), seq.begin(), seq.end());
        cur_x = nozzle_target_x;
        cur_y = nozzle_target_y;
        cur_z = z;
    } else
        out.emplace_back("; (UV disabled)\n");
    extruded_on_layer      = false;
    cured_for_layer_change = true;
    ++layer_count;
    // ---- Stage 4: powder ------------------------------------------------------------
    emit_powder(rec, z, F, k, out);
    // ---- Stage 5: compaction --------------------------------------------------------
    ++m_converted_layers;
    const bool compact = last_in_stack || (m_converted_layers % U.comp_interval == 0);
    if (compact) {
        out.push_back("; --- uMEX L" + std::to_string(k) + " S5 vibration compaction ---\n");
        mv(out, "G0", std::nullopt, std::nullopt, cur_z + U.comp_hop, zf, true, "powder");
        out.push_back("M62 P" + std::to_string(U.comp_pin) + " Q1\n");
        out.push_back("G04 P" + py_fixed(3, U.comp_t) + "\n");
        out.push_back("M62 P" + std::to_string(U.comp_pin) + " Q0\n");
        out.push_back("G04 P" + py_fixed(3, U.comp_settle) + "\n");
    }
    out.push_back("; --- uMEX L" + std::to_string(k) + " S6 layer advance (next layer sets Z) ---\n");
    // ---- state hand-back to the legacy line processor -------------------------------
    const LineInfo &last = infos[chunk.end - 1];
    last_e     = last.e_after;
    relative_e = last.rel_after;
    cur_f      = last.feed;
    return out;
}

void UMexSequencer::paste_travel(double x, double y, double z, double feed, std::optional<double> z_feed_layer, Lines &out)
{
    // Regenerated travel to a kept paste path (through the paste translator, so the usual A0.000 / dwell on
    // the extrusion -> travel transition applies).
    const double zf = truthy(z_feed_layer) ? *z_feed_layer : z_feed();
    const bool   up = z > cur_z + 1e-9;
    auto run = [&](const std::string &l) {
        const Lines r = process_line(l);
        out.insert(out.end(), r.begin(), r.end());
    };
    if (up)
        run("G0 Z" + py_fixed(3, z) + " F" + py_fixed(1, zf));
    if (py_hypot(x - cur_x, y - cur_y) > 1e-6)
        run("G0 X" + py_fixed(3, x) + " Y" + py_fixed(3, y) + " F" + py_fixed(1, feed));
    if (! up && std::abs(z - cur_z) > 1e-9)
        run("G0 Z" + py_fixed(3, z) + " F" + py_fixed(1, zf));
}

void UMexSequencer::feed_paste_line(const std::string &raw, double de, double feed, Lines &out)
{
    // Feeds one extruding move to the paste translator with its resolved RELATIVE dE.
    std::string s = raw;
    while (! s.empty() && (s.back() == '\r' || s.back() == '\n'))
        s.pop_back();
    const size_t ci = s.find(';');
    std::string code = ci != std::string::npos ? s.substr(0, ci) : s;
    const std::string comment = ci != std::string::npos ? s.substr(ci) : std::string();
    code = replace_words(code, "E", "E" + py_fixed(6, de));
    if (! find_word(code, "F"))
        code = rstrip(code) + " F" + py_fixed(1, feed);
    const std::string line = rstrip(code) + (comment.empty() ? std::string() : " " + comment);
    const bool saved = relative_e;
    relative_e = true;
    Lines r;
    try {
        r = process_line(line);
    } catch (...) {
        relative_e = saved;
        throw;
    }
    relative_e = saved;
    out.insert(out.end(), r.begin(), r.end());
}

// ---- Stage 4: powder (v2 "powder stage" section) -------------------------------------------------

std::pair<double, double> UMexSequencer::powder_xy(const RecPoint &p) const
{
    return { p.first - P.U.off_x, p.second - P.U.off_y };
}

double UMexSequencer::z_disp(double z_layer) const
{
    // Machine Z that puts the powder TIP at the layer top (+ clearance).
    return z_layer + P.U.z_clear - P.U.off_z;
}

double UMexSequencer::z_safe(double z_layer) const
{
    // Machine Z that keeps the LOWEST of the two tips travel_hop above the layer top.
    const double m = P.U.off_z < 0.0 ? P.U.off_z : 0.0;   // Python min(0.0, off_z)
    return z_layer + P.U.travel_hop - m;
}

void UMexSequencer::powder_transit(const RecPoint &pt, double z_layer, Lines &out)
{
    const UMexParams &U = P.U;
    const double zs = z_safe(z_layer), zd = z_disp(z_layer), zf = z_feed();
    const auto [mx, my] = powder_xy(pt);
    if (cur_z < zs - 1e-9)
        mv(out, "G0", std::nullopt, std::nullopt, zs, zf, true, "powder");
    mv(out, "G0", mx, my, std::nullopt, U.travel_feed, true, "powder");
    mv(out, "G1", std::nullopt, std::nullopt, zd, zf, true, "powder");
}

void UMexSequencer::pmove(Lines &out, const RecPoint &p, double F)
{
    const auto [mx, my] = powder_xy(p);
    mv(out, "G1", mx, my, std::nullopt, F, true, "powder");
}

void UMexSequencer::emit_powder(const PowderLayerRec &plan, double z, double F, int k, Lines &out)
{
    // Stage 4: feeder on + lag, traverse at F_powder with A0.000 and B untouched, early stop.
    const UMexParams &U = P.U;
    const double d_early = F / 60.0 * U.lag_off;
    const std::string header = "; --- uMEX L" + std::to_string(k) + " S4 powder infill F=" + py_fixed(1, F) +
                               " d_early=" + py_fixed(3, d_early) + " ---\n";
    out.push_back(header);
    if (m_stub_powder_stage)   // test stub (DIWTranslateOptions::stub_powder_stage): the header line only, no state change
        return;
    const std::string on  = "M62 P" + std::to_string(U.powder_pin) + " Q1\n";
    const std::string off = "M62 P" + std::to_string(U.powder_pin) + " Q0\n";
    const std::string per_cycle = py_fixed(4, U.m_dot * U.lag_off / 60.0);
    for (const PowderRunRec &run : plan.runs) {
        powder_transit(run.pts.front(), z, out);
        if (run.point) {
            const double rest  = run.t_feed - U.lag_off;
            const double dwell = U.lag_on + (0.0 > rest ? 0.0 : rest);   // max(t_feed - lag_off, 0.0)
            if (run.t_feed < U.lag_off) {
                const std::string tail = "s, shorter than powder_lag_off = " + py_repr(U.lag_off) +
                                         " s; the feeder cannot meter less than m_dot*lag_off = " + per_cycle +
                                         " g per cycle (over-delivery).";
                warn_agg("pt_quantum", [tail](int n, const std::string &layers, double vmin, double vmax) {
                    return std::to_string(n) + " point-dwell(s) in layers " + layers + " need a feed time of " + py_fixed(3, vmin) +
                           "-" + py_fixed(3, vmax) + " " + tail;
                }, k, run.t_feed);
            }
            out.emplace_back("; point dwell\n");
            out.push_back(on);
            out.push_back("G04 P" + py_fixed(3, dwell) + "\n");
            out.push_back(off);
            continue;
        }
        const std::vector<RecPoint> &pts = run.pts;
        PySum length;   // v2 PowderRun.length: sum(math.hypot(...)) over the segments
        for (size_t i = 0; i + 1 < pts.size(); ++i)
            length.add(py_hypot(pts[i + 1].first - pts[i].first, pts[i + 1].second - pts[i].second));
        const double total  = length.result();
        const double stop_s = total - d_early;
        out.push_back(on);
        out.push_back("G04 P" + py_fixed(3, U.lag_on) + "\n");
        bool stopped = false;
        if (stop_s <= 1e-9) {
            out.push_back(off);
            stopped = true;
            if (total < d_early - 1e-9) {
                const std::string tail = "mm) are shorter than the early-stop distance " + py_fixed(2, d_early) +
                                         " mm; the feeder is stopped at the run start and each still delivers m_dot*lag_off = " +
                                         per_cycle + " g (over-delivery).";
                warn_agg("short_run", [tail](int n, const std::string &layers, double vmin, double vmax) {
                    return std::to_string(n) + " powder run(s) in layers " + layers + " (" + py_fixed(2, vmin) + "-" +
                           py_fixed(2, vmax) + " " + tail;
                }, k, total);
            }
        }
        double cum = 0.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            const RecPoint &a = pts[i], &b = pts[i + 1];
            const double seg = py_hypot(b.first - a.first, b.second - a.second);
            if (seg < 1e-9)
                continue;
            if (! stopped && cum + seg >= stop_s - 1e-9) {
                const double t = (stop_s - cum) / seg;
                if (t >= 1.0 - 1e-9) {
                    pmove(out, b, F);
                    out.push_back(off);
                } else if (t <= 1e-9) {
                    out.push_back(off);
                    pmove(out, b, F);
                } else {
                    const RecPoint q{ a.first + t * (b.first - a.first), a.second + t * (b.second - a.second) };
                    pmove(out, q, F);
                    out.push_back(off);
                    pmove(out, b, F);
                }
                stopped = true;
            } else
                pmove(out, b, F);
            cum += seg;
        }
        if (! stopped)
            out.push_back(off);
    }
}

} // namespace DIW
} // namespace Slic3r
