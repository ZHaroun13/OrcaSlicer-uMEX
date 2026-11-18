#ifdef _MSC_VER
#pragma fp_contract(off)
#endif
#include "DIWPowderShadow.hpp"

#include "DIWPowderBlock.hpp"
#include "DIWPyCompat.hpp"
#include "DIWTranslator.hpp"
#include "../ExtrusionEntityCollection.hpp"
#include "../GCodeWriter.hpp"   // GCodeFormatter::axis_units
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../PrintBase.hpp"   // CanceledException

#include <boost/filesystem.hpp>
#include <boost/nowide/cstdlib.hpp>
#include <boost/nowide/fstream.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <charconv>
#include <cstdio>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

// Binder of the native powder planner (diw/PLANNER_DESIGN.md section 6). Every decision that the translator
// re-checks (k, placement, z, h, s, F, fu, fp, final layer, arcs / G91) is taken here from the text with the
// translator's own code (GCodeScanner, ChunkTracker, parse_core_layer, resolve_powder_feed, plan_fingerprint),
// so the two cannot disagree.

namespace Slic3r {
namespace DIW {

namespace {

const std::set<std::string> SPARSE_T = { "sparse infill", "internal infill" };
const std::set<std::string> SKIN_T   = { "top surface", "bottom surface", "top solid infill", "bottom solid infill" };

// Adds the lifetime of the object to *acc in ms (the T10 counters of the summary line); with a mutex, under it.
struct ScopedMs
{
    double                               *acc;
    std::mutex                           *mtx;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    ~ScopedMs()
    {
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (mtx) {
            std::lock_guard<std::mutex> lock(*mtx);
            *acc += ms;
        } else
            *acc += ms;
    }
};

bool sv_starts_with(std::string_view s, std::string_view p) { return s.size() >= p.size() && s.compare(0, p.size(), p) == 0; }

std::string getenv_str(const char *name)
{
    const char *v = boost::nowide::getenv(name);
    return v ? std::string(v) : std::string();
}

// ---- JSON helpers of the dump (mm values as Python repr strings, so they round-trip exactly) ----

double jnum(const nlohmann::json &j)
{
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        double            v = 0.;
        const auto        r = std::from_chars(s.data(), s.data() + s.size(), v, std::chars_format::general);
        if (r.ec != std::errc() || r.ptr != s.data() + s.size())
            throw std::runtime_error("bad number in shadow dump: " + s);
        return v;
    }
    return j.get<double>();
}

nlohmann::json jpts_nm(const Points &pts)
{
    nlohmann::json a = nlohmann::json::array();
    for (const Point &p : pts) a.push_back({ (long long) p.x(), (long long) p.y() });
    return a;
}

Points pts_nm(const nlohmann::json &j)
{
    Points out;
    for (const auto &p : j) out.emplace_back(coord_t(p.at(0).get<long long>()), coord_t(p.at(1).get<long long>()));
    return out;
}

nlohmann::json jpts_mm(const std::vector<Vec2d> &pts)
{
    nlohmann::json a = nlohmann::json::array();
    for (const Vec2d &p : pts) a.push_back({ py_repr(p.x()), py_repr(p.y()) });
    return a;
}

std::vector<Vec2d> pts_mm(const nlohmann::json &j)
{
    std::vector<Vec2d> out;
    for (const auto &p : j) out.emplace_back(jnum(p.at(0)), jnum(p.at(1)));
    return out;
}

nlohmann::json jtext(const std::vector<TextPath> &tt)
{
    nlohmann::json a = nlohmann::json::array();
    for (const TextPath &t : tt) {
        nlohmann::json de = nlohmann::json::array(), fe = nlohmann::json::array();
        for (double v : t.seg_de) de.push_back(py_repr(v));
        for (double v : t.seg_feed) fe.push_back(py_repr(v));
        nlohmann::json sw = nlohmann::json::array();
        for (double v : t.seg_width) sw.push_back(py_repr(v));
        a.push_back({ { "ptype", t.ptype }, { "kind", t.kind }, { "pts", jpts_mm(t.pts_mm) }, { "seg_de", de }, { "seg_feed", fe },
                      { "seg_width", sw },
                      { "width", py_repr(t.width) }, { "z", py_repr(t.z) },
                      { "travel_feed", t.travel_feed ? nlohmann::json(py_repr(*t.travel_feed)) : nlohmann::json() } });
    }
    return a;
}

std::string wkt(const ExPolygons &ee)
{
    if (ee.empty())
        return "MULTIPOLYGON EMPTY";
    auto ring = [](const Points &pts) {
        std::string s = "(";
        for (size_t i = 0; i <= pts.size(); ++i) {
            const Point &p = pts[i % pts.size()];
            if (i) s += ", ";
            s += py_repr(double(p.x()) * 1e-6) + " " + py_repr(double(p.y()) * 1e-6);
        }
        return s + ")";
    };
    std::string s = "MULTIPOLYGON (";
    for (size_t k = 0; k < ee.size(); ++k) {
        if (k) s += ", ";
        s += "(" + ring(ee[k].contour.points);
        for (const Polygon &h : ee[k].holes) s += ", " + ring(h.points);
        s += ")";
    }
    return s + ")";
}

// ---- frame self-check (gate 9): a uniform grid over segments ----

struct SegGrid
{
    struct Seg { Vec2d a, b; int tag; };
    double                                              cell = 1.0;   // mm
    double                                              pad  = 0.;
    std::vector<Seg>                                    segs;
    std::unordered_map<long long, std::vector<int>>     cells;

    static long long key(long long i, long long j) { return (i << 32) ^ (j & 0xffffffffLL); }
    void add(const Vec2d &a, const Vec2d &b, int tag)
    {
        const int id = int(segs.size());
        segs.push_back({ a, b, tag });
        const long long i0 = (long long) std::floor((std::min(a.x(), b.x()) - pad) / cell);
        const long long i1 = (long long) std::floor((std::max(a.x(), b.x()) + pad) / cell);
        const long long j0 = (long long) std::floor((std::min(a.y(), b.y()) - pad) / cell);
        const long long j1 = (long long) std::floor((std::max(a.y(), b.y()) + pad) / cell);
        for (long long i = i0; i <= i1; ++i)
            for (long long j = j0; j <= j1; ++j) cells[key(i, j)].push_back(id);
    }
    static double dist(const Vec2d &p, const Seg &s)
    {
        const Vec2d  ab = s.b - s.a;
        const double l2 = ab.squaredNorm();
        double       t  = l2 > 0. ? (p - s.a).dot(ab) / l2 : 0.;
        t               = std::max(0., std::min(1., t));
        return (p - (s.a + t * ab)).norm();
    }
    // Calls fn(tag) for every segment within tol of p.
    template<class Fn> void near(const Vec2d &p, double tol, Fn &&fn) const
    {
        const auto it = cells.find(key((long long) std::floor(p.x() / cell), (long long) std::floor(p.y() / cell)));
        if (it == cells.end())
            return;
        for (int id : it->second)
            if (dist(p, segs[id]) <= tol) fn(segs[id].tag);
    }
};

Vec2d nm_to_mm(const Point &p) { return Vec2d(double(p.x()) * 1e-6, double(p.y()) * 1e-6); }

// Empty when the frame check passes, else the reason.
std::string frame_check(const NativeLayer &nat, const std::vector<TextPath> &walls, double tol)
{
    // (a) every text wall vertex near a native wall piece
    SegGrid g;
    g.pad = tol;
    for (const WallPiece &w : nat.pieces) {
        const Points &p = w.pts.points;
        for (size_t i = 0; i + 1 < p.size(); ++i) g.add(nm_to_mm(p[i]), nm_to_mm(p[i + 1]), 0);
        if (w.closed && p.size() > 2) g.add(nm_to_mm(p.back()), nm_to_mm(p.front()), 0);
        if (p.size() == 1) g.add(nm_to_mm(p[0]), nm_to_mm(p[0]), 0);
    }
    for (const TextPath &t : walls)
        for (const Vec2d &v : t.pts_mm) {
            bool hit = false;
            g.near(v, tol, [&hit](int) { hit = true; });
            if (! hit)
                return "text wall vertex (" + py_repr(v.x()) + ", " + py_repr(v.y()) + ") is not on a captured wall";
        }
    // (b) every native ring touched by a text wall vertex
    SegGrid r;
    r.pad   = tol;
    int tag = 0;
    for (const Polygons &inst : nat.rings)
        for (const Polygon &poly : inst) {
            const Points &p = poly.points;
            for (size_t i = 0; i < p.size(); ++i) r.add(nm_to_mm(p[i]), nm_to_mm(p[(i + 1) % p.size()]), tag);
            ++tag;
        }
    std::vector<char> seen(size_t(tag), 0);
    for (const TextPath &t : walls)
        for (const Vec2d &v : t.pts_mm) r.near(v, tol, [&seen](int k) { seen[size_t(k)] = 1; });
    for (int k = 0; k < tag; ++k)
        if (! seen[size_t(k)])
            return "captured ring " + std::to_string(k) + " has no printed wall vertex";
    return std::string();
}

std::vector<TextPath> text_paths(const LayerModel &model, const char *kind, size_t min_pts)
{
    std::vector<TextPath> out;
    for (const PathRec &p : model.paths) {
        if (p.kind != kind || p.points.size() < min_pts)
            continue;
        TextPath t;
        t.ptype = p.ptype;
        t.kind  = p.kind;
        for (const auto &[x, y] : p.points) {
            t.pts_mm.emplace_back(x, y);
            t.pts_nm.points.push_back(to_grid(Vec2d(x, y)));
        }
        t.seg_de      = p.seg_de;
        t.seg_feed    = p.seg_feed;
        t.seg_width   = p.seg_width;
        t.width       = p.width;
        t.z           = p.z;
        t.travel_feed = p.travel_feed;
        out.push_back(std::move(t));
    }
    return out;
}

const char *reason_label(const std::string &r)
{
    if (r == "capture") return "capture failed";
    if (r == "disagree_text_only") return "the G-code text makes them core layers but the native classification does not";
    if (r == "disagree_native_only") return "the native classification makes them core layers but the G-code text does not";
    if (r == "merged_multi_core") return "more than one native core layer merged into one printed layer";
    if (r == "lost") return "their native capture was lost";
    if (r == "frame") return "frame self-check failed";
    if (r == "plan") return "planning failed";
    if (r == "record") return "record self-check failed";
    if (r == "sync") return "G-code was flushed while the layer was open";
    if (r == "no_z") return "no extrusion height";
    return nullptr;
}

} // namespace

// ------------------------------------------------------------------------------------------------ placeholders

bool parse_placeholder(std::string_view line, const std::string &token, uint64_t &id, bool &m)
{
    // ";DIWP " + token + " " + digits + " m=0|1": only the lines this shadow wrote (review fix: user text that
    // looks like a placeholder is never consumed or dropped)
    if (token.empty() || ! sv_starts_with(line, ";DIWP ") || line.size() < 7 + token.size() ||
        line.substr(6, token.size()) != token || line[6 + token.size()] != ' ')
        return false;
    while (! line.empty() && (line.back() == '\n' || line.back() == '\r')) line.remove_suffix(1);
    size_t i = 7 + token.size(), d0 = i;
    uint64_t v = 0;
    while (i < line.size() && line[i] >= '0' && line[i] <= '9') v = v * 10 + uint64_t(line[i++] - '0');
    if (i == d0 || i - d0 > 18)
        return false;
    if (line.substr(i) == " m=0")
        m = false;
    else if (line.substr(i) == " m=1")
        m = true;
    else
        return false;
    id = v;
    return true;
}

bool bead_bounds_slice(const std::vector<BeadPath> &paths, const ExPolygons &slices)
{
    if (slices.empty())
        return true;
    size_t nseg = 0;
    for (const BeadPath &b : paths)
        if (b.pts != nullptr && b.pts->size() >= 2)
            nseg += b.pts->size() - 1;
    if (nseg == 0)
        return true;
    std::vector<BoundingBox> bbs;
    for (const ExPolygon &e : slices) bbs.push_back(get_extents(e.contour));
    auto inside = [&](const Vec2d &q) {
        const Point p(coord_t(std::llround(q.x())), coord_t(std::llround(q.y())));
        for (size_t i = 0; i < slices.size(); ++i)
            if (bbs[i].contains(p) && slices[i].contains(p, false))
                return true;
        return false;
    };
    const size_t stride = std::max<size_t>(1, nseg / 64);
    size_t       seg = 0, n = 0, one_side = 0;
    for (const BeadPath &b : paths) {
        if (b.pts == nullptr)
            continue;
        for (size_t i = 0; i + 1 < b.pts->size(); ++i, ++seg) {
            if (seg % stride != 0)
                continue;
            const Vec2d a = (*b.pts)[i].cast<double>(), c = (*b.pts)[i + 1].cast<double>();
            const Vec2d d = c - a;
            const double len = d.norm();
            if (len <= 0.)
                continue;
            const Vec2d m = 0.5 * (a + c), nrm(-d.y() / len, d.x() / len);
            const double r = 0.75 * b.width;
            ++n;
            if (inside(m + r * nrm) != inside(m - r * nrm))
                ++one_side;
        }
    }
    return n == 0 || 4 * one_side >= n;
}

bool is_placeholder_line(std::string_view line, const std::string &token)
{
    uint64_t id = 0;
    bool     m  = false;
    return parse_placeholder(line, token, id, m);
}

std::string PowderShadow::placeholder_line(const std::string &token, uint64_t id, bool m)
{
    return ";DIWP " + token + " " + std::to_string(id) + (m ? " m=1\n" : " m=0\n");
}

namespace {
std::string token_or_random(const std::string &t)
{
    if (! t.empty())
        return t;
    // 64 random bits (random_device mixed with the clock), 16 hex digits
    std::random_device rd;
    uint64_t v = (uint64_t(rd()) << 32) ^ uint64_t(rd()) ^
                 uint64_t(std::chrono::high_resolution_clock::now().time_since_epoch().count()) * 0x9E3779B97F4A7C15ull;
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) v);
    return buf;
}
} // namespace

ShadowOptions ShadowOptions::from_env()
{
    ShadowOptions o;
    o.keep_placeholders = getenv_str("ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS") == "1";
    o.dump_dir          = getenv_str("ORCA_DIW_SHADOW_DUMP");
    o.fault             = getenv_str("ORCA_DIW_SHADOW_FAULT");
    return o;
}

// ------------------------------------------------------------------------------------------------ PowderShadow

PowderShadow::PowderShadow(const TranslatorParams &p, ShadowOptions opts)
    : P(p), m_opts(std::move(opts)), m_token(token_or_random(m_opts.placeholder_token))
{
    if (! P.umex)
        throw std::invalid_argument("PowderShadow: not a u-MEX powder job");
    m_fp = plan_fingerprint(P.U);
}

PowderShadow::PowderShadow(std::unique_ptr<TranslatorParams> p, ShadowOptions opts)
    : m_own_params(std::move(p)), P(*m_own_params), m_opts(std::move(opts)), m_token(token_or_random(m_opts.placeholder_token))
{
    if (! P.umex)
        throw std::invalid_argument("PowderShadow: not a u-MEX powder job");
    m_fp = plan_fingerprint(P.U);
}

// ------------------------------------------------------------------------------------------------ create (section 3)

std::shared_ptr<PowderShadow> PowderShadow::create(const Print &print, const DynamicPrintConfig &full, bool spiral_vase,
                                                   bool axis_remap, std::string *reason)
{
    std::string                   why;
    std::shared_ptr<PowderShadow> out;
    try {
        const auto *powder = full.option<ConfigOptionBool>("diw_powder_infill");
        if (! diw_translation_enabled(full) || powder == nullptr || ! powder->value)
            return nullptr;   // not a powder job: silent
        if (getenv_str("ORCA_DIW_NO_NATIVE_PLAN") == "1")
            why = "kill switch ORCA_DIW_NO_NATIVE_PLAN=1 is set";
        else if (print.is_BBL_printer())
            why = "BBL printer G-code (the layer markers are in the BBL dialect)";
        else if (spiral_vase)
            why = "spiral vase mode";
        else if (axis_remap)
            why = "G-code axis remap";
        else if (const auto *belt = full.option<ConfigOptionBool>("belt_printer"); belt != nullptr && belt->value)
            why = "belt printer";
        else if (const auto *eo = full.option<ConfigOptionPoints>("extruder_offset"); eo != nullptr) {
            for (const Vec2d &v : eo->values)
                if (v != eo->values.front()) {
                    why = "the extruders have different extruder_offset values";
                    break;
                }
        }
        if (why.empty()) {
            std::unique_ptr<TranslatorParams> tp;
            try {
                tp = std::make_unique<TranslatorParams>(diw_read_translator_params(full));
            } catch (const DIWConfigError &) {
                return nullptr;   // the translator reports the same error later
            }
            if (! tp->umex)
                return nullptr;
            ShadowOptions opts     = ShadowOptions::from_env();
            const auto   *hp       = full.option<ConfigOptionBool>("high_precision_gcode");
            opts.xy_decimals       = (hp != nullptr && hp->value) ? 4 : 3;
            opts.throw_if_canceled = [&print]() {
                if (print.canceled())
                    throw CanceledException();
            };
            out = std::make_shared<PowderShadow>(std::move(tp), std::move(opts));
        }
    } catch (const std::exception &ex) {
        why = std::string("internal error: ") + ex.what();
        out.reset();
    }
    if (reason != nullptr)
        *reason = why;
    return out;
}

// ------------------------------------------------------------------------------------------------ capture (section 5)

namespace {

bool is_wall_role(ExtrusionRole r) { return r == erPerimeter || r == erExternalPerimeter || r == erOverhangPerimeter; }
bool is_outer_role(ExtrusionRole r) { return r == erExternalPerimeter || r == erOverhangPerimeter; }

// Every leaf path with a non-empty polyline.
void for_each_leaf(const ExtrusionEntity &e, const std::function<void(const ExtrusionPath &)> &fn)
{
    if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(&e)) {
        for (const ExtrusionEntity *x : c->entities)
            if (x != nullptr)
                for_each_leaf(*x, fn);
    } else if (const auto *l = dynamic_cast<const ExtrusionLoop *>(&e)) {
        for (const ExtrusionPath &p : l->paths)
            if (! p.polyline.points.empty())
                fn(p);
    } else if (const auto *m = dynamic_cast<const ExtrusionMultiPath *>(&e)) {
        for (const ExtrusionPath &p : m->paths)
            if (! p.polyline.points.empty())
                fn(p);
    } else if (const auto *p = dynamic_cast<const ExtrusionPath *>(&e)) {
        if (! p->polyline.points.empty())
            fn(*p);
    }
}

// Walls (pieces), rings and open outer walls of one instance (section 5.2), on the text grid.
struct WallCollector
{
    const std::function<Point(const Point &)> &grid;
    int64_t                                    unit_nm;   // one unit of the last printed decimal, in nm
    NativeLayer                               &out;
    Polygons                                  &rings;
    const ExPolygons                          *slices = nullptr;   // the layer's lslices (object frame): bead_bounds_slice
    int                                       *single_bead = nullptr;   // walls refused as rings by bead_bounds_slice

    bool bounds(const std::vector<BeadPath> &paths) const
    {
        if (slices == nullptr || bead_bounds_slice(paths, *slices))
            return true;
        if (single_bead)
            ++*single_bead;
        return false;
    }

    Polyline path_nm(const ExtrusionPath &p) const
    {
        Polyline pl;
        pl.points.reserve(p.polyline.points.size());
        for (const auto &q : p.polyline.points) pl.points.push_back(grid(q.to_point()));
        return pl;
    }
    // nm on the text grid -> the double the G-code text parses to (the printed integer / 10^d, correctly rounded)
    std::vector<Vec2d> mm(const Polyline &pl) const
    {
        std::vector<Vec2d> v;
        v.reserve(pl.points.size());
        const double div = double(int64_t(1000000) / unit_nm);
        for (const Point &q : pl.points) v.emplace_back(double(q.x() / unit_nm) / div, double(q.y() / unit_nm) / div);
        return v;
    }
    void open_path(const ExtrusionPath &p)
    {
        if (! is_wall_role(p.role()) || p.polyline.points.empty())
            return;
        Polyline pl = path_nm(p);
        const Points pp = to_points(p.polyline.points);
        if (is_outer_role(p.role()) && bounds({ BeadPath{ &pp, double(p.width) / SCALING_FACTOR } }))
            out.open_outer.push_back(mm(pl));
        out.pieces.push_back({ std::move(pl), double(p.width), false });
    }
    void loop(const ExtrusionLoop &L)
    {
        if (L.paths.empty())
            return;
        std::vector<ExtrusionRole> roles;
        bool all_wall = true, same_width = true;
        for (const ExtrusionPath &p : L.paths) {
            roles.push_back(p.role());
            all_wall &= is_wall_role(p.role());
            same_width &= p.width == L.paths.front().width;
        }
        // Arachne disconnection guard: consecutive paths whose ends are more than 1 um apart
        bool disconnected = false;
        for (size_t i = 0; i < L.paths.size(); ++i) {
            const auto &a = L.paths[i].polyline.points;
            const auto &b = L.paths[(i + 1) % L.paths.size()].polyline.points;
            if (a.empty() || b.empty()) {
                disconnected = true;
                continue;
            }
            const Vec2d dd = unscale(a.back().to_point()) - unscale(b.front().to_point());
            if (dd.norm() > 1e-3)
                disconnected = true;
        }
        const Polygon loop_poly = L.polygon();   // before clip_end: no seam gap
        if (all_wall && same_width && ! disconnected) {
            // one closed piece (EndType::Joined); the per-path pieces would add nothing to its dilation
            Polyline pl;
            for (const Point &q : loop_poly.points) pl.points.push_back(grid(q));
            out.pieces.push_back({ std::move(pl), double(L.paths.front().width), true });
        } else {
            for (const ExtrusionPath &p : L.paths)
                if (is_wall_role(p.role()) && ! p.polyline.points.empty())
                    out.pieces.push_back({ path_nm(p), double(p.width), false });
        }
        std::vector<Points>   ppts;
        std::vector<BeadPath> beads;
        ppts.reserve(L.paths.size());
        for (const ExtrusionPath &p : L.paths) {
            ppts.push_back(to_points(p.polyline.points));
            beads.push_back({ &ppts.back(), double(p.width) / SCALING_FACTOR });
        }
        if (! disconnected && is_ring_loop(L.inset_idx, roles) && bounds(beads)) {
            Polygon poly;
            for (const Point &q : loop_poly.points) poly.points.push_back(grid(q));
            rings.push_back(std::move(poly));
        }
    }
    void walk(const ExtrusionEntity &e)
    {
        if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(&e)) {
            for (const ExtrusionEntity *x : c->entities)
                if (x != nullptr)
                    walk(*x);
        } else if (const auto *l = dynamic_cast<const ExtrusionLoop *>(&e)) {
            loop(*l);
        } else if (const auto *m = dynamic_cast<const ExtrusionMultiPath *>(&e)) {
            for (const ExtrusionPath &p : m->paths) open_path(p);
        } else if (const auto *p = dynamic_cast<const ExtrusionPath *>(&e)) {
            open_path(*p);
        }
    }
};

} // namespace

std::string PowderShadow::capture(const std::vector<CaptureLayer> &layers, const std::vector<const PrintInstance *> *ordering,
                                  size_t single_object_instance_idx, const ShadowFrame &frame,
                                  const std::function<bool(const ExtrusionPath &)> &path_uses_arcs)
{
    (void) ordering;   // the non-sequential instance loop of process_layer prints every instance of each object
    ScopedMs       timer{ &m_stats.capture_ms, &m_mutex };
    const uint64_t id   = next_id();
    bool           core = true;   // an exception before the classification is reported on the core layers
    NativeLayer    nl;
    try {
        // 5.1: native classification (v2 classify_layers, auto mode) from the in-memory roles
        bool has_sparse = false, has_skin = false, has_arc = false;
        auto arcs = [&](const ExtrusionPath &p) { has_arc |= path_uses_arcs && path_uses_arcs(p); };
        for (const CaptureLayer &lt : layers) {
            if (lt.object_layer != nullptr)
                for (const LayerRegion *region : lt.object_layer->regions()) {
                    for_each_leaf(region->fills, [&](const ExtrusionPath &p) {
                        const ExtrusionRole r = p.role();
                        has_sparse |= r == erInternalInfill;
                        has_skin |= r == erTopSolidInfill || r == erBottomSurface;
                        arcs(p);
                    });
                    for_each_leaf(region->perimeters, arcs);
                }
            if (lt.support_layer != nullptr)
                for_each_leaf(lt.support_layer->support_fills, arcs);
        }
        core = has_sparse && ! has_skin && ! has_arc;
        if (m_opts.fault == "capture")
            throw std::runtime_error("fault injection: capture");
        if (core) {
            // 5.2 / 5.3: walls and rings per instance, on the text grid
            const size_t  d       = frame.high_precision ? 4 : 3;
            const int64_t unit_nm = d == 4 ? 100 : 1000;
            nl.native_core        = true;
            int single_bead       = 0;
            for (const CaptureLayer &lt : layers) {
                if (lt.object_layer == nullptr || lt.original_object == nullptr)
                    continue;
                const PrintInstances &insts = lt.original_object->instances();
                size_t i0 = 0, i1 = insts.size();
                if (single_object_instance_idx != size_t(-1)) {
                    i0 = single_object_instance_idx;
                    i1 = std::min(insts.size(), single_object_instance_idx + 1);
                }
                for (size_t ii = i0; ii < i1; ++ii) {
                    const Vec2d origin = unscale(insts[ii].shift);   // GCode::set_origin(unscale(offset))
                    std::function<Point(const Point &)> grid;
                    if (frame.to_grid)
                        grid = [&frame, origin](const Point &p) { return frame.to_grid(p, origin); };
                    else
                        grid = [&frame, origin, d, unit_nm](const Point &p) {
                            const Vec2d  g = unscale(p) + origin - frame.extruder_offset;
                            const double x = g(0) - frame.writer_xy_offset.x(), y = g(1) - frame.writer_xy_offset.y();
                            return Point(coord_t(GCodeFormatter::axis_units(x, d) * unit_nm),
                                         coord_t(GCodeFormatter::axis_units(y, d) * unit_nm));
                        };
                    Polygons      rings;
                    WallCollector wc{ grid, unit_nm, nl, rings, &lt.object_layer->lslices, &single_bead };
                    for (const LayerRegion *region : lt.object_layer->regions()) wc.walk(region->perimeters);
                    nl.rings.push_back(std::move(rings));
                }
            }
            if (single_bead > 0) {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_single_bead += single_bead;
            }
        }
    } catch (const CanceledException &) {
        throw;
    } catch (const std::exception &ex) {
        nl       = NativeLayer();
        nl.error = ex.what();
        if (nl.error.empty())
            nl.error = "unknown error";
    }
    if (core || ! nl.error.empty())
        put_native(id, std::move(nl));
    return placeholder(id, core);
}

PowderShadow::~PowderShadow() = default;

uint64_t PowderShadow::next_id()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return ++m_next_id;
}

void PowderShadow::put_native(uint64_t id, NativeLayer layer)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_native[id] = std::move(layer);
}

std::optional<NativeLayer> PowderShadow::take_native(uint64_t id)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_native.find(id);
    if (it == m_native.end())
        return std::nullopt;
    NativeLayer l = std::move(it->second);
    m_native.erase(it);
    return l;
}

size_t PowderShadow::load_dump_dir(const std::string &dir)
{
    namespace fs = boost::filesystem;
    size_t n = 0;
    if (dir.empty() || ! fs::is_directory(fs::path(dir)))
        return 0;
    for (fs::directory_iterator it(dir), end; it != end; ++it) {
        const std::string name = it->path().filename().string();
        if (name.size() < 7 || name[0] != 'k' || it->path().extension().string() != ".json")
            continue;
        boost::nowide::ifstream f(it->path().string(), std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        const nlohmann::json j = nlohmann::json::parse(ss.str());
        if (j.contains("token") && j.at("token").is_string() && ! j.at("token").get<std::string>().empty())
            m_token = j.at("token").get<std::string>();   // the exporting shadow's placeholders
        if (! j.contains("id") || ! j.contains("native") || j.at("native").is_null())
            continue;
        const nlohmann::json &jn = j.at("native");
        NativeLayer nl;
        nl.native_core = jn.value("native_core", true);
        for (const auto &inst : jn.at("rings")) {
            Polygons pp;
            for (const auto &r : inst) pp.emplace_back(pts_nm(r));
            nl.rings.push_back(std::move(pp));
        }
        for (const auto &w : jn.at("pieces"))
            nl.pieces.push_back({ Polyline(pts_nm(w.at("pts"))), jnum(w.at("width")), w.at("closed").get<bool>() });
        for (const auto &o : jn.at("open_outer")) nl.open_outer.push_back(pts_mm(o));
        put_native(j.at("id").get<uint64_t>(), std::move(nl));
        ++n;
    }
    return n;
}

void PowderShadow::release(std::string_view line)
{
    if (! m_opts.keep_placeholders && is_placeholder_line(line, m_token))
        return;
    m_out.append(line.data(), line.size());
}

void PowderShadow::release_held()
{
    for (const std::string &l : m_held) release(l);
    m_held.clear();
    m_held_infos.clear();
    m_ph.clear();
}

void PowderShadow::flush(const Sink &sink)
{
    if (m_out.empty())
        return;
    std::string out;
    out.swap(m_out);
    sink(out);
}

void PowderShadow::count(const std::string &reason, int k, const std::string &what)
{
    m_stats.reasons[reason].push_back(k);
    if (! what.empty() && ! m_first_what.count(reason))
        m_first_what[reason] = what;
}

void PowderShadow::disable(const std::string &why)
{
    // Disabled mode (section 6.2): everything held goes out unchanged (placeholders removed), the rest passes.
    for (auto &ph : m_ph) take_native(ph.id);
    release_held();
    m_passthrough = true;
    m_stats.disabled = true;
    if (! m_first_what.count("binder"))
        m_first_what["binder"] = why;
}

void PowderShadow::feed(std::string_view bytes, const Sink &sink)
{
    ScopedMs timer{ &m_stats.binder_ms, nullptr };
    size_t pos = 0;
    while (pos < bytes.size()) {
        const size_t nl = bytes.find('\n', pos);
        if (nl == std::string_view::npos) {
            if (m_finished)
                m_out.append(bytes.data() + pos, bytes.size() - pos);
            else
                m_partial.append(bytes.data() + pos, bytes.size() - pos);
            break;
        }
        std::string line;
        line.swap(m_partial);
        line.append(bytes.data() + pos, nl + 1 - pos);
        pos = nl + 1;
        if (m_passthrough || m_finished) {
            // passthrough (config block seen, or disabled mode): a placeholder's capture is consumed with it
            uint64_t pid = 0;
            bool     pm  = false;
            if (parse_placeholder(line, m_token, pid, pm))
                take_native(pid);
            release(line);
            continue;
        }
        bool consumed = false;
        try {
            handle_line(std::move(line), consumed);
        } catch (const CanceledException &) {
            throw;
        } catch (const std::exception &ex) {
            disable(ex.what());   // the held lines first: they precede this line
            if (! consumed)
                release(line);
        }
    }
    flush(sink);
}

void PowderShadow::handle_line(std::string &&line, bool &consumed)
{
    const size_t   i  = m_line_no++;
    const LineInfo li = m_scanner.step(line);
    if (line.find("; CONFIG_BLOCK_START") != std::string::npos) {
        // split_layers stops here: the held chunk is the final one.
        if (std::optional<LayerChunk> c = m_tracker.finish(i))
            close_chunk(*c, true);
        m_passthrough = true;
        consumed      = true;
        release(line);
        return;
    }
    if (m_tracker.step(i, line, li) == ChunkTracker::opened) {
        if (std::optional<LayerChunk> c = m_tracker.take_closed())
            close_chunk(*c, false);
        m_unbindable = false;
    }
    if (m_tracker.current() == nullptr) {
        consumed = true;
        release(line);
        return;
    }
    uint64_t id = 0;
    bool     m  = false;
    const bool is_ph = parse_placeholder(line, m_token, id, m);
    if (m_unbindable) {
        // sync() released this chunk: its lines pass at once; the captures of its placeholders are consumed.
        if (is_ph) {
            take_native(id);
            if (m)
                m_ph.push_back({ size_t(-1), id, m });
        }
        consumed = true;
        release(line);
        return;
    }
    if (is_ph)
        m_ph.push_back({ m_held.size(), id, m });
    // T10 debug counter: the number of distinct chunks whose lines are held at once (1 by construction).
    const int ci = m_tracker.current()->index;
    if (m_held.empty()) {
        m_held_chunk   = ci;
        m_held_nchunks = 1;
    } else if (ci != m_held_chunk) {
        m_held_chunk = ci;
        ++m_held_nchunks;
    }
    m_stats.max_held_chunks = std::max(m_stats.max_held_chunks, m_held_nchunks);
    m_held_infos.push_back(li);
    m_held.push_back(std::move(line));
    consumed = true;
    if (m_opts.fault == "binder" && m_held.size() == 2)
        throw std::runtime_error("fault injection: binder");
}

void PowderShadow::close_chunk(const LayerChunk &chunk, bool final)
{
    if (m_unbindable) {
        if (! m_ph.empty())
            count("sync", chunk.index);
    } else if (! m_held.empty()) {
        const std::optional<std::string> block = finalize(chunk, final);
        // line 0 (the start marker), the block, then the other lines without the placeholders
        release(m_held.front());
        if (block) {
            if (m_held.front().empty() || m_held.front().back() != '\n')
                m_out += '\n';
            m_out += *block;
        }
        for (size_t j = 1; j < m_held.size(); ++j) release(m_held[j]);
    }
    if (chunk.z)
        m_prev_z = *chunk.z;
    m_held.clear();
    m_held_infos.clear();
    m_ph.clear();
    m_unbindable = false;
}

std::optional<std::string> PowderShadow::finalize(const LayerChunk &chunk, bool final)
{
    const int         k = chunk.index;
    const UMexParams &U = P.U;

    // Every placeholder's capture is consumed here, whatever the outcome.
    std::vector<std::pair<Placeholder, std::optional<NativeLayer>>> phs;
    for (const Placeholder &ph : m_ph) phs.emplace_back(ph, take_native(ph.id));

    // gate 0
    if (phs.empty() && ! m_opts.text_only)
        return std::nullopt;
    // gate 1, 2
    if (final) {
        count("final", k);
        return std::nullopt;
    }
    if (chunk.has_arc || chunk.has_relxyz) {
        count("arc_g91", k);
        return std::nullopt;
    }
    // gate 3: v2 classify_layers (auto rule) on the text vs the native classification
    bool has_sparse = false, has_skin = false;
    for (const std::string &t : chunk.types) {
        has_sparse |= SPARSE_T.count(t) > 0;
        has_skin |= SKIN_T.count(t) > 0;
    }
    const bool text_core = has_sparse && ! has_skin;
    int        n_m1      = 0;
    for (const auto &ph : phs) n_m1 += ph.first.m ? 1 : 0;
    const bool adapter_only = phs.empty();   // text-only replay without placeholders
    const bool native_core  = adapter_only ? text_core : n_m1 > 0;
    if (text_core != native_core) {
        count(text_core ? "disagree_text_only" : "disagree_native_only", k);
        return std::nullopt;
    }
    if (! text_core)
        return std::nullopt;
    // gate 4, 5
    const NativeLayer *nat = nullptr;
    if (! adapter_only) {
        if (n_m1 != 1) {
            count("merged_multi_core", k);
            return std::nullopt;
        }
        for (const auto &ph : phs)
            if (ph.first.m && ph.second)
                nat = &*ph.second;
        if (nat == nullptr && ! m_opts.text_only) {
            count("lost", k);
            return std::nullopt;
        }
        if (nat != nullptr && ! nat->error.empty()) {
            count("capture", k, nat->error);
            return std::nullopt;
        }
    }
    if (m_opts.throw_if_canceled)
        m_opts.throw_if_canceled();

    // gate 6: the translator's own layer model, z and h
    LayerChunk lc = chunk;
    lc.start      = 0;
    lc.end        = m_held.size();
    const LayerModel model = parse_core_layer(m_held, m_held_infos, lc, U.width_fallback);
    if (! model.z) {
        count("no_z", k);
        return std::nullopt;
    }
    PowderLayerRec rec;
    rec.k  = k;
    rec.z  = *model.z;
    rec.h  = truthy(chunk.height) ? *chunk.height : rec.z - m_prev_z;
    rec.fp = m_fp;
    bool       carrier = false;
    PlanInput  in;
    PlanDebug  dbg;
    const bool want_dbg = ! m_opts.dump_dir.empty();
    if (rec.h <= 0.0) {
        // gate 7: carrier, the translator gives v2's "non-positive layer height" warning
        rec.status = PowderLayerRec::Status::open;
        rec.s = rec.F = rec.fu = 0.0;
        carrier    = true;
    } else {
        // gate 8
        PowderFeed pf;
        try {
            pf = resolve_powder_feed(U.m_dot, U.rho_tap, U.s_hatch, rec.h, U.eta, U.f_max, U.policy);
        } catch (const DIWConfigError &) {
            // policy=error overflow: carrier with the clamp values; the translator raises v2's message here.
            PowderFeed pc;
            try {
                pc = resolve_powder_feed(U.m_dot, U.rho_tap, U.s_hatch, rec.h, U.eta, U.f_max, "clamp");
            } catch (const std::exception &ex) {
                count("plan", k, ex.what());
                return std::nullopt;
            }
            rec.status = PowderLayerRec::Status::open;
            rec.s      = U.s_hatch;
            rec.F = rec.fu = pc.f_calc;
            carrier        = true;
        }
        if (! carrier) {
            // gate 9 and 10
            in.U           = &U;
            in.h           = rec.h;
            in.s_eff       = pf.s_eff;
            in.text_walls  = text_paths(model, "wall", 1);
            in.text_paste  = text_paths(model, "paste", 2);
            in.text_powder = text_paths(model, "powder", 1);
            if (nat != nullptr) {
                for (const Polygons &inst : nat->rings)
                    for (const Polygon &poly : inst) in.rings.push_back({ poly });
                in.native_walls = nat->pieces;
                in.open_outer   = nat->open_outer;
                // A printed vertex is rounded to the text grid (<= 0.5 * 10^-d * sqrt(2)), and so are the native
                // segment's ends; the seam split point lies on the exact segment, so it can be off the rounded one
                // by both roundings (PLANNER_DESIGN.md "Deviations", stage 5).
                const double tol = std::pow(10.0, -double(m_opts.xy_decimals)) * std::sqrt(2.0) + 1e-6;
                const std::string why = frame_check(*nat, in.text_walls, tol);
                if (! why.empty()) {
                    count("frame", k, why);
                    return std::nullopt;
                }
            } else {
                // text-only adapter: v2's rings (snap + linemerge of the outer / overhang walls), no native band
                for (const TextPath &t : in.text_walls)
                    if ((t.ptype == "outer wall" || t.ptype == "overhang wall") && t.pts_mm.size() >= 2)
                        in.open_outer.push_back(t.pts_mm);
            }
            PowderLayerRec planned;
            try {
                if (m_opts.fault == "plan")
                    throw std::runtime_error("fault injection: plan");
                planned = plan_powder_layer_native(in, want_dbg ? &dbg : nullptr);
            } catch (const CanceledException &) {
                throw;
            } catch (const std::exception &ex) {
                count("plan", k, ex.what());
                return std::nullopt;
            }
            rec.status    = planned.status;
            rec.ca        = planned.ca;
            rec.oa        = planned.oa;
            rec.ra        = planned.ra;
            rec.hl        = planned.hl;
            rec.nc        = planned.nc;
            rec.runs      = std::move(planned.runs);
            rec.fallbacks = std::move(planned.fallbacks);
            rec.s         = pf.s_eff;
            rec.F         = pf.F;
            rec.fu        = pf.f_calc;
        }
    }
    // gate 11: the block must read back as itself
    std::string txt;
    try {
        txt = write_block(rec);
    } catch (const std::invalid_argument &ex) {
        count("record", k, ex.what());
        return std::nullopt;
    }
    const ParsedRecords back = parse_records(txt);
    ParsedRecords       mine;
    mine.blocks.push_back(rec);
    if (back.blocks.size() != 1 || ! back.errors.empty() || dump_records(back) != dump_records(mine)) {
        count("record", k, back.errors.empty() ? std::string("round trip differs") : back.errors.front().second);
        return std::nullopt;
    }
    ++m_stats.blocks;
    if (carrier)
        ++m_stats.carriers;

    if (want_dbg) {
        try {
            nlohmann::json j;
            j["k"]       = k;
            j["id"]      = nullptr;
            j["token"]   = m_token;
            for (const auto &ph : phs)
                if (ph.first.m) j["id"] = ph.first.id;
            j["adapter"] = nat == nullptr;
            if (nat != nullptr) {
                nlohmann::json jr = nlohmann::json::array(), jw = nlohmann::json::array(), jo = nlohmann::json::array();
                for (const Polygons &inst : nat->rings) {
                    nlohmann::json a = nlohmann::json::array();
                    for (const Polygon &p : inst) a.push_back(jpts_nm(p.points));
                    jr.push_back(a);
                }
                for (const WallPiece &w : nat->pieces)
                    jw.push_back({ { "pts", jpts_nm(w.pts.points) }, { "width", py_repr(w.width) }, { "closed", w.closed } });
                for (const auto &o : nat->open_outer) jo.push_back(jpts_mm(o));
                j["native"] = { { "native_core", nat->native_core }, { "rings", jr }, { "pieces", jw }, { "open_outer", jo } };
            } else
                j["native"] = nullptr;
            nlohmann::json oo = nlohmann::json::array();
            for (const auto &o : in.open_outer) oo.push_back(jpts_mm(o));
            j["input"] = { { "h", py_repr(rec.h) }, { "s_eff", py_repr(in.s_eff) }, { "open_outer", oo },
                           { "text_walls", jtext(in.text_walls) }, { "text_paste", jtext(in.text_paste) },
                           { "text_powder", jtext(in.text_powder) } };
            nlohmann::json comps = nlohmann::json::array();
            for (const PlanComponent &pc : dbg.components)
                comps.push_back({ { "rows", pc.rows }, { "H", py_repr(pc.H) }, { "runs", pc.runs }, { "point", pc.point },
                                  { "total", py_repr(pc.total) } });
            j["debug"]  = { { "cavity", wkt(dbg.cavity) }, { "omega", wkt(dbg.omega) }, { "narrow", wkt(dbg.narrow) },
                            { "revert", wkt(dbg.revert) }, { "components", comps }, { "notes", dbg.notes } };
            j["record"] = txt;
            const std::string path = (boost::filesystem::path(m_opts.dump_dir) / ("k" + std::to_string(k) + ".json")).string();
            boost::nowide::ofstream f(path, std::ios::binary | std::ios::trunc);
            const std::string s = j.dump(1);
            f.write(s.data(), std::streamsize(s.size()));
        } catch (const std::exception &) {
            // a dump is a debugging aid; it never changes the export
        }
    }
    return txt;
}

void PowderShadow::sync(const Sink &sink)
{
    ScopedMs timer{ &m_stats.binder_ms, nullptr };
    if (! m_passthrough && ! m_finished && m_tracker.current() != nullptr && ! m_unbindable) {
        std::vector<Placeholder> keep;
        for (const Placeholder &ph : m_ph) {
            take_native(ph.id);
            if (ph.m)
                keep.push_back({ size_t(-1), ph.id, true });
        }
        release_held();
        m_ph         = std::move(keep);
        m_unbindable = true;
    }
    flush(sink);
}

void PowderShadow::finish(const Sink &sink)
{
    ScopedMs timer{ &m_stats.binder_ms, nullptr };
    if (m_finished) {
        flush(sink);
        return;
    }
    if (! m_partial.empty()) {
        std::string line;
        line.swap(m_partial);
        if (m_passthrough)
            release(line);
        else {
            bool consumed = false;
            try {
                handle_line(std::move(line), consumed);
            } catch (const CanceledException &) {
                throw;
            } catch (const std::exception &ex) {
                disable(ex.what());
                if (! consumed)
                    release(line);
            }
        }
    }
    if (! m_passthrough) {
        try {
            if (std::optional<LayerChunk> c = m_tracker.finish(m_line_no))
                close_chunk(*c, true);
        } catch (const CanceledException &) {
            throw;
        } catch (const std::exception &ex) {
            disable(ex.what());
        }
        m_passthrough = true;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        for (const auto &kv : m_native) {
            (void) kv;
            m_stats.reasons["lost_placeholder"].push_back(0);
        }
        m_native.clear();
    }
    m_finished = true;
    flush(sink);
}

std::vector<std::string> PowderShadow::warnings() const
{
    std::vector<std::string> out;
    for (const auto &[reason, ks] : m_stats.reasons) {
        if (reason == "lost_placeholder") {
            out.push_back("u-MEX native powder planner: " + std::to_string(ks.size()) +
                          " captured layer(s) never reached the G-code output (placeholder lost)");
            continue;
        }
        const char *label = reason_label(reason);
        if (label == nullptr)
            continue;   // final, arc_g91: not a fault
        std::string list;
        for (size_t i = 0; i < ks.size() && i < 10; ++i) list += (i ? ", " : "") + std::to_string(ks[i]);
        if (ks.size() > 10)
            list += ", ... (" + std::to_string(ks.size() - 10) + " more)";
        std::string msg = "u-MEX native powder planner: " + std::to_string(ks.size()) + " core layer(s) (" + list +
                          ") left as paste: " + label;
        const auto it = m_first_what.find(reason);
        if (it != m_first_what.end())
            msg += ": " + it->second;
        out.push_back(msg);
    }
    if (m_stats.disabled) {
        const auto it = m_first_what.find("binder");
        out.push_back("u-MEX native powder planner disabled during the export (" +
                      (it != m_first_what.end() ? it->second : std::string("internal error")) +
                      "); the rest of the G-code carries no powder records");
    }
    return out;
}

std::string PowderShadow::summary() const
{
    std::string s = "DIW shadow planner: " + std::to_string(m_stats.blocks) + " block(s), " + std::to_string(m_stats.carriers) +
                    " carrier(s)";
    for (const auto &[reason, ks] : m_stats.reasons) s += ", " + reason + "=" + std::to_string(ks.size());
    if (m_stats.disabled)
        s += ", disabled";
    char t[160];
    std::snprintf(t, sizeof(t), "; max_held=%d capture_ms=%.1f binder_ms=%.1f single_bead=%d", m_stats.max_held_chunks,
                  m_stats.capture_ms, m_stats.binder_ms, m_single_bead);
    return s + t;
}

} // namespace DIW
} // namespace Slic3r
