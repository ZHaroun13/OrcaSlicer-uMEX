// Unit tests of the DIW powder planner (src/libslic3r/GCode/DIWPowderPlanner, diw/PLANNER_DESIGN.md T9).
// Built inside libslic3r_tests with BUILD_TESTS, and stand-alone as the dev-utils target diw_planner_tests.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/GCode/DIWPowderPlanner.hpp"
#include "libslic3r/GCode/DIWPowderReport.hpp"
#include "libslic3r/GCode/DIWPyCompat.hpp"
#include "libslic3r/GCode/DIWTranslator.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <nlohmann/json.hpp>

#include <cctype>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::DIW;

namespace {

constexpr double PI_T = 3.14159265358979323846;

Point Pmm(double x_mm, double y_mm) { return to_grid(Vec2d(x_mm, y_mm)); }

ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    ExPolygon e;
    e.contour.points = { Pmm(x0, y0), Pmm(x1, y0), Pmm(x1, y1), Pmm(x0, y1) };
    return e;
}

Polygon hole_mm(double x0, double y0, double x1, double y1)
{
    Polygon h;
    h.points = { Pmm(x0, y0), Pmm(x0, y1), Pmm(x1, y1), Pmm(x1, y0) };   // clockwise
    return h;
}

double dist_to_polyline_mm(const Vec2d &p, const Polyline &pl, bool closed)
{
    double     best = 1e300;
    const auto n    = pl.points.size();
    for (size_t i = 0; i + (closed ? 0 : 1) < n; ++i) {
        const Vec2d a = pl.points[i].cast<double>() * 1e-6, b = pl.points[(i + 1) % n].cast<double>() * 1e-6;
        const Vec2d ab = b - a;
        double      t  = ab.squaredNorm() > 0 ? (p - a).dot(ab) / ab.squaredNorm() : 0.;
        t              = std::max(0., std::min(1., t));
        best           = std::min(best, (p - (a + t * ab)).norm());
    }
    return best;
}

bool in_expolys(const ExPolygons &ee, const Vec2d &p_mm)
{
    for (const ExPolygon &e : ee)
        if (inside_rings(to_rings_mm(e), p_mm, false))
            return true;
    return false;
}

using Line2 = std::vector<std::pair<double, double>>;

// Canonical form of a merged line: closed lines rotated to their smallest vertex and in their smaller direction;
// open lines in their smaller direction.
Line2 canon(Line2 l)
{
    if (l.size() > 1 && l.front() == l.back()) {
        l.pop_back();
        const size_t k = size_t(std::min_element(l.begin(), l.end()) - l.begin());
        std::rotate(l.begin(), l.begin() + k, l.end());
        Line2 r(l.rbegin(), l.rend());
        std::rotate(r.begin(), r.begin() + (std::min_element(r.begin(), r.end()) - r.begin()), r.end());
        l = std::min(l, r);
        l.push_back(l.front());
        return l;
    }
    Line2 r(l.rbegin(), l.rend());
    return std::min(l, r);
}

} // namespace

TEST_CASE("arc tolerance gives 192 steps per 360 degrees", "[diw_planner]")
{
    for (double r_mm : { 0.2, 0.21, 0.6, 2.65 }) {
        const double r_u = r_mm * 1e6;
        CHECK(std::round(clipper_steps_per_360(r_u)) == 192.);
        CHECK(std::abs(arc_tol(r_u) - r_u * (1. - std::cos(PI_T / 192.))) <= 1e-9 * r_u);
    }
    CHECK(sector_steps(2. * PI_T) == SECTOR_STEPS_PER_360);
    CHECK(sector_steps(PI_T) == SECTOR_STEPS_PER_360 / 2);
    CHECK(sector_steps(1e-6) == 1);
}

TEST_CASE("padded round offset contains the exact disk sum", "[diw_planner]")
{
    std::mt19937_64 rng(4242);
    // An open zigzag with segments shorter than the radius, sharp turns and a reversal, and a closed loop.
    Polyline zig;
    zig.points = { Pmm(0, 0), Pmm(0.3, 0.1), Pmm(0.5, -0.2), Pmm(5, 0), Pmm(5.2, 3), Pmm(5.2, 0.5), Pmm(9, 0.6), Pmm(9.1, 0.6) };
    Polyline loop;
    for (int i = 0; i < 40; ++i) {
        const double a = 2 * PI_T * i / 40, rr = 3 + 0.7 * std::sin(5 * a);
        loop.points.push_back(Pmm(20 + rr * std::cos(a), rr * std::sin(a)));
    }
    for (const double r : { 0.21, 0.7 }) {
        const ExPolygons zo = round_offset({ zig }, r, OffsetEnd::Round);
        const ExPolygons lo = round_offset({ loop }, r, OffsetEnd::Joined);
        std::uniform_real_distribution<double> ux(-1.5, 10.5), uy(-1.5, 4.5), lx(15.5, 24.5), ly(-4.5, 4.5);
        int n_in = 0, n_bad = 0;
        while (n_in < 10000) {
            const Vec2d p(ux(rng), uy(rng));
            if (dist_to_polyline_mm(p, zig, false) <= r) {
                ++n_in;
                if (! in_expolys(zo, p)) ++n_bad;
            }
        }
        CHECK(n_bad == 0);
        n_in = n_bad = 0;
        while (n_in < 10000) {
            const Vec2d p(lx(rng), ly(rng));
            if (dist_to_polyline_mm(p, loop, true) <= r) {
                ++n_in;
                if (! in_expolys(lo, p)) ++n_bad;
            }
        }
        CHECK(n_bad == 0);
        // and not much more: every vertex of the result lies within the circumscription excess of the sum
        const double excess = r * (1. / std::cos(PI_T / SECTOR_STEPS_PER_360) - 1.) + 3e-6;
        double worst = 0.;
        for (const ExPolygon &e : zo)
            for (const Point &q : e.contour.points)
                worst = std::max(worst, dist_to_polyline_mm(q.cast<double>() * 1e-6, zig, false) - r);
        CHECK(worst <= excess);
        worst = 0.;
        for (const ExPolygon &e : lo)
            for (const Polygon *pg : { &e.contour }) {
                for (const Point &q : pg->points)
                    worst = std::max(worst, dist_to_polyline_mm(q.cast<double>() * 1e-6, loop, true) - r);
            }
        CHECK(worst <= excess);
    }
}

// Stage 5c: the dilations are built from raw offset paths (one Positive union per polyline / ExPolygon) instead of
// the union of one piece per segment and vertex. Both constructions use the same vertices, so the two sets may
// differ only by the grid rounding along their boundary (the rectangles of the pieces are extended by one grid unit)
// and, for the band, by the circumscribed cap arcs that the raw paths add on both sides of a reversal. Checked on
// 1800 random polylines and 600 random polygons (stage 5c also ran 20000 + 20000 of them once, all passing).
TEST_CASE("raw offset paths give the piece unions", "[diw_planner]")
{
    auto area_mm2 = [](const ExPolygons &ee) {
        double a = 0.;
        for (const ExPolygon &e : ee) a += e.area() * 1e-12;
        return a;
    };
    auto boundary_mm = [](const ExPolygons &ee) {
        double l = 0.;
        for (const ExPolygon &e : ee) {
            l += e.contour.length() * 1e-6;
            for (const Polygon &h : e.holes) l += h.length() * 1e-6;
        }
        return l;
    };
    // B - A within 3 grid units of the boundary (A contains B up to the grid rounding), A - B within the
    // circumscription excess (r (1/cos(pi/N) - 1)) + 3 grid units of it (the caps that raw paths add at reversals)
    auto same = [&](const ExPolygons &a, const ExPolygons &b, double excess_mm = 0.) {
        const double l = boundary_mm(a) + boundary_mm(b);
        return area_mm2(diff_ex(b, a)) <= l * 3e-6 + 1e-12 && area_mm2(diff_ex(a, b)) <= l * (excess_mm + 3e-6) + 1e-12;
    };
    std::mt19937_64 rng(5150);
    std::uniform_real_distribution<double> u(0., 1.);
    int cases = 0, bad = 0;
    for (int it = 0; it < 300; ++it) {
        // random polylines: long and very short segments, sharp turns, exact reversals, repeated points
        Polyline pl;
        const int n = 1 + int(u(rng) * 40);
        Vec2d     c(10 * u(rng), 10 * u(rng));
        for (int i = 0; i < n; ++i) {
            const double kind = u(rng);
            Vec2d        step = kind < 0.2 ? Vec2d(1e-5 * (u(rng) - 0.5), 1e-5 * (u(rng) - 0.5))
                              : kind < 0.3 ? Vec2d(0., 0.)
                                           : Vec2d(3 * (u(rng) - 0.5), 3 * (u(rng) - 0.5));
            if (kind > 0.9 && pl.points.size() >= 2)   // reversal along the previous segment
                step = -0.5 * (pl.points.back() - pl.points[pl.points.size() - 2]).cast<double>() * 1e-6;
            c += step;
            pl.points.push_back(to_grid(c));
        }
        for (const double r : { 0.21, 0.7, 1.5601 }) {
            for (const OffsetEnd end : { OffsetEnd::Round, OffsetEnd::Joined }) {
                const ExPolygons a = round_offset({ pl }, r, end);
                const ExPolygons b = round_offset_pieces({ pl }, r, end);
                ++cases;
                if (! same(a, b, r * (1. / std::cos(PI_T / SECTOR_STEPS_PER_360) - 1.)))
                    ++bad;
            }
        }
    }
    CHECK(cases == 1800);
    CHECK(bad == 0);
    // mitre dilation of random star polygons (sharp tips beyond the mitre limit, reflex vertices, nm zigzags, thin
    // notches) with a hole
    int mbad = 0;
    for (int it = 0; it < 200; ++it) {
        ExPolygon e;
        const int    n  = 3 + int(u(rng) * 30);
        const Vec2d  c(20 * u(rng), 20 * u(rng));
        for (int i = 0; i < n; ++i) {
            const double ang = 2 * PI_T * (i + 0.3 * u(rng)) / n, rr = 2 + 8 * u(rng);
            e.contour.points.push_back(to_grid(c + rr * Vec2d(std::cos(ang), std::sin(ang))));
            if (u(rng) < 0.3)   // a few-nm zigzag
                e.contour.points.push_back(e.contour.points.back() + Point(coord_t(5 * (u(rng) - 0.5)), coord_t(5 * (u(rng) - 0.5))));
            if (u(rng) < 0.2)   // a thin notch towards the centre
                e.contour.points.push_back(to_grid(c + 0.1 * rr * Vec2d(std::cos(ang + 1e-4), std::sin(ang + 1e-4))));
        }
        if (e.contour.area() <= 0.)
            continue;
        if (u(rng) < 0.5)
            e.holes.push_back(hole_mm(c.x() - 0.5, c.y() - 0.5, c.x() + 0.5, c.y() + 0.5));
        const ExPolygons clean = union_ex(ExPolygons{ e });
        for (const double d : { 1e5, 1e6, 3e6 })
            if (! same(mitre_dilate(clean, d, 5.0), mitre_dilate_pieces(clean, d, 5.0))) ++mbad;
    }
    CHECK(mbad == 0);
}

TEST_CASE("erosion stays inside the exact erosion", "[diw_planner]")
{
    ExPolygon sq = rect_mm(0, 0, 10, 10);
    sq.holes.push_back(hole_mm(3, 3, 7, 7));
    const ExPolygons om = round_erode({ sq }, 0.601);
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> u(-0.5, 10.5);
    int bad = 0;
    for (int i = 0; i < 20000; ++i) {
        const Vec2d p(u(rng), u(rng));
        if (! in_expolys(om, p)) continue;
        // distance to the complement of sq = distance to its boundary (p inside)
        const double d = std::min({ p.x(), p.y(), 10 - p.x(), 10 - p.y(),
                                    std::max({ 3 - p.x(), p.x() - 7, 3 - p.y(), p.y() - 7 }) <= 0 ? 0. :
                                    std::hypot(std::max(0., std::max(3 - p.x(), p.x() - 7)), std::max(0., std::max(3 - p.y(), p.y() - 7))) });
        if (d < 0.601 - 1e-9) ++bad;
    }
    CHECK(bad == 0);
    double a = 0;
    for (const ExPolygon &e : om) a += e.area() * 1e-12;
    // exact erosion: (10-1.202)^2 - (4+1.202)^2 + (4 - pi) * 0.601^2 (rounded corners of the grown hole)
    const double exact = std::pow(10 - 1.202, 2) - std::pow(4 + 1.202, 2) + (4 - PI_T) * 0.601 * 0.601;
    CHECK(std::abs(a - exact) < 2e-4);
}

TEST_CASE("mitre opening: GEOS limited mitre", "[diw_planner]")
{
    // square: plain mitres (ratio sqrt 2 < 5), the dilation is the grown square
    const ExPolygons sq = mitre_dilate({ rect_mm(0, 0, 10, 10) }, 1e6, 5.0);
    REQUIRE(sq.size() == 1);
    CHECK(std::abs(sq.front().area() * 1e-12 - 144.0) < 1e-4);
    const BoundingBox bb = get_extents(sq.front().contour);
    CHECK(bb.min == Point(-1000000, -1000000));
    CHECK(bb.max == Point(11000000, 11000000));
    // a 5 degree tip: the mitre (11.5 d) is cut square to the bisector at 5 d from the vertex (GEOS), not at
    // 1 d (Clipper2's squared mitre)
    ExPolygon tri;
    tri.contour.points = { Pmm(0, 0), Pmm(20, -0.875), Pmm(20, 0.875) };
    const ExPolygons td = mitre_dilate({ tri }, 1e6, 5.0);
    REQUIRE(td.size() == 1);
    const BoundingBox tb = get_extents(td.front().contour);
    CHECK(std::abs(tb.min.x() - (-5000000)) <= 2);
    // with a hole: the hole shrinks by d
    ExPolygon h = rect_mm(0, 0, 10, 10);
    h.holes.push_back(hole_mm(3, 3, 7, 7));
    const ExPolygons hd = mitre_dilate({ h }, 1e6, 5.0);
    REQUIRE(hd.size() == 1);
    REQUIRE(hd.front().holes.size() == 1);
    CHECK(std::abs(hd.front().area() * 1e-12 - (144.0 - 4.0)) < 1e-4);
}

TEST_CASE("hatch: square, rows on the edges, odd-row reversal", "[diw_planner]")
{
    PlanComponent info;
    const auto runs = hatch_component(rect_mm(0, 0, 10, 10), 1.0, 0.0, &info);
    CHECK(info.rows == 11);
    REQUIRE(runs.size() == 1);
    const auto &r = runs.front();
    REQUIRE(r.size() == 22);
    CHECK(r[0] == Vec2d(0, 0));   // row 0 lies on the bottom edge
    CHECK(r[1] == Vec2d(10, 0));
    CHECK(r[2] == Vec2d(10, 1));  // row 1 runs backwards
    CHECK(r[3] == Vec2d(0, 1));
    CHECK(r[21] == Vec2d(10, 10)); // row 10 lies on the top edge (even: forward)
}

TEST_CASE("hatch: a connector across a hole is rejected", "[diw_planner]")
{
    ExPolygon sq = rect_mm(0, 0, 10, 10);
    sq.holes.push_back(hole_mm(3, 3, 7, 7));
    CHECK_FALSE(covered(sq, Vec2d(3, 4), Vec2d(7, 4)));
    PlanComponent info;
    const auto    runs = hatch_component(sq, 2.0, 0.0, &info);
    CHECK(info.rows == 6);
    CHECK(runs.size() > 1);
    for (const auto &r : runs)
        for (size_t i = 0; i + 1 < r.size(); ++i)
            CHECK(covered(sq, r[i], r[i + 1]));
}

TEST_CASE("hatch: 30 degree rotation", "[diw_planner]")
{
    const auto runs = hatch_component(rect_mm(0, 0, 10, 10), 0.7, 30.0);
    REQUIRE(! runs.empty());
    const RingsMM rings = to_rings_mm(rect_mm(0, 0, 10, 10));
    int n_rows = 0;
    for (const auto &r : runs)
        for (size_t i = 0; i + 1 < r.size(); ++i) {
            const Vec2d d = r[i + 1] - r[i];
            const double ang = std::atan2(d.y(), d.x()) * 180. / PI_T;
            const bool row = std::abs(std::remainder(ang - 30., 180.)) < 1e-9;
            n_rows += row;
            for (const Vec2d &q : { r[i], r[i + 1] })
                CHECK((inside_rings(rings, q, false) || std::abs(std::min({ q.x(), q.y(), 10 - q.x(), 10 - q.y() })) < 1e-9));
        }
    CHECK(n_rows > 10);
}

TEST_CASE("covered: boundary connector, tangent touch, exit and re-entry", "[diw_planner]")
{
    const ExPolygon sq = rect_mm(0, 0, 10, 10);
    CHECK(covered(sq, Vec2d(10, 0), Vec2d(10, 1)));       // along the boundary
    CHECK(covered(sq, Vec2d(0, 0), Vec2d(10, 10)));       // diagonal through two corners
    // C shape: outer 10x10 with the notch [3,10]x[3,7]
    ExPolygon c;
    c.contour.points = { Pmm(0, 0), Pmm(10, 0), Pmm(10, 3), Pmm(3, 3), Pmm(3, 7), Pmm(10, 7), Pmm(10, 10), Pmm(0, 10) };
    CHECK(covered(c, Vec2d(1, 2), Vec2d(5, 3)));          // touches the notch edge from inside (tangent)
    CHECK(covered(c, Vec2d(3, 1), Vec2d(3, 9)));           // along x=3: inside, then on the notch edge, then
                                                           // inside again (GEOS: buffer(1e-6) covers it)
    CHECK_FALSE(covered(c, Vec2d(8, 1), Vec2d(8, 9)));    // exits through the notch and re-enters
}

TEST_CASE("centroid with holes and InteriorPointArea", "[diw_planner]")
{
    ExPolygon sq = rect_mm(0, 0, 10, 10);
    sq.holes.push_back(hole_mm(1, 1, 4, 4));
    const Vec2d c = area_centroid(sq);
    CHECK(std::abs(c.x() - 5.247252747252747) < 1e-12);   // shapely 2.2.0 / GEOS 3.14.1
    CHECK(std::abs(c.y() - 5.247252747252747) < 1e-12);
    ExPolygon cs;
    cs.contour.points = { Pmm(0, 0), Pmm(10, 0), Pmm(10, 3), Pmm(3, 3), Pmm(3, 7), Pmm(10, 7), Pmm(10, 10), Pmm(0, 10) };
    const Vec2d cc = area_centroid(cs);
    CHECK(std::abs(cc.x() - 4.416666666666667) < 1e-12);
    CHECK_FALSE(inside_rings(to_rings_mm(cs), cc, true));  // the centroid lies in the notch
    const Vec2d ip = interior_point_area(cs);
    CHECK(ip == Vec2d(1.5, 5.0));                          // shapely representative_point
}

TEST_CASE("fallback clip against brute-force sampling", "[diw_planner]")
{
    ExPolygon sq = rect_mm(0, 0, 10, 10);
    sq.holes.push_back(hole_mm(3, 3, 7, 7));
    ExPolygon tri;
    tri.contour.points = { Pmm(12, 0), Pmm(18, 1), Pmm(13, 8) };
    RingsMM clip = to_rings_mm(sq);
    for (auto &r : to_rings_mm(tri)) clip.push_back(r);
    std::mt19937_64 rng(99);
    std::uniform_real_distribution<double> u(-2, 20);
    for (int k = 0; k < 20; ++k) {
        const Vec2d a(u(rng), u(rng)), b(u(rng), u(rng));
        const auto  iv  = clip_segment(clip, a, b);
        double      len = 0.;
        for (const auto &v : iv) {
            CHECK(v.first < v.second);
            len += v.second - v.first;
        }
        const int N = 100000;
        int       in = 0;
        for (int i = 0; i < N; ++i)
            if (inside_rings(clip, a + ((i + 0.5) / N) * (b - a), false)) ++in;
        CHECK(std::abs(len - double(in) / N) <= 10.0 / N);
        for (size_t i = 0; i + 1 < iv.size(); ++i)
            CHECK(iv[i].second < iv[i + 1].first);   // adjacent kept intervals are merged
    }
}

TEST_CASE("ring predicate", "[diw_planner]")
{
    CHECK(is_ring_loop(0, { erExternalPerimeter }));
    CHECK(is_ring_loop(0, { erOverhangPerimeter, erExternalPerimeter }));
    // an inner loop whose first path is an overhang piece (ExtrusionLoop::role() would say overhang)
    CHECK_FALSE(is_ring_loop(1, { erOverhangPerimeter, erPerimeter }));
    CHECK_FALSE(is_ring_loop(2, { erOverhangPerimeter }));
    // inset_idx -1: a thin wall of inner perimeter role is not a ring; an all-external one is
    CHECK_FALSE(is_ring_loop(-1, { erPerimeter }));
    CHECK_FALSE(is_ring_loop(-1, { erExternalPerimeter, erPerimeter }));
    CHECK(is_ring_loop(-1, { erExternalPerimeter, erOverhangPerimeter }));
}

TEST_CASE("LineMerger port against shapely fixtures", "[diw_planner]")
{
    std::ifstream f(std::string(TEST_DATA_DIR) + "/diw_linemerge.json", std::ios::binary);
    REQUIRE(f.good());
    std::stringstream ss;
    ss << f.rdbuf();
    const auto root = nlohmann::json::parse(ss.str());
    REQUIRE(root.at("sets").size() == 50);
    int n_sets = 0;
    for (const auto &set : root.at("sets")) {
        std::vector<std::vector<Vec2d>> lines;
        for (const auto &l : set.at("lines")) {
            std::vector<Vec2d> pts;
            for (const auto &p : l) pts.emplace_back(p.at(0).get<double>(), p.at(1).get<double>());
            lines.push_back(pts);
        }
        std::vector<Line2> got, want;
        for (const auto &m : line_merge(lines)) {
            Line2 l;
            for (const Vec2d &v : m) l.emplace_back(v.x(), v.y());
            got.push_back(canon(l));
        }
        for (const auto &m : set.at("merged")) {
            Line2 l;
            for (const auto &p : m) l.emplace_back(p.at(0).get<double>(), p.at(1).get<double>());
            want.push_back(canon(l));
        }
        std::sort(got.begin(), got.end());
        std::sort(want.begin(), want.end());
        CHECK(got == want);
        ++n_sets;
    }
    CHECK(n_sets == 50);
}

TEST_CASE("v2 ring formation: snap and merge", "[diw_planner]")
{
    // an outer wall split in two at the seam, ends 0.03 apart (snapped), plus a separate closed loop
    std::vector<std::vector<Vec2d>> walls = {
        { { 0, 0 }, { 10, 0 }, { 10, 10 } },
        { { 10, 10 }, { 0, 10 }, { 0.03, 0.0 } },
        { { 20, 0 }, { 25, 0 }, { 25, 5 }, { 20.02, 0.0 } },   // >= 3 points, ends 0.02 apart: snapped closed
    };
    const auto rings = rings_from_open_walls(walls);
    CHECK(rings.size() == 2);
}

// ---------------------------------------------------------------------------------------------------------------
// Audit fix (diw/AUDIT_FIX.md): G91.2, A and B are per-block deltas (dA = K_E*dE, dB = ratio*kB*dE), only on extrusion/retraction.

namespace {

// Layer 1: extrude, retract, travel, re-prime, extrude, travel; layer 2: extrude. Relative E.
const char *AUDIT_SRC =
    ";LAYER_CHANGE\nM83\nG92 E0\nG1 X10 Y10 F1200\nG1 X20 Y10 E1.0 F1800\nG1 E-0.5 F2400\nG1 X30 Y30 F6000\n"
    "G1 E0.5 F2400\nG1 X40 Y30 E1.0 F1800\nG1 X50 Y50 F6000\n;LAYER_CHANGE\nG1 X10 Y10 E1.0 F1800\nM84\n";

std::vector<std::string> audit_translate(const DIWTranslateOptions &opts, int a_mode = -1)
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
    if (a_mode >= 0)
        cfg.set_key_value("diw_a_axis_mode", new ConfigOptionEnumsGeneric{ a_mode });
    const std::string out = diw_translate_gcode(AUDIT_SRC, cfg, opts);
    std::vector<std::string> lines;
    std::istringstream is(out);
    for (std::string l; std::getline(is, l);) lines.push_back(l);
    return lines;
}

bool has_word(const std::string &line, char w, double *v = nullptr)
{
    const std::string code = line.substr(0, line.find(';'));
    std::istringstream is(code);
    for (std::string t; is >> t;)
        if (t[0] == w && t.size() > 1 && (std::isdigit(uint8_t(t[1])) || t[1] == '-' || t[1] == '.')) {
            if (v) *v = std::stod(t.substr(1));
            return true;
        }
    return false;
}

} // namespace

TEST_CASE("audit fix: incremental is the default A mode, G91.2 in the header", "[diw_planner][audit]")
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    const auto *o = cfg.option<ConfigOptionEnumsGeneric>("diw_a_axis_mode");
    REQUIRE(o != nullptr);
    CHECK(o->values.front() == 3);
    const auto lines = audit_translate(DIWTranslateOptions());
    REQUIRE(lines.size() > 3);
    CHECK(lines[1].rfind("G91.2", 0) == 0);
    int n912 = 0;
    for (const auto &l : lines) {
        n912 += l.rfind("G91.2", 0) == 0;
        CHECK(l.rfind("G90.2", 0) != 0);
    }
    CHECK(n912 == 1);
    // the v2 modes keep the old header
    const auto old = audit_translate(DIWTranslateOptions(), 0);
    for (const auto &l : old) CHECK(l.rfind("G91.2", 0) != 0);
}

TEST_CASE("audit fix: dA = K_E*dE, dB = ratio*kB*dE per block, negative dB on retraction, nothing on travel", "[diw_planner][audit]")
{
    const double K_E = 3.48;   // diw_k_screw default
    const auto lines = audit_translate(DIWTranslateOptions());
    // blocks carrying A: E 1.0, 0.5 (re-prime), 1.0, 1.0 -> dA = K_E*dE; dB proportional to dE with a fixed factor c
    const double dE_a[4] = { 1.0, 0.5, 1.0, 1.0 };
    int n_a = 0;
    double a_sum = 0., b_sum = 0., c = 0.;
    for (const auto &l : lines) {
        double a = 0., b = 0.;
        const bool has_a = has_word(l, 'A', &a);
        const bool has_b = has_word(l, 'B', &b);
        CHECK(l.find("A0.000") == std::string::npos);
        if (l.rfind("G0 ", 0) == 0) { CHECK_FALSE(has_a); CHECK_FALSE(has_b); }
        if (has_a) {
            REQUIRE(n_a < 4);
            CHECK(has_b);
            if (n_a == 0) c = b;                  // dB for dE = 1.0
            CHECK(a == Catch::Approx(K_E * dE_a[n_a]).margin(1e-3));
            CHECK(b == Catch::Approx(c * dE_a[n_a]).margin(2e-5));
            ++n_a;
            a_sum += a;
        }
        if (has_b) b_sum += b;
    }
    CHECK(n_a == 4);
    CHECK(c == Catch::Approx(0.001225 * 1.01).margin(2e-5));
    CHECK(a_sum == Catch::Approx(K_E * 3.5).margin(1e-3));
    // net E = 1.0 - 0.5 + 0.5 + 1.0 + 1.0 = 3.0: the running dB sum equals ratio*kB*net E (residual carried, no drift)
    CHECK(b_sum == Catch::Approx(c * 3.0).margin(2e-5));
    // the retraction: negative dB, no A
    int n_ret = 0;
    for (const auto &l : lines) {
        double b = 0.;
        if (l.rfind("G1 F", 0) == 0 && has_word(l, 'B', &b) && ! has_word(l, 'A')) {
            ++n_ret;
            CHECK(b == Catch::Approx(-0.5 * c).margin(2e-5));
            CHECK_FALSE(has_word(l, 'A'));
        }
    }
    CHECK(n_ret == 1);
    // the v2 mode still writes A0.000 on the first travel after an extrusion
    const auto old = audit_translate(DIWTranslateOptions(), 0);
    bool any_a0 = false;
    for (const auto &l : old) any_a0 |= l.find("A0.000") != std::string::npos;
    CHECK(any_a0);
}


TEST_CASE("audit fix: G04 before M62 setting", "[diw_planner][audit]")
{
    const auto off = audit_translate(DIWTranslateOptions());
    DIWTranslateOptions on_opts;
    on_opts.m62_pre_dwell = 0.25;
    const auto on = audit_translate(on_opts);
    auto count = [](const std::vector<std::string> &v, const char *pre) { size_t n = 0; for (const auto &l : v) n += l.rfind(pre, 0) == 0; return n; };
    const size_t m62 = count(off, "M62");
    REQUIRE(m62 > 0);
    CHECK(count(on, "M62") == m62);
    CHECK(count(on, "G04") > count(off, "G04"));
    for (size_t i = 0; i < on.size(); ++i)
        if (on[i].rfind("M62", 0) == 0) {
            REQUIRE(i > 0);
            CHECK(on[i - 1].rfind("G04", 0) == 0);   // every M62 follows a dwell
        }
    // default is off: identical to an explicit 0
    DIWTranslateOptions zero;
    zero.m62_pre_dwell = 0.;
    CHECK(audit_translate(zero) == off);
}

TEST_CASE("audit fix: suck-back setting", "[diw_planner][audit]")
{
    const auto off = audit_translate(DIWTranslateOptions());
    for (const auto &l : off) CHECK(l.find("suck-back") == std::string::npos);
    DIWTranslateOptions o;
    o.suckback_e = 0.4;
    o.suckback_f = 200.;
    const auto on = audit_translate(o);
    int n_sb = 0, n_rp = 0;
    for (const auto &l : on) {
        double b = 0.;
        if (l.find("suck-back re-prime") != std::string::npos) { ++n_rp; CHECK_FALSE(has_word(l, 'A')); CHECK(has_word(l, 'B', &b)); CHECK(b > 0.); }
        else if (l.find("suck-back") != std::string::npos) { ++n_sb; CHECK_FALSE(has_word(l, 'A')); CHECK(has_word(l, 'B', &b)); CHECK(b < 0.); }
    }
    CHECK(n_sb >= 2);
    CHECK(n_rp >= 1);
    CHECK(n_rp <= n_sb);
    // A deltas add up to the same total as without suck-back (suck-back moves B only; re-prime adds no A)
    double a_sum_on = 0., a_sum_off = 0.;
    for (const auto &l : on) { double a; if (has_word(l, 'A', &a)) { CHECK(a > 0.); a_sum_on += a; } }
    for (const auto &l : off) { double a; if (has_word(l, 'A', &a)) a_sum_off += a; }
    CHECK(a_sum_on == Catch::Approx(a_sum_off).margin(1e-3));
}


TEST_CASE("audit fix: retract feed clamp setting", "[diw_planner][audit]")
{
    auto retract_f = [](const std::vector<std::string> &v) {
        for (const auto &l : v) {   // the retraction: B only plus a feed, in layer 1
            double f = 0.;
            if (l.rfind("G1 F", 0) == 0 && has_word(l, 'B') && has_word(l, 'F', &f)) return f;
        }
        return -1.;
    };
    const double def = retract_f(audit_translate(DIWTranslateOptions()));
    DIWTranslateOptions o;
    o.retract_f_max = 120.;
    CHECK(def == Catch::Approx(300.));
    CHECK(retract_f(audit_translate(o)) == Catch::Approx(120.));
}

// ---------------------------------------------------------------------------------------------------------------
// Increment 6: added time and powder mass report (diw/POWDER_PLAN.md, GCode/DIWPowderReport).

namespace {

// Translated-text fixture, hand computed (nominal feed, mm/min; pins: powder 5, compaction 6, UV 3; m_dot 6 g/min,
// lag_on = lag_off = 0.15 s). Start (0,0,0).
//   S1 end   G04 0.2                                              relief   0.2
//   S2       Z+6 @6000 0.06, X+10 @3000 0.2, X+30 @3000 0.6       wipe     0.86
//   S3       XY 40 @3000 0.8, UV dwell 5                          UV       5.8
//   S4       Z-4 @6000 0.04, X-30 @3000 0.6, lag 0.15, 22.5 mm @3000 0.45, 7.5 mm @3000 0.15,
//            point window G04 0.35                                powder   1.74
//   S5       Z+2 @6000 0.02, G04 3.0, G04 0.5                     compaction 3.52
//   paste UV G0 60 mm @3000 1.2 each way, dwell 5                 UV      +7.4
//   G04 0.1 outside any stage                                     other    0.1
// Powder windows: 0.15 + 0.45 = 0.6 s -> deposits 0.6 - 0.15 + 0.15 = 0.6 s (the run is 30 mm at 3000 mm/min = 0.6 s);
// point window 0.35 s -> 0.35 s. Deposition 0.95 s, mass 6 / 60 * 0.95 = 0.095 g.
const char *REPORT_FIXTURE =
    "G90\nG0 X0 Y0 Z0 F6000\n"
    "; --- uMEX L2 S1 end: auger stop, pressure-relief dwell ---\nG04 P0.200\n"
    "; --- uMEX L2 S2 nozzle wipe ---\nG0 Z6.000 F6000\nG0 X10.000 Y0.000 F3000\nG1 X40.000 F3000\n"
    "; --- uMEX L2 S3 UV cure at bounding-box centre + offset ---\nG90\nG0 X40.000 Y40.000 F3000\nM62 P3 Q1\nG04 P5.00\nM62 P3 Q0\n"
    "; --- uMEX L2 S4 powder infill F=3000.0 d_early=7.500 ---\nG0 Z2.000 F6000\nG0 X10.000 Y40.000 F3000\nM62 P5 Q1\nG04 P0.150\n"
    "G1 X32.500 Y40.000 F3000\nM62 P5 Q0\nG1 X40.000 Y40.000 F3000\n"
    "; point dwell\nM62 P5 Q1\nG04 P0.350\nM62 P5 Q0\n"
    "; --- uMEX L2 S5 vibration compaction ---\nG0 Z4.000 F6000\nM62 P6 Q1\nG04 P3.000\nM62 P6 Q0\nG04 P0.500\n"
    "; --- uMEX L2 S6 layer advance (next layer sets Z) ---\n;LAYER_CHANGE\n"
    "G90\nG0 X100.000 Y40.000 F3000\nM62 P3 Q1\nG04 P5.00\nM62 P3 Q0\nG0 X40.000 Y40.000 F3000\n"
    "G1 X41.000 Y40.000 F1200\nG04 P0.100\n"
    "; estimated printing time (normal mode) = 10m 51s\n";

DIW::PowderReportParams report_params()
{
    DIW::PowderReportParams p;
    p.powder_pin = 5; p.comp_pin = 6; p.uv_pin = 3;
    p.m_dot = 6.; p.lag_on = 0.15; p.lag_off = 0.15;
    return p;
}

} // namespace

TEST_CASE("report: hand-computed fixture", "[diw_planner][report]")
{
    const DIW::PowderReport r = DIW::analyze_translated_gcode(REPORT_FIXTURE, report_params());
    CHECK(r.part_s[DIW::PowderReport::PressureRelief] == Catch::Approx(0.2).margin(1e-9));
    CHECK(r.part_s[DIW::PowderReport::Wipe]           == Catch::Approx(0.86).margin(1e-9));
    CHECK(r.part_s[DIW::PowderReport::Uv]             == Catch::Approx(13.2).margin(1e-9));
    CHECK(r.part_s[DIW::PowderReport::PowderFeed]     == Catch::Approx(1.74).margin(1e-9));
    CHECK(r.part_s[DIW::PowderReport::Compaction]     == Catch::Approx(3.52).margin(1e-9));
    CHECK(r.part_s[DIW::PowderReport::Other]          == Catch::Approx(0.1).margin(1e-9));
    CHECK(r.total_s() == Catch::Approx(19.62).margin(1e-9));
    CHECK(r.dwell_s   == Catch::Approx(14.3).margin(1e-9));
    CHECK(r.move_s    == Catch::Approx(5.32).margin(1e-9));
    CHECK(r.powder_layers == 1);
    CHECK(r.powder_windows == 2);
    CHECK(r.powder_deposition_s == Catch::Approx(0.95).margin(1e-9));
    CHECK(r.powder_mass_g == Catch::Approx(0.095).margin(1e-9));
    CHECK(r.slicer_estimate_s == Catch::Approx(651.).margin(1e-9));
    const std::string footer = DIW::report_footer(r);
    CHECK(footer.find("; powder mass = 0.0950 g\n") != std::string::npos);
    CHECK(footer.find("; added time = 19.620 s, 20s\n") != std::string::npos);
    CHECK(footer.find("slicer estimate = 10m 51s, with added time = 11m 11s") != std::string::npos);
}

TEST_CASE("report: no powder means zero powder mass", "[diw_planner][report]")
{
    // a paste job: start-up UV flicker, a positioned UV cure, no powder stage, no feeder window
    const char *paste =
        "G90\nM62 P3 Q1 ; UV Self-Check ON\nG04 P3.00 ; Hold for 3 seconds\nM62 P3 Q0 ; UV Self-Check OFF\n"
        "G1 X10 Y10 F1200\nG1 X20 Y10 B0.01 A1.0\n"
        "G90\nG0 X20.000 Y90.000 F3000\nM62 P3 Q1\nG04 P5.00\nM62 P3 Q0\nG0 X20.000 Y10.000 F3000\n";
    DIW::PowderReportParams p = report_params();
    p.powder_pin = p.comp_pin = -1;   // a paste job has neither
    const DIW::PowderReport r = DIW::analyze_translated_gcode(paste, p);
    CHECK(r.powder_mass_g == 0.);
    CHECK(r.powder_deposition_s == 0.);
    CHECK(r.powder_windows == 0);
    CHECK(r.powder_layers == 0);
    CHECK(r.part_s[DIW::PowderReport::PowderFeed] == 0.);
    // UV: 3 s flicker (no move around it), 80 mm out and 80 mm back @3000 = 1.6 s each way, 5 s dwell
    CHECK(r.part_s[DIW::PowderReport::Uv] == Catch::Approx(3. + 5. + 1.6 + 1.6).margin(1e-9));
    CHECK(r.total_s() == Catch::Approx(11.2).margin(1e-9));
    CHECK(DIW::report_footer(r).find("; powder mass = 0.0000 g\n") != std::string::npos);
    // an empty file reports zero everything
    const DIW::PowderReport e = DIW::analyze_translated_gcode("", report_params());
    CHECK(e.total_s() == 0.);
    CHECK(e.powder_mass_g == 0.);
}

TEST_CASE("report: the parts sum to the total", "[diw_planner][report]")
{
    const DIW::PowderReport r = DIW::analyze_translated_gcode(REPORT_FIXTURE, report_params());
    double parts = 0.;
    for (int i = 0; i < DIW::PowderReport::PartCount; ++i)
        parts += r.part_s[i];
    CHECK(parts == Catch::Approx(r.total_s()).margin(1e-12));
    CHECK(r.dwell_s + r.move_s == Catch::Approx(r.total_s()).margin(1e-9));
    // the footer prints the same numbers: the six listed parts add up to the listed total (3-decimal rounding)
    double listed = 0.;
    for (const std::string &l : DIW::report_summary_lines(r))
        if (l.rfind("added time by part:", 0) == 0)
            for (size_t sp = l.find(" s"); sp != std::string::npos; sp = l.find(" s", sp + 1)) {
                const size_t b = l.rfind(' ', sp - 1);
                listed += std::stod(l.substr(b + 1, sp - b - 1));
            }
    CHECK(listed == Catch::Approx(r.total_s()).margin(0.003));
}

TEST_CASE("report: malformed M62 or G04 fails with the line number", "[diw_planner][report]")
{
    CHECK_THROWS_WITH(DIW::analyze_translated_gcode("G90\nM62 P3\n", report_params()), Catch::Matchers::ContainsSubstring("line 2"));
    CHECK_THROWS_WITH(DIW::analyze_translated_gcode("G04\n", report_params()), Catch::Matchers::ContainsSubstring("line 1"));
}

TEST_CASE("report: footer is comments only and leaves the translation bytes unchanged", "[diw_planner][report]")
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
    const std::string plain = diw_translate_gcode(AUDIT_SRC, cfg, DIWTranslateOptions());
    DIWTranslateOptions opts;
    opts.append_report = true;
    std::vector<std::string> lines;
    opts.report_lines = &lines;
    const std::string with = diw_translate_gcode(AUDIT_SRC, cfg, opts);
    REQUIRE(with.size() > plain.size());
    CHECK(with.compare(0, plain.size(), plain) == 0);   // the motion bytes are the translation, byte for byte
    std::istringstream is(with.substr(plain.size()));
    int n = 0;
    for (std::string l; std::getline(is, l); ++n)
        CHECK(l.rfind("; ", 0) == 0);
    CHECK(n >= 4);
    CHECK(! lines.empty());
    CHECK(with.find("; powder mass = 0.0000 g\n") != std::string::npos);   // a paste job has no powder
    // no bracket or parenthesis, so PlanetCNC reads the lines as plain comments
    CHECK(with.substr(plain.size()).find_first_of("[]()") == std::string::npos);
    // off by default: no footer
    CHECK(plain.find("uMEX report") == std::string::npos);
}

TEST_CASE("increment 7: new config keys default to off", "[diw_planner][config]")
{
    const DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    for (const char *key : { "diw_m62_pre_dwell", "diw_suckback_length", "diw_suckback_feed" }) {
        const ConfigOptionFloat *o = cfg.option<ConfigOptionFloat>(key);
        REQUIRE(o != nullptr);
        CHECK(o->value == 0.);
    }
}

TEST_CASE("increment 7: the report carries the added time per layer and the estimate marker", "[diw_planner][report]")
{
    const DIW::PowderReport r = DIW::analyze_translated_gcode(REPORT_FIXTURE, report_params());
    double sum = 0.;
    for (double v : r.layer_added_s)
        sum += v;
    CHECK(sum == Catch::Approx(r.total_s()).margin(1e-9));
    CHECK(r.estimate_includes_s < 0.);

    // The slicer's estimate already contains 19.62 s: the report gives back the estimate without it.
    const std::string with_marker = std::string(REPORT_FIXTURE) + DIW::estimate_marker(r.total_s());
    const DIW::PowderReport m = DIW::analyze_translated_gcode(with_marker, report_params());
    CHECK(m.estimate_includes_s == Catch::Approx(r.total_s()).margin(1e-3));
    CHECK(m.slicer_estimate_s == Catch::Approx(651. - r.total_s()).margin(1e-3));
    CHECK(m.total_s() == Catch::Approx(r.total_s()).margin(1e-9));
}
