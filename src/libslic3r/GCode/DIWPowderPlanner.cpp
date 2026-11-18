// u-MEX powder planner, slicer-core side. See DIWPowderPlanner.hpp and diw/PLANNER_DESIGN.md section 7.
// Floating point follows v2 operation by operation where it matters (hatch rows, chaining, lengths, dwell);
// no FMA contraction (/fp:precise and the pragma below).
#ifdef _MSC_VER
#pragma fp_contract(off)
#endif

#include "DIWPowderPlanner.hpp"

#include "DIWPyCompat.hpp"
#include "../BoundingBox.hpp"
#include "../ClipperUtils.hpp"

#include <clipper2/clipper.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <array>
#include <tuple>
#include <utility>

namespace Slic3r {
namespace DIW {

namespace C2 = Clipper2Lib;

namespace {

constexpr double PI_D      = 3.14159265358979323846;
constexpr double ARC_THETA = 2.0 * PI_D / SECTOR_STEPS_PER_360;
constexpr double C_OPEN_NM = 100.0;            // narrow is taken of C opened by this much (P5): strips < 200 nm dropped
constexpr double OPENING_SIMPLIFY_NM = 10.0;   // Douglas-Peucker tolerance of Omega before the mitre opening (P5)

// ------------------------------------------------------------------------------------------------ grid helpers

inline double area_mm2(const ExPolygon &ex) { return ex.area() * 1e-12; }

C2::Paths64 to_paths(const ExPolygons &ee)
{
    C2::Paths64 out;
    for (const ExPolygon &e : ee) {
        C2::Path64 p;
        p.reserve(e.contour.points.size());
        for (const Point &pt : e.contour.points) p.emplace_back(pt.x(), pt.y());
        out.emplace_back(std::move(p));
        for (const Polygon &h : e.holes) {
            C2::Path64 q;
            q.reserve(h.points.size());
            for (const Point &pt : h.points) q.emplace_back(pt.x(), pt.y());
            out.emplace_back(std::move(q));
        }
    }
    return out;
}

Polygons to_polygons(const C2::Paths64 &pp)
{
    Polygons out;
    out.reserve(pp.size());
    for (const C2::Path64 &p : pp) {
        Polygon poly;
        poly.points.reserve(p.size());
        for (const C2::Point64 &pt : p) poly.points.emplace_back(coord_t(pt.x), coord_t(pt.y));
        out.emplace_back(std::move(poly));
    }
    return out;
}

C2::Paths64 to_paths(const Polygons &pp)
{
    C2::Paths64 out;
    out.reserve(pp.size());
    for (const Polygon &poly : pp) {
        C2::Path64 p;
        p.reserve(poly.points.size());
        for (const Point &pt : poly.points) p.emplace_back(pt.x(), pt.y());
        out.emplace_back(std::move(p));
    }
    return out;
}

Polygon to_polygon(const C2::Path64 &p)
{
    Polygon poly;
    poly.points.reserve(p.size());
    for (const C2::Point64 &pt : p) poly.points.emplace_back(coord_t(pt.x), coord_t(pt.y));
    return poly;
}

// PolyTree64 -> ExPolygons (outer paths counter-clockwise, holes clockwise, islands in holes as new ExPolygons).
void tree_to_expolygons(const C2::PolyPath64 &node, ExPolygons &out)
{
    for (size_t i = 0; i < node.Count(); ++i) {
        const C2::PolyPath64 *outer = node.Child(i);
        ExPolygon e;
        e.contour = to_polygon(outer->Polygon());
        for (size_t j = 0; j < outer->Count(); ++j)
            e.holes.push_back(to_polygon(outer->Child(j)->Polygon()));
        out.push_back(std::move(e));
        for (size_t j = 0; j < outer->Count(); ++j)
            tree_to_expolygons(*outer->Child(j), out);
    }
}

// Clipper2 booleans (exact on the integer grid; faster than the Clipper 1 based ClipperUtils for the many
// overlapping band pieces). Collinear vertices are removed.
ExPolygons c2_boolean(C2::ClipType ct, const C2::Paths64 &subj, const C2::Paths64 &clip, C2::FillRule fr)
{
    C2::Clipper64 c;
    c.PreserveCollinear(false);
    c.AddSubject(subj);
    if (! clip.empty())
        c.AddClip(clip);
    C2::PolyTree64 tree;
    c.Execute(ct, fr, tree);
    ExPolygons out;
    tree_to_expolygons(tree, out);
    return out;
}
ExPolygons c2_union(const Polygons &pp) { return c2_boolean(C2::ClipType::Union, to_paths(pp), {}, C2::FillRule::NonZero); }
ExPolygons c2_diff(const ExPolygons &a, const ExPolygons &b)
{
    return c2_boolean(C2::ClipType::Difference, to_paths(a), to_paths(b), C2::FillRule::NonZero);
}
ExPolygons c2_intersection(const ExPolygons &a, const ExPolygons &b)
{
    return c2_boolean(C2::ClipType::Intersection, to_paths(a), to_paths(b), C2::FillRule::NonZero);
}

// Clipper2 offset of closed polygons (used for the opening and the fallback clip pad, as in the design).
ExPolygons c2_offset(const ExPolygons &in, double delta, C2::JoinType jt, double miter_limit, double arc_tolerance)
{
    if (in.empty())
        return {};
    C2::ClipperOffset co(miter_limit, arc_tolerance, /*preserve_collinear*/ false, /*reverse*/ false);
    co.AddPaths(to_paths(in), jt, C2::EndType::Polygon);
    C2::PolyTree64 tree;
    co.Execute(delta, tree);
    ExPolygons out;
    tree_to_expolygons(tree, out);
    return out;
}

// ------------------------------------------------------------------------------------------------ dilation pieces

inline Point rnd(const Vec2d &v) { return Point(coord_t(std::llround(v.x())), coord_t(std::llround(v.y()))); }
inline Vec2d unit_at(double a) { return Vec2d(std::cos(a), std::sin(a)); }

void push_ccw(Polygons &out, Polygon &&poly)
{
    // A piece that rounding made degenerate or clockwise would subtract under NonZero: drop it (its true
    // area is below one grid unit squared).
    if (poly.points.size() >= 3 && poly.area() > 0.)
        out.emplace_back(std::move(poly));
}

// The outer vertices of the polygon circumscribed about the arc of radius r around v from angle a0,
// counter-clockwise by sweep: its edges are tangent to the circle at a0, at every multiple of 2 pi / N strictly
// inside the arc, and at a0 + sweep, and each vertex is the crossing of two consecutive tangents (at the middle
// angle, at r / cos(half the gap)). The gaps are at most 2 pi / N, so the vertices lie within r (1/cos(pi/N) - 1)
// of the circle. The tangent directions are on a fixed global grid (stage 5c; before, the arc was cut into equal
// steps from a0): two arcs of nearly the same centre and radius, such as the band of a native wall and of its
// G-code text, then have parallel edges and cross O(1) times instead of once per step, which made their union slow;
// and nm noise in the input moves only the end vertices, not the phase of the whole arc.
template<class Out> void circumscribed_arc(const Vec2d &v, double r, double a0, double sweep, Out &&out)
{
    if (! (sweep > 1e-12))
        return;
    const double a1   = a0 + sweep;
    double       prev = a0;
    for (double k = std::floor(a0 / ARC_THETA) + 1.;; k += 1.) {
        const double t = k * ARC_THETA;
        if (! (t < a1 - 1e-12))
            break;
        if (t > prev + 1e-12) {
            out(v + (r / std::cos(0.5 * (t - prev))) * unit_at(0.5 * (prev + t)));
            prev = t;
        }
    }
    out(v + (r / std::cos(0.5 * (a1 - prev))) * unit_at(0.5 * (prev + a1)));
}

// Circular sector around v from angle a0, counter-clockwise by sweep, with radius r: v, the arc start, the
// circumscribed vertices (circumscribed_arc), the arc end. Contains the exact sector.
void add_sector(Polygons &out, const Vec2d &v, double r, double a0, double sweep)
{
    if (! (sweep > 1e-12))
        return;
    Polygon poly;
    poly.points.reserve(size_t(sector_steps(sweep)) + 4);
    poly.points.emplace_back(rnd(v));
    poly.points.emplace_back(rnd(v + r * unit_at(a0)));
    circumscribed_arc(v, r, a0, sweep, [&poly](const Vec2d &q) { poly.points.emplace_back(rnd(q)); });
    poly.points.emplace_back(rnd(v + r * unit_at(a0 + sweep)));
    push_ccw(out, std::move(poly));
}

void add_disc(Polygons &out, const Vec2d &v, double r)
{
    const int    m = SECTOR_STEPS_PER_360;
    const double R = r / std::cos(0.5 * ARC_THETA);
    Polygon poly;
    poly.points.reserve(m);
    for (int i = 0; i < m; ++i)
        poly.points.emplace_back(rnd(v + R * unit_at((i + 0.5) * ARC_THETA)));
    push_ccw(out, std::move(poly));
}

// Rectangle of half width r around a->b, extended by one grid unit at both ends.
void add_rect(Polygons &out, const Vec2d &a, const Vec2d &b, double r)
{
    Vec2d d = b - a;
    d /= d.norm();
    const Vec2d nl(-d.y(), d.x());
    const Vec2d a2 = a - d, b2 = b + d;
    Polygon poly;
    poly.points = { rnd(a2 - r * nl), rnd(b2 - r * nl), rnd(b2 + r * nl), rnd(a2 + r * nl) };
    push_ccw(out, std::move(poly));
}

// Pieces whose union contains the polyline dilated by r_u (grid units, the caller adds the rounding margin):
// one rectangle per segment, a sector on the convex side of every interior vertex, half-disc caps at the
// ends of an open polyline. Every point of the exact sum has its nearest polyline point either inside a
// segment (rectangle), at a vertex within that vertex's normal cone (sector) or at an end (cap).
void add_sausage(Polygons &out, const Points &in, bool closed, double r_u)
{
    Points p;
    p.reserve(in.size());
    for (const Point &q : in)
        if (p.empty() || q != p.back())
            p.push_back(q);
    if (closed && p.size() > 1 && p.front() == p.back())
        p.pop_back();
    if (p.empty())
        return;
    if (p.size() == 1) {
        add_disc(out, p.front().cast<double>(), r_u);
        return;
    }
    if (p.size() == 2)
        closed = false;
    const size_t n    = p.size();
    const size_t nseg = closed ? n : n - 1;
    auto P   = [&](size_t i) { return p[i % n].cast<double>(); };
    auto dir = [&](size_t i) { Vec2d d = P(i + 1) - P(i); return Vec2d(d / d.norm()); };
    for (size_t i = 0; i < nseg; ++i)
        add_rect(out, P(i), P(i + 1), r_u);
    const size_t v0 = closed ? 0 : 1, v1 = closed ? n : n - 1;
    for (size_t i = v0; i < v1; ++i) {
        const Vec2d  d1  = dir((i + n - 1) % n), d2 = dir(i);
        const double crs = d1.x() * d2.y() - d1.y() * d2.x();
        const double dot = d1.x() * d2.x() + d1.y() * d2.y();
        const double phi = std::atan2(crs, dot);
        if (phi > 0.)       // left turn: the gap is on the right side, from d1's right normal counter-clockwise
            add_sector(out, P(i), r_u, std::atan2(-d1.x(), d1.y()), phi);
        else if (phi < 0.)  // right turn: the gap is on the left side, from d2's left normal counter-clockwise
            add_sector(out, P(i), r_u, std::atan2(d2.x(), -d2.y()), -phi);
    }
    if (! closed) {
        const Vec2d ds = dir(0), de = dir(n - 2);
        add_sector(out, P(0), r_u, std::atan2(ds.x(), -ds.y()), PI_D);        // from the left normal, round the back
        add_sector(out, P(n - 1), r_u, std::atan2(-de.x(), de.y()), PI_D);    // from the right normal, round the front
    }
}

// The edges of a polygon set (grid units), bucketed in horizontal strips, to test whether an arc cap of a band
// dilation lies outside it. Only the part of the band inside S matters (C = S - band, Omega = S - grown band or
// the erosion of S minus it), so a cap with no point in S may be replaced by its chord: S - band is the same set,
// with the same boundary edges and crossings inside S (PLANNER_DESIGN.md "Deviations", stage 5c). This drops most
// arc vertices on the outer side of the outer walls, whose union with the other polylines dominated P3 / P4.
class OutsideTest
{
public:
    explicit OutsideTest(const ExPolygons &ex)
    {
        double y0 = std::numeric_limits<double>::max(), y1 = -y0;
        auto add = [&](const Points &pts) {
            for (size_t i = 0, n = pts.size(); i < n; ++i) {
                const Vec2d a = pts[i].cast<double>(), b = pts[(i + 1) % n].cast<double>();
                m_edges.push_back({ a, b });
                y0 = std::min({ y0, a.y(), b.y() });
                y1 = std::max({ y1, a.y(), b.y() });
            }
        };
        for (const ExPolygon &e : ex) {
            add(e.contour.points);
            for (const Polygon &h : e.holes) add(h.points);
        }
        if (m_edges.empty())
            return;
        m_y0 = y0;
        m_n  = std::max<size_t>(1, std::min<size_t>(4096, m_edges.size() / 4));
        m_h  = std::max(1., (y1 - y0) / double(m_n));
        m_strips.assign(m_n, {});
        for (size_t i = 0; i < m_edges.size(); ++i) {
            const auto [s0, s1] = strips(std::min(m_edges[i][0].y(), m_edges[i][1].y()), std::max(m_edges[i][0].y(), m_edges[i][1].y()));
            for (size_t k = s0; k <= s1; ++k) m_strips[k].push_back(uint32_t(i));
        }
    }

    // True when the convex counter-clockwise polygon q (an arc cap: the arc start, the circumscribed vertices, the
    // arc end, closed by the chord) has no point of the set: no edge of the set within MARGIN of it and one of
    // its points outside the set (even-odd).
    bool outside(const std::vector<Vec2d> &q) const
    {
        if (m_edges.empty())
            return true;
        double x0 = q[0].x(), x1 = x0, y0 = q[0].y(), y1 = y0;
        for (const Vec2d &v : q) {
            x0 = std::min(x0, v.x()); x1 = std::max(x1, v.x());
            y0 = std::min(y0, v.y()); y1 = std::max(y1, v.y());
        }
        x0 -= MARGIN; x1 += MARGIN; y0 -= MARGIN; y1 += MARGIN;
        const auto [s0, s1] = strips(y0, y1);
        for (size_t k = s0; k <= s1; ++k)
            for (const uint32_t i : m_strips[k]) {
                const Vec2d &a = m_edges[i][0], &b = m_edges[i][1];
                if (std::max(a.x(), b.x()) < x0 || std::min(a.x(), b.x()) > x1 || std::max(a.y(), b.y()) < y0 ||
                    std::min(a.y(), b.y()) > y1)
                    continue;
                if (inside_convex(q, a))
                    return false;
                for (size_t j = 0, n = q.size(); j < n; ++j)
                    if (seg_seg_dist(a, b, q[j], q[(j + 1) % n]) < MARGIN)
                        return false;
            }
        // no edge near the cap: the whole cap is on one side; test its first point (even-odd ray to +x)
        const Vec2d &p  = q.front();
        bool         in = false;
        const size_t k  = strips(p.y(), p.y()).first;
        for (const uint32_t i : m_strips[k]) {
            const Vec2d &a = m_edges[i][0], &b = m_edges[i][1];
            if ((a.y() > p.y()) != (b.y() > p.y()) && p.x() < a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y()))
                in = ! in;
        }
        return ! in;
    }

private:
    static constexpr double MARGIN = 4.;   // grid units
    std::vector<std::array<Vec2d, 2>>  m_edges;
    std::vector<std::vector<uint32_t>> m_strips;
    double                             m_y0 = 0., m_h = 1.;
    size_t                             m_n  = 0;

    std::pair<size_t, size_t> strips(double ya, double yb) const
    {
        auto idx = [&](double y) {
            const double t = std::floor((y - m_y0) / m_h);
            return size_t(std::clamp(t, 0., double(m_n - 1)));
        };
        return { idx(ya), idx(yb) };
    }
    static bool inside_convex(const std::vector<Vec2d> &q, const Vec2d &p)
    {
        for (size_t j = 0, n = q.size(); j < n; ++j) {
            const Vec2d e = q[(j + 1) % n] - q[j], d = p - q[j];
            if (e.x() * d.y() - e.y() * d.x() < 0.)
                return false;
        }
        return true;
    }
    static double seg_pt(const Vec2d &p, const Vec2d &a, const Vec2d &b)
    {
        const Vec2d  ab = b - a;
        const double l2 = ab.squaredNorm();
        const double t  = l2 > 0. ? std::clamp((p - a).dot(ab) / l2, 0., 1.) : 0.;
        return (p - (a + t * ab)).norm();
    }
    static double seg_seg_dist(const Vec2d &a, const Vec2d &b, const Vec2d &c, const Vec2d &d)
    {
        auto orient = [](const Vec2d &o, const Vec2d &u, const Vec2d &w) {
            return (u.x() - o.x()) * (w.y() - o.y()) - (u.y() - o.y()) * (w.x() - o.x());
        };
        const double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b);
        if (((o1 > 0.) != (o2 > 0.)) && ((o3 > 0.) != (o4 > 0.)))
            return 0.;
        return std::min({ seg_pt(a, c, d), seg_pt(b, c, d), seg_pt(c, a, b), seg_pt(d, a, b) });
    }
};

// One polyline to dilate: a band / paste centreline or a boundary ring of the eroded set (grid units, local frame).
struct Sausage
{
    Points             pts;
    bool               closed = false;
    double             r_u    = 0.;        // radius in grid units, with the margin for the rounding of the pieces
    const OutsideTest *drop   = nullptr;   // arc caps outside this set are replaced by their chords
};

// The dilation of one polyline by r_u as one Clipper2 Positive union of its raw offset path(s), the way
// ClipperOffset builds a round offset, but with the arcs of add_sausage: along each side, a straight edge at r_u
// per segment, at a convex vertex the circumscribed vertices of the sector add_sausage puts there (from the end of
// one edge's offset to the start of the next), at a concave vertex the offset end, the vertex and the next offset
// start (Clipper2's three-point concave join; the backward loop it forms is removed by the Positive union), and at
// the ends of an open polyline the circumscribed half-disc caps. A closed polyline gives two raw paths (the right
// side of the loop and the right side of the reversed loop), an open one a single path (right side forward, end cap,
// left side backward, start cap), a single point the circumscribed disc. The set is the union of add_sausage's
// pieces (the same arc vertices; straight edges at r_u instead of r_u with ends extended by one grid unit), so it
// contains the exact disk sum and lies within r (1 / cos(pi / N) - 1) + 1 grid unit of it; the union of one path
// pair has far fewer crossings than the union of one rectangle and one sector per vertex (PLANNER_DESIGN.md
// "Deviations", stage 5c). At a (near) reversal (cos <= -0.999) both sides get the cap arc around the turning
// point, as ClipperOffset does: a three-point join there would leave the cap to the other side, which an exact
// reversal (cross product -0 on both sides) does not get either.
ExPolygons dilate_polyline(const Sausage &sg)
{
    Points p;
    p.reserve(sg.pts.size());
    for (const Point &q : sg.pts)
        if (p.empty() || q != p.back())
            p.push_back(q);
    bool closed = sg.closed;
    if (closed && p.size() > 1 && p.front() == p.back())
        p.pop_back();
    if (p.empty() || ! (sg.r_u > 0.))
        return {};
    const double r = sg.r_u;
    if (p.size() == 1) {
        Polygons disc;
        add_disc(disc, p.front().cast<double>(), r);
        return c2_union(disc);
    }
    if (p.size() == 2)
        closed = false;
    C2::Paths64 raw;
    C2::Path64 *cur = nullptr;
    auto emit = [&](const Vec2d &v) {
        const Point q = rnd(v);
        if (cur->empty() || cur->back().x != q.x() || cur->back().y != q.y())
            cur->emplace_back(q.x(), q.y());
    };
    // arc around v from angle a0, counter-clockwise by sweep: the circumscribed vertices of add_sector, then the end
    std::vector<Vec2d> cap;
    auto arc = [&](const Vec2d &v, double a0, double sweep) {
        if (! (sweep > 1e-12))
            return;
        if (sg.drop != nullptr && sweep <= PI_D) {
            // the cap (arc start, circumscribed vertices, arc end; convex for a sweep <= pi) outside S: the chord
            cap.clear();
            cap.push_back(v + r * unit_at(a0));
            circumscribed_arc(v, r, a0, sweep, [&cap](const Vec2d &w) { cap.push_back(w); });
            cap.push_back(v + r * unit_at(a0 + sweep));
            if (cap.size() > 2 && sg.drop->outside(cap)) {
                emit(cap.back());
                return;
            }
        }
        circumscribed_arc(v, r, a0, sweep, emit);
        emit(v + r * unit_at(a0 + sweep));
    };
    // the right side of q (consecutive distinct points), joins at the interior vertices (at every vertex when the
    // side is closed); an open side starts at the first segment's offset start and ends at the last one's end
    auto side = [&](const std::vector<Vec2d> &q, bool cyclic) {
        const size_t n    = q.size();
        const size_t nseg = cyclic ? n : n - 1;
        std::vector<Vec2d> d(nseg), nr(nseg);
        for (size_t i = 0; i < nseg; ++i) {
            d[i] = q[(i + 1) % n] - q[i];
            d[i] /= d[i].norm();
            nr[i] = Vec2d(d[i].y(), -d[i].x());
        }
        const size_t v0 = cyclic ? 0 : 1, v1 = cyclic ? n : n - 1;
        if (! cyclic)
            emit(q[0] + r * nr[0]);
        for (size_t j = v0; j < v1; ++j) {
            const size_t k   = (j + nseg - 1) % nseg;
            const double crs = d[k].x() * d[j].y() - d[k].y() * d[j].x();
            const double dot = d[k].x() * d[j].x() + d[k].y() * d[j].y();
            const double phi = std::atan2(crs, dot);
            emit(q[j] + r * nr[k]);
            if (dot <= -0.999)       // (near) reversal: the cap around the turning point on both sides (Clipper2 #593)
                arc(q[j], std::atan2(nr[k].y(), nr[k].x()), phi > 0. ? phi : 2. * PI_D + phi);
            else if (phi > 0.)       // left turn: the right side is convex
                arc(q[j], std::atan2(nr[k].y(), nr[k].x()), phi);
            else if (phi < 0.) {     // right turn: concave, Clipper2's three-point join
                emit(q[j]);
                emit(q[j] + r * nr[j]);
            }
        }
        if (! cyclic)
            emit(q[n - 1] + r * nr[nseg - 1]);
        return nr;
    };
    std::vector<Vec2d> fwd;
    fwd.reserve(p.size());
    for (const Point &q : p) fwd.push_back(q.cast<double>());
    const std::vector<Vec2d> bwd(fwd.rbegin(), fwd.rend());
    if (closed) {
        cur = &raw.emplace_back();
        side(fwd, true);
        cur = &raw.emplace_back();
        side(bwd, true);
    } else {
        cur = &raw.emplace_back();
        const std::vector<Vec2d> nf = side(fwd, false);
        arc(fwd.back(), std::atan2(nf.back().y(), nf.back().x()), PI_D);   // end cap, from the right normal
        const std::vector<Vec2d> nb = side(bwd, false);
        arc(bwd.back(), std::atan2(nb.back().y(), nb.back().x()), PI_D);   // start cap
    }
    for (C2::Path64 &q : raw)
        while (q.size() > 1 && q.front() == q.back())
            q.pop_back();
    return c2_boolean(C2::ClipType::Union, raw, {}, C2::FillRule::Positive);
}

// Union of the dilated polylines: every polyline is dilated on its own (in parallel above PAR_MIN_POINTS input
// vertices), then the results are united. Each result has winding 0 / 1 (holes clockwise), so the NonZero union of
// the results is the union of the dilations. Deterministic: the order is fixed.
constexpr size_t PAR_MIN_POINTS = 0;

size_t count_points(const std::vector<Sausage> &ss)
{
    size_t n = 0;
    for (const Sausage &s : ss) n += s.pts.size();
    return n;
}

// A closed polyline as a canonical vertex cycle: repeated and collinear (straight-through, exact integer test)
// vertices removed, started at the smallest vertex, counter-clockwise. Two closed polylines with the same cycle are
// the same closed curve.
Points closed_cycle(const Points &in)
{
    Points p;
    for (const Point &q : in)
        if (p.empty() || q != p.back())
            p.push_back(q);
    while (p.size() > 1 && p.front() == p.back())
        p.pop_back();
    for (bool changed = true; changed && p.size() >= 3;) {
        changed = false;
        for (size_t i = 0; i < p.size() && p.size() >= 3; ++i) {
            const Point &a = p[(i + p.size() - 1) % p.size()], &b = p[i], &c = p[(i + 1) % p.size()];
            const int64_t ux = int64_t(b.x()) - a.x(), uy = int64_t(b.y()) - a.y();
            const int64_t vx = int64_t(c.x()) - b.x(), vy = int64_t(c.y()) - b.y();
            if (ux * vy - uy * vx == 0 && ux * vx + uy * vy > 0) {
                p.erase(p.begin() + i);
                changed = true;
                --i;
            }
        }
    }
    if (p.size() < 3)
        return {};
    double a2 = 0.;
    for (size_t i = 0; i < p.size(); ++i)
        a2 += double(p[i].x()) * double(p[(i + 1) % p.size()].y()) - double(p[(i + 1) % p.size()].x()) * double(p[i].y());
    if (a2 < 0.)
        std::reverse(p.begin(), p.end());
    std::rotate(p.begin(), std::min_element(p.begin(), p.end(), [](const Point &u, const Point &v) {
                    return std::make_pair(u.x(), u.y()) < std::make_pair(v.x(), v.y());
                }), p.end());
    return p;
}

ExPolygons union_sausages(const std::vector<Sausage> &in_ss)
{
    // A closed polyline that is the same closed curve as another one with a radius at least as large adds nothing:
    // the dilations of one curve are nested in the radius (rectangles, sectors and caps all grow with it; a cap of
    // the larger one replaced by its chord lies outside S, and so does the part of the smaller one's cap beyond
    // that chord). This drops the G-code text copy of a native wall loop when the text reproduces it exactly
    // (straight walls on a micrometre grid), whose union with the loop was a large share of P3 / P4
    // (PLANNER_DESIGN.md "Deviations", stage 5c).
    std::vector<Sausage> ss;
    {
        std::vector<Points> cyc(in_ss.size());
        std::vector<char>   keep(in_ss.size(), 1);
        for (size_t i = 0; i < in_ss.size(); ++i)
            if (in_ss[i].closed)
                cyc[i] = closed_cycle(in_ss[i].pts);
        for (size_t i = 0; i < in_ss.size(); ++i) {
            if (cyc[i].empty() || ! keep[i])
                continue;
            for (size_t j = 0; j < in_ss.size(); ++j)
                if (j != i && keep[j] && ! cyc[j].empty() && in_ss[j].drop == in_ss[i].drop && cyc[j] == cyc[i] &&
                    (in_ss[j].r_u > in_ss[i].r_u || (in_ss[j].r_u == in_ss[i].r_u && j < i))) {
                    keep[i] = 0;
                    break;
                }
        }
        for (size_t i = 0; i < in_ss.size(); ++i)
            if (keep[i])
                ss.push_back(in_ss[i]);
    }
    std::vector<ExPolygons> parts(ss.size());
    auto one = [&ss, &parts](size_t i) { parts[i] = dilate_polyline(ss[i]); };
    if (count_points(ss) < PAR_MIN_POINTS)
        for (size_t i = 0; i < ss.size(); ++i) one(i);
    else
        tbb::parallel_for(tbb::blocked_range<size_t>(0, ss.size()), [&one](const tbb::blocked_range<size_t> &r) {
            for (size_t i = r.begin(); i < r.end(); ++i) one(i);
        });
    if (parts.size() == 1)
        return std::move(parts.front());
    Polygons all;
    for (ExPolygons &q : parts)
        append(all, to_polygons(std::move(q)));
    return all.empty() ? ExPolygons() : c2_union(all);
}

// f1 and f2, in parallel when par is set.
template<class F1, class F2> void run2(bool par, F1 &&f1, F2 &&f2)
{
    if (par)
        tbb::parallel_invoke(f1, f2);
    else {
        f1();
        f2();
    }
}

} // namespace

// The polygons dilated by d (grid units) with mitre joins limited the way GEOS limits them (buffer(join_style=
// "mitre", mitre_limit=limit)): a mitre longer than limit * d is cut off square to the bisector at limit * d from
// the vertex, so the result is continuous in the corner angle (Clipper2 squares such corners at 1 * d instead).
// Built per ExPolygon as one Clipper2 Positive union of the raw offset path of each ring, the way ClipperOffset
// builds a mitre offset: per edge the edge moved out by d, at a convex vertex the mitre point (or the two cut points),
// at a reflex vertex the offset end, the vertex and the next offset start (Clipper2's three-point concave join).
// That is the set of mitre_dilate_pieces (the union of the polygon, one rectangle per edge and one wedge per convex
// vertex) with the same vertices, and much cheaper to unite (PLANNER_DESIGN.md "Deviations", stage 5c).
ExPolygons mitre_dilate(const ExPolygons &in, double d, double limit)
{
    std::vector<ExPolygons> parts(in.size());
    for (size_t ie = 0; ie < in.size(); ++ie) {
        C2::Paths64 raw;
        C2::Path64 *cur = nullptr;
        auto emit = [&](const Vec2d &v) {
            const Point q = rnd(v);
            if (cur->empty() || cur->back().x != q.x() || cur->back().y != q.y())
                cur->emplace_back(q.x(), q.y());
        };
        auto ring = [&](const Points &rp) {
            Points p;
            for (const Point &q : rp)
                if (p.empty() || q != p.back())
                    p.push_back(q);
            while (p.size() > 1 && p.front() == p.back())
                p.pop_back();
            const size_t n = p.size();
            if (n < 3)
                return;
            auto P = [&](size_t i) { return p[i % n].cast<double>(); };
            cur = &raw.emplace_back();
            for (size_t i = 0; i < n; ++i) {
                const Vec2d v = P(i);
                Vec2d       d1 = v - P(i + n - 1), d2 = P(i + 1) - v;
                d1 /= d1.norm();
                d2 /= d2.norm();
                const Vec2d n1(d1.y(), -d1.x()), n2(d2.y(), -d2.x());   // material on the left: outward = right normal
                const double crs = d1.x() * d2.y() - d1.y() * d2.x();
                if (crs <= 0.) {
                    // reflex or straight: the edge offsets meet behind the vertex
                    emit(v + d * n1);
                    if (crs < 0.) {
                        emit(v);
                        emit(v + d * n2);
                    }
                    continue;
                }
                Vec2d        bis = n1 + n2;
                const double bl  = bis.norm();
                if (! (bl > 1e-12)) {
                    emit(v + d * n1);
                    emit(v + d * n2);
                    continue;
                }
                bis /= bl;
                const double cos_half = n1.dot(bis);
                emit(v + d * n1);
                if (d / cos_half <= limit * d)
                    emit(v + (d / cos_half) * bis);
                else {
                    const double s1 = (limit * d - d * n1.dot(bis)) / d1.dot(bis);
                    const double s2 = (limit * d - d * n2.dot(bis)) / d2.dot(bis);
                    emit(v + d * n1 + s1 * d1);
                    emit(v + d * n2 + s2 * d2);
                }
                emit(v + d * n2);
            }
            while (cur->size() > 1 && cur->front() == cur->back())
                cur->pop_back();
        };
        ring(in[ie].contour.points);
        for (const Polygon &h : in[ie].holes)
            ring(h.points);
        parts[ie] = c2_boolean(C2::ClipType::Union, raw, {}, C2::FillRule::Positive);
    }
    if (parts.size() == 1)
        return std::move(parts.front());
    Polygons all;
    for (ExPolygons &q : parts)
        append(all, to_polygons(std::move(q)));
    return all.empty() ? ExPolygons() : c2_union(all);
}

// The stage-2 construction of mitre_dilate (the reference of the T9 test): the union of the polygons, one rectangle
// per edge and one mitre wedge per convex vertex.
ExPolygons mitre_dilate_pieces(const ExPolygons &in, double d, double limit)
{
    Polygons pieces;
    auto push = [&pieces](Polygon &&poly) {
        const double a = poly.area();
        if (poly.points.size() < 3 || a == 0.)
            return;
        if (a < 0.)
            poly.reverse();
        pieces.emplace_back(std::move(poly));
    };
    auto ring = [&](const Points &raw) {
        Points p;
        for (const Point &q : raw)
            if (p.empty() || q != p.back())
                p.push_back(q);
        while (p.size() > 1 && p.front() == p.back())
            p.pop_back();
        const size_t n = p.size();
        if (n < 3)
            return;
        auto P = [&](size_t i) { return p[i % n].cast<double>(); };
        for (size_t i = 0; i < n; ++i) {
            const Vec2d a = P(i), b = P(i + 1);
            Vec2d       t = b - a;
            t /= t.norm();
            const Vec2d nr(t.y(), -t.x());   // material on the left: outward is the right normal
            Polygon     rect;
            rect.points = { rnd(a - t), rnd(b + t), rnd(b + t + d * nr), rnd(a - t + d * nr) };
            push(std::move(rect));
        }
        for (size_t i = 0; i < n; ++i) {
            const Vec2d v = P(i);
            Vec2d       d1 = v - P(i + n - 1), d2 = P(i + 1) - v;
            d1 /= d1.norm();
            d2 /= d2.norm();
            if (d1.x() * d2.y() - d1.y() * d2.x() <= 0.)   // reflex or straight: the edge rectangles cover it
                continue;
            const Vec2d n1(d1.y(), -d1.x()), n2(d2.y(), -d2.x());
            Vec2d       bis = n1 + n2;
            const double bl = bis.norm();
            if (! (bl > 1e-12))
                continue;
            bis /= bl;
            const double cos_half = n1.dot(bis);   // mitre length d / cos_half
            Polygon      wedge;
            if (d / cos_half <= limit * d) {
                wedge.points = { rnd(v), rnd(v + d * n1), rnd(v + (d / cos_half) * bis), rnd(v + d * n2) };
            } else {
                // cut square to the bisector at limit * d: the points of the two offset lines at that distance
                // offset line 1: v + d n1 + s d1; its point with (x - v).bis = limit d
                const double s1 = (limit * d - d * n1.dot(bis)) / d1.dot(bis);
                const double s2 = (limit * d - d * n2.dot(bis)) / d2.dot(bis);
                wedge.points = { rnd(v), rnd(v + d * n1), rnd(v + d * n1 + s1 * d1), rnd(v + d * n2 + s2 * d2), rnd(v + d * n2) };
            }
            push(std::move(wedge));
        }
    };
    for (const ExPolygon &e : in) {
        pieces.push_back(e.contour);
        for (const Polygon &h : e.holes)
            pieces.push_back(h);
        ring(e.contour.points);
        for (const Polygon &h : e.holes)
            ring(h.points);
    }
    return c2_union(pieces);
}

namespace {

// Radius in grid units with the rounding margin (pieces are rounded to the grid: <= 0.71 unit per vertex).
inline double padded_units(double r_mm) { return r_mm * 1e6 + 1.0; }

// ------------------------------------------------------------------------------------------------ mm geometry

inline double cross2(const Vec2d &a, const Vec2d &b) { return a.x() * b.y() - a.y() * b.x(); }

double seg_dist(const Vec2d &p, const Vec2d &a, const Vec2d &b)
{
    const Vec2d  ab = b - a, ap = p - a;
    const double l2 = ab.squaredNorm();
    double t = l2 > 0. ? ap.dot(ab) / l2 : 0.;
    t = std::clamp(t, 0., 1.);
    return (p - (a + t * ab)).norm();
}

double boundary_dist(const RingsMM &rings, const Vec2d &p)
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto &r : rings)
        for (size_t i = 0, n = r.size(); i < n; ++i)
            best = std::min(best, seg_dist(p, r[i], r[(i + 1) % n]));
    return best;
}

bool evenodd(const RingsMM &rings, const Vec2d &p)
{
    bool in = false;
    for (const auto &r : rings)
        for (size_t i = 0, n = r.size(), j = n - 1; i < n; j = i++) {
            const Vec2d &a = r[i], &b = r[j];
            if ((a.y() > p.y()) != (b.y() > p.y())) {
                const double x = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
                if (p.x() < x)
                    in = ! in;
            }
        }
    return in;
}

double ring_signed_area(const std::vector<Vec2d> &r)
{
    double a = 0.;
    for (size_t i = 0, n = r.size(); i < n; ++i)
        a += cross2(r[i], r[(i + 1) % n]);
    return 0.5 * a;
}

std::vector<Vec2d> to_mm(const Points &pts)
{
    std::vector<Vec2d> out;
    out.reserve(pts.size());
    for (const Point &p : pts)
        out.emplace_back(double(p.x()) * 1e-6, double(p.y()) * 1e-6);
    return out;
}

// Row intervals of a horizontal line y through the rings (closed-set semantics, as GEOS intersects).
std::vector<std::pair<double, double>> row_intervals(const RingsMM &rings, double y)
{
    std::vector<double>                    xs;
    std::vector<std::pair<double, double>> iv;
    for (const auto &r : rings)
        for (size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2d &p = r[i], &q = r[(i + 1) % n];
            const double py = p.y(), qy = q.y();
            if (py == qy) {
                if (py == y)
                    iv.emplace_back(std::min(p.x(), q.x()), std::max(p.x(), q.x()));
                continue;
            }
            if (std::min(py, qy) <= y && y < std::max(py, qy))
                xs.push_back(p.x() + (y - py) * (q.x() - p.x()) / (qy - py));
        }
    std::sort(xs.begin(), xs.end());
    for (size_t i = 0; i + 1 < xs.size(); i += 2)
        iv.emplace_back(xs[i], xs[i + 1]);
    std::sort(iv.begin(), iv.end());
    std::vector<std::pair<double, double>> merged;
    for (const auto &v : iv) {
        if (! merged.empty() && v.first <= merged.back().second)
            merged.back().second = std::max(merged.back().second, v.second);
        else
            merged.push_back(v);
    }
    return merged;
}

double run_length(const std::vector<Vec2d> &r)
{
    PySum s;
    for (size_t i = 1; i < r.size(); ++i)
        s.add(py_hypot(r[i].x() - r[i - 1].x(), r[i].y() - r[i - 1].y()));
    return s.result();
}

} // namespace

// ------------------------------------------------------------------------------------------------ public helpers

double arc_tol(double r_u)
{
    const double s = std::sin(PI_D / 384.0);
    return r_u * 2.0 * s * s;
}

double clipper_steps_per_360(double r_u)
{
    const double tol = std::min(r_u, arc_tol(r_u));
    return std::min(PI_D / std::acos(1.0 - tol / r_u), r_u * PI_D);
}

int sector_steps(double sweep)
{
    return std::max(1, int(std::ceil(sweep / ARC_THETA - 1e-9)));
}

Point to_grid(const Vec2d &mm) { return Point(coord_t(std::llround(mm.x() * 1e6)), coord_t(std::llround(mm.y() * 1e6))); }

ExPolygons round_offset(const Polylines &lines, double r_mm, OffsetEnd end)
{
    std::vector<Sausage> ss;
    const double r_u = padded_units(r_mm);
    for (const Polyline &pl : lines)
        ss.push_back({ pl.points, end == OffsetEnd::Joined, r_u });
    return union_sausages(ss);
}

ExPolygons round_offset_pieces(const Polylines &lines, double r_mm, OffsetEnd end)
{
    Polygons pieces;
    const double r_u = padded_units(r_mm);
    for (const Polyline &pl : lines)
        add_sausage(pieces, pl.points, end == OffsetEnd::Joined, r_u);
    return c2_union(pieces);
}

ExPolygons round_erode(const ExPolygons &ex, double r_mm)
{
    if (ex.empty())
        return {};
    std::vector<Sausage> ss;
    const double r_u = padded_units(r_mm);
    for (const ExPolygon &e : ex) {
        ss.push_back({ e.contour.points, true, r_u });
        for (const Polygon &h : e.holes)
            ss.push_back({ h.points, true, r_u });
    }
    return c2_diff(ex, union_sausages(ss));
}

RingsMM to_rings_mm(const ExPolygon &ex)
{
    RingsMM out;
    out.push_back(to_mm(ex.contour.points));
    for (const Polygon &h : ex.holes)
        out.push_back(to_mm(h.points));
    return out;
}

bool inside_rings(const RingsMM &rings, const Vec2d &p, bool strictly)
{
    if (strictly && boundary_dist(rings, p) == 0.)
        return false;
    return evenodd(rings, p);
}

bool covered(const RingsMM &rings, const Vec2d &p, const Vec2d &q)
{
    const Vec2d         r = q - p;
    std::vector<double> ts{ 0., 1. };
    for (const auto &ring : rings)
        for (size_t i = 0, n = ring.size(); i < n; ++i) {
            const Vec2d &e0 = ring[i], &e1 = ring[(i + 1) % n];
            const Vec2d  sv = e1 - e0, w = e0 - p;
            const double den = cross2(r, sv);
            if (den != 0.) {
                const double t = cross2(w, sv) / den, u = cross2(w, r) / den;
                if (t > 0. && t < 1. && u >= 0. && u <= 1.)
                    ts.push_back(t);
            } else if (cross2(w, r) == 0.) {
                const double l2 = r.squaredNorm();
                if (l2 > 0.)
                    for (const Vec2d &e : { e0, e1 }) {
                        const double t = (e - p).dot(r) / l2;
                        if (t > 0. && t < 1.)
                            ts.push_back(t);
                    }
            }
        }
    std::sort(ts.begin(), ts.end());
    auto at   = [&](double t) { return Vec2d(p + t * r); };
    auto near = [&](const Vec2d &x) { return boundary_dist(rings, x) <= 1e-6; };
    for (size_t i = 0; i + 1 < ts.size(); ++i) {
        const double t0 = ts[i], t1 = ts[i + 1];
        if (t1 <= t0)
            continue;
        const Vec2d mid = at(0.5 * (t0 + t1));
        if (evenodd(rings, mid))
            continue;
        if (near(mid) && near(at(t0 + 0.25 * (t1 - t0))) && near(at(t0 + 0.75 * (t1 - t0))))
            continue;
        return false;
    }
    return true;
}

bool covered(const ExPolygon &ex, const Vec2d &p, const Vec2d &q) { return covered(to_rings_mm(ex), p, q); }

std::vector<std::vector<Vec2d>> hatch_component(const ExPolygon &omega, double s, double angle_deg, PlanComponent *info)
{
    if (! (s > 0.))
        throw DIWConfigError("hatch spacing must be > 0");
    RingsMM    rings  = to_rings_mm(omega);
    const bool rotate = std::abs(angle_deg) > 1e-12;
    if (rotate) {
        // shapely affinity.rotate(poly, -angle_deg, origin=(0, 0))
        const double rad = (-angle_deg) * PI_D / 180.0;
        double       cp = std::cos(rad), sp = std::sin(rad);
        if (std::abs(cp) < 2.5e-16) cp = 0.0;
        if (std::abs(sp) < 2.5e-16) sp = 0.0;
        for (auto &r : rings)
            for (Vec2d &v : r) {
                const double x = v.x(), y = v.y();
                v = Vec2d(cp * x + (-sp) * y + 0.0, sp * x + cp * y + 0.0);
            }
    }
    double miny = std::numeric_limits<double>::infinity(), maxy = -miny;
    for (const Vec2d &v : rings.front()) {
        miny = std::min(miny, v.y());
        maxy = std::max(maxy, v.y());
    }
    const double height = maxy - miny;
    const int    n      = int(std::floor(height / s + 1e-9)) + 1;
    const double y0     = miny + (height - (n - 1) * s) / 2.0;
    using Seg = std::pair<Vec2d, Vec2d>;
    std::vector<Seg> flat;
    for (int i = 0; i < n; ++i) {
        const double     y = y0 + i * s;
        std::vector<Seg> segs;
        for (const auto &iv : row_intervals(rings, y)) {
            const Vec2d a(iv.first, y), b(iv.second, y);
            if (py_hypot(b.x() - a.x(), b.y() - a.y()) > 1e-4)
                segs.emplace_back(a, b);
        }
        std::sort(segs.begin(), segs.end(), [](const Seg &l, const Seg &r) {
            return std::make_tuple(l.first.x(), l.first.y(), l.second.x(), l.second.y()) <
                   std::make_tuple(r.first.x(), r.first.y(), r.second.x(), r.second.y());
        });
        if (i % 2 == 1) {
            std::reverse(segs.begin(), segs.end());
            for (Seg &sg : segs)
                std::swap(sg.first, sg.second);
        }
        flat.insert(flat.end(), segs.begin(), segs.end());
    }
    std::vector<std::vector<Vec2d>> runs;
    std::vector<Vec2d>              cur;
    for (const Seg &sg : flat) {
        const Vec2d &a = sg.first, &b = sg.second;
        if (cur.empty()) {
            cur = { a, b };
            continue;
        }
        const double gap    = py_hypot(a.x() - cur.back().x(), a.y() - cur.back().y());
        const bool   joined = gap <= 3.0 * s && (gap < 1e-9 || covered(rings, cur.back(), a));
        if (joined) {
            if (gap >= 1e-9)
                cur.push_back(a);
            cur.push_back(b);
        } else {
            runs.push_back(std::move(cur));
            cur = { a, b };
        }
    }
    if (! cur.empty())
        runs.push_back(std::move(cur));
    if (rotate) {
        const double th = angle_deg * (PI_D / 180.0);
        const double cs = std::cos(th), sn = std::sin(th);
        for (auto &r : runs)
            for (Vec2d &v : r) {
                const double px = v.x(), py = v.y();
                v = Vec2d(px * cs - py * sn, px * sn + py * cs);
            }
    }
    if (info) {
        info->rows = n;
        info->H    = height;
        info->runs = int(runs.size());
    }
    return runs;
}

Vec2d area_centroid(const ExPolygon &ex)
{
    const RingsMM rings = to_rings_mm(ex);
    const Vec2d   base  = rings.front().front();
    double cgx = 0., cgy = 0., areasum2 = 0.;
    for (size_t k = 0; k < rings.size(); ++k) {
        const auto &r   = rings[k];
        const bool  ccw = ring_signed_area(r) > 0.;
        const bool  pos = k == 0 ? ! ccw : ccw;   // GEOS: shell CW / hole CCW count positive
        const double sign = pos ? 1.0 : -1.0;
        for (size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2d &p1 = r[i], &p2 = r[(i + 1) % n];
            const double cx = base.x() + p1.x() + p2.x();
            const double cy = base.y() + p1.y() + p2.y();
            const double a2 = (p1.x() - base.x()) * (p2.y() - base.y()) - (p2.x() - base.x()) * (p1.y() - base.y());
            cgx += sign * a2 * cx;
            cgy += sign * a2 * cy;
            areasum2 += sign * a2;
        }
    }
    if (areasum2 == 0.)
        return base;
    return Vec2d(cgx / 3.0 / areasum2, cgy / 3.0 / areasum2);
}

Vec2d interior_point_area(const ExPolygon &ex)
{
    const RingsMM rings = to_rings_mm(ex);
    double miny = std::numeric_limits<double>::infinity(), maxy = -miny;
    for (const Vec2d &v : rings.front()) {
        miny = std::min(miny, v.y());
        maxy = std::max(maxy, v.y());
    }
    // ScanLineYOrdinateFinder
    const double centre = (miny + maxy) / 2.0;
    double       hiY = maxy, loY = miny;
    for (const auto &r : rings)
        for (const Vec2d &v : r) {
            const double y = v.y();
            if (y <= centre) {
                if (y > loY) loY = y;
            } else if (y > centre) {
                if (y < hiY) hiY = y;
            }
        }
    const double        scanY = (hiY + loY) / 2.0;
    std::vector<double> xs;
    for (const auto &r : rings)
        for (size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2d &p0 = r[i], &p1 = r[(i + 1) % n];
            if (p0.y() > scanY && p1.y() > scanY) continue;
            if (p0.y() < scanY && p1.y() < scanY) continue;
            if (p0.y() == p1.y()) continue;
            if (p0.y() == scanY && p1.y() < scanY) continue;
            if (p1.y() == scanY && p0.y() < scanY) continue;
            double x = p0.x();
            if (p0.x() != p1.x()) {
                const double m = (p1.y() - p0.y()) / (p1.x() - p0.x());
                x = p0.x() + ((scanY - p0.y()) / m);
            }
            xs.push_back(x);
        }
    if (xs.size() < 2)
        return rings.front().front();
    std::sort(xs.begin(), xs.end());
    double best = -1., bx = xs.front();
    for (size_t i = 0; i + 1 < xs.size(); i += 2) {
        const double w = xs[i + 1] - xs[i];
        if (w > best) {
            best = w;
            bx   = (xs[i] + xs[i + 1]) / 2.0;
        }
    }
    return Vec2d(bx, scanY);
}

std::vector<std::pair<double, double>> clip_segment(const RingsMM &clip, const Vec2d &a, const Vec2d &b)
{
    const Vec2d         r = b - a;
    std::vector<double> ts{ 0., 1. };
    for (const auto &ring : clip)
        for (size_t i = 0, n = ring.size(); i < n; ++i) {
            const Vec2d &e0 = ring[i], &e1 = ring[(i + 1) % n];
            const Vec2d  sv = e1 - e0, w = e0 - a;
            const double den = cross2(r, sv);
            if (den != 0.) {
                const double t = cross2(w, sv) / den, u = cross2(w, r) / den;
                if (t > 0. && t < 1. && u >= 0. && u <= 1.)
                    ts.push_back(t);
            }
        }
    std::sort(ts.begin(), ts.end());
    std::vector<std::pair<double, double>> out;
    for (size_t i = 0; i + 1 < ts.size(); ++i) {
        const double t0 = ts[i], t1 = ts[i + 1];
        if (! (t1 > t0))
            continue;
        if (! evenodd(clip, a + (0.5 * (t0 + t1)) * r))
            continue;
        if (! out.empty() && out.back().second == t0)
            out.back().second = t1;
        else
            out.emplace_back(t0, t1);
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ LineMerger

std::vector<std::vector<Vec2d>> line_merge(const std::vector<std::vector<Vec2d>> &lines)
{
    struct Edge { std::vector<Vec2d> pts; int from, to; bool marked = false; };
    struct DE { int edge; bool fwd; };
    struct Node { std::vector<DE> out; bool marked = false; };
    auto key = [](const Vec2d &v) { return std::make_pair(v.x(), v.y()); };
    std::map<std::pair<double, double>, int> node_ids;   // GEOS NodeMap: ordered by (x, y)
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    auto node_of = [&](const Vec2d &v) {
        auto it = node_ids.find(key(v));
        if (it != node_ids.end())
            return it->second;
        const int id = int(nodes.size());
        nodes.emplace_back();
        node_ids.emplace(key(v), id);
        return id;
    };
    for (const auto &l : lines) {
        std::vector<Vec2d> c;
        for (const Vec2d &v : l)
            if (c.empty() || v != c.back())
                c.push_back(v);
        if (c.size() <= 1)
            continue;
        const int e = int(edges.size());
        Edge      ed;
        ed.from = node_of(c.front());
        ed.to   = node_of(c.back());
        ed.pts  = std::move(c);
        edges.push_back(std::move(ed));
        nodes[edges[e].from].out.push_back({ e, true });
        nodes[edges[e].to].out.push_back({ e, false });
    }
    auto to_node = [&](const DE &d) { return d.fwd ? edges[d.edge].to : edges[d.edge].from; };
    std::vector<std::vector<Vec2d>> result;
    auto build_string = [&](DE start) {
        std::vector<DE> des;
        DE              cur   = start;
        bool            valid = true;
        do {
            des.push_back(cur);
            edges[cur.edge].marked = true;
            const Node &tn = nodes[to_node(cur)];
            if (tn.out.size() != 2) { valid = false; break; }
            const DE sym{ cur.edge, ! cur.fwd };
            const DE nx = (tn.out[0].edge == sym.edge && tn.out[0].fwd == sym.fwd) ? tn.out[1] : tn.out[0];
            cur = nx;
        } while (! (cur.edge == start.edge && cur.fwd == start.fwd));
        (void) valid;
        std::vector<Vec2d> coords;
        int nf = 0, nr = 0;
        for (const DE &d : des) {
            (d.fwd ? nf : nr)++;
            const auto &pts = edges[d.edge].pts;
            auto add = [&](const Vec2d &v) { if (coords.empty() || coords.back() != v) coords.push_back(v); };
            if (d.fwd)
                for (const Vec2d &v : pts) add(v);
            else
                for (auto it = pts.rbegin(); it != pts.rend(); ++it) add(*it);
        }
        if (nr > nf)
            std::reverse(coords.begin(), coords.end());
        result.push_back(std::move(coords));
    };
    auto build_at = [&](int nid) {
        for (const DE &d : std::vector<DE>(nodes[nid].out))
            if (! edges[d.edge].marked)
                build_string(d);
    };
    for (const auto &kv : node_ids) {   // nodes of degree != 2 first, in NodeMap order
        Node &nd = nodes[kv.second];
        if (nd.out.size() != 2) {
            build_at(kv.second);
            nd.marked = true;
        }
    }
    for (const auto &kv : node_ids) {   // then the isolated loops
        Node &nd = nodes[kv.second];
        if (! nd.marked) {
            build_at(kv.second);
            nd.marked = true;
        }
    }
    return result;
}

std::vector<std::vector<Vec2d>> rings_from_open_walls(const std::vector<std::vector<Vec2d>> &walls)
{
    std::vector<std::vector<Vec2d>> loops;
    for (const auto &w : walls) {
        if (w.size() < 2)
            continue;
        std::vector<Vec2d> pts = w;
        if (pts.size() >= 3 && py_hypot(pts.front().x() - pts.back().x(), pts.front().y() - pts.back().y()) < LOOP_CLOSE_TOL_MM)
            pts.back() = pts.front();
        loops.push_back(std::move(pts));
    }
    std::vector<std::vector<Vec2d>> rings;
    for (auto &c : line_merge(loops)) {
        if (c.size() >= 4 && py_hypot(c.front().x() - c.back().x(), c.front().y() - c.back().y()) < LOOP_CLOSE_TOL_MM) {
            if (c.front() == c.back())
                c.pop_back();
            rings.push_back(std::move(c));
        }
    }
    return rings;
}

bool is_ring_loop(int inset_idx, const std::vector<ExtrusionRole> &path_roles)
{
    if (inset_idx == 0)
        return true;
    if (inset_idx > 0 || path_roles.empty())
        return false;
    for (ExtrusionRole r : path_roles)
        if (r != erExternalPerimeter && r != erOverhangPerimeter)
            return false;
    return true;
}

// ------------------------------------------------------------------------------------------------ the planner

PowderLayerRec plan_powder_layer_native(const PlanInput &in, PlanDebug *dbg)
{
    if (in.U == nullptr)
        throw std::invalid_argument("plan_powder_layer_native: no parameters");
    const UMexParams &U = *in.U;
    PowderLayerRec    rec;
    rec.status = PowderLayerRec::Status::open;
    auto note  = [&](const std::string &s) { if (dbg) dbg->notes.push_back(s); };
    static const bool timing = std::getenv("ORCA_DIW_PLAN_TIMING") != nullptr;
    auto t_last = std::chrono::steady_clock::now();
    auto lap = [&](const char *what) {
        if (! timing || ! dbg) return;
        const auto now = std::chrono::steady_clock::now();
        dbg->notes.push_back(std::string("t:") + what + "=" + std::to_string(std::chrono::duration<double, std::milli>(now - t_last).count()));
        t_last = now;
    };

    // ---- local frame: P0-P5 run on the inputs translated by -origin (whole grid units), so that a translated
    // input gives an exactly translated output (Clipper2 truncates intersection coordinates towards zero).
    std::vector<Polygon> grid_open_rings;
    for (const auto &c : rings_from_open_walls(in.open_outer)) {
        Polygon poly;
        for (const Vec2d &v : c)
            poly.points.push_back(to_grid(v));
        grid_open_rings.push_back(std::move(poly));
    }
    coord_t minx = std::numeric_limits<coord_t>::max(), miny = minx;
    auto    see  = [&](const Points &pts) {
        for (const Point &p : pts) {
            minx = std::min(minx, p.x());
            miny = std::min(miny, p.y());
        }
    };
    for (const NativeRing &r : in.rings) see(r.poly.points);
    for (const Polygon &r : grid_open_rings) see(r.points);
    for (const WallPiece &w : in.native_walls) see(w.pts.points);
    for (const TextPath &t : in.text_walls) see(t.pts_nm.points);
    for (const TextPath &t : in.text_paste) see(t.pts_nm.points);
    if (minx == std::numeric_limits<coord_t>::max())
        minx = miny = 0;
    const Point origin(minx - 10000000, miny - 10000000);
    auto        local = [&](const Points &pts) {
        Points out(pts);
        for (Point &p : out) p -= origin;
        return out;
    };

    // ---- P0 / P1: rings and S --------------------------------------------------------------------
    Polygons rings;
    for (const NativeRing &r : in.rings) {
        Points d;
        for (const Point &p : r.poly.points)
            if (d.empty() || p != d.back())
                d.push_back(p);
        if (d.size() > 1 && d.front() == d.back())
            d.pop_back();
        Polygon poly(local(d));
        if (d.size() >= 3 && poly.area() != 0.)
            rings.push_back(std::move(poly));
    }
    for (const Polygon &r : grid_open_rings)
        rings.emplace_back(local(r.points));
    if (rings.empty()) {
        note("no ring");
        return rec;
    }
    ExPolygons S = c2_boolean(C2::ClipType::Union, to_paths(rings), {}, C2::FillRule::EvenOdd);
    if (S.empty()) {
        note("S empty");
        return rec;
    }
    const OutsideTest outside_S(S);

    lap("P01");
    // ---- P2 / P3: band, paste exclusion, cavity --------------------------------------------------------
    // P4 needs the same pieces grown by D (see below), so both sets are built in one pass.
    const double d_eff = U.delta + POWDER_GUARD_MM;
    const double D     = double(std::llround(d_eff * 1e6));
    // one dilation per polyline (in parallel, see union_sausages)
    std::vector<Sausage> pieces, grown;
    // every segment of a band / paste centreline, to test whether the ring edges are all band centrelines (P4)
    std::set<std::pair<std::pair<coord_t, coord_t>, std::pair<coord_t, coord_t>>> band_segs;
    auto seg_key = [](const Point &a, const Point &b) {
        auto pa = std::make_pair(a.x(), a.y()), pb = std::make_pair(b.x(), b.y());
        return pa < pb ? std::make_pair(pa, pb) : std::make_pair(pb, pa);
    };
    auto add_both = [&](const Points &pts, bool closed, double half_width_mm) {
        const Points lp = local(pts);
        pieces.push_back({ lp, closed, padded_units(half_width_mm), &outside_S });
        grown.push_back({ lp, closed, padded_units(half_width_mm) + padded_units(D * 1e-6), &outside_S });
        for (size_t i = 0; i + 1 < lp.size(); ++i) band_segs.insert(seg_key(lp[i], lp[i + 1]));
        if (closed && lp.size() > 2) band_segs.insert(seg_key(lp.back(), lp.front()));
    };
    for (const WallPiece &w : in.native_walls)
        add_both(w.pts.points, w.closed, w.width / 2.0);
    // Text walls and paste: v2's band / exclusion (the whole path at its first-segment width), plus every segment
    // printed wider than that at its own ;WIDTH: (review fix: variable-width gap fill grows inside one path, so the
    // first-segment width under-covers the printed bead). Round caps at the segment ends cover the joints.
    auto add_text = [&](const TextPath &t) {
        if (t.pts_nm.points.size() < 2)
            return;
        add_both(t.pts_nm.points, t.pts_mm.front() == t.pts_mm.back(), t.width / 2.0);
        for (size_t i = 0; i < t.seg_width.size() && i + 1 < t.pts_nm.points.size(); ++i)
            if (t.seg_width[i] > t.width && t.pts_nm.points[i] != t.pts_nm.points[i + 1])
                add_both(Points{ t.pts_nm.points[i], t.pts_nm.points[i + 1] }, false, t.seg_width[i] / 2.0);
    };
    for (const TextPath &t : in.text_walls) add_text(t);
    for (const TextPath &t : in.text_paste) add_text(t);
    // S eroded by D is implied by X dilated by w/2 + D when every ring edge is a band centreline segment: the
    // boundary of S lies on the ring edges, so (boundary of S) dilated by D lies inside the grown pieces.
    bool rings_on_band = ! band_segs.empty();
    for (const Polygon &r : rings) {
        for (size_t i = 0, n = r.points.size(); i < n && rings_on_band; ++i)
            rings_on_band = band_segs.count(seg_key(r.points[i], r.points[(i + 1) % n])) > 0;
        if (! rings_on_band)
            break;
    }
    lap("P2pieces");
    // ---- P3 (cavity) and P4 (Omega) run in parallel ---------------------------------------------------
    // P4: C = S - X with X = band U paste, so C eroded by D = (S eroded by D) - (X dilated by D) exactly, and X
    // dilated by D is the wall / paste centrelines dilated by w/2 + D. Both sides are built from the same
    // conservative pieces as before (the dilation contains the exact disk sum, round_erode is contained in the
    // exact erosion), so Omega stays inside the exact erosion of the exact C. This replaces round_erode(C, D),
    // whose union of one piece per vertex of C (C's boundary is the fine arcs of the band) took ~0.2-0.8 s per
    // layer (PLANNER_DESIGN.md "Deviations", stage 5). Components of C that the area filter dropped cannot
    // contain a disk of radius D, so they contribute nothing here either. When the ring edges are all band
    // centrelines (always so for native rings), S eroded by D is not computed: S - (X dilated by w/2 + D) is
    // then the same set.
    ExPolygons C, SD;
    const bool par = count_points(pieces) + count_points(grown) >= PAR_MIN_POINTS;
    run2(par,
        [&]() {
            ExPolygons C0 = pieces.empty() ? S : c2_diff(S, union_sausages(pieces));
            for (ExPolygon &e : C0)
                if (area_mm2(e) > 1e-6)
                    C.push_back(std::move(e));
        },
        [&]() {
            SD = rings_on_band ? S : round_erode(S, D * 1e-6);
            if (! grown.empty() && ! SD.empty())
                SD = c2_diff(SD, union_sausages(grown));
        });
    lap("P3P4");
    PySum ca;
    for (const ExPolygon &e : C) ca.add(area_mm2(e));
    ExPolygons omega;
    if (! C.empty())
        for (ExPolygon &e : SD)
            if (area_mm2(e) > 1e-6)
                omega.push_back(std::move(e));
    PySum oa;
    for (const ExPolygon &e : omega) oa.add(area_mm2(e));

    // ---- P5: narrow and revert ------------------------------------------------------------------------
    // narrow is taken of C without its strips thinner than 2 * C_OPEN_NM (C intersected with its opening by
    // C_OPEN_NM): two wall bands that are tangent in the G-code (wall spacing == width) leave a zero-width seam
    // in C that nm noise opens into a strip linking separate narrow parts into one reverted region
    // (PLANNER_DESIGN.md "Deviations", stage 2). 100 nm is v2's own noise margin (opening.buffer(1e-4)). ca and
    // Omega use C itself, as v2.
    // The opening of C and the mitre opening of Omega are independent: with debug geometry (which reports narrow)
    // both are computed, in parallel. Without it the mitre opening comes first: when no component of C - opening_m
    // reaches paste_min_area, no component of narrow (a subset of it) does either, so revert is empty and the
    // opening of C is skipped.
    ExPolygons Cn, opening_m;
    bool       need_cn = true;
    auto cn_fn = [&]() {
        if (! C.empty() && need_cn)
            Cn = c2_intersection(C, c2_offset(c2_offset(C, -C_OPEN_NM, C2::JoinType::Miter, 2.0, 0.),
                                              C_OPEN_NM, C2::JoinType::Miter, 2.0, 0.));
    };
    auto opening_fn = [&]() {
            if (omega.empty())
                return;
            // The mitre opening uses GEOS's limited mitre (mitre_dilate), not Clipper2's squared one, and is taken
            // of Omega simplified by OPENING_SIMPLIFY_NM (Douglas-Peucker): the union of the grid-rounded erosion
            // pieces leaves edges a few nm long whose normals point anywhere, and a mitre join between such edges
            // grows a spike of up to 5 D, which made revert jump by ~0.1 mm2 under 1 nm noise (PLANNER_DESIGN.md
            // "Deviations", stage 2). The simplification moves the boundary by <= 10 nm.
            ExPolygons omega_s;
            for (const ExPolygon &e : omega)
                append(omega_s, e.simplify(OPENING_SIMPLIFY_NM));
            const ExPolygons opening = mitre_dilate(omega_s, D, 5.0);
            opening_m                = c2_offset(opening, 100., C2::JoinType::Round, 2.0, arc_tol(100.));
    };
    if (par && dbg != nullptr)   // the debug geometry reports narrow: both are needed
        run2(true, cn_fn, opening_fn);
    else {
        opening_fn();
        if (dbg == nullptr && ! omega.empty()) {
            need_cn = false;
            for (const ExPolygon &e : c2_diff(C, opening_m))
                if (area_mm2(e) >= U.paste_min_area) {
                    need_cn = true;
                    break;
                }
        }
        cn_fn();
    }
    lap("P5open");
    ExPolygons narrow = ! need_cn ? ExPolygons() : omega.empty() ? Cn : c2_diff(Cn, opening_m);
    lap("P5narrow");
    ExPolygons revert;
    PySum      ra;
    for (const ExPolygon &e : narrow)
        if (area_mm2(e) >= U.paste_min_area) {
            revert.push_back(e);
            ra.add(area_mm2(e));
        }

    rec.ca = ca.result();
    rec.oa = oa.result();
    rec.ra = ra.result();
    rec.nc = int(omega.size());
    rec.hl = 0.0;
    // back to the G-code frame (exact integer translation)
    for (ExPolygons *ee : { &C, &omega, &narrow, &revert })
        translate(*ee, origin);

    lap("P5rest");
    // ---- P6 / P7: order and hatch ---------------------------------------------------------------------
    std::stable_sort(omega.begin(), omega.end(), [](const ExPolygon &a, const ExPolygon &b) {
        const BoundingBox ba = get_extents(a.contour), bb = get_extents(b.contour);
        return std::make_pair(ba.min.y(), ba.min.x()) < std::make_pair(bb.min.y(), bb.min.x());
    });
    for (const ExPolygon &w : omega) {
        PlanComponent info;
        auto          runs = hatch_component(w, in.s_eff, U.hatch_angle, &info);
        PySum         tot;
        for (const auto &r : runs) tot.add(run_length(r));
        const double total = tot.result();
        info.total         = total;
        if (total < U.point_len) {
            Vec2d c = area_centroid(w);
            if (! inside_rings(to_rings_mm(w), c, true))
                c = interior_point_area(w);
            const double m = (((area_mm2(w) * in.h) * 1e-3) * U.rho_tap) * U.eta;
            PowderRunRec run;
            run.point  = true;
            run.pts    = { { c.x(), c.y() } };
            run.t_feed = (60.0 * m) / U.m_dot;
            rec.runs.push_back(std::move(run));
            info.point = true;
            info.runs  = 0;
        } else {
            for (const auto &r : runs) {
                PowderRunRec run;
                for (const Vec2d &v : r) run.pts.emplace_back(v.x(), v.y());
                rec.runs.push_back(std::move(run));
            }
            rec.hl += total;
        }
        if (dbg) dbg->components.push_back(info);
    }

    lap("P7hatch");
    // ---- P8: paste fallback -------------------------------------------------------------------------
    if (! rec.runs.empty() && ! revert.empty()) {
        const double     pad  = std::min(0.5 * U.width_fallback, 0.5 * U.delta);
        const double     padu = std::ceil(pad * 1e6);
        const ExPolygons clip = c2_offset(revert, padu, C2::JoinType::Round, 2.0, arc_tol(padu));
        RingsMM          clip_rings;
        for (const ExPolygon &e : clip)
            for (auto &r : to_rings_mm(e))
                clip_rings.push_back(std::move(r));
        for (const TextPath &p : in.text_powder) {
            std::vector<Vec2d>  cur;
            std::vector<double> cde, cfeed;
            auto flush = [&]() {
                if (! cur.empty() && ! cde.empty()) {
                    FallbackPathRec fb;
                    fb.ptype = p.ptype;
                    for (const Vec2d &v : cur) fb.pts.emplace_back(v.x(), v.y());
                    fb.de          = cde;
                    fb.feed        = cfeed;
                    fb.width       = p.width;
                    fb.travel_feed = p.travel_feed;
                    fb.z           = p.z;
                    rec.fallbacks.push_back(std::move(fb));
                }
            };
            for (size_t i = 0; i + 1 < p.pts_mm.size() && i < p.seg_de.size(); ++i) {
                const Vec2d &a = p.pts_mm[i], &b = p.pts_mm[i + 1];
                const double dx = b.x() - a.x(), dy = b.y() - a.y();
                const double L  = std::sqrt(dx * dx + dy * dy);
                if (L < 1e-9)
                    continue;
                for (const auto &iv : clip_segment(clip_rings, a, b)) {
                    const Vec2d  c0 = iv.first == 0. ? a : Vec2d(a + iv.first * (b - a));
                    const Vec2d  c1 = iv.second == 1. ? b : Vec2d(a + iv.second * (b - a));
                    const double ex = c1.x() - c0.x(), ey = c1.y() - c0.y();
                    const double len = std::sqrt(ex * ex + ey * ey);
                    if (len < 0.05)
                        continue;
                    const double de_piece = p.seg_de[i] * len / L;
                    if (! (! cur.empty() && py_hypot(c0.x() - cur.back().x(), c0.y() - cur.back().y()) < 1e-6)) {
                        flush();
                        cur = { c0 };
                        cde.clear();
                        cfeed.clear();
                    }
                    cur.push_back(c1);
                    cde.push_back(de_piece);
                    cfeed.push_back(p.seg_feed[i]);
                }
            }
            flush();
        }
    }

    lap("P8fb");
    rec.status = rec.runs.empty() ? PowderLayerRec::Status::small : PowderLayerRec::Status::ok;
    if (dbg) {
        dbg->cavity = C;
        dbg->omega  = omega;
        dbg->narrow = narrow;
        dbg->revert = revert;
    }
    return rec;
}

} // namespace DIW
} // namespace Slic3r
