#ifndef slic3r_GCode_DIWPowderPlanner_hpp_
#define slic3r_GCode_DIWPowderPlanner_hpp_

// u-MEX powder planner, slicer-core side (diw/PLANNER_DESIGN.md section 7): a pure function that turns one core
// layer's walls, paste paths and sparse-infill paths into the powder plan of a ";DIW:" record block
// (diw/POWDER_RECORDS.md). It mirrors v2's plan_powder_layer (diw_engine.py 1025-1131) step by step:
//   P0/P1  rings (native closed loops, plus v2's snap + LineMerger over open outer walls) -> S (even-odd)
//   P2     wall band B = native walls (+) text walls, each dilated by half its width
//   P3     cavity C = S - (B u E), E = paste paths dilated by half their width
//   P4     Omega = C eroded by delta + POWDER_GUARD_MM
//   P5     narrow = C - opening(Omega), revert = narrow parts >= paste_min_area
//   P6/P7  per Omega component (sorted by its bounding box): boustrophedon hatch, or a point dwell
//   P8     paste fallback: the sparse-infill paths clipped to the reverted regions (+ a pad)
// It depends on nothing from Print or Layer. Coordinates are on a fixed 1 nm integer grid (UNIT_MM), not on
// SCALING_FACTOR. Hatch, coverage, centroid and the fallback clip are analytic, in double precision.
//
// Dilations and the erosion contain the exact Minkowski disk sum: straight edges sit exactly at the radius, arcs are
// circumscribed (tangent directions on a global grid of 1024 steps per 360 degrees). Since stage 5c they are built
// per polyline from one raw offset path (the edge offsets, the circumscribed arcs at convex vertices, Clipper2's
// three-point joins at concave ones) and one Positive union, instead of the union of one rectangle per segment and
// one sector per vertex (stage 2, kept as round_offset_pieces / mitre_dilate_pieces for the tests); arc caps of the
// band that lie outside S are replaced by their chords (PLANNER_DESIGN.md "Deviations", stages 2 and 5c). The
// geometry is computed in a local frame (the inputs' bounding box
// corner minus 10 mm, an integer translation), so translating the input by whole grid units translates the
// output exactly.

#include "../ExPolygon.hpp"
#include "../ExtrusionEntity.hpp"
#include "../Point.hpp"
#include "../Polygon.hpp"
#include "../Polyline.hpp"

#include "DIWPowderBlock.hpp"
#include "DIWUMexParams.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {
namespace DIW {

constexpr double UNIT_MM           = 1e-6;   // planner grid: 1 integer unit = 1 nm
constexpr double LOOP_CLOSE_TOL_MM = 0.05;   // v2: a wall path whose ends are closer than this is a closed loop
constexpr int    ARC_STEPS_PER_360 = 192;    // Clipper2 round offsets (opening margin, fallback clip pad): arc_tol()
constexpr int    SECTOR_STEPS_PER_360 = 1024; // circumscribed sectors of the band / paste / erosion pieces

// One wall piece of the native band. closed: the polyline is a closed loop (front()==back() is not stored).
struct WallPiece  { Polyline pts; double width = 0.; bool closed = false; };
// A closed in-memory loop in the G-code frame, nm grid.
struct NativeRing { Polygon poly; };

// One text path of parse_core_layer (v2 PathRec): points both as parsed (mm) and on the nm grid.
struct TextPath
{
    std::string           ptype, kind;
    std::vector<Vec2d>    pts_mm;
    Polyline              pts_nm;
    std::vector<double>   seg_de, seg_feed;
    // ;WIDTH: of each segment (empty: every segment has `width`). Review fix (variable-width paste, e.g. gap fill):
    // a segment printed wider than the path's first-segment width is dilated by its own half width as well.
    std::vector<double>   seg_width;
    double                width = 0., z = 0.;
    std::optional<double> travel_feed;
};

struct PlanInput
{
    std::vector<NativeRing>          rings;        // native S input (closed loops)
    std::vector<std::vector<Vec2d>>  open_outer;   // open outer / overhang walls (mm): v2 snap + LineMerger -> rings
    std::vector<WallPiece>           native_walls; // native band input
    std::vector<TextPath>            text_walls, text_paste, text_powder;   // from parse_core_layer
    double                           h = 0., s_eff = 0.;
    const UMexParams                *U = nullptr;
};

// Per Omega component, in plan order.
struct PlanComponent
{
    int    rows  = 0;       // hatch rows n = floor(H/s + 1e-9) + 1
    double H     = 0.;      // shell height in the hatch frame (mm)
    int    runs  = 0;       // line runs (0 for a point dwell)
    bool   point = false;
    double total = 0.;      // hatch length of the component (also for a point dwell: the rejected total)
};

struct PlanDebug
{
    ExPolygons                 cavity, omega, narrow, revert;   // nm grid
    std::vector<PlanComponent> components;
    std::vector<std::string>   notes;
};

// Fills status, STAT, runs and fallbacks; the caller sets k, z, h, s, F, fu and fp.
// s_out (PRODUCT_DESIGN.md 3.4, the binder's orphan check): receives S, the even-odd union of the rings (G-code frame,
// nm grid), or stays empty when the layer has no ring. It does not enter the record.
PowderLayerRec plan_powder_layer_native(const PlanInput &in, PlanDebug *dbg = nullptr, ExPolygons *s_out = nullptr);

// ------------------------------------------------------------------------------------------------
// Building blocks, exposed for the unit tests (tests/libslic3r/test_diw_planner.cpp).
// ------------------------------------------------------------------------------------------------

// Clipper2 round-join arc tolerance for radius r (grid units): r * 2 sin^2(pi/384) = r (1 - cos(pi/192)).
double arc_tol(double r_u);
// Clipper2's steps_per_360 for a round offset of radius r_u with arc_tol(r_u) (192 for any r_u >= 1 mm).
double clipper_steps_per_360(double r_u);
// Number of equal arc steps of 2 pi / 1024 that cover the given sweep (radians): ceil(sweep / (2 pi / 1024)); a
// circumscribed arc on the global grid has at most one more vertex.
int    sector_steps(double sweep);

enum class OffsetEnd { Joined, Round };
// The union of the polylines (nm grid) dilated by r_mm. Joined: each polyline is a closed loop. The result
// contains the exact Minkowski disk sum; straight parts lie within 1 nm of it, arcs within r (1/cos(pi/1024) - 1).
ExPolygons round_offset(const Polylines &lines, double r_mm, OffsetEnd end);
// The same set built as the union of one rectangle per segment and one circumscribed sector per convex vertex
// and end cap (the stage-2 construction), kept as the reference of the T9 test.
ExPolygons round_offset_pieces(const Polylines &lines, double r_mm, OffsetEnd end);
// The polygon eroded by r_mm (nm grid): ex minus the dilation of its boundary. Contained in the exact erosion.
ExPolygons round_erode(const ExPolygons &ex, double r_mm);

// The polygons (nm grid) dilated by d_u grid units with GEOS's limited mitre joins (shapely buffer(join_style=
// "mitre", mitre_limit=limit)): a mitre longer than limit * d_u is cut square to the bisector at limit * d_u.
ExPolygons mitre_dilate(const ExPolygons &in, double d_u, double limit);
// The same set built as the union of the polygons, one rectangle per edge and one wedge per convex vertex (the
// stage-2 construction), kept as the reference of the T9 test.
ExPolygons mitre_dilate_pieces(const ExPolygons &in, double d_u, double limit);

// v2 hatch_polygon on one Omega component (nm grid): runs in mm, in v2 order (boustrophedon, chained).
std::vector<std::vector<Vec2d>> hatch_component(const ExPolygon &omega, double s, double angle_deg,
                                                PlanComponent *info = nullptr);

// Polygon rings in mm (ring 0 = shell; no repeated closing point).
using RingsMM = std::vector<std::vector<Vec2d>>;
RingsMM to_rings_mm(const ExPolygon &ex);
// v2's poly.buffer(1e-6).covers(LineString([p, q])) on rings in mm.
bool    covered(const RingsMM &rings, const Vec2d &p, const Vec2d &q);
bool    covered(const ExPolygon &ex, const Vec2d &p, const Vec2d &q);
// Even-odd point in rings; strictly: false on the boundary.
bool    inside_rings(const RingsMM &rings, const Vec2d &p, bool strictly);

// GEOS area centroid (triangle fan from the shell's first vertex, holes subtracted), in mm.
Vec2d area_centroid(const ExPolygon &ex);
// GEOS InteriorPointArea (shapely representative_point), in mm.
Vec2d interior_point_area(const ExPolygon &ex);

// v2's seg.intersection(clip) for one segment a->b (mm): the pieces inside the clip rings (even-odd), in segment
// direction, as parameter intervals [t0, t1] of a->b (adjacent kept intervals merged).
std::vector<std::pair<double, double>> clip_segment(const RingsMM &clip, const Vec2d &a, const Vec2d &b);

// GEOS LineMerger (shapely.linemerge) over open lines (mm): merges at exactly equal end points through
// nodes of degree 2; isolated loops become closed lines.
std::vector<std::vector<Vec2d>> line_merge(const std::vector<std::vector<Vec2d>> &lines);
// v2's ring formation (P1b): snap (>= 3 points, ends < 0.05 apart), line_merge, keep merged lines with >= 4
// coordinates whose ends are < 0.05 apart. Rings in mm, without a repeated closing point.
std::vector<std::vector<Vec2d>> rings_from_open_walls(const std::vector<std::vector<Vec2d>> &walls);

// Ring predicate of the native capture (section 5.2): an outermost loop, or a loop of unknown inset whose
// paths are all external / overhang perimeters. ExtrusionLoop::role() and is_inner_wall() are not used.
bool is_ring_loop(int inset_idx, const std::vector<ExtrusionRole> &path_roles);

// mm -> nm grid (llround(x * 1e6)).
Point to_grid(const Vec2d &mm);

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPowderPlanner_hpp_
