"""Native powder planner vs v2 (diw/PLANNER_DESIGN.md T1, T3, T4 a-c): diw_translate --plan-json against v2's own
plan_powder_layer and its intermediate geometry, on the same inputs.

usage:
  python compare_planner.py <v2 engine dir> <diw_translate.exe> <work dir> <corpus.gcode ...> [--allow-version-drift]

Cases (T1):
  - corpus: every core layer that v2_hooks.capture_geometry reaches on each corpus file (plus the generated notch)
    under compare_records.VARIANTS plus hatch_angle 45 / 90 and delta 0.2 / 0.4 / 0.6. The PlanInput carries v2's
    text paths verbatim (walls, paste, sparse infill), v2's h and s_eff, and the outer / overhang walls as open
    lines: the planner forms the rings with its snap + LineMerger port, as v2 does. native_walls are empty.
  - synthetic: a 2 d_eff + 2 um slot, triangle tips of 15 / 22 / 25 degrees, square and round holes, two
    components with equal min y, an Omega with integral H/s, an L with a paste path, each at hatch angles
    0, 30, 45, 90 and -30.
Checks per case (tolerances of PLANNER_DESIGN.md section 10.2):
  st_nc     st and nc equal
  areas     |d ca|, |d oa| <= 1e-5 v + 1e-4; |d ra| <= 1e-4 ra + 1e-3 and the same revert count, unless a v2 narrow
            region is within 1e-3 mm2 of paste_min_area or the cavity has a tip under 23 degrees (logged)
  shape     Hausdorff(boundary C), Hausdorff(boundary Omega) <= 2 um; Omega_native inside Omega_v2 (+) 0.3 um
  hatch     per Omega component (paired in plan order, confirmed by an Omega Hausdorff <= 2 um): components with
            dist(H/s, Z) < 2 um / s are excluded (D5) and counted (< 5 % overall); rows and runs equal, Hausdorff of
            the runs <= 2 um, |d total| <= 1e-5 total + 4 um * rows; point dwells: position <= 2 um, |d d| / d <= 1e-5
  fb        same count (a permutation is logged), points <= 2 um, |d de| <= de / len * 4 um, feed / w / tf / z / ty
            equal; a case whose pieces lie within 1 um of the 0.05 mm minimum is excluded (logged)
  T3        every RUN segment and point dwell, against the text walls and paste: dist - w/2 >= delta + guard - 10 nm;
            with the run points rounded to 3 decimals >= delta; every RUN point inside S. v2's own minimum is
            reported beside the native one.
T4 on the corpus default-variant cases and the synthetics:
  shift     (a) input shifted by (+7, -3) mm: output equal to the shifted original within 1e-9 mm
  jitter    (b) 10 seeds of +-1..10 nm per point plus a 1e-9 rad rotation about the bbox centre (a subset): st, nc,
            areas within 1e-6 relative, Hausdorff(Omega) <= |shift| + 20 nm, runs <= |shift| + 50 nm (D5 components
            may gain or lose one row), T3 holds
  rot90     (c) input rotated by 90 degrees with hatch_angle + 90: the same runs, rotated, within 1 um

Derived tolerances (diw/PLANNER_DESIGN.md "Deviations", stage 2; each use is logged): the cavity Hausdorff is taken
after a 10 um opening (cusp slivers); a failure that disappears against v2 at quad_segs 256 is v2's own arc
discretization; ra inside v2's own spread over quad_segs 32..1024 is accepted when that spread exceeds the
tolerance (ill-conditioned mitre opening); point dwells use the first-order area / centroid bound of the measured
component Hausdorff; T4(b) bounds areas by boundary length x shift (ra also by v2's own quad_segs spread), scales
the Omega bound by 1/sin(a/2) at acute corners and the run bound by 1/sin(theta)
at grazing rows.

A missing exe or a planner error is a FAIL, never a skip. Prints "N checks, F failures"; exit 0 only without failures.
"""
import copy
import json
import math
import os
import platform
import random
import subprocess
import sys
import time

import v2_blocks as B
import v2_hooks as H
from compare_powder import PINNED, parse_config_block, v2_config
from compare_records import VARIANTS, make_notch

PLAN_VARIANTS = dict(VARIANTS)
PLAN_VARIANTS.update({
    "angle45": ["diw_powder_hatch_angle=45"],
    "angle90": ["diw_powder_hatch_angle=90"],
    "delta0.2": ["diw_shell_powder_clearance=0.2"],
    "delta0.4": ["diw_shell_powder_clearance=0.4"],
    "delta0.6": ["diw_shell_powder_clearance=0.6"],
})
SYN_ANGLES = (0.0, 30.0, 45.0, 90.0, -30.0)

UM = 1e-3                     # 1 micrometre in mm
HAUS = 2 * UM
CONTAIN = 0.3 * UM
GUARD = 0.001                 # v2 POWDER_GUARD_MM
R = repr


# ----------------------------------------------------------------------------- bookkeeping

class Checks:
    def __init__(self):
        self.n = 0
        self.fails = []
        self.logs = []
        self.by_kind = {}
        self.records = []

    def check(self, kind, name, ok, detail=""):
        self.records.append((kind, name, ok, detail))
        self.n += 1
        k = self.by_kind.setdefault(kind, [0, 0])
        k[0] += 1
        if not ok:
            k[1] += 1
            self.fails.append(f"{kind} {name}: {detail}")
        return ok

    def log(self, msg):
        self.logs.append(msg)


# ----------------------------------------------------------------------------- PlanInput JSON

def _tp(p):
    return {"ptype": p.ptype, "kind": p.kind, "pts": [[R(float(x)), R(float(y))] for x, y in p.points],
            "seg_de": [R(float(v)) for v in p.seg_de], "seg_feed": [R(float(v)) for v in p.seg_feed],
            "width": R(float(p.width)), "z": R(float(p.z)),
            "travel_feed": None if p.travel_feed is None else R(float(p.travel_feed))}


def plan_input(model, prm, h, s_eff, z=0.0, F=0.0, fu=0.0):
    walls = [p for p in model.paths if p.kind == "wall"]
    return {
        "z": R(float(z)), "h": R(float(h)), "s_eff": R(float(s_eff)), "F": R(float(F)), "fu": R(float(fu)), "fp": 0,
        "U": {"delta": R(prm.delta), "hatch_angle": R(float(prm.hatch_angle)), "point_len": R(prm.point_len),
              "rho_tap": R(prm.rho_tap), "eta": R(prm.eta), "m_dot": R(prm.m_dot),
              "paste_min_area": R(prm.paste_min_area), "width_fallback": R(prm.width_fallback)},
        "open_outer": [[[R(float(x)), R(float(y))] for x, y in p.points] for p in walls
                       if p.ptype in ("outer wall", "overhang wall") and len(p.points) >= 2],
        "text_walls": [_tp(p) for p in walls],
        "text_paste": [_tp(p) for p in model.paths if p.kind == "paste" and len(p.points) >= 2],
        "text_powder": [_tp(p) for p in model.paths if p.kind == "powder"],
    }


def map_points(inp, fn):
    """A copy of a PlanInput JSON with fn applied to every mm point (same point -> same image)."""
    out = copy.deepcopy(inp)
    memo = {}

    def f(pt):
        key = (pt[0], pt[1])
        if key not in memo:
            x, y = fn(float(pt[0]), float(pt[1]))
            memo[key] = [R(x), R(y)]
        return memo[key]
    out["open_outer"] = [[f(p) for p in l] for l in out["open_outer"]]
    for k in ("text_walls", "text_paste", "text_powder"):
        for t in out[k]:
            t["pts"] = [f(p) for p in t["pts"]]
    return out


def run_planner(exe, work, tag, inputs):
    """Runs diw_translate --plan-json on all inputs at once. Returns [(rec or None, dbg dict)] in input order."""
    cases = []
    for i, inp in enumerate(inputs):
        c = dict(inp)
        c["k"] = i + 1
        cases.append(c)
    pin = os.path.join(work, f"{tag}.plan.json")
    pout = os.path.join(work, f"{tag}.blocks.txt")
    pdbg = os.path.join(work, f"{tag}.debug.json")
    with open(pin, "w", encoding="ascii", newline="\n") as fh:
        json.dump({"cases": cases}, fh)
    for p in (pout, pdbg):
        if os.path.exists(p):
            os.remove(p)
    r = subprocess.run([exe, "--plan-json", pin, pout, pdbg], capture_output=True, cwd=os.path.dirname(exe))
    if r.returncode != 0:
        raise RuntimeError(f"diw_translate --plan-json exit {r.returncode}: {r.stderr.decode('utf-8', 'replace')}")
    with open(pout, "rb") as fh:
        blocks, errors = B.parse_records(fh.read())
    if errors:
        raise RuntimeError(f"native blocks do not parse: {errors[:5]}")
    by_k = {b.k: b for b in blocks}
    with open(pdbg, encoding="utf-8") as fh:
        dbg = json.load(fh)["cases"]
    return [(by_k.get(i + 1), dbg[i]) for i in range(len(inputs))]


# ----------------------------------------------------------------------------- geometry helpers

def hausdorff(a, b, sg):
    if a.is_empty and b.is_empty:
        return 0.0
    if a.is_empty or b.is_empty:
        return math.inf
    return a.hausdorff_distance(b)


def runs_geom(runs, sg):
    lines = [sg.LineString(r.pts) for r in runs if r.kind == "line"]
    return sg.MultiLineString(lines) if lines else sg.MultiLineString()


def min_tip_deg(geom, sg):
    best = 180.0
    for poly in _polys(geom):
        for ring in [poly.exterior] + list(poly.interiors):
            c = list(ring.coords)[:-1]
            n = len(c)
            for i in range(n):
                a, b, d = c[i - 1], c[i], c[(i + 1) % n]
                v1 = (a[0] - b[0], a[1] - b[1])
                v2 = (d[0] - b[0], d[1] - b[1])
                l1, l2 = math.hypot(*v1), math.hypot(*v2)
                if l1 < 1e-9 or l2 < 1e-9:
                    continue
                cosang = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (l1 * l2)))
                best = min(best, math.degrees(math.acos(cosang)))
    return best


def _polys(g):
    if g is None or g.is_empty:
        return []
    if g.geom_type == "Polygon":
        return [g]
    if hasattr(g, "geoms"):
        out = []
        for x in g.geoms:
            out += _polys(x)
        return out
    return []


def v2_components(E, sg, V):
    """v2's Omega components in plan order with rows, H, runs and point flag (recomputed as plan_powder_layer)."""
    polys = sorted(V.geom["omega_polys"], key=lambda g: (round(g.bounds[1], 6), round(g.bounds[0], 6)))
    out = []
    ang = V.prm.hatch_angle
    for poly in polys:
        runs = E.hatch_polygon(poly, V.s, ang, sg)
        total = sum(E.PowderRun(r).length for r in runs)
        rp = sg.affinity.rotate(poly, -ang, origin=(0.0, 0.0)) if abs(ang) > 1e-12 else poly
        Hh = rp.bounds[3] - rp.bounds[1]
        rows = int(math.floor(Hh / V.s + 1e-9)) + 1
        out.append({"poly": poly, "runs": runs, "total": total, "point": total < V.prm.point_len, "H": Hh, "rows": rows})
    return out


def split_by_component(runs, comps):
    out, i = [], 0
    for c in comps:
        n = 1 if c["point"] else c["runs"]
        out.append(runs[i:i + n])
        i += n
    return out, i == len(runs)


# ----------------------------------------------------------------------------- T3 clearance

def clearance(sg, shapely, model, runs, S, delta, extra=()):
    """(min slack of record values, min slack after 3-decimal rounding, all points inside S) for a run list.
    extra: further obstacles as (width, LineString) pairs (the native closed wall loops, T2 / T3)."""
    import numpy as np
    obst = {}
    for p in model.paths:
        if p.kind in ("wall", "paste") and len(p.points) >= 2:
            obst.setdefault(float(p.width), []).append(sg.LineString(p.points))
    for w, line in extra:
        obst.setdefault(float(w), []).append(line)
    if not runs or not obst:
        return math.inf, math.inf, True

    def geoms(rnd):
        g = []
        for r in runs:
            pts = [(round(x, 3), round(y, 3)) for x, y in r.pts] if rnd else list(r.pts)
            if r.kind == "point" or len(pts) == 1:
                g.append(sg.Point(pts[0]))
            else:
                g += [sg.LineString([a, b]) if a != b else sg.Point(a) for a, b in zip(pts, pts[1:])]
        return np.array(g, dtype=object)

    out = []
    for rnd in (False, True):
        G = geoms(rnd)
        best = np.full(len(G), np.inf)
        for w, lines in obst.items():
            mls = sg.MultiLineString(lines)
            best = np.minimum(best, shapely.distance(G, mls) - w / 2.0)
        out.append(float(best.min()))
    xs = np.array([x for r in runs for x, _ in r.pts])
    ys = np.array([y for r in runs for _, y in r.pts])
    inside = bool(shapely.contains_xy(S.buffer(1e-9), xs, ys).all()) if S is not None else True
    return out[0] - (delta + GUARD), out[1] - delta, inside


# ----------------------------------------------------------------------------- T1 comparison

class V2Case:
    """v2's side of one case."""
    def __init__(self, name, model, prm, h, s, z, F, fu, plan, geom):
        self.name, self.model, self.prm, self.h, self.s, self.z, self.F, self.fu = name, model, prm, h, s, z, F, fu
        self.plan, self.geom = plan, geom
        self.st = "open" if plan is None else ("small" if not plan.runs else "ok")


def compare_case(E, sg, shapely, chk, stats, V, rec, dbg, mask=None, t3_extra=(), S_native=None):
    """mask: a region left out of the boundary Hausdorff of C and Omega (T2(c): discs around the seam points);
    t3_extra: further T3 obstacles (width, LineString); S_native: the solid the RUN points must lie in (default v2's)."""
    name = V.name
    if rec is None or dbg.get("error"):
        chk.check("st_nc", name, False, f"planner error: {dbg.get('error')}")
        return
    nc_v2 = 0 if V.plan is None else V.plan.n_components
    if not chk.check("st_nc", name, rec.st == V.st and rec.nc == nc_v2,
                     f"st/nc native {rec.st}/{rec.nc} v2 {V.st}/{nc_v2}"):
        return
    if V.st == "open":
        return
    p = V.plan
    # ---- areas
    bad = []
    for key, a, b in (("ca", rec.ca, p.cavity_area), ("oa", rec.oa, p.omega_area)):
        if abs(a - b) > 1e-5 * abs(b) + 1e-4:
            bad.append(f"{key} {a!r} vs {b!r}")
    rv_n = len(_polys(shapely.from_wkt(dbg["revert"])))
    rv_v = len(V.geom["revert"].geoms)
    if abs(rec.ra - p.revert_area) > 1e-4 * p.revert_area + 1e-3 or rv_n != rv_v:
        pma = V.prm.paste_min_area
        near = [g.area for g in _polys(V.geom["narrow"]) if abs(g.area - pma) <= 1e-3]
        tip = min_tip_deg(V.geom["cavity"], sg)
        lo, hi = v2_ra_range(E, sg, V)
        tol = 1e-4 * p.revert_area + 1e-3
        # Review fix 5: the design allows only a revert COUNT mismatch when a v2 narrow region is within 1e-3 mm2 of
        # paste_min_area or the cavity has a tip under 23 deg. Each component that is reverted on one side only has an
        # area within 1e-3 of paste_min_area (it crosses the threshold), so ra may differ by at most that much per
        # count difference (plus the tolerance and v2's own ra spread over quad_segs). Equal counts get no exception.
        cnt = abs(rv_n - rv_v)
        spread = max(hi - lo, 0.0)
        allow = cnt * (pma + 1e-3)
        if (near or tip < 23.0) and cnt > 0 and abs(rec.ra - p.revert_area) <= tol + allow + spread:
            chk.log(f"ra exception {name}: ra {rec.ra!r} vs {p.revert_area!r}, revert count {rv_n} vs {rv_v}"
                    f" (narrow near paste_min_area: {near}, min cavity tip {tip:.1f} deg; |dra| bound "
                    f"{tol + allow + spread:.4g} mm2)")
            stats["ra_exceptions"] += 1
            ILL_RA[name] = max(spread, tol) + allow
        elif hi - lo > tol and lo - tol <= rec.ra <= hi + tol and rv_n == rv_v:
            # The mitre opening is ill-conditioned at acute Omega corners: v2 itself does not converge in
            # quad_segs (its ra spans [lo, hi] over 32..1024). Native inside that span is accepted and logged.
            chk.log(f"ra ill-conditioned {name}: native {rec.ra!r}, v2 {p.revert_area!r}, v2 over quad_segs "
                    f"{RA_QUADS} spans [{lo!r}, {hi!r}]")
            stats["ra_illcond"] = stats.get("ra_illcond", 0) + 1
            ILL_RA[name] = hi - lo
        else:
            bad.append(f"ra {rec.ra!r} vs {p.revert_area!r}, revert count {rv_n} vs {rv_v}")
    chk.check("areas", name, not bad, "; ".join(bad))
    # ---- shapes
    C_n, O_n = shapely.from_wkt(dbg["cavity"]), shapely.from_wkt(dbg["omega"])

    def bnd(g):
        b = g.boundary
        return b if mask is None or b.is_empty else b.difference(mask)
    hc = hausdorff(bnd(opened(C_n)), bnd(opened(V.geom["cavity"])), sg)
    stats["haus_C_raw"] = max(stats.get("haus_C_raw", 0.0), hausdorff(bnd(C_n), bnd(V.geom["cavity"]), sg))
    ho = hausdorff(bnd(O_n), bnd(V.geom["omega"]), sg)
    out = O_n.difference(V.geom["omega"].buffer(CONTAIN, quad_segs=32)).area if not O_n.is_empty else 0.0
    stats["haus_C"] = max(stats["haus_C"], hc)
    stats["haus_O"] = max(stats["haus_O"], ho)
    chk.check("shape", name, hc <= HAUS and ho <= HAUS and out <= 1e-9,
              f"Hausdorff C (10 um openings) {hc:.3g}, Omega {ho:.3g}, Omega outside v2+0.3um {out:.3g} mm2")
    # ---- hatch per component
    comps_v = v2_components(E, sg, V)
    comps_n = dbg["components"]
    polys_n = _polys(O_n)
    runs_v, okv = split_by_component([B.RunRec(r.kind, list(r.pts), r.t_feed) for r in p.runs], comps_v_like(comps_v))
    runs_n, okn = split_by_component(rec.runs, comps_n)
    problems = []
    if len(comps_v) != len(comps_n) or not okv or not okn or len(polys_n) != len(comps_n):
        problems.append(f"component split: v2 {len(comps_v)} native {len(comps_n)} ({okv}, {okn})")
    else:
        order = list(range(len(comps_n)))
        hs = [hausdorff(polys_n[i].boundary, comps_v[i]["poly"].boundary, sg) for i in order]
        if any(h > HAUS for h in hs):
            used, order = set(), []
            for i in range(len(comps_n)):
                cand = min((j for j in range(len(comps_v)) if j not in used),
                           key=lambda j: hausdorff(polys_n[i].boundary, comps_v[j]["poly"].boundary, sg))
                used.add(cand)
                order.append(cand)
            hs = [hausdorff(polys_n[i].boundary, comps_v[order[i]]["poly"].boundary, sg) for i in range(len(order))]
            chk.log(f"component order {name}: paired by Hausdorff {order}")
        for i, j in enumerate(order):
            cn, cv = comps_n[i], comps_v[j]
            stats["components"] += 1
            if hs[i] > HAUS:
                problems.append(f"comp {i}: Omega Hausdorff {hs[i]:.3g}")
                continue
            s = V.s
            d5 = any(abs(x / s - round(x / s)) < 2 * UM / s for x in (float(cn["H"]), cv["H"]))
            if d5:
                # D5: a row lies on a horizontal Omega edge (H/s integral), where GEOS and the analytic hatch may split
                # or drop that edge row differently. Review fix 9: the component is still compared, without its edge
                # rows: both run sets are clipped, in the hatch frame, to the band half a spacing inside the shells'
                # common y range. Only a component with nothing left to compare counts as excluded.
                if cn["point"] and cv["point"]:
                    pass                        # a point dwell has no rows: compared below as usual
                elif cn["point"] != cv["point"]:
                    stats["d5_excluded"] += 1
                    continue
                else:
                    ang = V.prm.hatch_angle

                    def rot(g):
                        return sg.affinity.rotate(g, -ang, origin=(0.0, 0.0)) if abs(ang) > 1e-12 else g
                    pn, pv = rot(polys_n[i]), rot(cv["poly"])
                    y0 = max(pn.bounds[1], pv.bounds[1]) + s / 2.0
                    y1 = min(pn.bounds[3], pv.bounds[3]) - s / 2.0
                    if y1 <= y0:
                        stats["d5_excluded"] += 1
                        continue
                    x0 = min(pn.bounds[0], pv.bounds[0]) - 1.0
                    x1 = max(pn.bounds[2], pv.bounds[2]) + 1.0
                    band = sg.Polygon([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
                    gn = rot(runs_geom(runs_n[i], sg)).intersection(band)
                    gv = rot(runs_geom(runs_v[j], sg)).intersection(band)
                    stats["d5_edge_rows"] = stats.get("d5_edge_rows", 0) + 1
                    if gn.is_empty and gv.is_empty:
                        stats["d5_excluded"] += 1
                        continue
                    if gn.is_empty != gv.is_empty:
                        problems.append(f"comp {i}: D5 interior rows: one side empty")
                        continue
                    hr = hausdorff(gn, gv, sg)
                    stats["haus_runs"] = max(stats["haus_runs"], hr)
                    if hr > HAUS:
                        problems.append(f"comp {i}: D5 interior rows Hausdorff {hr:.3g}")
                    continue
            if cn["point"] != cv["point"] or (cn["rows"] != cv["rows"] and not d5):
                problems.append(f"comp {i}: point/rows native {cn['point']}/{cn['rows']} v2 {cv['point']}/{cv['rows']}")
                continue
            tn, tv = float(cn["total"]), cv["total"]
            if not d5 and abs(tn - tv) > 1e-5 * tv + 4 * UM * cv["rows"]:
                problems.append(f"comp {i}: total {tn!r} vs {tv!r}")
            if cn["point"]:
                a, b = runs_n[i][0], runs_v[j][0]
                dpos = math.hypot(a.pts[0][0] - b.pts[0][0], a.pts[0][1] - b.pts[0][1])
                stats["max_point_pos"] = max(stats["max_point_pos"], dpos)
                stats["max_point_d"] = max(stats["max_point_d"], abs(a.d - b.d) / b.d)
                # d is proportional to the component area and the dwell sits at its centroid: a boundary
                # difference h (the measured component Hausdorff, <= 2 um) over the perimeter P changes the area by
                # <= P h and moves the centroid by <= 2 P h R / A. For small components this exceeds the design's
                # flat 1e-5 / 2 um, so the first-order bound is used (logged when it is the looser one).
                Pm, Am, Rm = poly_props(cv["poly"])
                tol_d = 1e-5 + Pm * hs[i] / Am
                tol_pos = max(HAUS, 2 * Pm * hs[i] * Rm / Am)
                if dpos > tol_pos or abs(a.d - b.d) > tol_d * b.d:
                    problems.append(f"comp {i}: point dwell pos {dpos:.3g} (tol {tol_pos:.3g}) d {a.d!r} vs {b.d!r}"
                                    f" (rel tol {tol_d:.3g})")
                elif dpos > HAUS or abs(a.d - b.d) > 1e-5 * b.d:
                    chk.log(f"point dwell {name} comp {i}: pos {dpos:.3g}, d rel {abs(a.d - b.d) / b.d:.3g} within the"
                            f" area bound (component area {Am:.3g}, perimeter {Pm:.3g}, Hausdorff {hs[i]:.3g})")
            else:
                if len(runs_n[i]) != len(runs_v[j]):
                    problems.append(f"comp {i}: runs native {len(runs_n[i])} v2 {len(runs_v[j])}")
                    continue
                hr = hausdorff(runs_geom(runs_n[i], sg), runs_geom(runs_v[j], sg), sg)
                stats["haus_runs"] = max(stats["haus_runs"], hr)
                if hr > HAUS:
                    problems.append(f"comp {i}: runs Hausdorff {hr:.3g}")
    chk.check("hatch", name, not problems, "; ".join(problems[:4]))
    # ---- fallback paths
    if V.st == "ok":
        compare_fb(chk, stats, name, rec, p)
    # ---- T3
    if V.st == "ok":
        S = V.geom["solid"] if S_native is None else S_native
        sl, slr, ins = clearance(sg, shapely, V.model, rec.runs, S, V.prm.delta, t3_extra)
        vruns = [B.RunRec(r.kind, list(r.pts), r.t_feed) for r in p.runs]
        vsl, vslr, _ = clearance(sg, shapely, V.model, vruns, V.geom["solid"], V.prm.delta)
        stats["t3_min"] = min(stats["t3_min"], sl)
        stats["t3_min_rounded"] = min(stats["t3_min_rounded"], slr)
        stats["t3_v2_min"] = min(stats["t3_v2_min"], vsl)
        chk.check("T3", name, sl >= -1e-5 and slr >= 0.0 and ins,
                  f"slack {sl:.3g} (v2 {vsl:.3g}), rounded {slr:.3g}, inside S {ins}")


RA_QUADS = (32, 64, 128, 256, 512, 1024)
_RA_RANGE = {}
ILL_RA = {}                 # case name -> ra spread accepted for it (ill-conditioned revert, see compare_case)


def v2_ra_range(E, sg, V):
    """(min, max) of v2's revert area over quad_segs RA_QUADS (module global patched per call; v2 not modified)."""
    if V.name not in _RA_RANGE:
        old = E.UMEX_QUAD_SEGS
        vals = []
        try:
            for q in RA_QUADS:
                E.UMEX_QUAD_SEGS = q
                p = E.plan_powder_layer(V.model, V.prm, V.h, V.s, sg)
                vals.append(p.revert_area if p is not None else 0.0)
        finally:
            E.UMEX_QUAD_SEGS = old
        _RA_RANGE[V.name] = (min(vals), max(vals))
    return _RA_RANGE[V.name]


def poly_props(g):
    """(perimeter, area, max distance from the centroid to a vertex) of a polygon."""
    c = g.centroid
    R = max(math.hypot(x - c.x, y - c.y) for ring in [g.exterior] + list(g.interiors) for x, y in ring.coords)
    return g.length, g.area, R


def min_convex_deg(geom):
    """Smallest interior angle (degrees) at a convex vertex of the polygons (shells counter-clockwise)."""
    from shapely.geometry.polygon import orient
    best = 180.0
    for poly in _polys(geom):
        poly = orient(poly, 1.0)
        for ring in [poly.exterior] + list(poly.interiors):
            c = list(ring.coords)[:-1]
            n = len(c)
            for i in range(n):
                a, b, d = c[i - 1], c[i], c[(i + 1) % n]
                d1 = (b[0] - a[0], b[1] - a[1])
                d2 = (d[0] - b[0], d[1] - b[1])
                l1, l2 = math.hypot(*d1), math.hypot(*d2)
                if l1 < 1e-9 or l2 < 1e-9 or d1[0] * d2[1] - d1[1] * d2[0] <= 0:
                    continue
                cosang = max(-1.0, min(1.0, -(d1[0] * d2[0] + d1[1] * d2[1]) / (l1 * l2)))
                best = min(best, math.degrees(math.acos(cosang)))
    return best


def runs_close(ra, rb, omega_b, tol, angle_deg):
    """Every vertex of runs ra is within tol of runs rb, or, where a row meets the Omega boundary at a grazing
    angle theta, within tol / sin(theta) (the row end moves along the row by the boundary shift / sin(theta));
    sin(theta) is floored at 0.05. Returns the worst ratio distance / allowed."""
    import shapely
    gb = runs_geom(rb, shapely.geometry)
    if gb.is_empty:
        return 0.0
    bnd = omega_b.boundary
    th = math.radians(angle_deg)
    u = (math.cos(th), math.sin(th))
    worst = 0.0
    for r in ra:
        for x, y in r.pts:
            pt = shapely.geometry.Point(x, y)
            d = pt.distance(gb)
            if d <= tol:
                continue
            # boundary direction at the nearest boundary point
            s = bnd.project(pt)
            p0, p1 = bnd.interpolate(max(0.0, s - 1e-4)), bnd.interpolate(s + 1e-4)
            tx, ty = p1.x - p0.x, p1.y - p0.y
            tl = math.hypot(tx, ty)
            sin_t = abs(u[0] * ty - u[1] * tx) / tl if tl > 0 else 1.0
            worst = max(worst, d / (tol / max(sin_t, 0.05)))
    return worst


OPEN_EPS = 10 * UM          # cusp tips thinner than 2 OPEN_EPS are left out of the cavity Hausdorff


def opened(g):
    """Morphological opening by OPEN_EPS: removes the cusp tips of sliver cavities between tangent wall bands, where a
    normal difference d moves the tip tangentially by sqrt(2 r d) (11.5 um for v2's own 0.2 um chord error)."""
    if g.is_empty:
        return g
    return g.buffer(-OPEN_EPS, quad_segs=8).buffer(OPEN_EPS, quad_segs=8)


ARBITRATED = ("areas", "shape", "hatch", "fb")
FINE_QUAD_SEGS = 256


def v2_fine(E, sg, V):
    """v2 itself, run with UMEX_QUAD_SEGS = 256 (module global patched for the call; v2 is not modified on disk)."""
    old = E.UMEX_QUAD_SEGS
    try:
        E.UMEX_QUAD_SEGS = FINE_QUAD_SEGS
        plan = E.plan_powder_layer(V.model, V.prm, V.h, V.s, sg)
        geom = H.v2_geometry(E, V.model, V.prm, sg)
    finally:
        E.UMEX_QUAD_SEGS = old
    return V2Case(V.name, V.model, V.prm, V.h, V.s, V.z, V.F, V.fu, plan, geom)


def compare_arbitrated(E, sg, shapely, chk, stats, V, rec, dbg, **kw):
    """compare_case; a tolerance failure of areas / shape / hatch / fb that disappears against v2 run at
    quad_segs = 256 is v2's own arc discretization (D4 / D7): it is logged, not failed. kw: see compare_case."""
    tmp = Checks()
    coarse = blank_stats(stats)
    compare_case(E, sg, shapely, tmp, coarse, V, rec, dbg, **kw)
    failed = {k for k, _, ok, _ in tmp.records if not ok and k in ARBITRATED}
    fine_failed = set()
    fine = None
    if failed and V.plan is not None:
        Vf = v2_fine(E, sg, V)
        tmpf = Checks()
        fine = blank_stats(stats)
        compare_case(E, sg, shapely, tmpf, fine, Vf, rec, dbg, **kw)
        fine_failed = {k for k, _, ok, _ in tmpf.records if not ok}
        fine_detail = {k: d for k, _, ok, d in tmpf.records if not ok}
        nc_f = 0 if Vf.plan is None else Vf.plan.n_components
        nc_c = 0 if V.plan is None else V.plan.n_components
        if "st_nc" in fine_failed or Vf.st != V.st or nc_f != nc_c:
            # Review fix 7: compare_case stops at a failed st/nc check, so the fine run then has no areas / shape /
            # hatch / fb records at all. v2 at quad_segs 256 is then a different plan (status or component count):
            # it cannot arbitrate anything, and the coarse failures stand.
            chk.log(f"v2 discretization {V.name}: v2 at quad_segs={FINE_QUAD_SEGS} has another status / component "
                    f"count ({Vf.st}/{nc_f} vs {V.st}/{nc_c}): not arbitrated")
            fine_failed = set(ARBITRATED)
            fine_detail = {k: fine_detail.get(k, "") or f"v2 at quad_segs={FINE_QUAD_SEGS} gives st/nc {Vf.st}/{nc_f}"
                           for k in ARBITRATED}
            fine = None
    final_ok = all(ok or (kind in failed and kind not in fine_failed) for kind, _, ok, _ in tmp.records)
    # Review fix 8: the statistics are those of the accepted comparison. Every check passes against coarse v2: the
    # coarse metrics. Arbitrated against v2 at quad_segs 256: per metric the better of the two references (each kind
    # passed against one of them). A failure: the coarse metrics (they show the failure).
    if final_ok and fine is not None:
        merge_stats(stats, coarse, best_of=fine)
    else:
        merge_stats(stats, coarse)
    for kind, name, ok, detail in tmp.records:
        if not ok and kind in failed and kind not in fine_failed:
            stats["v2_discretization"] = stats.get("v2_discretization", 0) + 1
            chk.log(f"v2 discretization {kind} {name}: vs v2 [{detail}]; within tolerance of v2 at quad_segs={FINE_QUAD_SEGS}")
            chk.check(kind, name, True)
        elif not ok and kind in failed:
            chk.check(kind, name, False, f"{detail} || vs v2 quad_segs={FINE_QUAD_SEGS}: {fine_detail.get(kind, '')}")
        else:
            chk.check(kind, name, ok, detail)
    chk.logs += tmp.logs


def blank_stats(like):
    """A fresh statistics dict with the keys of `like` (counters 0, maxima 0.0, minima inf)."""
    return {k: (0 if isinstance(v, int) and not isinstance(v, bool) else (math.inf if k.startswith("t3_") else 0.0))
            for k, v in like.items() if isinstance(v, (int, float))}


def merge_stats(dst, src, best_of=None):
    """Adds the statistics of one accepted comparison to dst: counters are added, t3_* minima take the minimum and the
    other floats (Hausdorff, positions) the maximum. best_of: a second comparison of the same block against another
    reference; each float metric is then taken from the reference where it is better (smaller, larger for t3_*)."""
    for k, v in src.items():
        if best_of is not None and k in best_of and isinstance(v, float):
            w = best_of[k]
            v = max(v, w) if k.startswith("t3_") else min(v, w)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            dst[k] = v
        elif isinstance(v, int):
            dst[k] = dst.get(k, 0) + v
        elif k.startswith("t3_"):
            dst[k] = min(dst.get(k, math.inf), v)
        else:
            dst[k] = max(dst.get(k, 0.0), v)


def comps_v_like(comps_v):
    return [{"point": c["point"], "runs": len(c["runs"])} for c in comps_v]


def compare_fb(chk, stats, name, rec, plan):
    fv = [B.FbRec(w=p.width, tf=p.travel_feed, z=p.z, ty=p.ptype, pts=list(p.points),
                  segs=list(zip(p.seg_de, p.seg_feed))) for p in plan.fallback_paths]
    fn = rec.fbs

    def lens(fbs):
        return [math.hypot(b[0] - a[0], b[1] - a[1]) for f in fbs for a, b in zip(f.pts, f.pts[1:])]

    def dpos(a, b):
        return max(math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(a.pts, b.pts))

    def accept(pairs):
        # review fix 8: fb_pos is measured on the pairs that were accepted, not on every pair tried
        for a, b in pairs:
            stats["fb_pos"] = max(stats["fb_pos"], dpos(a, b))

    def same(a, b):
        if len(a.pts) != len(b.pts):
            return f"n {len(a.pts)} vs {len(b.pts)}"
        if (R(a.w), R(a.tf) if a.tf is not None else "-", R(a.z), a.ty) != \
                (R(b.w), R(b.tf) if b.tf is not None else "-", R(b.z), b.ty):
            return "w/tf/z/ty"
        dp = dpos(a, b)
        if dp > HAUS:
            return f"points {dp:.3g}"
        for (da, fa), (db, fb_), p0, p1 in zip(a.segs, b.segs, b.pts, b.pts[1:]):
            L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            if R(fa) != R(fb_):
                return "feed"
            if abs(da - db) > (abs(db) / L) * 4 * UM if L > 0 else abs(da - db) > 1e-12:
                return f"de {da!r} vs {db!r}"
        return None

    near = [l for l in lens(fn) + lens(fv) if abs(l - 0.05) <= UM]
    if len(fn) == len(fv):
        errs = [same(a, b) for a, b in zip(fn, fv)]
        if not any(errs):
            accept(zip(fn, fv))
            chk.check("fb", name, True)
            return
        # a permutation (GEOS piece order) is logged, not failed
        rest = list(fv)
        perm_ok = True
        pairs = []
        for a in fn:
            j = next((j for j, b in enumerate(rest) if same(a, b) is None), None)
            if j is None:
                perm_ok = False
                break
            pairs.append((a, rest.pop(j)))
        if perm_ok:
            accept(pairs)
            chk.log(f"fb order {name}: same pieces in another order")
            chk.check("fb", name, True)
            return
        if near:
            chk.log(f"fb excluded {name}: pieces within 1 um of 0.05 mm ({errs})")
            stats["fb_excluded"] += 1
            chk.check("fb", name, True)
            return
        if name in ILL_RA and not any(e in ("w/tf/z/ty", "feed") or (e or "").startswith("n ") for e in errs):
            chk.log(f"fb {name}: revert accepted as ill-conditioned, fallback points differ ({errs})")
            stats["fb_illcond"] = stats.get("fb_illcond", 0) + 1
            chk.check("fb", name, True)
            return
        chk.check("fb", name, False, f"{[e for e in errs if e][:3]}")
        return
    if name in ILL_RA:
        chk.log(f"fb {name}: revert accepted as ill-conditioned (tip / narrow), fallback count {len(fn)} vs {len(fv)}")
        stats["fb_illcond"] = stats.get("fb_illcond", 0) + 1
        chk.check("fb", name, True)
        return
    if near:
        chk.log(f"fb excluded {name}: count {len(fn)} vs {len(fv)}, pieces within 1 um of 0.05 mm")
        stats["fb_excluded"] += 1
        chk.check("fb", name, True)
        return
    chk.check("fb", name, False, f"count native {len(fn)} v2 {len(fv)}")


# ----------------------------------------------------------------------------- synthetic layers

def synthetic_cases(E, sg, prm0, h=0.6, z=1.2):
    w = 0.6
    F, s_eff, fu, _ = E.resolve_powder_feed(prm0.m_dot, prm0.rho_tap, prm0.s_hatch, h, prm0.eta, prm0.f_max, prm0.policy)
    d_eff = prm0.delta + GUARD

    def r3(pts):
        return [(round(x, 3), round(y, 3)) for x, y in pts]

    def path(ptype, pts, width=w, travel=6000.0):
        pts = r3(pts)
        segs = list(zip(pts, pts[1:]))
        de = [round(0.05 * math.hypot(b[0] - a[0], b[1] - a[1]), 5) for a, b in segs]
        return E.PathRec(ptype=ptype, kind=E._path_kind(ptype), points=pts, seg_de=de, seg_feed=[1200.0] * len(de),
                         seg_raw=["G1"] * len(de), width=width, z=z, pre_lines=[], travel_feed=travel)

    def loop(pts):
        return list(pts) + [pts[0]]

    def inner(poly_pts, d):
        g = sg.Polygon(poly_pts).buffer(-d, join_style="mitre")
        return [loop(list(p.exterior.coords)[:-1]) for p in _polys(g)]

    def sparse(poly, pitch=2.0, ang=45.0):
        out = []
        minx, miny, maxx, maxy = poly.bounds
        c = ((minx + maxx) / 2, (miny + maxy) / 2)
        L = math.hypot(maxx - minx, maxy - miny)
        k = 0
        t = -L
        while t <= L:
            ln = sg.LineString([(c[0] - L, c[1] + t), (c[0] + L, c[1] + t)])
            ln = sg.affinity.rotate(ln, ang, origin=c)
            for piece in _polys_lines(poly.intersection(ln)):
                pts = list(piece.coords)
                out.append(path("sparse infill", pts if k % 2 == 0 else pts[::-1], travel=None if k == 0 else 6000.0))
            t += pitch
            k += 1
        return out

    shapes = {}
    # slot: two squares joined by a vertical corridor whose cavity is 2 d_eff + 2 um wide
    c = 2 * d_eff + 0.002 + w
    x0, x1 = 4.5 - c / 2, 4.5 + c / 2
    slot = [(0, 0), (9, 0), (9, 9), (x1, 9), (x1, 12), (9, 12), (9, 21), (0, 21), (0, 12), (x0, 12), (x0, 9), (0, 9)]
    shapes["slot"] = [path("outer wall", loop(slot))] + sparse(sg.Polygon(slot).buffer(-1.0))
    for tip in (15, 22, 25):
        hw = 20 * math.tan(math.radians(tip / 2))
        tri = [(0, 0), (hw, 20), (-hw, 20)]
        shapes[f"tip{tip}"] = ([path("outer wall", loop(tri))] + [path("inner wall", l) for l in inner(tri, w)] +
                               sparse(sg.Polygon(tri).buffer(-1.0)))
    sq = [(0, 0), (20, 0), (20, 20), (0, 20)]
    hole = [(5, 5), (5, 9), (9, 9), (9, 5)]
    circ = [(14 + 3 * math.cos(2 * math.pi * i / 32), 14 - 3 * math.sin(2 * math.pi * i / 32)) for i in range(32)]
    shapes["holes"] = ([path("outer wall", loop(sq)), path("outer wall", loop(hole)), path("outer wall", loop(circ))] +
                       [path("inner wall", l) for l in inner(sq, w)] +
                       sparse(sg.Polygon(sq, [hole, circ]).buffer(-1.0)))
    a = [(0, 0), (8, 0), (8, 8), (0, 8)]
    b = [(12, 0), (20, 0), (20, 8), (12, 8)]
    shapes["two_equal_miny"] = ([path("outer wall", loop(a)), path("outer wall", loop(b))] +
                                sparse(sg.Polygon(a).buffer(-1.0)) + sparse(sg.Polygon(b).buffer(-1.0)))
    side = 10 * s_eff + w + 2 * d_eff        # Omega height 10 s at angle 0 (D5)
    sq2 = [(0, 0), (side, 0), (side, side), (0, side)]
    shapes["integral_Hs"] = [path("outer wall", loop(sq2))] + sparse(sg.Polygon(sq2).buffer(-1.0))
    ell = [(0, 0), (20, 0), (20, 8), (8, 8), (8, 20), (0, 20)]
    shapes["lshape_paste"] = ([path("outer wall", loop(ell)), path("gap infill", [(2, 2), (6, 6), (6, 15)], width=0.5),
                               path("internal solid infill", [(10, 2), (18, 2), (18, 5)], width=0.6)] +
                              sparse(sg.Polygon(ell).buffer(-1.0)))
    out = []
    for nm, paths in shapes.items():
        for p in paths:                       # on the bed, away from the origin
            p.points = [(round(x + 40.0, 3), round(y + 40.0, 3)) for x, y in p.points]
        model = E.LayerModel(header=[], paths=paths, tail=[], z=z, z_feed=None)
        for ang in SYN_ANGLES:
            prm = copy.copy(prm0)
            prm.hatch_angle = ang
            out.append((f"syn.{nm}.a{ang:g}", model, prm, h, s_eff, z, F, fu))
    return out


def _polys_lines(g):
    if g is None or g.is_empty:
        return []
    if g.geom_type == "LineString":
        return [g]
    if hasattr(g, "geoms"):
        out = []
        for x in g.geoms:
            out += _polys_lines(x)
        return out
    return []


# ----------------------------------------------------------------------------- T4

def same_output(a, b, fn, tol):
    """Native outputs a (base) and b (transformed): b == fn(a) within tol. Returns None or a problem."""
    if a.st != b.st or a.nc != b.nc:
        return f"st/nc {a.st}/{a.nc} vs {b.st}/{b.nc}"
    for k in ("ca", "oa", "ra", "hl"):
        x, y = getattr(a, k), getattr(b, k)
        if abs(x - y) > 1e-9 * max(1.0, abs(x)):
            return f"{k} {x!r} vs {y!r}"
    if len(a.runs) != len(b.runs) or len(a.fbs) != len(b.fbs):
        return f"runs/fb count {len(a.runs)}/{len(a.fbs)} vs {len(b.runs)}/{len(b.fbs)}"
    worst = 0.0
    for ra, rb in zip(a.runs, b.runs):
        if ra.kind != rb.kind or len(ra.pts) != len(rb.pts):
            return "run structure"
        for p, q in zip(ra.pts, rb.pts):
            x, y = fn(*p)
            worst = max(worst, math.hypot(x - q[0], y - q[1]))
    for fa, fb in zip(a.fbs, b.fbs):
        if len(fa.pts) != len(fb.pts):
            return "fb structure"
        for p, q in zip(fa.pts, fb.pts):
            x, y = fn(*p)
            worst = max(worst, math.hypot(x - q[0], y - q[1]))
        for (da, _), (db, _) in zip(fa.segs, fb.segs):
            if abs(da - db) > 1e-9 * max(1.0, abs(da)):
                return f"fb de {da!r} vs {db!r}"
    if worst > tol:
        return f"points off by {worst:.3g} mm"
    return None


def same_run_set(a, b, fn, tol, ra_tol=0.0):
    """T4(c): the runs of b are the runs of a mapped by fn, in any component order, within tol."""
    if a.st != b.st or a.nc != b.nc:
        return f"st/nc {a.st}/{a.nc} vs {b.st}/{b.nc}"
    for k in ("ca", "oa", "ra", "hl"):
        x, y = getattr(a, k), getattr(b, k)
        if abs(x - y) > max(1e-5 * max(1.0, abs(x)), ra_tol if k == "ra" else 0.0):
            return f"{k} {x!r} vs {y!r}"
    if len(a.runs) != len(b.runs) or len(a.fbs) != len(b.fbs):
        return f"runs/fb count {len(a.runs)}/{len(a.fbs)} vs {len(b.runs)}/{len(b.fbs)}"
    rest = list(b.runs)
    for ra in a.runs:
        img = [fn(*p) for p in ra.pts]
        j = next((j for j, rb in enumerate(rest) if rb.kind == ra.kind and len(rb.pts) == len(img) and
                  all(math.hypot(x - q[0], y - q[1]) <= tol for (x, y), q in zip(img, rb.pts))), None)
        if j is None:
            return f"run with {len(ra.pts)} points at {img[0]} has no rotated twin"
        rest.pop(j)
    if ra_tol > 0.0:        # ill-conditioned revert (accepted in T1): the fallback clip ends may move
        return None
    for fa, fb in zip(a.fbs, b.fbs):
        if len(fa.pts) != len(fb.pts) or any(math.hypot(fn(*p)[0] - q[0], fn(*p)[1] - q[1]) > tol
                                             for p, q in zip(fa.pts, fb.pts)):
            return "fallback paths differ"
    return None


def map_model(E, model, fn):
    paths = [E.PathRec(ptype=p.ptype, kind=p.kind, points=[fn(float(x), float(y)) for x, y in p.points],
                       seg_de=p.seg_de, seg_feed=p.seg_feed, seg_raw=p.seg_raw, width=p.width, z=p.z,
                       pre_lines=p.pre_lines, travel_feed=p.travel_feed) for p in model.paths]
    return E.LayerModel(header=model.header, paths=paths, tail=model.tail, z=model.z, z_feed=model.z_feed)


def jitter_fn(seed, inp):
    rng = random.Random(seed)
    xs = [float(p[0]) for l in inp["open_outer"] for p in l] + [float(p[0]) for k in ("text_walls", "text_paste", "text_powder")
                                                                  for t in inp[k] for p in t["pts"]]
    ys = [float(p[1]) for l in inp["open_outer"] for p in l] + [float(p[1]) for k in ("text_walls", "text_paste", "text_powder")
                                                                  for t in inp[k] for p in t["pts"]]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    th = 1e-9
    ct, st = math.cos(th), math.sin(th)
    memo = {}

    def fn(x, y):
        if (x, y) not in memo:
            jx = rng.choice((-1, 1)) * rng.randint(1, 10) * 1e-6
            jy = rng.choice((-1, 1)) * rng.randint(1, 10) * 1e-6
            u, v = x + jx - cx, y + jy - cy
            memo[(x, y)] = (cx + u * ct - v * st, cy + u * st + v * ct)
        return memo[(x, y)]
    return fn


# ----------------------------------------------------------------------------- main

def main(argv):
    allow_drift = "--allow-version-drift" in argv
    argv = [a for a in argv if a != "--allow-version-drift"]
    if len(argv) < 5:
        print(__doc__)
        return 2
    engine_dir, exe, work = argv[1], os.path.abspath(argv[2]), os.path.abspath(argv[3])
    corpus = argv[4:]
    sys.path.insert(0, engine_dir)
    import diw_engine as E  # noqa: E402
    import shapely  # noqa: E402
    versions = {"python": platform.python_version(), "shapely": shapely.__version__, "geos": shapely.geos_version_string}
    print(f"versions: {versions}")
    if versions != PINNED and not allow_drift:
        print(f"version drift from {PINNED}; pass --allow-version-drift to run anyway")
        return 2
    if not os.path.isfile(exe):
        print(f"FAIL: no diw_translate at {exe}")
        return 2
    with open(exe, "rb") as fh:
        if b"--plan-json" not in fh.read():
            print("FAIL: diw_translate has no --plan-json")
            return 2
    os.makedirs(work, exist_ok=True)
    sg = E._shapely()
    from shapely.geometry import MultiLineString  # noqa: E402
    sg.MultiLineString = MultiLineString
    chk = Checks()
    stats = {"components": 0, "d5_excluded": 0, "ra_exceptions": 0, "fb_excluded": 0, "haus_C": 0.0, "haus_O": 0.0,
             "haus_runs": 0.0, "max_point_pos": 0.0, "max_point_d": 0.0, "fb_pos": 0.0, "t3_min": math.inf,
             "t3_min_rounded": math.inf, "t3_v2_min": math.inf}
    t0 = time.time()

    # ---- collect v2 cases
    cases = []          # (V2Case, PlanInput)
    t4_base = []        # indices into cases for T4
    inputs = []
    for path in corpus:
        with open(path, "rb") as fh:
            inputs.append((os.path.splitext(os.path.basename(path))[0], fh.read(), PLAN_VARIANTS))
    if inputs:
        inputs.append(("notch", make_notch(inputs[0][1]), {"notch": [], "notch_angle30": ["diw_powder_hatch_angle=30"]}))
    prm_syn = None
    for cname, data, variants in inputs:
        for vname, ov in variants.items():
            src = os.path.join(work, f"{cname}.{vname}.in.gcode")
            with open(src, "wb") as fh:
                fh.write(data)
            diw = parse_config_block(data)
            for kv in ov:
                k, v = kv.split("=", 1)
                diw[k] = v
            cfg = v2_config(diw, E)
            cfg["enable_u_mex_powder_infill"] = True
            layers = H.capture_geometry(E, cfg, src)
            os.remove(src)
            if prm_syn is None and layers:
                prm_syn = layers[0].prm
            for L in layers:
                V = V2Case(f"{cname}.{vname}.k{L.k}", L.model, L.prm, L.h, L.s, L.z, L.F, L.fu, L.plan, L.geom)
                if vname in ("default", "notch"):
                    t4_base.append(len(cases))
                cases.append((V, plan_input(L.model, L.prm, L.h, L.s, L.z, L.F, L.fu)))
            print(f"captured {cname}.{vname}: {len(layers)} core layers ({time.time() - t0:.0f} s)")
    if prm_syn is not None:
        for nm, model, prm, h, s, z, F, fu in synthetic_cases(E, sg, prm_syn):
            plan = E.plan_powder_layer(model, prm, h, s, sg)
            geom = H.v2_geometry(E, model, prm, sg)
            V = V2Case(nm, model, prm, h, s, z, F, fu, plan, geom)
            t4_base.append(len(cases))
            cases.append((V, plan_input(model, prm, h, s, z, F, fu)))
    print(f"{len(cases)} cases ({len(t4_base)} T4 bases)")

    # ---- T1 + T3
    results = run_planner(exe, work, "t1", [c[1] for c in cases])
    for (V, _), (rec, dbg) in zip(cases, results):
        compare_arbitrated(E, sg, shapely, chk, stats, V, rec, dbg)
    print(f"T1 done ({time.time() - t0:.0f} s)")

    # ---- T4 (a) shift and (c) rot90 on the bases
    shifted = [map_points(cases[i][1], lambda x, y: (x + 7.0, y - 3.0)) for i in t4_base]
    rot = []
    for i in t4_base:
        inp = map_points(cases[i][1], lambda x, y: (-y, x))
        inp["U"]["hatch_angle"] = R(float(inp["U"]["hatch_angle"]) + 90.0)
        rot.append(inp)
    res_s = run_planner(exe, work, "t4a", shifted)
    res_r = run_planner(exe, work, "t4c", rot)
    for n, i in enumerate(t4_base):
        V = cases[i][0]
        base = results[i][0]
        a, b = res_s[n][0], res_r[n][0]
        if base is None or a is None or b is None:
            chk.check("T4a", V.name, False, "planner error")
            chk.check("T4c", V.name, False, "planner error")
            continue
        pa = same_output(base, a, lambda x, y: (x + 7.0, y - 3.0), 1e-9)
        if pa is None:
            ca = [c["rows"] for c in results[i][1].get("components", [])]
            cb = [c["rows"] for c in res_s[n][1].get("components", [])]
            pa = None if ca == cb else f"rows {ca} vs {cb}"
        chk.check("T4a", V.name, pa is None, pa or "")
        lo, hi = v2_ra_range(E, sg, V) if abs(base.ra - b.ra) > 1e-5 * max(1.0, base.ra) else (0.0, 0.0)
        pc = same_run_set(base, b, lambda x, y: (-y, x), 1 * UM, max(ILL_RA.get(V.name, 0.0), hi - lo))
        chk.check("T4c", V.name, pc is None, pc or "")
    print(f"T4 a/c done ({time.time() - t0:.0f} s)")

    # ---- T4 (b) jitter, on a subset: per input file and synthetic shape, up to 3 st=ok bases
    sub, seen = [], {}
    for i in t4_base:
        V = cases[i][0]
        key = V.name.rsplit(".", 1)[0] if V.name.startswith("syn.") else V.name.split(".")[0]
        if V.st == "ok" and seen.get(key, 0) < 3:
            seen[key] = seen.get(key, 0) + 1
            sub.append(i)
    jit_inputs, jit_meta = [], []
    for i in sub:
        for seed in range(10):
            fn = jitter_fn(1000 + seed, cases[i][1])
            jit_inputs.append(map_points(cases[i][1], fn))
            jit_meta.append((i, seed, fn))
    res_j = run_planner(exe, work, "t4b", jit_inputs)
    shift = 10e-6 * math.sqrt(2) + 1e-9 * 200.0
    for (i, seed, fn), (rec, dbg) in zip(jit_meta, res_j):
        V = cases[i][0]
        base, bdbg = results[i]
        name = f"{V.name}.seed{seed}"
        if rec is None or base is None:
            chk.check("T4b", name, False, f"planner error {dbg.get('error')}")
            continue
        probs = []
        if rec.st != base.st or rec.nc != base.nc:
            probs.append(f"st/nc {rec.st}/{rec.nc} vs {base.st}/{base.nc}")
        else:
            for k, g in (("ca", "cavity"), ("oa", "omega"), ("ra", "revert")):
                x, y = getattr(base, k), getattr(rec, k)
                L = shapely.from_wkt(bdbg[g]).boundary.length
                bound = 1e-6 * abs(x) + L * (shift + 20e-6)
                if k == "ra" and abs(x - y) > bound:
                    lo, hi = v2_ra_range(E, sg, V)      # v2's own spread: ill-conditioned mitre opening
                    bound = max(bound, hi - lo)
                if abs(x - y) > bound:
                    probs.append(f"{k} {x!r} vs {y!r} (bound {bound:.3g})")
            om_b = shapely.from_wkt(bdbg["omega"])
            ho = hausdorff(shapely.from_wkt(dbg["omega"]).boundary, om_b.boundary, sg)
            # a convex Omega corner of angle a moves by shift / sin(a / 2) (amplification capped at 20)
            amp = min(20.0, 1.0 / math.sin(math.radians(min_convex_deg(om_b)) / 2.0))
            if ho > (shift + 20e-6) * amp:
                probs.append(f"Omega Hausdorff {ho:.3g} (bound {(shift + 20e-6) * amp:.3g})")
            rows_b = [c["rows"] for c in bdbg["components"]]
            rows_j = [c["rows"] for c in dbg["components"]]
            d5 = any(abs(float(c["H"]) / V.s - round(float(c["H"]) / V.s)) < 2 * UM / V.s for c in bdbg["components"])
            if rows_b != rows_j and not (d5 and all(abs(a - b) <= 1 for a, b in zip(rows_b, rows_j))):
                probs.append(f"rows {rows_b} vs {rows_j}")
            hr = hausdorff(runs_geom(rec.runs, sg), runs_geom(base.runs, sg), sg)
            stats.setdefault("t4b_runs", 0.0)
            stats["t4b_runs"] = max(stats["t4b_runs"], hr)
            if hr > shift + 50e-6 and not d5:
                om_j = shapely.from_wkt(dbg["omega"])
                w1 = runs_close(rec.runs, base.runs, om_b, shift + 50e-6, V.prm.hatch_angle)
                w2 = runs_close(base.runs, rec.runs, om_j, shift + 50e-6, V.prm.hatch_angle)
                if max(w1, w2) > 1.0:
                    probs.append(f"runs Hausdorff {hr:.3g} (grazing-corrected ratio {max(w1, w2):.3g})")
                else:
                    chk.log(f"T4b {name}: runs Hausdorff {hr:.3g} at rows grazing the Omega boundary (ratio "
                            f"{max(w1, w2):.3g} of shift / sin(theta))")
            if rec.st == "ok":
                sl, slr, ins = clearance(sg, shapely, map_model(E, V.model, fn), rec.runs, V.geom["solid"], V.prm.delta)
                if not (sl >= -1e-5 and slr >= 0.0 and ins):
                    probs.append(f"T3 slack {sl:.3g} rounded {slr:.3g} inside {ins}")
        chk.check("T4b", name, not probs, "; ".join(probs[:3]))
    print(f"T4 b done ({time.time() - t0:.0f} s)")

    # ---- report
    frac = stats["d5_excluded"] / max(1, stats["components"])
    chk.check("D5", "excluded fraction", frac < 0.05, f"{stats['d5_excluded']}/{stats['components']} = {frac:.3%}")
    for m in chk.logs[:60]:
        print(f"LOG  {m}")
    if len(chk.logs) > 60:
        print(f"LOG  ... {len(chk.logs) - 60} more")
    for f in chk.fails[:80]:
        print(f"FAIL {f}")
    if len(chk.fails) > 80:
        print(f"FAIL ... {len(chk.fails) - 80} more")
    print("per kind (checks, failures): " + ", ".join(f"{k} {v[0]}/{v[1]}" for k, v in chk.by_kind.items()))
    print("stats: " + ", ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in stats.items()))
    with open(os.path.join(work, "compare_planner.report.json"), "w", encoding="utf-8") as fh:
        json.dump({"versions": versions, "checks": chk.n, "failures": chk.fails, "logs": chk.logs,
                   "by_kind": chk.by_kind, "stats": {k: (None if v == math.inf else v) for k, v in stats.items()}}, fh,
                  indent=1)
    print(f"{chk.n} checks, {len(chk.fails)} failures ({time.time() - t0:.0f} s)")
    return 0 if not chk.fails else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
