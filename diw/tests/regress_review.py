"""Regression tests of the increment-4 review fixes (diw/PLANNER_DESIGN.md, Deviations, "Review fixes").

usage (from build/src/Release):
  py -3.15 -E -s regress_review.py --engine <v2 dir> --release <build/src/Release> --datadir <orca datadir>
         --work <dir> --slices <shadow_export matrix work>/slices

  R1  custom G-code with placeholder-like ";DIWP" lines (layer_change_gcode): the lines are kept byte for byte, the
      records and the planner summary equal those of the export without them (C++ side: T8 Catch2 case
      "placeholder-like custom G-code is ordinary text").
  R2  a cube plus a thin tube printed as one bead (Arachne, and classic with detect_thin_wall): the bead loop is no
      ring (summary single_bead > 0), every block has one Omega component (the cube's), and no RUN point lies in the
      tube or its bore.
  R3  a cube plus slender tapered prisms (classic walls, delta 0.2): every RUN segment and point dwell keeps
      delta + guard from every printed wall / paste segment at that segment's OWN ;WIDTH: (read from the text, not
      from the planner's models); the plate reaches the defect (gap fill growing inside one path).
  R4  a capture error copied into the reference: the native piece widths of the classic cube25 matrix slice's dump
      x1.4, replanned offline; compare_t2c must fail (no D2 native band acceptance, pieces refused as unverified).
  R5  T1 ra: ra + 50 mm2 on a cube25 corpus layer whose cavity has a tip < 23 deg must fail.
  R6  the v2-ill-conditioned exact-clearance path (bridge_obj.arachne.base matrix slice, the layer that takes it):
      with two thirds of the hatch runs removed, hatch must fail.
  R7  quad_segs 256 arbitration: when v2 at 256 has another component count, an area error must fail.
  R8  statistics of the accepted comparison: fb_pos on the matched pairs only; compare_arbitrated reports the
      metrics of the reference it accepted.
  R9  the D5 exclusion bound of shadow_export (d5_fraction_ok).
Prints "N checks, F failures"; exit code 1 on any failure. A missing input is a failure, never a skip.
"""
import copy
import json
import math
import os
import re
import shutil
import subprocess
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import shadow_export as SE  # noqa: E402

UM = 1e-3


class Checks:
    def __init__(self):
        self.n, self.fails = 0, []

    def check(self, name, ok, detail=""):
        self.n += 1
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {detail}", flush=True)
        if not ok:
            self.fails.append(f"{name}: {detail}")
        return ok


def opt(argv, name):
    return argv[argv.index(name) + 1] if name in argv else None


def profile_value(key):
    with open(os.path.join(HERE, "profiles", "flat_machine_diw.json"), encoding="utf-8-sig") as fh:
        return json.load(fh)[key]


def export(ctx, name, stls, ov, dump=True, extra=()):
    """Slices stls with the DIW profiles + ov (shadow on, raw file kept); returns (rc, data, summary, out dir)."""
    out = os.path.join(ctx.work, name)
    settings = SE.write_profiles(os.path.join(ctx.work, "profiles"), name, ov)
    env = {}
    if dump:
        env["ORCA_DIW_SHADOW_DUMP"] = os.path.join(ctx.work, name + ".dump")
        if os.path.isdir(env["ORCA_DIW_SHADOW_DUMP"]):
            shutil.rmtree(env["ORCA_DIW_SHADOW_DUMP"])
        os.makedirs(env["ORCA_DIW_SHADOW_DUMP"])
    rc, data, log, _ = SE.slice_part(ctx.orca, ctx.datadir, stls, out, env, settings=settings, extra_args=list(extra))
    return rc, data, SE.parse_summary(log), out


def chunks_of(E, data):
    lines = data.decode("utf-8", "replace").splitlines(keepends=True)
    infos = E.scan_gcode(lines)
    return lines, infos, {c.index: c for c in E.split_layers(lines, infos)}


# ----------------------------------------------------------------------------------------------- R1

def r1(ctx, chk):
    E = ctx.E
    import v2_blocks as B
    stls = [os.path.join(ctx.mdir, "box20.stl")]
    user = ";DIWP 3 m=1\n;DIWP 2 m=0\n;DIWP 0123456789abcdef 4 m=1"
    lcg = profile_value("layer_change_gcode")
    rc0, d0, s0, _ = export(ctx, "r1_plain", stls, {}, dump=False)
    rc1, d1, s1, _ = export(ctx, "r1_user", stls, {"layer_change_gcode": lcg + "\n" + user}, dump=False)
    if not chk.check("R1.export", rc0 == 0 and rc1 == 0 and d0 and d1 and s0 and s1, f"rc {rc0}/{rc1}"):
        return
    # the custom G-code also appears in the config block as one escaped line: count whole lines only
    n_lc = d1.count(b"\n;AFTER_LAYER_CHANGE\n")
    counts = [d1.count(("\n" + l + "\n").encode()) for l in user.split("\n")]
    chk.check("R1.user_lines_kept", n_lc > 0 and all(c == n_lc for c in counts),
              f"{n_lc} layer changes; whole-line counts of the user lines {counts}")
    rec0 = [l for l in d0.splitlines() if l.startswith(b";DIW:")]
    rec1 = [l for l in d1.splitlines() if l.startswith(b";DIW:")]
    chk.check("R1.record_lines_equal", rec0 == rec1 and len(rec0) > 0, f"{len(rec0)} vs {len(rec1)} record lines")
    chk.check("R1.summary", s0["reasons"] == s1["reasons"] and s0["blocks"] == s1["blocks"] and
              not any(k.startswith(("disagree", "merged", "lost")) for k in s1["reasons"]),
              f"{s0['reasons']} / {s1['reasons']}")


# ----------------------------------------------------------------------------------------------- R2

def tube_centre(E, lines, infos, chunk):
    m = E.parse_core_layer(lines, infos, chunk, 1.4)
    for p in m.paths:
        if p.kind == "wall" and p.ptype == "outer wall" and len(p.points) > 8:
            xs = [x for x, _ in p.points]
            ys = [y for _, y in p.points]
            if max(xs) - min(xs) < 13.0 and max(ys) - min(ys) < 13.0:
                return ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0)
    return None


def r2(ctx, chk):
    E = ctx.E
    import v2_blocks as B
    stls = [os.path.join(ctx.mdir, "box20.stl"), os.path.join(ctx.mdir, "tube_thin.stl")]
    for tag, ov in (("arachne", {"wall_generator": "arachne"}),
                    ("classic_thin", {"wall_generator": "classic", "detect_thin_wall": "1"})):
        rc, data, summ, _ = export(ctx, f"r2_{tag}", stls, ov)
        if not chk.check(f"R2.{tag}.export", rc == 0 and data is not None and summ is not None, f"rc {rc}"):
            continue
        sb = summ.get("single_bead")
        chk.check(f"R2.{tag}.single_bead", sb is not None and sb > 0, f"single_bead={sb}")
        blocks = B.parse_records(data)[0]
        lines, infos, chunks = chunks_of(E, data)
        ok_blocks = [b for b in blocks if b.st == "ok"]
        if not chk.check(f"R2.{tag}.blocks", len(ok_blocks) > 0, f"{len(blocks)} blocks, {len(ok_blocks)} ok"):
            continue
        bad_nc, bad_pts, no_tube = [], [], []
        for b in ok_blocks:
            if b.nc != 1:
                bad_nc.append((b.k, b.nc))
            c = tube_centre(E, lines, infos, chunks[b.k])
            if c is None:
                no_tube.append(b.k)
                continue
            for r in b.runs:
                for x, y in r.pts:
                    if math.hypot(x - c[0], y - c[1]) < 6.0 + 0.5:
                        bad_pts.append((b.k, round(x, 3), round(y, 3)))
        chk.check(f"R2.{tag}.tube_printed", not no_tube, f"layers without the tube's outer wall: {no_tube[:5]}")
        chk.check(f"R2.{tag}.nc", not bad_nc, f"blocks with nc != 1 (the bore planned as cavity): {bad_nc[:5]}")
        chk.check(f"R2.{tag}.bore_empty", not bad_pts, f"RUN points in the tube / its bore: {bad_pts[:5]}")


# ----------------------------------------------------------------------------------------------- R3

def r3(ctx, chk):
    E, sg, shapely = ctx.E, ctx.sg, ctx.shapely
    import numpy as np
    import v2_blocks as B
    stls = [os.path.join(ctx.mdir, n + ".stl") for n in ("box20", "spike_a", "spike_b", "spike_c")]
    delta = 0.2
    rc, data, summ, _ = export(ctx, "r3_spikes", stls, {"wall_generator": "classic", "diw_shell_powder_clearance": repr(delta)})
    if not chk.check("R3.export", rc == 0 and data is not None, f"rc {rc}"):
        return
    blocks = [b for b in B.parse_records(data)[0] if b.st == "ok"]
    lines, infos, chunks = chunks_of(E, data)
    worst, grow, n_runs = math.inf, 0.0, 0
    where = None
    for b in blocks:
        c = chunks[b.k]
        segs, widths = [], []
        first_w, cur = None, None
        for i in range(c.start, c.end):
            li = infos[i]
            if not li.is_motion:
                continue
            if not li.is_extruding_move:
                cur = None
                continue
            w = li.width if li.width else 1.4
            if cur is None or li.ftype != cur:
                cur, first_w = li.ftype, w
            grow = max(grow, w - first_w)
            if E._path_kind(li.ftype) in ("wall", "paste"):
                segs.append(sg.LineString([(li.x0, li.y0), (li.x1, li.y1)]) if (li.x0, li.y0) != (li.x1, li.y1)
                            else sg.Point(li.x0, li.y0))
                widths.append(w)
        if not segs:
            continue
        O = np.array(segs, dtype=object)
        W = np.array(widths)
        for r in b.runs:
            pts = list(r.pts)
            gs = [sg.Point(pts[0])] if r.kind == "point" or len(pts) == 1 else \
                [sg.LineString([p, q]) for p, q in zip(pts, pts[1:]) if p != q]
            for g in gs:
                n_runs += 1
                sl = float((shapely.distance(g, O) - W / 2.0).min()) - (delta + 0.001)
                if sl < worst:
                    worst, where = sl, (b.k, r.kind, pts[0])
    chk.check("R3.reaches_variable_width", grow > 0.5, f"largest width growth inside one path {grow:.3f} mm")
    chk.check("R3.blocks", len(blocks) > 0 and n_runs > 0, f"{len(blocks)} blocks, {n_runs} run pieces")
    chk.check("R3.per_segment_clearance", worst >= -1e-5, f"min slack {worst:.3g} mm against delta + guard at the "
              f"segment's own ;WIDTH: (at {where})")


# ----------------------------------------------------------------------------------------------- R4 / R6 helpers

def slice_case(ctx, sdir):
    """v2 layers, native blocks, dumps and v2 config of a shadow_export matrix slice directory."""
    E = ctx.E
    import v2_hooks as H
    import v2_blocks as B
    from compare_powder import parse_config_block, v2_config
    kill_path = os.path.join(sdir, "kill", "plate_1.gcode")
    on_path = os.path.join(sdir, "on", "plate_1.gcode")
    kill = open(kill_path, "rb").read()
    cfg = v2_config(parse_config_block(kill), E)
    cfg["enable_u_mex_powder_infill"] = True
    layers = {L.k: L for L in H.capture_geometry(E, cfg, kill_path)}
    P = E.UMexParams.from_config(cfg)[0]
    return kill_path, on_path, layers, P


def t2c(ctx, L, P, rec, dj, stats=None):
    import compare_planner as CP
    E, sg, shapely = ctx.E, ctx.sg, ctx.shapely
    native = dj["native"]
    V = CP.V2Case(f"k{L.k}", L.model, L.prm, L.h, L.s, L.z, L.F, L.fu, L.plan, L.geom)
    chk = SE.Collector()
    st = SE.new_stats() if stats is None else stats
    SE.compare_t2c(E, sg, shapely, chk, st, V, rec, dj["debug"], native, mask=SE.seam_mask(L.model, P.delta, 0.0, sg),
                   t3_extra=SE.native_closed_loops(native, sg), S_native=SE.native_solid(native, sg))
    return chk, st


def r4(ctx, chk):
    import v2_blocks as B
    sdir = os.path.join(ctx.slices, "cube25.classic.base")
    keep = os.path.join(sdir, "keep", "plate_1.gcode")
    ddir = os.path.join(sdir, "dump")
    if not chk.check("R4.inputs", os.path.isfile(keep) and os.path.isdir(ddir), sdir):
        return
    kill_path, on_path, layers, P = slice_case(ctx, sdir)
    work = os.path.join(ctx.work, "r4")
    if os.path.isdir(work):
        shutil.rmtree(work)
    os.makedirs(work)
    for tag, factor in (("same", 1.0), ("x1.4", 1.4)):
        d2 = os.path.join(work, tag + "_dump")
        shutil.copytree(ddir, d2)
        for f in os.listdir(d2):
            p = os.path.join(d2, f)
            j = json.load(open(p, encoding="utf-8"))
            if j.get("native"):
                for w in j["native"]["pieces"]:
                    w["width"] = repr(float(w["width"]) * factor)
            json.dump(j, open(p, "w", encoding="utf-8"))
        out = os.path.join(work, tag + ".gcode")
        rc, err = SE.run_tr(ctx.tr, ["--config-from-gcode", "--plan-records", keep, out],
                            {"ORCA_DIW_SHADOW_DUMP": d2, "ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS": "1"})
        if not chk.check(f"R4.{tag}.replan", rc == 0 and os.path.isfile(out), f"rc {rc} {err[-200:]}"):
            continue
        nat = {b.k: b for b in B.parse_records(open(out, "rb").read())[0]}
        dumps = SE.load_dumps(d2)
        tot, fails, st = 0, 0, SE.new_stats()
        for k, L in sorted(layers.items()):
            if k not in nat or L.st == "open" or k not in dumps:
                continue
            c, _ = t2c(ctx, L, P, nat[k], dumps[k], st)
            tot += 1
            fails += sum(1 for r in c.records if not r[2])
        if tag == "same":
            chk.check("R4.same.passes", tot > 0 and fails == 0, f"{tot} layers, {fails} failed checks")
        else:
            chk.check("R4.x1.4.fails", tot > 0 and fails > 0 and st.get("d2_native_band", 0) == 0,
                      f"{tot} layers, {fails} failed checks, d2_native_band={st.get('d2_native_band', 0)}, "
                      f"native_pieces_unverified={st.get('native_pieces_unverified', 0)}")


def r6(ctx, chk):
    import v2_blocks as B
    sdir = os.path.join(ctx.slices, "bridge_obj.arachne.base")
    if not chk.check("R6.inputs", os.path.isdir(os.path.join(sdir, "dump")), sdir):
        return
    kill_path, on_path, layers, P = slice_case(ctx, sdir)
    nat = {b.k: b for b in B.parse_records(open(on_path, "rb").read())[0]}
    dumps = SE.load_dumps(os.path.join(sdir, "dump"))
    found = None
    for k in sorted(set(nat) & set(layers) & set(dumps)):
        if layers[k].st != "ok" or nat[k].st != "ok":
            continue
        c, st = t2c(ctx, layers[k], P, nat[k], dumps[k])
        if st.get("omega_illcond_exact"):
            found = k
            chk.check("R6.unmodified", all(r[2] for r in c.records), f"k{k}: {[(r[0], r[2]) for r in c.records]}")
            break
    if not chk.check("R6.reaches_illcond_path", found is not None, "a layer judged by the exact clearance"):
        return
    rec = copy.copy(nat[found])
    lines = [r for r in rec.runs if r.kind != "point"]
    rec.runs = [r for r in rec.runs if r.kind == "point"] + lines[: max(1, len(lines) // 3)]
    c, st = t2c(ctx, layers[found], P, rec, dumps[found])
    hatch = [r[2] for r in c.records if r[0] == "hatch"]
    chk.check("R6.runs_dropped_fails", hatch and not all(hatch), f"k{found}: runs {len(nat[found].runs)} -> "
              f"{len(rec.runs)}; hatch {hatch}")


# ----------------------------------------------------------------------------------------------- R5 / R7 / R8

def corpus_cases(ctx):
    import compare_planner as CP
    import v2_hooks as H
    from compare_powder import parse_config_block, v2_config
    E = ctx.E
    data = open(os.path.join(HERE, "corpus", "cube25.gcode"), "rb").read()
    src = os.path.join(ctx.work, "cube25.in.gcode")
    open(src, "wb").write(data)
    cfg = v2_config(parse_config_block(data), E)
    cfg["enable_u_mex_powder_infill"] = True
    cs = []
    for L in H.capture_geometry(E, cfg, src):
        V = CP.V2Case(f"cube25.k{L.k}", L.model, L.prm, L.h, L.s, L.z, L.F, L.fu, L.plan, L.geom)
        cs.append((V, CP.plan_input(L.model, L.prm, L.h, L.s, L.z, L.F, L.fu)))
    res = CP.run_planner(ctx.tr, ctx.work, "rv", [c[1] for c in cs])
    return cs, res


def r5_r7(ctx, chk):
    import compare_planner as CP
    E, sg, shapely = ctx.E, ctx.sg, ctx.shapely
    cs, res = corpus_cases(ctx)
    idx = next((i for i, (V, _) in enumerate(cs) if V.st == "ok" and CP.min_tip_deg(V.geom["cavity"], sg) < 23.0), None)
    if not chk.check("R5.tip_layer", idx is not None, "a cube25 layer whose cavity has a tip < 23 deg"):
        return
    V, _ = cs[idx]
    rec, dbg = res[idx]
    c0 = CP.Checks()
    CP.compare_case(E, sg, shapely, c0, SE.new_stats(), V, rec, dbg)
    chk.check("R5.unmodified", all(r[2] for r in c0.records), f"{V.name}: {[(r[0], r[2]) for r in c0.records]}")
    r2 = copy.copy(rec)
    r2.ra = rec.ra + 50.0
    c1 = CP.Checks()
    CP.compare_case(E, sg, shapely, c1, SE.new_stats(), V, r2, dbg)
    areas = [r[2] for r in c1.records if r[0] == "areas"]
    chk.check("R5.ra_plus_50_fails", areas == [False], f"{V.name}: areas {areas}")
    # R7: v2 at quad_segs 256 with another component count cannot arbitrate an area error
    r3 = copy.copy(rec)
    r3.ca, r3.oa = rec.ca + 1.0, rec.oa + 1.0
    orig = CP.v2_fine

    def fine_other(E_, sg_, V_):
        Vf = orig(E_, sg_, V_)
        Vf.plan = copy.copy(Vf.plan)
        Vf.plan.n_components = Vf.plan.n_components + 1
        return Vf
    CP.v2_fine = fine_other
    try:
        c2 = CP.Checks()
        st = SE.new_stats()
        CP.compare_arbitrated(E, sg, shapely, c2, st, V, r3, dbg)
    finally:
        CP.v2_fine = orig
    areas = [r[2] for r in c2.records if r[0] == "areas"]
    chk.check("R7.fine_nc_differs_not_arbitrated", areas == [False] and not st.get("v2_discretization"),
              f"areas {areas}, v2_discretization {st.get('v2_discretization')}")


def r8(ctx, chk):
    import compare_planner as CP
    import v2_blocks as B
    # (a) fb_pos is measured on the accepted (permuted) pairs only
    a = B.FbRec(w=0.6, tf=6000.0, z=1.2, ty="sparse infill", pts=[(0.0, 0.0), (5.0, 0.0)], segs=[(0.1, 1200.0)])
    b = B.FbRec(w=0.6, tf=6000.0, z=1.2, ty="sparse infill", pts=[(0.0, 9.0), (5.0, 9.0)], segs=[(0.1, 1200.0)])

    def path(f):
        return types.SimpleNamespace(width=f.w, travel_feed=f.tf, z=f.z, ptype=f.ty, points=f.pts,
                                     seg_de=[s[0] for s in f.segs], seg_feed=[s[1] for s in f.segs])
    rec = types.SimpleNamespace(fbs=[a, b])
    plan = types.SimpleNamespace(fallback_paths=[path(b), path(a)])
    c = CP.Checks()
    st = SE.new_stats()
    CP.compare_fb(c, st, "r8.fb", rec, plan)
    chk.check("R8.fb_pos_matched_pairs", [r[2] for r in c.records] == [True] and st["fb_pos"] == 0.0,
              f"fb {[r[2] for r in c.records]}, fb_pos {st['fb_pos']} (9 mm when measured on the unmatched pairs)")
    # (b) compare_arbitrated: a coarse failure accepted against v2 at quad_segs 256 reports the fine metrics
    real, orig_fine = CP.compare_case, CP.v2_fine
    calls = []

    def staged(E, sg, shapely, chk_, stats, V, rec, dbg, **kw):
        calls.append(1)
        chk_.check("st_nc", V.name, True)
        if len(calls) == 1:
            stats["haus_O"] = max(stats["haus_O"], 0.5)
            chk_.check("areas", V.name, False, "coarse")
        else:
            stats["haus_O"] = max(stats["haus_O"], 1e-4)
            chk_.check("areas", V.name, True)
    plan = types.SimpleNamespace(runs=[1], n_components=1)
    V = CP.V2Case("r8.k1", None, None, 0, 0, 0, 0, 0, plan, {})
    CP.compare_case = staged
    CP.v2_fine = lambda E, sg, V_: CP.V2Case(V_.name, None, None, 0, 0, 0, 0, 0, plan, {})
    try:
        c = CP.Checks()
        st = SE.new_stats()
        CP.compare_arbitrated(None, None, None, c, st, V, types.SimpleNamespace(), {})
    finally:
        CP.compare_case, CP.v2_fine = real, orig_fine
    chk.check("R8.arbitrated_metrics", all(r[2] for r in c.records) and st["haus_O"] == 1e-4,
              f"records {[(r[0], r[2]) for r in c.records]}, haus_O {st['haus_O']} (0.5 = the rejected comparison)")


def r9(ctx, chk):
    cases = [((117, 3350), True), ((10, 20), False), ((1, 10), True), ((2, 10), False), ((0, 5), True), ((30, 100), False)]
    got = [(c, SE.d5_fraction_ok(*c)) for c, _ in cases]
    chk.check("R9.d5_fraction_ok", all(g == want for (_, g), (_, want) in zip(got, cases)), f"{got}")


# ----------------------------------------------------------------------------------------------- main

def main(argv):
    ctx = types.SimpleNamespace()
    engine, release = opt(argv, "--engine"), opt(argv, "--release")
    ctx.datadir, ctx.work, ctx.slices = opt(argv, "--datadir"), opt(argv, "--work"), opt(argv, "--slices")
    only = opt(argv, "--only")
    chk = Checks()
    if not all((engine, release, ctx.datadir, ctx.work, ctx.slices)):
        print(__doc__)
        return 2
    ctx.orca = os.path.join(release, "orca-slicer.exe")
    ctx.tr = os.path.join(release, "diw_translate.exe")
    for p in (ctx.orca, ctx.tr, engine, ctx.datadir):
        if not chk.check("inputs", os.path.exists(p), p):
            print(f"{chk.n} checks, {len(chk.fails)} failures")
            return 1
    os.makedirs(ctx.work, exist_ok=True)
    ctx.work = os.path.abspath(ctx.work)
    ctx.E, ctx.sg, ctx.shapely = SE._engine(engine)
    ctx.mdir = SE.make_meshes(ctx.work)
    tests = [("R1", r1), ("R2", r2), ("R3", r3), ("R4", r4), ("R5", r5_r7), ("R6", r6), ("R8", r8), ("R9", r9)]
    for name, fn in tests:
        if only and name not in only.split(","):
            continue
        try:
            fn(ctx, chk)
        except Exception as ex:  # a crash is a failure
            import traceback
            traceback.print_exc()
            chk.check(f"{name}.crash", False, repr(ex))
    for f in chk.fails:
        print("  FAIL " + f)
    print(f"{chk.n} checks, {len(chk.fails)} failures")
    return 1 if chk.fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
