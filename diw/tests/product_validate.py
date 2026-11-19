"""G6: u-MEX powder-job validation from the CLI (diw/PRODUCT_DESIGN.md sections 6 and 8).

usage: python product_validate.py --orca <orca-slicer.exe> --translate <diw_translate.exe> --datadir <orca datadir>
                                  --work <dir> [-j N]

Slices the 25 mm cube (planner_meshes.py) with the DIW printer and powder process test profiles
(profiles/flat_*.json, ORCA_DIW_EXPORT_UNTRANSLATED=1) plus one override set per row, and checks:
  E  each hard limit of 6.2 (v2's checks, at their 1e-9 boundaries): the export fails, no plate file is written,
     the log holds "got error when validate: <message>" with exactly the message diw_translate --validate gives for
     the same settings (--config-from-gcode <plain export> key=value ...), and that the opt_key is the table's;
     the accepted side of each boundary slices (rc 0);
  W  each warning of 6.2 and the three retraction warnings of 6.3: rc 0, the warning is logged ("got warnings:"),
     equal to diw_translate --validate's text where it has one, the export before "; CONFIG_BLOCK_START" equals
     the plain export (norm) unless the value itself changes the plan (clearance: rc and warning only), and the
     warning is absent on the accepted side of its 1e-9 boundary;
  S  scarf (6.1): a global scarf is forced off on a powder job (export equals the plain one before the config block,
     the config block says seam_slope_type = none); a per-object scarf fails with 6.1's message; with
     ORCA_DIW_TEST_KEEP_SCARF=1 both are kept (config block / export reached);
  R  6.3: wall_loops = 0, sparse_infill_density = 0 (global, and on both objects of a two-object plate), >= 100 (F4: global, and on one
     object of two) and
     infill_combination = 1 fail with 6.3's messages; density 0 on one object of two slices; the structural
     refusals (spiral vase, belt printer, differing extruder offsets) fail with "u-MEX powder infill cannot be used
     with <reason>." unless Orca's own rules reject the config first (then listed: covered by Catch2);
  N  the same invalid values on a non-DIW printer (diw_printer 0) and on a DIW paste-only job slice (rc 0);
  K  ORCA_DIW_TEST_SKIP_VALIDATE=1 lets a powder job with infill_combination = 1 slice (the skip works).
Prints "N checks, F failures" and exits non-zero on a failure. A missing executable or input is a failure.
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import shadow_export as S  # noqa: E402

ERR_RE = re.compile(rb"got error when validate: ([^\r\n]*)")
WARN_RE = re.compile(rb"got warnings: ([^\r\n]*)")


def structural_refused_elsewhere(r):
    """True only when Orca's own validate error is present (an ERR_RE match that is not u-MEX) and the run failed."""
    errs = ERR_RE.findall(r["log"])
    return r["rc"] != 0 and bool(errs) and not any(b"u-MEX" in e for e in errs)


def selftest():
    """F1 regression: the refused_elsewhere branch must not accept an export-time H1 refusal on its own."""
    fails = []
    h1 = {"rc": 1, "log": b"Slic3r::CLI::run found error, exit\nu-MEX powder infill cannot be used: spiral vase mode\n"}
    orca = {"rc": 1, "log": b"got error when validate: Spiral vase mode does not work when something is set\n"}
    umex = {"rc": 1, "log": b"got error when validate: u-MEX powder infill cannot be used with spiral vase mode.\n"}
    ok0 = {"rc": 0, "log": b"got error when validate: Orca something\n"}
    for name, r, want in (("h1_only", h1, False), ("orca_error", orca, True), ("umex_error", umex, False),
                          ("rc0", ok0, False)):
        got = structural_refused_elsewhere(r)
        if got != want:
            fails.append(f"{name}: got {got}, want {want}")
    for f in fails:
        print("  FAIL " + f)
    print(f"4 checks, {len(fails)} failures")
    return 1 if fails else 0

M_SCARF = ("u-MEX powder jobs cannot use a scarf joint seam: it ramps the outer wall below the layer and the powder "
           "cavity is planned on flat layers. Set \"Scarf joint seam\" to None in this object's settings.")
M_WALLS = "u-MEX powder infill needs at least one wall loop: the powder cavity is closed by the outer wall."
M_DENS = ("u-MEX powder infill needs sparse infill: the powder replaces the sparse infill of the core layers. Set a "
          "sparse infill density above 0 or turn u-MEX powder infill off.")
M_DENS100 = ("u-MEX powder infill needs sparse infill: at a sparse infill density of 100% the infill is solid and there is no "
             "cavity for the powder. Set a sparse infill density below 100% in this object's settings or turn u-MEX powder "
             "infill off.")
M_COMB = ("u-MEX powder jobs cannot combine infill: sparse infill printed every N layers would get the powder of one "
          "layer only. Turn \"Infill combination\" off.")
W_LONG = ("Retractions of 0.9 mm are longer than the DIW limit of 0.5 mm (\"Maximum syringe retraction\"); the DIW "
          "translation may remove retractions longer than the limit, as v2 does.")
W_OFF = "Syringe retraction is off on this DIW printer; the DIW translation removes retraction moves."
W_SPEED = "Retraction speed 45 mm/s is clamped to the DIW limit of 5 mm/s."

# (name, overrides, opt_key, message prefix): hard limits of 6.2, rejected side of each boundary
ERRORS = [
    ("comp_gt8", {"diw_compaction_time": "8.000000001"}, "diw_compaction_time", "compaction_duration = "),
    ("comp_le0", {"diw_compaction_time": "0"}, "diw_compaction_time", "compaction_duration must be > 0"),
    ("hatch_le0", {"diw_powder_hatch_spacing": "0"}, "diw_powder_hatch_spacing", "powder_hatch_spacing must be > 0"),
    ("mdot_le0", {"diw_powder_mass_flow": "0"}, "diw_powder_mass_flow", "powder_mass_flow_rate must be > 0"),
    ("rho_le0", {"diw_powder_tap_density": "0"}, "diw_powder_tap_density", "powder_tap_density must be > 0"),
    ("eta_le0", {"diw_powder_fill_multiplier": "0"}, "diw_powder_fill_multiplier", "powder_fill_multiplier must be > 0"),
    ("fmax_le0", {"diw_powder_speed_max": "0"}, "diw_powder_speed_max", "powder_f_max must be > 0"),
    ("dpow_le0", {"diw_powder_nozzle_diameter": "0"}, "diw_powder_nozzle_diameter", "powder_nozzle_dia must be > 0"),
    ("lagon_lt0", {"diw_powder_lag_on": "-0.001"}, "diw_powder_lag_on", "powder_lag_on must be >= 0"),
    ("lagoff_lt0", {"diw_powder_lag_off": "-0.001"}, "diw_powder_lag_off", "powder_lag_off must be >= 0"),
    ("delta_lt0", {"diw_shell_powder_clearance": "-0.001"}, "diw_shell_powder_clearance", "shell_powder_clearance must be >= 0"),
    ("settle_lt0", {"diw_compaction_settle_time": "-0.001"}, "diw_compaction_settle_time", "compaction_settle_dwell must be >= 0"),
    ("thop_lt0", {"diw_powder_travel_hop": "-0.001"}, "diw_powder_travel_hop", "powder_travel_hop_mm must be >= 0"),
    ("chop_lt0", {"diw_compaction_z_hop": "-0.001"}, "diw_compaction_z_hop", "compaction_z_hop_mm must be >= 0"),
    ("interval_lt1", {"diw_compaction_interval": "0"}, "diw_compaction_interval", "compaction_interval must be >= 1"),
    ("volt_low", {"diw_powder_motor_voltage": "3.649999998"}, "diw_powder_motor_voltage", "powder_motor_voltage must be within"),
    ("volt_high", {"diw_powder_motor_voltage": "12.000000002"}, "diw_powder_motor_voltage", "powder_motor_voltage must be within"),
    ("pins", {"diw_compaction_pin": "5", "diw_powder_pin": "5"}, "diw_powder_pin", "digital output pins must be distinct: "),
]
# accepted side of the error boundaries (rc 0)
ACCEPT = [
    ("comp_eq8", {"diw_compaction_time": "8"}),
    ("lag_eq0", {"diw_powder_lag_on": "0", "diw_powder_lag_off": "0"}),
    ("volt_low_ok", {"diw_powder_motor_voltage": "3.6499999995"}),
    ("volt_high_ok", {"diw_powder_motor_voltage": "12.0000000005"}),
    ("interval_eq1", {"diw_compaction_interval": "1"}),
]
# (name, overrides, opt_key or None (6.3: no diw_translate counterpart), message prefix, warned?, same_body?)
WARNINGS = [
    ("comp_lt1", {"diw_compaction_time": "0.5"}, "diw_compaction_time", "compaction_duration = 0.5 s is below", True, True),
    ("comp_eq1", {"diw_compaction_time": "1"}, "diw_compaction_time", "compaction_duration = ", False, True),
    ("volt_warn", {"diw_powder_motor_voltage": "3.650000002"}, "diw_powder_motor_voltage", "powder_motor_voltage = ", True, True),
    ("volt_nowarn", {"diw_powder_motor_voltage": "3.6500000005"}, "diw_powder_motor_voltage", "powder_motor_voltage = ", False, True),
    ("delta_warn", {"diw_shell_powder_clearance": "0.749999998"}, "diw_shell_powder_clearance", "shell_powder_clearance = ", True, False),
    ("delta_nowarn", {"diw_shell_powder_clearance": "0.7499999995"}, "diw_shell_powder_clearance", "shell_powder_clearance = ", False, False),
    ("offz_warn", {"diw_powder_offset_z": "0.000000002"}, "diw_powder_offset_z", "powder_offset_z = ", True, True),
    ("offz_nowarn", {"diw_powder_offset_z": "0.0000000005"}, "diw_powder_offset_z", "powder_offset_z = ", False, True),
    ("offsets_zero", {}, "diw_powder_offset_x", "POWDER NOZZLE OFFSETS powder_offset_x/y/z ARE ALL 0.0", True, True),
    ("offsets_set", {"diw_powder_offset_x": "1"}, "diw_powder_offset_x", "POWDER NOZZLE OFFSETS", False, True),
    ("retract_long", {"diw_retraction_max_length": "0.5"}, None, W_LONG, True, True),
    ("retract_off", {"diw_retraction_enable": "0"}, None, W_OFF, True, True),
    ("retract_speed", {}, None, W_SPEED, True, True),
    ("retract_speed_ok", {"diw_retraction_max_speed": "45"}, None, "Retraction speed ", False, True),
]


def body(data):
    return S.norm(data.split(b"; CONFIG_BLOCK_START", 1)[0])


def config_block(data):
    return dict(re.findall(rb"^; ([a-z0-9_]+) = (.*)$", data.split(b"; CONFIG_BLOCK_START", 1)[1], re.M))


def run_slice(job):
    """One CLI slice. job: name, overrides, env, objs (optional [(stl, params)]), work, orca, datadir, mesh."""
    sd = os.path.join(job["work"], job["name"])
    os.makedirs(sd, exist_ok=True)
    settings = S.write_profiles(os.path.join(sd, "profiles"), "p", job["overrides"])
    out = os.path.join(sd, "out")
    if job.get("objs"):
        os.makedirs(out, exist_ok=True)
        objs = [{"path": p, "count": 1, "filaments": [1], "print_params": dict(pp)} for p, pp in job["objs"]]
        al = os.path.join(sd, "assemble.json")
        with open(al, "w", encoding="utf-8") as fh:
            json.dump({"plates": [{"plate_name": "plate_1", "need_arrange": True, "objects": objs}]}, fh, indent=1)
        stls, args = [], ["--load-assemble-list", al]
    else:
        stls, args = [job["mesh"]], []
    rc, data, log, _ = S.slice_part(job["orca"], job["datadir"], stls, out, dict(job.get("env") or {}), settings, args)
    return {"name": job["name"], "rc": rc, "data": data, "log": log,
            "plate": os.path.join(out, "plate_1.gcode"), "files": sorted(os.listdir(out)) if os.path.isdir(out) else []}


def tr_validate(tr, plain_path, overrides):
    r = subprocess.run([tr, "--config-from-gcode", plain_path, "--validate", "diw_powder_infill=1"] +
                       [f"{k}={v}" for k, v in overrides.items()], capture_output=True, cwd=os.path.dirname(tr))
    lines = r.stdout.decode("utf-8", "replace").splitlines()
    err = [l.split("\t", 2) for l in lines if l.startswith("error\t")]
    warns = [l.split("\t", 2) for l in lines if l.startswith("warning\t")]
    return r.returncode, (err[0][1], err[0][2]) if err else None, [(w[1], w[2]) for w in warns]


def main(argv):
    if "--selftest" in argv:
        return selftest()
    orca, tr, datadir, work = (S.opt(argv, k) for k in ("--orca", "--translate", "--datadir", "--work"))
    nj = int(S.opt(argv, "-j", "6"))
    rep = S.Report()
    if not (orca and tr and datadir and work):
        print(__doc__)
        return 2
    for nm, p in (("exe.orca", orca), ("exe.translate", tr)):
        rep.check(nm, os.path.isfile(p), p)
    rep.check("datadir", os.path.isdir(datadir), datadir)
    if rep.fails:
        print(f"{rep.n} checks, {len(rep.fails)} failures")
        return 1
    with open(tr, "rb") as fh:
        rep.check("exe.translate.flags", b"--validate" in fh.read(), "--validate present")
    os.makedirs(work, exist_ok=True)
    mdir = S.make_meshes(work)
    cube = os.path.join(mdir, "cube25.stl")
    two_a, two_b = os.path.join(mdir, "two_a.stl"), os.path.join(mdir, "two_b.stl")
    base = {"work": work, "orca": orca, "datadir": datadir, "mesh": cube}

    jobs = [dict(base, name="plain", overrides={})]
    jobs += [dict(base, name=f"E.{n}", overrides=ov) for n, ov, _, _ in ERRORS]
    jobs += [dict(base, name=f"A.{n}", overrides=ov) for n, ov in ACCEPT]
    jobs += [dict(base, name=f"W.{n}", overrides=ov) for n, ov, *_ in WARNINGS]
    jobs += [
        dict(base, name="S.global_scarf", overrides={"seam_slope_type": "all"}),
        dict(base, name="S.global_scarf_keep", overrides={"seam_slope_type": "all"}, env={"ORCA_DIW_TEST_KEEP_SCARF": "1"}),
        dict(base, name="S.object_scarf", overrides={}, objs=[(cube, {"seam_slope_type": "external"})]),
        dict(base, name="S.object_scarf_keep", overrides={}, objs=[(cube, {"seam_slope_type": "external"})],
             env={"ORCA_DIW_TEST_KEEP_SCARF": "1"}),
        dict(base, name="R.wall_loops0", overrides={"wall_loops": "0"}),
        dict(base, name="R.density0", overrides={"sparse_infill_density": "0%"}),
        dict(base, name="R.density0_both", overrides={}, objs=[(two_a, {"sparse_infill_density": "0%"}),
                                                                 (two_b, {"sparse_infill_density": "0%"})]),
        dict(base, name="R.density0_one", overrides={}, objs=[(two_a, {"sparse_infill_density": "0%"}), (two_b, {})]),
        dict(base, name="R.density100", overrides={"sparse_infill_density": "100%"}),
        dict(base, name="R.density100_one", overrides={}, objs=[(two_a, {"sparse_infill_density": "100%"}), (two_b, {})]),
        dict(base, name="R.density99", overrides={"sparse_infill_density": "99%"}),
        dict(base, name="R.infill_combination", overrides={"infill_combination": "1"}),
        dict(base, name="R.spiral", overrides={"spiral_mode": "1"}),
        dict(base, name="R.belt", overrides={"belt_printer": "1"}),
        dict(base, name="R.extruder_offset", overrides={"extruder_offset": "0x0,1x0"}),
        dict(base, name="K.skip_validate", overrides={"infill_combination": "1"}, env={"ORCA_DIW_TEST_SKIP_VALIDATE": "1"}),
    ]
    # values inside PrintConfig.cpp's min / max that Print::validate rejects on a powder job
    bad_all = {"diw_compaction_time": "8.000000001", "diw_powder_motor_voltage": "12.000000002", "wall_loops": "0", "infill_combination": "1",
               "seam_slope_type": "all", "diw_compaction_pin": "5", "diw_powder_pin": "5"}
    jobs += [dict(base, name="N.non_diw", overrides=dict(bad_all, diw_printer="0")),
             dict(base, name="N.paste_only", overrides=dict(bad_all, diw_powder_infill="0"))]
    print(f"{len(jobs)} slices, {nj} workers", flush=True)
    with ProcessPoolExecutor(max_workers=nj) as ex:
        res = {r["name"]: r for r in ex.map(run_slice, jobs)}

    plain = res["plain"]
    if not rep.check("plain.rc", plain["rc"] == 0 and plain["data"] is not None, f"rc={plain['rc']}"):
        print(f"{rep.n} checks, {len(rep.fails)} failures")
        return 1
    plain_body = body(plain["data"])

    def failed_with(r, msg, prefix=False):
        errs = [m.decode("utf-8", "replace") for m in ERR_RE.findall(r["log"])]
        ok_msg = any(e.startswith(msg) if prefix else e == msg for e in errs)
        return r["rc"] != 0 and "plate_1.gcode" not in r["files"] and ok_msg, f"rc={r['rc']} files={r['files']} errors={errs[:2]}"

    def warned(r, msg, prefix=True):
        ws = [m.decode("utf-8", "replace") for m in WARN_RE.findall(r["log"])]
        return any(w.startswith(msg) if prefix else w == msg for w in ws), ws

    # ---- E: hard limits
    range_refused = []
    for n, ov, key, prefix in ERRORS:
        r = res[f"E.{n}"]
        rc, err, _ = tr_validate(tr, plain["plate"], ov)
        rep.check(f"E.{n}.translate_validate", rc == 3 and err is not None and err[0] == key and err[1].startswith(prefix),
                  f"rc={rc} {err}")
        if err is None:
            continue
        rng = re.search(rb"^" + re.escape(key.encode()) + rb": [^\r\n]* not in range [^\r\n]*", r["log"], re.M)
        if rng is not None and b"Param values in 3mf/config error" in r["log"]:
            # PrintConfig.cpp's min / max of the key reject the value before Print::validate (the CLI's config range
            # check): the export is still refused before slicing; the message is checked by diw_translate --validate
            # above and by Catch2 T13
            range_refused.append(n)
            rep.check(f"E.{n}.cli_range", r["rc"] != 0 and "plate_1.gcode" not in r["files"],
                      f"refused by the config range check: {rng.group(0).decode()!r}")
            continue
        ok, d = failed_with(r, err[1])
        rep.check(f"E.{n}.cli", ok, f"want {err[1]!r}; {d}")
    for n, ov in ACCEPT:
        r = res[f"A.{n}"]
        rc, err, _ = tr_validate(tr, plain["plate"], ov)
        rep.check(f"A.{n}", r["rc"] == 0 and r["data"] is not None and rc == 0 and err is None, f"rc={r['rc']} validate rc={rc} {err}")

    # ---- W: warnings
    for n, ov, key, msg, want, same in WARNINGS:
        r = res[f"W.{n}"]
        if not rep.check(f"W.{n}.rc", r["rc"] == 0 and r["data"] is not None, f"rc={r['rc']}"):
            continue
        hit, ws = warned(r, msg, prefix=key is not None)
        rep.check(f"W.{n}.warning", hit == want, f"{'want' if want else 'no'} {msg[:60]!r}; logged {[w[:60] for w in ws]}")
        if key is not None:
            rc, err, twarns = tr_validate(tr, plain["plate"], ov)
            mine = [w for w in ws if w.startswith(msg)]
            theirs = [t for k, t in twarns if t.startswith(msg) and k == key]
            rep.check(f"W.{n}.translate_validate", rc == 0 and err is None and mine == theirs,
                      f"cli {[m[:50] for m in mine]} / diw_translate {[t[:50] for t in theirs]}")
        if same:
            rep.check(f"W.{n}.gcode", body(r["data"]) == plain_body,
                      "identical before the config block" if body(r["data"]) == plain_body else S.first_diff(body(r["data"]), plain_body))

    # ---- S: scarf
    g = res["S.global_scarf"]
    if rep.check("S.global.rc", g["rc"] == 0 and g["data"] is not None, f"rc={g['rc']}"):
        cb = config_block(g["data"])
        rep.check("S.global.forced_none", cb.get(b"seam_slope_type") == b"none", f"seam_slope_type = {cb.get(b'seam_slope_type')}")
        rep.check("S.global.gcode", body(g["data"]) == plain_body, "identical to the no-scarf export before the config block"
                  if body(g["data"]) == plain_body else S.first_diff(body(g["data"]), plain_body))
    k = res["S.global_scarf_keep"]
    if rep.check("S.global_keep.rc", k["rc"] == 0 and k["data"] is not None, f"rc={k['rc']}"):
        cb = config_block(k["data"])
        rep.check("S.global_keep.kept", cb.get(b"seam_slope_type") == b"all", f"seam_slope_type = {cb.get(b'seam_slope_type')}")
    ok, d = failed_with(res["S.object_scarf"], M_SCARF)
    rep.check("S.object.refused", ok, d)
    k = res["S.object_scarf_keep"]
    rep.check("S.object_keep.rc", k["rc"] == 0 and k["data"] is not None, f"rc={k['rc']} {ERR_RE.findall(k['log'])[:1]}")

    # ---- R: 6.3
    for n, msg in (("wall_loops0", M_WALLS), ("density0", M_DENS), ("density0_both", M_DENS), ("infill_combination", M_COMB),
                   ("density100", M_DENS100), ("density100_one", M_DENS100)):
        ok, d = failed_with(res[f"R.{n}"], msg)
        rep.check(f"R.{n}", ok, d)
    r = res["R.density99"]   # F4: 99 % is still sparse infill
    rep.check("R.density99", r["rc"] == 0 and r["data"] is not None, f"rc={r['rc']} {ERR_RE.findall(r['log'])[:1]}")
    r = res["R.density0_one"]
    rep.check("R.density0_one", r["rc"] == 0 and r["data"] is not None, f"rc={r['rc']} {ERR_RE.findall(r['log'])[:1]}")
    catch2_only = []
    for n, reason in (("spiral", "spiral vase mode"), ("belt", "G-code axis remap"),
                      ("extruder_offset", "extruders that have different extruder_offset values")):
        r = res[f"R.{n}"]
        msg = f"u-MEX powder infill cannot be used with {reason}."
        ok, d = failed_with(r, msg)
        if not ok and structural_refused_elsewhere(r):
            # Orca's own Print::validate rejected the config first (an ERR_RE error that is not u-MEX): covered by
            # Catch2 (powder_refusal_reason). An export-time H1 refusal alone ("u-MEX powder infill cannot be used:")
            # is NOT this case: the Print::validate DIW rule is then missing, and the row fails below.
            errs = ERR_RE.findall(r["log"])
            catch2_only.append((n, errs[:2]))
            rep.check(f"R.{n}.refused_elsewhere", True, f"Orca refused first: {errs[:2]}")
            continue
        rep.check(f"R.{n}", ok, d)

    # ---- N: non-DIW printer and DIW paste-only jobs are not checked
    for n in ("non_diw", "paste_only"):
        r = res[f"N.{n}"]
        rep.check(f"N.{n}", r["rc"] == 0 and r["data"] is not None and not any(b"u-MEX" in e for e in ERR_RE.findall(r["log"])),
                  f"rc={r['rc']} {ERR_RE.findall(r['log'])[:1]}")
    # ---- K: the test-only skip
    r = res["K.skip_validate"]
    rep.check("K.skip_validate", r["rc"] == 0 and r["data"] is not None, f"rc={r['rc']} {ERR_RE.findall(r['log'])[:1]}")

    print(f"hard-limit rows refused by the CLI config range check (PrintConfig.cpp min / max) before Print::validate: "
          f"{range_refused}")
    if catch2_only:
        print(f"structural rows refused by Orca before the DIW rules (Catch2 only): {catch2_only}")
    for f in rep.fails:
        print("  FAIL " + f[:400])
    print(f"{rep.n} checks, {len(rep.fails)} failures")
    return 1 if rep.fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
