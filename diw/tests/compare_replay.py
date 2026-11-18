"""Offline shadow binder vs v2 (diw/PLANNER_DESIGN.md stage 3 checkpoint): `diw_translate --plan-records`.

usage:
  python compare_replay.py <v2 engine dir> <diw_translate.exe> <work dir> <corpus.gcode ...> [--allow-version-drift]

For every corpus file and compare_records variant (plus the generated notch and its variant), with v2 in real
powder mode as the reference (v2_hooks.capture):
  - plan:      diw_translate --config-from-gcode --plan-records <src> <out> diw_powder_infill=1 <overrides> exits 0
               (text-only adapter: rings from v2's snap + linemerge of the text walls, no native band);
  - stream:    the same with --feed-size 13 (the binder fed in 13-byte pieces) gives the same bytes;
  - strip:     removing every ;DIW: line gives the source back byte for byte (shadow mode adds records only);
  - records:   the records parse without error;
  - placement: every block sits right after the start-marker line of its layer, at the line v2_hooks.capture
               uses (LayerChunk.start + 1), so the translator's placement rule holds;
  - core_set:  the k set equals the set of layers v2 planned (h <= 0 / policy=error carriers excepted);
  - begin:     BEGIN k z h s F fu fp are string-equal to v2's block (and st equal);
  - accept:    diw_translate in powder mode accepts the file (exit 0);
  - accept_stub: the same with --stub-powder-stage, and its output equals the translation of v2's own records
               (records_gcode of v2_hooks.capture) outside the converted layers' S4 headers / fallback lines
               (reported, not asserted: the planners differ within the T1 tolerances).
Prints "N checks, F failures" and exits non-zero on any failure. A missing executable is a failure.
"""
import os
import platform
import subprocess
import sys

import v2_blocks as B
import v2_hooks as H
from compare_powder import PINNED, parse_config_block, v2_config
from compare_records import NOTCH_VARIANTS, VARIANTS, make_notch

BEGIN_KEYS = ("k", "z", "h", "s", "F", "fu", "fp")


def begin_fields(line):
    toks = dict(t.split("=", 1) for t in line.split()[1:])
    return toks


class Report:
    def __init__(self):
        self.n = 0
        self.fails = []

    def check(self, name, ok, detail=""):
        self.n += 1
        print(f"{'ok  ' if ok else 'FAIL'} {name}: {detail}")
        if not ok:
            self.fails.append(f"{name}: {detail}")
        return ok


def run(exe, argv):
    r = subprocess.run([exe] + argv, capture_output=True, cwd=os.path.dirname(exe))
    return r.returncode, r.stderr.decode("utf-8", "replace").strip()


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
    os.makedirs(work, exist_ok=True)
    rep = Report()
    if not rep.check("exe", os.path.isfile(exe), exe):
        print(f"\n{rep.n} checks, {len(rep.fails)} failures")
        return 1
    with open(exe, "rb") as fh:
        rep.check("exe.flag", b"--plan-records" in fh.read(), "--plan-records present")

    inputs = []
    for path in corpus:
        with open(path, "rb") as fh:
            inputs.append((os.path.splitext(os.path.basename(path))[0], fh.read(), VARIANTS))
    if inputs:
        inputs.append(("notch", make_notch(inputs[0][1]), NOTCH_VARIANTS))
    totals = {"blocks": 0, "v2_blocks": 0, "st_ok": 0, "max_rel_ca": 0.0, "max_rel_oa": 0.0, "stub_same": 0, "stub_cases": 0}
    for cname, data, variants in inputs:
        for vname, ov in variants.items():
            name = f"{cname}.{vname}"
            src = os.path.join(work, f"{name}.in.gcode")
            with open(src, "wb") as fh:
                fh.write(data)
            diw = parse_config_block(data)
            for kv in ov:
                k, v = kv.split("=", 1)
                diw[k] = v
            cfg = v2_config(diw, E)
            cfg["enable_u_mex_powder_infill"] = True
            rec_gcode, layers, _ = H.capture(E, cfg, src, os.path.join(work, f"{name}.v2cap.gcode"))
            v2_path = os.path.join(work, f"{name}.v2rec.gcode")
            with open(v2_path, "wb") as fh:
                fh.write(rec_gcode)
            ovs = ["diw_powder_infill=1"] + ov

            out = os.path.join(work, f"{name}.native.gcode")
            rc, err = run(exe, ["--config-from-gcode", "--plan-records", src, out] + ovs)
            if not rep.check(f"{name}.plan", rc == 0 and os.path.isfile(out), f"exit {rc}: {err}"):
                continue
            with open(out, "rb") as fh:
                nat = fh.read()
            out13 = os.path.join(work, f"{name}.native13.gcode")
            rc, err13 = run(exe, ["--config-from-gcode", "--plan-records", "--feed-size", "13", src, out13] + ovs)
            same = rc == 0 and open(out13, "rb").read() == nat
            rep.check(f"{name}.stream", same, "13-byte feeds give the same bytes" if same else f"exit {rc}: {err13}")
            rep.check(f"{name}.strip", B.strip_records(nat) == data, "")

            blocks, errors = B.parse_records(nat)
            rep.check(f"{name}.records", not errors, f"{len(blocks)} blocks" + (f", errors {errors[:3]}" if errors else ""))
            # placement: source lines before each BEGIN (records removed) == v2 LayerChunk.start + 1
            src_before, n_src, pos = {}, 0, 0
            for line in nat.split(b"\n"):
                if line.startswith(b";DIW:"):
                    if line.startswith(b";DIW:BEGIN"):
                        src_before[int(begin_fields(line.decode("ascii"))["k"])] = n_src
                else:
                    n_src += 1
            v2_by_k = {L.k: L for L in layers}
            P = E.UMexParams.from_config(cfg)[0]
            # carriers (section 6.3 gates 7 and 8): layers v2 never plans (h <= 0, or policy=error overflow)
            carriers = {b.k for b in blocks if b.st == "open" and (b.h <= 0.0 or (P.policy == "error" and b.F == b.fu))}
            bad_pos = [k for k, n in src_before.items() if k in v2_by_k and n != v2_by_k[k].start + 1]
            bad_pos += [k for k in src_before if k not in v2_by_k and k not in carriers]
            rep.check(f"{name}.placement", not bad_pos, f"{len(src_before)} blocks after their start marker" +
                      (f"; misplaced k {bad_pos[:10]}" if bad_pos else ""))
            ks_nat = sorted(b.k for b in blocks if b.k not in carriers)
            ks_v2 = sorted(v2_by_k)
            rep.check(f"{name}.core_set", ks_nat == ks_v2,
                      f"{len(ks_nat)} core layers" + ("" if ks_nat == ks_v2 else
                                                      f"; native-only {sorted(set(ks_nat) - set(ks_v2))[:10]}, "
                                                      f"v2-only {sorted(set(ks_v2) - set(ks_nat))[:10]}"))
            nat_begin = {}
            for line in nat.decode("ascii", "replace").split("\n"):
                if line.startswith(";DIW:BEGIN"):
                    f = begin_fields(line)
                    nat_begin[int(f["k"])] = f
            diffs = []
            for L in layers:
                f = nat_begin.get(L.k)
                if f is None:
                    continue
                g = begin_fields(L.block.split("\n")[0])
                for key in BEGIN_KEYS + ("st",):
                    if f[key] != g[key]:
                        diffs.append(f"k={L.k} {key}: native {f[key]} v2 {g[key]}")
            rep.check(f"{name}.begin", not diffs, f"{len(nat_begin)} BEGIN lines string-equal" if not diffs else "; ".join(diffs[:6]))

            totals["blocks"] += len(blocks)
            totals["v2_blocks"] += len(layers)
            by_k = {b.k: b for b in blocks}
            for L in layers:
                b = by_k.get(L.k)
                if b is None or L.plan is None:
                    continue
                totals["st_ok"] += b.st == "ok"
                for key, mine, theirs in (("ca", b.ca, L.plan.cavity_area), ("oa", b.oa, L.plan.omega_area)):
                    if theirs:
                        totals[f"max_rel_{key}"] = max(totals[f"max_rel_{key}"], abs(mine - theirs) / abs(theirs))

            tr = os.path.join(work, f"{name}.native.tr.gcode")
            rc, err = run(exe, ["--config-from-gcode", out, tr] + ovs)
            rep.check(f"{name}.accept", rc == 0, f"exit {rc}" + (f": {err}" if rc else ""))
            trs = os.path.join(work, f"{name}.native.stub.gcode")
            rc, err = run(exe, ["--config-from-gcode", "--stub-powder-stage", out, trs] + ovs)
            ok = rc == 0
            rep.check(f"{name}.accept_stub", ok, f"exit {rc}" + (f": {err}" if rc else ""))
            if ok:
                trv = os.path.join(work, f"{name}.v2rec.stub.gcode")
                rc2, _ = run(exe, ["--config-from-gcode", "--stub-powder-stage", v2_path, trv] + ovs)
                if rc2 == 0:
                    totals["stub_cases"] += 1
                    totals["stub_same"] += open(trs, "rb").read() == open(trv, "rb").read()
    print(f"\ninfo: {totals['blocks']} native blocks, {totals['v2_blocks']} v2 blocks, max relative ca diff "
          f"{totals['max_rel_ca']:.3g}, oa {totals['max_rel_oa']:.3g}; stubbed translation byte-identical to v2's "
          f"records on {totals['stub_same']}/{totals['stub_cases']} inputs (informational: fallback paths differ "
          f"within the T1 tolerances)")
    for f in rep.fails:
        print(f"FAIL {f}")
    print(f"\n{rep.n} checks, {len(rep.fails)} failures")
    return 1 if rep.fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
