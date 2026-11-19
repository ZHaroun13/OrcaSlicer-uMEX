"""Audit-fix gate (diw/AUDIT_FIX.md): incremental ABC (G91.2). A and B are per-block deltas, dA = K_E*dE, dB = ratio*kB*dE.

usage:
  py -3.15 -E -s audit_gate.py <v2 engine dir> <diw_translate.exe> <work dir> [extra.gcode ...]

Per input (the edge cases of diw_cases.py, the corpus given on the command line, the in-file 'cube25' check):
  A1  golden for A: the running sum of the emitted dA equals A_abs = K_E * cumsum(dE) (dE of every dE > 0 call of v2's
      calculate_delta_b, in order) at every A block (|diff| <= 1e-3); number of A words equals the number of dE > 0 calls;
      each dA is K_E*dE within the 1e-4 quantisation (+ carried residual);
  B1  golden for B: the running sum of the emitted dB equals v2's absolute B (flow_scaled oracle, 'G92 B0.0000' resets
      accumulated as a baseline) at every B block (|diff| <= 1e-3 + 5e-5 per v2 reset: v2 rounds B to 4 decimals at each reset); each dB equals v2's per-block delta (calculate_delta_b)
      within 1.5e-5 (1e-5 quantisation with the residual carried); the final sum is within 1e-5 of the exact total;
  A2  everything except the A/B words, the bare 'G1 A0.000' lines, the G91.2 line and the G92 E reset is identical to v2;
  A3  every dA is >= 0 (A never moves backwards);
  A4  no A or B on any line other than a G1/G2/G3 block; A only with a positive dB; a B without A is a retraction (negative);
      no A or B on travel (cube25: 0);
  A5  exactly one G91.2, on the line after the header, before any motion, and no G90.2;
  A6  the old modes still produce v2 bytes (no G91.2, no G90.2);
  A7  settings: --m62-dwell puts a G04 before every M62 and changes nothing else; --suckback adds B-only moves
      (negative dB for the suck-back, positive for the re-prime) and leaves the A words unchanged; --retract-f-max
      is accepted; defaults are off.
Run diw_translate.exe from build/src/Release (DLLs). Exit code 0 when everything passes.
"""
import os
import re
import subprocess
import sys

from diw_cases import CASES

K_E = 3.48          # diw_k_screw default (v2 k_screw)
A_WORD = re.compile(r"\sA-?[0-9.]+")
B_WORD = re.compile(r"\sB-?[0-9.]+")
B_VAL = re.compile(r"(?:^|\s)B(-?[0-9.]+)")
A_VAL = re.compile(r"(?:^|\s)A(-?[0-9.]+)")


def lines_of(path):
    with open(path, "rb") as f:
        return f.read().replace(b"\r\n", b"\n").decode("utf-8", "replace").split("\n")


def code(line):
    return line.split(";", 1)[0]


def run_v2(E, src, out):
    cfg = dict(E.DEFAULT_CONFIG)
    cfg["enable_u_mex_powder_infill"] = False
    pp = E.DIWPostProcessor(cfg)
    calls = []                 # (dE, dB) of every calculate_delta_b call
    orig = pp.calculate_delta_b

    def spy(delta_e):
        r = orig(delta_e)
        calls.append((delta_e, r))
        return r

    pp.calculate_delta_b = spy
    pp.process_file(src, out)
    return calls


def translate(exe, src, out, *flags):
    return subprocess.run([exe, src, out, *flags], capture_output=True, text=True)


def v2_b_totals(v2_lines):
    """Per v2 B-bearing line: the true cumulative B (absolute words plus the baseline carried over 'G92 ... B0.0000')."""
    base, last, tot, resets = 0.0, 0.0, [], 0
    for l in v2_lines:
        m = B_VAL.search(code(l))
        if not m:
            continue
        v = float(m.group(1))
        if l.startswith("G92"):
            base += last
            last = 0.0
            resets += 1
            continue
        last = v
        tot.append((base + v, resets))
    return tot


def is_filtered_g92(l):
    return l.startswith("; (FILTERED) G92")


def check_one(E, exe, work, name, data, fails):
    src = os.path.join(work, f"{name}.in.gcode")
    with open(src, "wb") as f:
        f.write(data)
    v2o, cpp = os.path.join(work, f"{name}.v2.gcode"), os.path.join(work, f"{name}.new.gcode")
    calls = run_v2(E, src, v2o)
    r = translate(exe, src, cpp, "--product-defaults")
    if r.returncode != 0:
        fails.append(f"{name}: diw_translate exit {r.returncode} {r.stderr.strip()}")
        return None
    a, b = lines_of(v2o)[1:], lines_of(cpp)[1:]

    # A5
    n912 = [i for i, l in enumerate(b) if l.startswith("G91.2")]
    if n912 != [0] and data.strip():
        fails.append(f"{name}: A5 G91.2 at lines {n912}, expected [0]")
    if any(l.startswith("G90.2") for l in b):
        fails.append(f"{name}: A5 G90.2 present")
    b_nog = [l for l in b if not l.startswith("G91.2")]

    # A2: identical outside the A/B words. v2's bare 'G1 A0.000' lines and its 'G92 ... B0.0000' reset have no
    # counterpart (the reset is a filtered comment in the incremental mode, the totals just run on).
    def strip_axes(l):
        return B_WORD.sub("", A_WORD.sub("", l))
    sa = [strip_axes(l) for l in a]
    sa = [l for l in sa if l.strip() != "G1" and not re.match(r"^G92(\s*;.*)?$", l.strip())]
    sb = [strip_axes(l) for l in b_nog if not is_filtered_g92(l)]
    sb = [l for l in sb if l.strip() != "G1"]
    if sa != sb:
        i = next((k for k in range(min(len(sa), len(sb))) if sa[k] != sb[k]), min(len(sa), len(sb)))
        fails.append(f"{name}: A2 differs outside A/B at line {i + 1}: v2 {sa[i:i+1]} new {sb[i:i+1]}")
        return None

    # per-line structure (A3, A4)
    a_blocks, b_blocks, n_travel = [], [], 0
    for i, l in enumerate(b_nog):
        c = code(l)
        ma, mb = A_VAL.search(c), B_VAL.search(c)
        if (ma or mb) and not re.match(r"[Gg][123]\s", c):
            fails.append(f"{name}: A4 A/B on a non-motion (G1/G2/G3) line {i + 2}: {l!r}")
        if re.match(r"G0\s", c) and (ma or mb):
            n_travel += 1
        if ma:
            da = float(ma.group(1))
            if da < 0:
                fails.append(f"{name}: A3 negative dA at line {i + 2}: {l!r}")
            if not mb or float(mb.group(1)) <= 0:
                fails.append(f"{name}: A4 A without a positive dB at line {i + 2}: {l!r}")
            a_blocks.append(da)
        if mb:
            b_blocks.append((float(mb.group(1)), bool(ma), l))
        if mb and not ma and float(mb.group(1)) > 0 and "suck-back" not in l:
            fails.append(f"{name}: A4 positive dB without A (not a re-prime) at line {i + 2}: {l!r}")

    # A1: running sum of dA vs K_E * cumsum(dE > 0); per block dA vs K_E * dE
    exp_de = [d for d, _ in calls if d > 0]
    if len(a_blocks) != len(exp_de):
        fails.append(f"{name}: A1 A words {len(a_blocks)} != dE>0 calls {len(exp_de)}")
    else:
        run, ref = 0.0, 0.0
        for k, (da, de) in enumerate(zip(a_blocks, exp_de)):
            run += da
            ref += K_E * de
            if abs(run - ref) > 1e-3 or abs(da - K_E * de) > 1.5e-4:
                fails.append(f"{name}: A1 block {k}: dA={da} (expected {K_E * de:.4f}), sum {run:.4f} vs {ref:.4f}")
                break

    # B1: running sum of dB vs v2 absolute B (with reset baseline); per block dB vs v2's calculate_delta_b
    v2_tot = v2_b_totals(a)
    exp_db = [db for d, db in calls if d != 0]
    got_db = [v for v, _, _ in b_blocks]
    if len(got_db) != len(v2_tot):
        fails.append(f"{name}: B1 B words {len(got_db)} != v2 B lines {len(v2_tot)}")
    else:
        run = 0.0
        for k, (db, (tot, nres)) in enumerate(zip(got_db, v2_tot)):
            run += db
            # v2 drops the sub-1e-4 residual of its rounded absolute B at every G92 reset (<= 5e-5 each)
            if abs(run - tot) > 1e-3 + 5e-5 * nres:
                fails.append(f"{name}: B1 block {k}: running dB {run:.5f} vs v2 absolute B {tot:.4f}")
                break
    if len(got_db) == len(exp_db):
        for k, (db, ex) in enumerate(zip(got_db, exp_db)):
            if abs(db - ex) > 1.5e-5:
                fails.append(f"{name}: B1 block {k}: dB={db} expected {ex:.6f}")
                break
        if abs(sum(got_db) - sum(exp_db)) > 1e-5:
            fails.append(f"{name}: B1 final sum {sum(got_db):.6f} vs exact {sum(exp_db):.6f}")
    else:
        fails.append(f"{name}: B1 B words {len(got_db)} != calculate_delta_b calls with dE != 0 {len(exp_db)}")
    return len(a_blocks), n_travel


def main(argv):
    engine_dir, exe, work = argv[1], argv[2], argv[3]
    extra = argv[4:]
    sys.path.insert(0, engine_dir)
    import diw_engine as E  # noqa: E402

    os.makedirs(work, exist_ok=True)
    inputs = dict(CASES)
    for p in extra:
        with open(p, "rb") as f:
            inputs[os.path.basename(p)] = f.read()
    fails = []
    for name, data in inputs.items():
        res = check_one(E, exe, work, name, data, fails)
        if res is not None:
            print(f"ok   A1-A5,B1 {name} ({res[0]} A words, {res[1]} A/B on G0 travel)")
            if res[1] != 0:
                fails.append(f"{name}: {res[1]} A/B words on G0 travel, expected 0")
    # cube25 must be present in the audit (required by the lead)
    if not any(n.startswith("cube25") for n in inputs):
        fails.append("cube25.gcode not given on the command line: the audit check was not run")

    # A6: old modes untouched
    src = os.path.join(work, "a6.in.gcode")
    probe = None
    for n, d in inputs.items():
        if b"E" in d and b"LAYER_CHANGE" in d and len(d) > 200:
            probe = (n, d)
            break
    if probe is None:
        probe = ("relative_e_arcs_line_numbers", CASES["relative_e_arcs_line_numbers"])
    with open(src, "wb") as f:
        f.write(probe[1])
    for mode in ("flow_scaled", "proportional", "cumulative"):
        out = os.path.join(work, f"a6.{mode}.gcode")
        r = translate(exe, src, out, f"diw_a_axis_mode={mode}")
        txt = b"\n".join(l.encode() for l in lines_of(out))
        if r.returncode != 0 or b"G90.2" in txt or b"G91.2" in txt:
            fails.append(f"A6 {mode}: rc {r.returncode} or G90.2/G91.2 present")
        else:
            print(f"ok   A6 {mode} has no G90.2/G91.2")

    # A7: settings (on the probe input)
    base = os.path.join(work, "a7.base.gcode")
    translate(exe, src, base, "--product-defaults")
    bl = lines_of(base)
    n_m62 = sum(1 for l in bl if l.startswith("M62"))
    dw = os.path.join(work, "a7.dwell.gcode")
    r = translate(exe, src, dw, "--product-defaults", "--m62-dwell", "0.25")
    dl = lines_of(dw)
    if r.returncode != 0:
        fails.append(f"A7 m62-dwell rc {r.returncode}")
    else:
        bad = [i for i, l in enumerate(dl) if l.startswith("M62") and not dl[i - 1].startswith("G04")]
        if bad:
            fails.append(f"A7 M62 without preceding G04 at lines {[i + 1 for i in bad[:5]]}")
        stripped = []
        for i, l in enumerate(dl):
            if l.startswith("G04") and i + 1 < len(dl) and dl[i + 1].startswith("M62") and "0.25" in l:
                continue
            stripped.append(l)
        if stripped[1:] != bl[1:] and n_m62:
            fails.append("A7 m62-dwell changed more than the dwell lines")
        print(f"ok   A7 m62-dwell ({n_m62} M62 in probe)")
    sb = os.path.join(work, "a7.suckback.gcode")
    r = translate(exe, src, sb, "--product-defaults", "--suckback", "0.4", "--suckback-feed", "200")
    sl = lines_of(sb)
    if r.returncode != 0:
        fails.append(f"A7 suckback rc {r.returncode}")
    else:
        aw = [A_VAL.search(code(l)).group(1) for l in sl if A_VAL.search(code(l))]
        ab = [A_VAL.search(code(l)).group(1) for l in bl if A_VAL.search(code(l))]
        if aw != ab:
            fails.append("A7 suck-back changed the A words")
        for l in sl:
            m = B_VAL.search(code(l))
            if "suck-back" in l:
                if A_VAL.search(code(l)):
                    fails.append(f"A7 A on a suck-back line: {l!r}")
                    break
                if not m or (float(m.group(1)) > 0) != ("re-prime" in l):
                    fails.append(f"A7 suck-back sign wrong (negative dB for the suck-back, positive for the re-prime): {l!r}")
                    break
        print(f"ok   A7 suckback ({sum('suck-back' in l for l in sl)} lines)")
    rm = os.path.join(work, "a7.retract.gcode")
    r = translate(exe, src, rm, "--product-defaults", "--retract-f-max", "120")
    if r.returncode != 0:
        fails.append(f"A7 retract-f-max rc {r.returncode}")
    else:
        print("ok   A7 retract-f-max accepted")

    for f in fails:
        print("FAIL", f)
    print(f"\n{len(fails)} failure(s)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
