"""Core-layer sequencing test (POWDER_PLAN increment 3a): diw_translate.exe driven by v2's own plans as `;DIW:` records
vs plain v2 in real powder mode.

usage:
  python compare_sequencer.py <v2 engine dir> <diw_translate.exe> <work dir> <corpus.gcode ...> [--stub-s4] [--allow-version-drift]

Per corpus file and variant (compare_records.VARIANTS + SEQ_VARIANTS; plus the generated notch geometry with
compare_records.NOTCH_VARIANTS + NOTCH_SEQ_VARIANTS):
  - capture:  v2 in real powder mode records its plans as blocks (v2_hooks.capture) into a copy of the source;
  - expected: plain v2, real powder mode, run on the ORIGINAL file (no records);
  - actual:   diw_translate --config-from-gcode <records file> diw_powder_infill=1 <variant overrides>;
  - sequence: expected == actual after the header line (as compare_powder.norm), first difference with context;
  - blocks_are_cores: every layer v2 classifies as core has a block. v2 marks a layer core but plans nothing
    (so no block is written) when the layer height is <= 0 or planning raises; the C++ side cannot know about
    such a layer, and it would also change v2's last_in_stack for the layer below. This check proves that no
    parity case hits that edge (POWDER_PLAN.md, "Increment 3a").
With --stub-s4, stage 4 (powder emission, increment 3b) is stubbed on both sides under one rule: where S4 would
emit, it emits exactly its header comment line
    "; --- uMEX L{k} S4 powder infill F={F:.1f} d_early={F / 60.0 * lag_off:.3f} ---"
and nothing else, with no state change (v2: _emit_powder monkeypatched; C++: --stub-powder-stage).
Without --stub-s4 both sides emit the real stage 4 (increment 3b).

Line-number prefixes (first corpus file, default variant): one extruding XY move in the first core layer gets an
"n7 " prefix (lower_n: v2 _RE_CMD has no IGNORECASE, so the line is no command there) and an "N7 " prefix (upper_n);
each runs the same capture / expected / actual / sequence parity.

Synthetic S4 runs (first corpus file, default variant): the runs of its first st=ok block are replaced by hand-made
runs that take every early-stop branch of v2 _emit_powder (zero-length segment, stop at a segment end, stop at a
segment start, stop inside, stop at the run start, run shorter than d_early) plus point dwells at, below and above
powder_lag_off. expected: v2 replaying those records (v2_hooks.replay); actual: diw_translate on the same file.

Negative cases (first corpus file, default variant; always with --stub-powder-stage): a tampered fingerprint,
an F mismatch, a block in the wrong layer, a block after its layer's first motion line, and a record parse
error must each stop diw_translate with exit 3 and a clear message.

Exit code 0 only with 0 failures and 0 skips.
"""
import contextlib
import json
import math
import os
import platform
import re
import subprocess
import sys

import v2_blocks as B
import v2_hooks as H
from compare_powder import PINNED, parse_config_block, v2_config
from compare_records import NOTCH_VARIANTS, VARIANTS, make_notch

# Sequencer-only variants (on top of compare_records'): S5 by interval and last_in_stack (on the notch, the layer
# below an st=open layer is not last in its stack), the cumulative-A / no-UV branches of S1 and S3, and the
# post-extrusion dwell that replaces the auger stop dwell.
SEQ_VARIANTS = {
    "interval3": ["diw_compaction_interval=3"],
    "cumulative_noUV": ["diw_a_axis_mode=cumulative", "diw_uv_on_gcode="],
    "post_dwell": ["diw_post_extrusion_dwell=0.3"],
    # S4 (increment 3b): no early stop (d_early = 0) and no transit hop; a negative powder Z offset (z_safe uses
    # min(0, off_z)) with a clearance and a long lag; point dwells shorter than lag_off.
    "s4_lag0_nohop": ["diw_powder_lag_off=0", "diw_powder_travel_hop=0"],
    "s4_negz_longlag": ["diw_powder_offset_z=-0.3", "diw_powder_z_clearance=0.2", "diw_powder_lag_on=0.05",
                        "diw_powder_lag_off=0.4"],
    "s4_points_lag": ["diw_powder_point_dwell_length=100000", "diw_powder_lag_off=0.6"],
}
NOTCH_SEQ_VARIANTS = {"notch_interval3": ["diw_compaction_interval=3"]}

S4_STUB_FMT ="; --- uMEX L{k} S4 powder infill F={F:.1f} d_early={d:.3f} ---\n"


@contextlib.contextmanager
def stub_s4(E, on):
    """v2 _emit_powder replaced by the S4 header line only (the same rule as --stub-powder-stage)."""
    if not on:
        yield
        return
    owner = H._owner(E.DIWPostProcessor, "_emit_powder")
    orig = owner.__dict__["_emit_powder"]

    def stub(self, plan, z, F, k, out):
        out.append(S4_STUB_FMT.format(k=k, F=F, d=F / 60.0 * self.P.lag_off))
    owner._emit_powder = stub
    try:
        yield
    finally:
        owner._emit_powder = orig


def norm_bytes(data):
    return data.replace(b"\r\n", b"\n").split(b"\n")[1:]


def diff_context(a, b, ctx=4):
    i = next((k for k in range(min(len(a), len(b))) if a[k] != b[k]), min(len(a), len(b)))
    lo = max(0, i - ctx)
    out = [f"first difference at line {i + 2} of the output (line {i + 1} after the header)"]
    for tag, lines in (("v2 ", a), ("c++", b)):
        out.append(f"   {tag}:")
        for k in range(lo, min(len(lines), i + ctx + 1)):
            out.append(f"     {'>' if k == i else ' '} {k + 2:6} {lines[k]!r}")
        if i >= len(lines):
            out.append(f"     > {i + 2:6} <end>")
    return "\n".join(out)


def run_cpp(exe, args, log):
    r = subprocess.run([exe] + args, capture_output=True, cwd=os.path.dirname(exe))
    with open(log, "wb") as fh:
        fh.write(r.stdout + b"\n--- stderr ---\n" + r.stderr)
    return r.returncode, r.stderr.decode("utf-8", "replace").strip()


def s4_branches(F, lag_off, run):
    """The branches of v2 _emit_powder (same arithmetic, same order) one line run takes."""
    d_early = F / 60.0 * lag_off
    pts = run.pts
    total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    stop_s = total - d_early
    hit, stopped = set(), False
    if stop_s <= 1e-9:
        stopped = True
        hit.add("short" if total < d_early - 1e-9 else "stop_at_start")
    cum = 0.0
    for a, b in zip(pts, pts[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-9:
            hit.add("zero_segment")
            continue
        if not stopped and cum + seg >= stop_s - 1e-9:
            t = (stop_s - cum) / seg
            hit.add("t>=1" if t >= 1.0 - 1e-9 else "t<=0" if t <= 1e-9 else "t inside")
            stopped = True
        cum += seg
    return hit


S4_TARGETS = ("zero_segment", "t>=1", "t<=0", "t inside", "stop_at_start", "short")


def synthetic_s4(E, exe, work, stub, flags, base, record):
    """Hand-made runs in the first st=ok block of the base case, built so that every early-stop branch of
    v2 _emit_powder is taken (S4_TARGETS), plus point dwells at, below and above powder_lag_off.
    expected: v2 replaying the edited records (v2_hooks.replay); actual: diw_translate on the same records."""
    name, src, cfg, rec_gcode, layers, ov = base
    L = next((l for l in layers if l.st == "ok"), None)
    if L is None:
        record("s4_synthetic", "FAIL", f"{name} has no st=ok block")
        return
    rec = B.parse_records(L.block.encode("ascii"))[0][0]
    lag_off = H._params(E, cfg).lag_off
    d = rec.F / 60.0 * lag_off
    x0, y0 = rec.runs[0].pts[0]

    def pick(target, make, epss):
        for eps in epss:
            r = make(eps)
            if target in s4_branches(rec.F, lag_off, r):
                return r
        return None

    line = lambda pts: B.RunRec("line", pts)
    runs = [
        line([(x0, y0), (x0, y0), (x0 + d + 4.0, y0), (x0 + d + 4.0, y0), (x0 + d + 4.0, y0 + 2.0)]),
        pick("t>=1", lambda e: line([(x0, y0 + 1.0), (x0 + 3.0, y0 + 1.0), (x0 + 3.0, y0 + 1.0 + d + e)]),
             [0.0, 1e-12, -1e-12, 1e-10, -1e-10]),
        pick("t<=0", lambda e: line([(x0, y0 + 2.0), (x0 + 3.0, y0 + 2.0), (x0 + 3.0, y0 + 2.0 + d + e)]),
             [3e-9, 2e-9, 4e-9, 5e-9, 1.5e-9]),
        pick("stop_at_start", lambda e: line([(x0, y0 + 3.0), (x0, y0 + 3.0 + d + e)]),
             [5e-10, 2e-10, 0.0, -2e-10, -5e-10]),
        line([(x0, y0 + 4.0), (x0 + d / 2.0, y0 + 4.0)]),
        B.RunRec("point", [(x0 + 1.0, y0 + 5.0)], lag_off),
        B.RunRec("point", [(x0 + 2.0, y0 + 5.0)], lag_off * 0.5),
        B.RunRec("point", [(x0 + 3.0, y0 + 5.0)], lag_off + 0.3),
    ]
    if any(r is None for r in runs):
        record("s4_synthetic.build", "FAIL", "could not construct a run for every early-stop branch")
        return
    hit = set().union(*(s4_branches(rec.F, lag_off, r) for r in runs if r.kind == "line"))
    missing = [t for t in S4_TARGETS if t not in hit]
    record("s4_synthetic.branches", "FAIL" if missing else "ok",
           f"missing {missing}" if missing else f"runs take every early-stop branch {list(S4_TARGETS)}; "
                                                f"point dwells at, below and above lag_off={lag_off!r}")
    rec.runs = runs
    body = rec_gcode.replace(L.block.encode("ascii"), B.write_rec(rec).encode("ascii"), 1)
    sname = f"{name}.s4_synthetic"
    rec_path = os.path.join(work, f"{sname}.rec.gcode")
    with open(rec_path, "wb") as fh:
        fh.write(body)
    exp_path = os.path.join(work, f"{sname}.v2replay{'.stub' if stub else ''}.gcode")
    with stub_s4(E, stub):
        _, problems = H.replay(E, cfg, src, body, exp_path)
    if problems:
        record(f"{sname}.replay", "FAIL", "; ".join(problems))
        return
    cpp_path = os.path.join(work, f"{sname}.cpp{'.stub' if stub else ''}.gcode")
    rc, err = run_cpp(exe, flags + [rec_path, cpp_path, "diw_powder_infill=1"] + ov, cpp_path.replace(".gcode", ".log"))
    if rc != 0:
        record(f"{sname}.sequence", "FAIL", f"diw_translate exit {rc}: {err}")
        return
    a = norm_bytes(open(exp_path, "rb").read())
    b = norm_bytes(open(cpp_path, "rb").read())
    n_pd = sum(1 for l in a if l == b"; point dwell")
    if a == b:
        record(f"{sname}.sequence", "ok", f"{len(a)} lines identical (block k={rec.k}: {len(runs)} runs, "
                                          f"{n_pd} point dwell(s) in the output)")
    else:
        record(f"{sname}.sequence", "FAIL", diff_context(a, b))


def main(argv):
    allow_drift = "--allow-version-drift" in argv
    stub = "--stub-s4" in argv
    argv = [a for a in argv if a not in ("--allow-version-drift", "--stub-s4")]
    if len(argv) < 5:
        print(__doc__)
        return 2
    engine_dir, exe, work = argv[1], os.path.abspath(argv[2]), os.path.abspath(argv[3])
    corpus = argv[4:]
    sys.path.insert(0, engine_dir)
    import diw_engine as E  # noqa: E402
    import shapely  # noqa: E402
    versions = {"python": platform.python_version(), "shapely": shapely.__version__, "geos": shapely.geos_version_string}
    print(f"versions: {versions}, stub S4: {stub}")
    if versions != PINNED and not allow_drift:
        print(f"version drift from {PINNED}; pass --allow-version-drift to run anyway")
        return 2
    if not os.path.isfile(exe):
        print(f"cannot read {exe}")
        return 2
    os.makedirs(work, exist_ok=True)
    report = {"versions": versions, "stub_s4": stub, "cases": {}}
    counts = {"ok": 0, "FAIL": 0, "SKIP": 0}

    def record(name, status, detail):
        counts[status] += 1
        report["cases"][name] = {"status": status, "detail": detail}
        print(f"{status:4} {name}: {detail}")

    flags = ["--config-from-gcode"] + (["--stub-powder-stage"] if stub else [])
    inputs = []
    for path in corpus:
        with open(path, "rb") as fh:
            inputs.append((os.path.splitext(os.path.basename(path))[0], fh.read(), {**VARIANTS, **SEQ_VARIANTS}))
    if inputs:
        inputs.append(("notch", make_notch(inputs[0][1]), {**NOTCH_VARIANTS, **NOTCH_SEQ_VARIANTS}))
    def parity(name, data, ov):
        """capture / expected / actual / sequence (+ blocks_are_cores) for one input and one variant."""
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
        rec_path = os.path.join(work, f"{name}.rec.gcode")
        with open(rec_path, "wb") as fh:
            fh.write(rec_gcode)
        exp_path = os.path.join(work, f"{name}.v2{'.stub' if stub else ''}.gcode")
        with stub_s4(E, stub):
            stats = E.DIWPostProcessor(cfg).process_file(src, exp_path)
        n_core = stats["umex"]["classification"]["n_core"]
        ks = [l.k for l in layers]
        ok = n_core == len(layers)
        record(f"{name}.blocks_are_cores", "ok" if ok else "FAIL",
               f"{n_core} core layer(s) in v2, {len(layers)} block(s)" +
               ("" if ok else " (a core layer without a block: h <= 0 or planning failed; parity cannot hold)"))

        cpp_path = os.path.join(work, f"{name}.cpp{'.stub' if stub else ''}.gcode")
        rc, err = run_cpp(exe, flags + [rec_path, cpp_path, "diw_powder_infill=1"] + ov,
                          cpp_path.replace(".gcode", ".log"))
        sts = {s: sum(1 for l in layers if l.st == s) for s in ("ok", "open", "small")}
        if rc != 0:
            record(f"{name}.sequence", "FAIL", f"diw_translate exit {rc}: {err}")
        else:
            a = norm_bytes(open(exp_path, "rb").read())
            b = norm_bytes(open(cpp_path, "rb").read())
            n_s1 = sum(1 for l in a if b" S1 paste contour " in l)
            if a == b:
                record(f"{name}.sequence", "ok",
                       f"{len(a)} lines identical; blocks {sts} k={ks}, {n_s1} layer(s) sequenced")
            else:
                record(f"{name}.sequence", "FAIL", f"blocks {sts}\n" + diff_context(a, b))
        return src, cfg, rec_gcode, layers

    neg_base = None
    for ci, (cname, data, variants) in enumerate(inputs):
        for vname, ov in variants.items():
            name = f"{cname}.{vname}"
            src, cfg, rec_gcode, layers = parity(name, data, ov)
            if ci == 0 and vname == next(iter(variants)):
                neg_base = (name, rec_gcode, layers, ov)
                s4_base = (name, src, cfg, rec_gcode, layers, ov)

    # ------------------------------------------------------------------ line-number prefixes
    # v2 _RE_CMD = ^(?:N\d+\s+)?([GMgm])0*(\d+) has no IGNORECASE: "N7 G1 ..." is a G1, but "n7 G1 ..." is no
    # command at all (a pending non-motion line that is passed through, its dE folded into the next segment).
    # One extruding wall move inside a core layer gets each prefix: in cube25 when it is in the corpus, the wall
    # move "G1 X59.6 Y80.4 E15.43339" of core layer 3 (the case that diverged before the fix); otherwise the third
    # extruding XY move of the first core layer of the first corpus file.
    if neg_base is None:
        record("line_number", "FAIL", "no corpus input")
    else:
        cube = next((i for i in inputs if i[0] == "cube25"), None)
        cname, data, variants = cube or inputs[0]
        ov = variants[next(iter(variants))]
        txt = data.decode("utf-8")
        target, k0 = None, None
        if cube:
            at = re.search(r"^G1 X59\.6 Y80\.4 E15\.43339\r?$", txt, re.M)
            target = at.start() if at else None
        else:
            k0 = neg_base[2][0].k if neg_base[2] else None
        starts = [m.start() for m in re.finditer(r"^;LAYER_CHANGE[^\n]*\n", txt, re.M)]
        if target is None and k0 is not None and k0 <= len(starts):
            lo, hi = starts[k0 - 1], (starts[k0] if k0 < len(starts) else len(txt))
            moves = [m for m in re.finditer(r"^G1 X[-\d.]+ Y[-\d.]+ E[-\d.]+[ \t]*\r?$", txt[lo:hi], re.M)]
            if len(moves) >= 3:
                target = lo + moves[2].start()
        if target is None:
            record("line_number", "FAIL", f"{cname}: no extruding XY move in core layer {k0}")
        else:
            for tag, prefix in (("lower_n", "n7 "), ("upper_n", "N7 ")):
                parity(f"{cname}.line_number_{tag}", (txt[:target] + prefix + txt[target:]).encode("utf-8"), ov)

    # ------------------------------------------------------------------ synthetic S4 runs
    if neg_base is None:
        record("s4_synthetic", "FAIL", "no corpus input")
    else:
        synthetic_s4(E, exe, work, stub, flags, s4_base, record)

    # ------------------------------------------------------------------ negative cases
    if neg_base is None:
        record("negative", "FAIL", "no corpus input")
    else:
        name, rec_gcode, layers, ov = neg_base
        L = next((l for l in layers if l.st == "ok"), None)
        if L is None:
            record("negative", "FAIL", f"{name} has no st=ok block")
        else:
            blk = L.block.encode("ascii")
            rec = B.parse_records(blk)[0][0]
            cases = {}

            r = B.parse_records(blk)[0][0]
            r.fp ^= 0x1
            cases["fingerprint"] = (rec_gcode.replace(blk, B.write_rec(r).encode("ascii"), 1),
                                    [f"block k={rec.k} ", "fingerprint", "different settings"])
            r = B.parse_records(blk)[0][0]
            r.F = r.F + 1.0
            cases["F_mismatch"] = (rec_gcode.replace(blk, B.write_rec(r).encode("ascii"), 1),
                                   [f"block k={rec.k} ", f"has F={r.F!r} ", "the current settings give F=", "rejected"])
            # wrong layer: the block moved to right after the start marker of a layer without a block
            ks = {l.k for l in layers}
            data_wo = rec_gcode.replace(blk, b"", 1)
            starts = [m.end() for m in re.finditer(rb"^;LAYER_CHANGE[^\n]*\n", data_wo, re.M)]
            free = next((i for i in range(len(starts)) if (i + 1) not in ks), None)
            if free is None:
                record("negative.wrong_layer", "FAIL", "no layer without a block")
            else:
                at = starts[free]
                cases["wrong_layer"] = (data_wo[:at] + blk + data_wo[at:],
                                        [f"block k={rec.k} ", f"sits in printed layer {free + 1}", "k must equal"])
            # after the first motion line of its own layer
            at = rec_gcode.index(blk)
            m = re.compile(rb"^G[0-3]\b[^\n]*\n", re.M).search(rec_gcode, at + len(blk))
            moved = rec_gcode[:at] + rec_gcode[at + len(blk):m.end()] + blk + rec_gcode[m.end():]
            cases["after_motion"] = (moved, [f"block k={rec.k} ", "after the first motion line"])
            # parse error: one digit of an XY line changed, CRC not recomputed
            lines = L.block.split("\n")
            xi = next(i for i, l in enumerate(lines) if l.startswith(";DIW:XY "))
            bad = lines[:]
            bad[xi] = re.sub(r"\d", lambda mm: "5" if mm.group(0) != "5" else "6", bad[xi], count=1)
            bad_gcode = rec_gcode.replace(blk, "\n".join(bad).encode("ascii"), 1)
            errs = B.parse_records(bad_gcode)[1]
            if not errs:
                record("negative.parse_error", "FAIL", "the tampered block parsed without error")
            else:
                ln, reason = errs[0]
                cases["parse_error"] = (bad_gcode, [f"record at line {ln} ", f"({reason})"])

            for cname, (body, needles) in cases.items():
                p_in = os.path.join(work, f"neg.{cname}.gcode")
                with open(p_in, "wb") as fh:
                    fh.write(body)
                p_out = p_in.replace(".gcode", ".out.gcode")
                rc, err = run_cpp(exe, ["--config-from-gcode", "--stub-powder-stage", p_in, p_out, "diw_powder_infill=1"] + ov,
                                  p_in.replace(".gcode", ".log"))
                missing = [n for n in needles if n not in err]
                if rc == 3 and not missing:
                    record(f"negative.{cname}", "ok", f"exit 3: {err}")
                else:
                    record(f"negative.{cname}", "FAIL", f"exit {rc} (want 3), missing {missing}: {err}")

    with open(os.path.join(work, "compare_sequencer_report.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"\n{counts['ok']} ok, {counts['FAIL']} failure(s), {counts['SKIP']} skipped")
    return 0 if counts["FAIL"] == 0 and counts["SKIP"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
