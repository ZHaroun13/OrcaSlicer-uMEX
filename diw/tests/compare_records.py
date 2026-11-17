"""`;DIW:` powder records v1 test: Python reference (v2_blocks / v2_hooks) vs v2, and vs diw_translate.exe.

usage:
  python compare_records.py <v2 engine dir> <diw_translate.exe> <work dir> <corpus.gcode ...> [--allow-version-drift]

Per corpus file and variant (v2 in real powder mode, no umex_n_bottom_layers override):
  - capture: v2's own plans written as blocks into a copy of the source;
  - strip:     removing every ;DIW: line gives the source back byte for byte;
  - roundtrip: parse -> rewrite gives the same blocks, no errors, one block per planned layer;
  - replay:    v2 driven by the parsed records is byte-identical to plain v2 (records lose nothing);
  - exercises: the variant really exercises what it is named for, judged against the input's base variant
    (default / notch), so an override that v2_config or the planner ignored fails;
  - exercises_control: the same predicate on the base variant's own plans is false (the predicate can fail);
  - cpp_dump / cpp_rewrite / cpp_fingerprint: diw_translate --dump-records / --rewrite-records /
    --fingerprint must equal the Python dump / blocks / fingerprint byte for byte.
Plus synthetic hostile inputs: the Python dump must show the expected errors, and the C++ dump must equal it.
Plus the translator's record filter (section 4): translate(F) == translate(strip_records(F)) for the first corpus
file's records with lone-CR lines injected, in paste mode. Powder mode reads the records (increment 3a), so there
the lone-CR record line must stop the export with the reader's line and reason, and a ";DIW:" after a lone CR must
stay a comment while the blocks are still sequenced (compare_sequencer.py checks the sequencing itself).

A missing or unreadable diw_translate.exe stops the run (exit 2); a binary without one of the flags fails every
check that needs it. Exit code 0 when every check passes.

Tool interface (POWDER_RECORDS.md section 7; results on stdout, a copy is kept in the work dir):
  diw_translate --dump-records <in>
  diw_translate --rewrite-records <in>
  diw_translate [--config-from-gcode] --fingerprint <in> [key=value ...]   -> "<fingerprint string>\n<hex>\n"
"""
import json
import os
import platform
import re
import subprocess
import sys

import v2_blocks as B
import v2_hooks as H
from compare_powder import PINNED, parse_config_block, v2_config

VARIANTS = {
    "default": [],
    "angle30": ["diw_powder_hatch_angle=30"],
    "tiny_hatch": ["diw_powder_hatch_spacing=0.05"],
    "clamp": ["diw_powder_overflow_policy=clamp"],
    "no_overflow": ["diw_powder_speed_max=200"],
    "small": ["diw_shell_powder_clearance=20"],
    "point_runs": ["diw_powder_point_dwell_length=100000"],
    "offsets": ["diw_powder_offset_x=12.5", "diw_powder_offset_y=-3", "diw_powder_offset_z=0.4"],
}


def _geom(l):
    """The RUN / FB / XY / DE lines of a captured layer's block (without BEGIN, STAT and END, which carry fp / crc)."""
    return l.block.split("\n")[2:-2]


def _vs_base(L, D, fn):
    """True when fn(layer, base layer of the same k) holds for some layer of L."""
    d = {l.k: l for l in D}
    return any(fn(l, d[l.k]) for l in L if l.k in d)


# What each variant must exercise (otherwise the variant does not test what it is named for).
# Called as fn(L, D, P, out, base_out): the variant's captured layers, the base variant's layers (default /
# notch) of the same input, the variant's UMexParams, and the plain v2 outputs of the variant and the base.
# Every predicate that names an override compares with the base, so an ignored override fails; main() also
# runs it on the base itself (exercises_control), where it must be false.
EXPECT = {
    # under increase_spacing s_eff does not depend on s_hatch, but f_calc (fu) does
    "tiny_hatch": lambda L, D, P, o, bo: (P.s_hatch == 0.05 and any(l.fu != l.F for l in L) and
                                          _vs_base(L, D, lambda l, b: l.fu != b.fu)),
    # clamp keeps s = s_hatch and clamps the feed; the base (increase_spacing) widens s instead
    "clamp": lambda L, D, P, o, bo: (P.policy == "clamp" and
                                     any(l.st == "ok" and l.s == P.s_hatch and l.fu > l.F for l in L) and
                                     _vs_base(L, D, lambda l, b: l.s != b.s)),
    "no_overflow": lambda L, D, P, o, bo: (all(l.fu == l.F for l in L) and
                                           _vs_base(L, D, lambda l, b: l.F != b.F)),
    "small": lambda L, D, P, o, bo: any(l.st == "small" for l in L),
    "point_runs": lambda L, D, P, o, bo: any(r.kind == "point" for l in L if l.plan for r in l.plan.runs),
    "angle30": lambda L, D, P, o, bo: (any(l.st == "ok" for l in L) and
                                       _vs_base(L, D, lambda l, b: l.st == "ok" and _geom(l) != _geom(b))),
    "notch_angle30": lambda L, D, P, o, bo: (any(l.st == "ok" for l in L) and
                                             _vs_base(L, D, lambda l, b: l.st == "ok" and _geom(l) != _geom(b))),
    # offsets are not planning inputs (same records and fingerprint), so they must show in v2's output
    "offsets": lambda L, D, P, o, bo: o != bo,
    "default": lambda L, D, P, o, bo: any(l.st == "ok" for l in L),
    "notch": lambda L, D, P, o, bo: (
        any(l.st == "open" for l in L) and
        any(p.travel_feed is None for l in L if l.st == "ok" for p in l.plan.fallback_paths) and
        any(p.travel_feed is not None for l in L if l.st == "ok" for p in l.plan.fallback_paths)),
}
# Variants of the generated notch geometry (see make_notch); the corpus file's own settings apply.
NOTCH_VARIANTS = {"notch": [], "notch_angle30": ["diw_powder_hatch_angle=30"]}

# Lone-CR lines for the translator's record filter (section 4): a lone \r ends neither a record line nor any
# other line for the filter, so the first piece is one record line (dropped whole, G1 included) and the
# second is not a record (kept whole; the translator then sees ";DIW:BEGIN ..." as an ordinary comment).
LONE_CR = b";DIW:XY 1.0 2.0\rG1 X5 Y5 E1\nG1 X1\r;DIW:BEGIN v=1 k=1\n"


# ----------------------------------------------------------------------------- synthetic inputs

def _synth_rec(k=3):
    return B.PowderLayerRec(
        k=k, st="ok", z=2.1, h=0.7, s=1.2, F=3000.0, fu=3809.5238095238096, fp=0x1234abcd,
        ca=120.5, oa=80.25, ra=3.0, hl=55.5, nc=2,
        runs=[B.RunRec("line", [(float(i), 0.5 * i) for i in range(11)]),
              B.RunRec("point", [(10.0, -0.0)], 1.25)],
        fbs=[B.FbRec(w=1.4, tf=None, z=2.1, ty="sparse infill", pts=[(1.0, 1.0), (2.0, 1.0), (2.0, 2.0)],
                     segs=[(0.01, 600.0), (1e-05, 600.0)]),
             B.FbRec(w=1.45, tf=3000.0, z=2.1, ty="internal infill", pts=[(0.0, 0.0), (1.5, 0.0)],
                     segs=[(0.02, 900.0)])])


def _blk(**kw):
    return B.write_rec(_synth_rec(**kw))


def _lines(s):
    return s.split("\n")[:-1]


def _join(ls):
    return "".join(l + "\n" for l in ls)


def _edit(block, idx, fn):
    ls = _lines(block)
    ls[idx] = fn(ls[idx])
    return _join(ls)


def _resign(block):
    """Recomputes n and crc of a block whose body lines were edited (to get past the CRC check)."""
    ls = _lines(block)[:-1]
    body = _join(ls)
    return body + f";DIW:END n={len(ls) - 1} crc={B.crc32(body.encode('ascii')):08x}\n"


def synthetic_cases():
    """name -> (bytes, expected [(line, reason)] or None to only compare with C++)."""
    G = ";LAYER_CHANGE\n;Z:2.1\n"
    b = _blk()
    nb = len(_lines(b))
    open_blk = B.write_rec(B.PowderLayerRec(k=4, st="open", z=2.8, h=0.7, s=1.2, F=3000.0, fu=3000.0, fp=1))
    small_blk = B.write_rec(B.PowderLayerRec(k=5, st="small", z=3.5, h=0.7, s=1.2, F=3000.0, fu=3000.0, fp=1,
                                             ca=1.5, oa=0.0, ra=1.5, hl=0.0, nc=0))
    fb_line = next(i for i, l in enumerate(_lines(b)) if l.startswith(";DIW:FB"))
    c = {}
    c["valid_all_kinds"] = (G + b + "G1 X1 Y1\n" + G + open_blk + G + small_blk, [])
    c["crlf_accepted"] = ((G + b + open_blk).replace("\n", "\r\n"), [])
    c["no_trailing_newline"] = (b[:-1], [])
    c["tampered_digit"] = (G + _edit(b, 3, lambda l: l.replace("1.0", "1.5", 1)), [(2 + nb, B.E_CRC)])
    c["missing_end"] = (G + _join(_lines(b)[:-1]), [(3, B.E_UNTERMINATED)])
    c["missing_end_then_gcode"] = (G + _join(_lines(b)[:-1]) + "G1 X1\n", [(3, B.E_UNTERMINATED)])
    c["nested_begin"] = (_join(_lines(b)[:-1]) + open_blk, [(nb, B.E_NESTED)])
    c["nan"] = (_edit(b, 0, lambda l: l.replace("z=2.1", "z=nan")), [(1, B.E_BAD_NUMBER)])
    c["inf_in_xy"] = (_edit(b, 3, lambda l: l.replace("1.0", "inf", 1)), [(4, B.E_BAD_NUMBER)])
    c["non_canonical_real"] = (_edit(b, 0, lambda l: l.replace("h=0.7", "h=0.70")), [(1, B.E_BAD_NUMBER)])
    c["integer_real"] = (_edit(b, 0, lambda l: l.replace("F=3000.0", "F=3000")), [(1, B.E_BAD_NUMBER)])
    c["key_order"] = (_edit(b, 0, lambda l: l.replace("z=2.1 h=0.7", "h=0.7 z=2.1")), [(1, B.E_KEY_ORDER)])
    c["unknown_key"] = (_edit(b, 1, lambda l: l.replace("ca=", "cx=")), [(2, B.E_UNKNOWN_KEY)])
    c["missing_key"] = (_edit(b, 1, lambda l: l.replace(" nc=2", "")), [(2, B.E_MISSING_KEY)])
    c["extra_key"] = (_edit(b, 1, lambda l: l + " nc=2"), [(2, B.E_KEY_ORDER)])
    c["unknown_tag"] = (_edit(b, 1, lambda l: l.replace("STAT", "STATS")), [(2, B.E_UNKNOWN_TAG)])
    c["duplicate_k"] = (b + G + b, [(nb + 3, B.E_DUP_K)])
    c["version_2"] = (_edit(b, 0, lambda l: l.replace("v=1", "v=2")) + open_blk, [(1, B.E_VERSION)])
    c["trailing_space"] = (_edit(b, 2, lambda l: l + " "), [(3, B.E_BAD_LINE)])
    c["double_space"] = (_edit(b, 2, lambda l: l.replace(" ", "  ", 1)), [(3, B.E_BAD_LINE)])
    c["tab"] = (_edit(b, 2, lambda l: l.replace(" ", "\t", 1)), [(3, B.E_BAD_LINE)])
    c["layer_in_ty"] = (_resign(_edit(b, fb_line, lambda l: l.replace("ty=sparse_infill", "ty=layer"))),
                        [(fb_line + 1, B.E_BAD_LINE)])
    c["ty_escaped_layer_ok"] = (_resign(_edit(b, fb_line, lambda l: l.replace("ty=sparse_infill", "ty=la%79er"))), [])
    c["ty_non_canonical"] = (_resign(_edit(b, fb_line, lambda l: l.replace("ty=sparse_infill", "ty=sp%61rse"))),
                             [(fb_line + 1, B.E_BAD_NUMBER)])
    c["bracket"] = (_edit(b, 0, lambda l: l + " (x)"), [(1, B.E_BAD_LINE)])
    c["end_without_begin"] = (G + ";DIW:END n=0 crc=00000000\n" + b, [(3, B.E_END_NO_BEGIN)])
    c["stat_outside"] = (";DIW:STAT ca=0.0 oa=0.0 ra=0.0 hl=0.0 nc=0\n" + b, [(1, B.E_BAD_LINE)])
    c["short_xy"] = (_resign(_join([l for i, l in enumerate(_lines(b)) if i != 4])), [(5, B.E_COUNT)])
    c["xy_overshoot"] = (_resign(_edit(b, 4, lambda l: l + " 1.0 2.0")), [(5, B.E_COUNT)])
    c["nine_pairs"] = (_resign(_edit(b, 3, lambda l: l + " 9.0 9.0")), [(4, B.E_BAD_LINE)])
    c["odd_xy"] = (_resign(_edit(b, 4, lambda l: l + " 1.0")), [(5, B.E_BAD_LINE)])
    c["end_n_wrong"] = (_edit(b, nb - 1, lambda l: re.sub(r"n=\d+", "n=99", l)), [(nb, B.E_COUNT)])
    c["run_n_1"] = (_resign(_join([";DIW:RUN t=line n=1" if l.startswith(";DIW:RUN t=line") else l
                                   for l in _lines(b)])), [(3, B.E_COUNT)])
    c["run_point_with_n"] = (_resign(_edit(b, 5, lambda l: l.replace("d=1.25", "n=1"))), [(6, B.E_KEY_ORDER)])
    c["k_zero"] = (_edit(b, 0, lambda l: l.replace("k=3", "k=0")), [(1, B.E_BAD_NUMBER)])
    c["k_leading_zero"] = (_edit(b, 0, lambda l: l.replace("k=3", "k=03")), [(1, B.E_BAD_NUMBER)])
    c["open_with_stat"] = (_resign(_edit(open_blk, 1, lambda l: l.replace("ca=0.0", "ca=1.0"))), [(2, B.E_BAD_NUMBER)])
    c["open_with_run"] = (_resign(_join(_lines(open_blk)[:2] + [";DIW:RUN t=point d=1.0", ";DIW:XY 1.0 1.0"] +
                                        _lines(open_blk)[2:])), [(3, B.E_BAD_LINE)])
    c["ok_without_runs"] = (_resign(_join([l.replace("st=small", "st=ok") for l in _lines(small_blk)])), [(3, B.E_COUNT)])
    c["run_after_fb"] = (_resign(_join(_lines(b)[:fb_line] + _lines(b)[fb_line:-1] +
                                       [";DIW:RUN t=point d=1.0", ";DIW:XY 1.0 1.0"] + _lines(b)[-1:])),
                         [(nb, B.E_BAD_LINE)])
    c["gcode_inside_block"] = (_join(_lines(b)[:3] + ["G1 X1"] + _lines(b)[3:]), [(1, B.E_UNTERMINATED)])
    c["orphan_after_unterminated"] = (_join(_lines(b)[:3] + ["G1 X1"]) + ";DIW:XY 1.0 1.0\n" + G + open_blk + b,
                                      [(1, B.E_UNTERMINATED)])
    c["error_then_skip_to_next"] = (_edit(b, 2, lambda l: l + " ") + open_blk, [(3, B.E_BAD_LINE)])
    c["neg_zero_and_exp"] = (_resign(_edit(b, 3, lambda l: l.replace("0.0 0.0", "-0.0 1e-05", 1))), [])
    c["upper_hex"] = (_edit(b, nb - 1, lambda l: l.upper().replace(";DIW:END N=", ";DIW:END n=").replace("CRC=", "crc=")),
                      None)
    c["non_ascii"] = (b.replace("STAT", "STATé", 1), [(2, B.E_BAD_LINE)])
    c["empty"] = ("", [])
    c["xy_after_begin"] = (_resign(_join(_lines(b)[:1] + [";DIW:XY 1.0 1.0"] + _lines(b)[1:])), [(2, B.E_BAD_LINE)])
    c["end_after_begin"] = (_resign(_join(_lines(b)[:1] + [";DIW:END n=0 crc=00000000"])), [(2, B.E_COUNT)])
    # a lone \r ends no line: inside a record it is a bad byte; before ";DIW:" it hides the record
    c["lone_cr_in_record"] = (_edit(b, 3, lambda l: l + "\rG1 X5 Y5 E1"), [(4, B.E_BAD_LINE)])
    c["lone_cr_before_begin"] = (G + "G1 X1\r" + b, [(4, B.E_BAD_LINE)])
    c["orphans"] =(";DIW:XY 1.0 1.0\n;DIW:DE 1.0 1.0\nG1 X1\n;DIW:RUN t=point d=1.0\n" + b, [(1, B.E_BAD_LINE)])
    return c


# ----------------------------------------------------------------------------- generated geometry

def make_notch(cube):
    """cube25's header and config around 6 generated layers whose cavity has a narrow arm (paste fallback
    paths, travel feed both set and None) and whose layer 3 has an open wall loop (st=open).
    Outer wall only, absolute E, 0.7 mm layers; layer 6 (final) is never converted."""
    lines = cube.decode("utf-8").split("\n")
    first = next(i for i, l in enumerate(lines) if l.startswith(";LAYER_CHANGE"))
    stop = max(i for i, l in enumerate(lines) if l.startswith("; stop printing object"))
    loop = [(60, 60), (80, 60), (80, 69), (92, 69), (92, 71), (80, 71), (80, 80), (60, 80), (60, 60)]
    out = lines[:first]
    for n in range(1, 7):
        z = round(0.7 * n, 1)
        e = 0.0
        out += [";LAYER_CHANGE", f";Z:{z}", ";HEIGHT:0.7", "G92 E0", f"G1 Z{z} F600", ";TYPE:Outer wall",
                ";WIDTH:1.4", "G1 F1200"]

        def ext(a, b):
            nonlocal e
            e += 0.1 * ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
            return f"G1 X{b[0]} Y{b[1]} E{e:.5f}"
        for a, b in zip(loop, loop[1:]):
            if n == 3 and a == (92, 71):
                out.append(f"G1 X{b[0]} Y{b[1]} F9000")       # open loop: one edge is a travel
            else:
                out.append(ext(a, b))
        out += [";TYPE:Sparse infill", ";WIDTH:1.4"]
        if n % 2:
            out.append("G1 X62 Y70 F7200")
            out += ["G1 F1500", ext((62, 70), (91, 70))]
        else:                                                  # no XY travel in the layer: travel_feed None
            out += ["G1 F1500", ext((60, 60), (62, 70)), ext((62, 70), (91, 70))]
        out += ["G1 X62 Y62 F7200", ext((62, 62), (78, 78)), "G1 X60 Y60 F9000"]
    out += lines[stop:]
    return "\n".join(out).encode("utf-8")


# ----------------------------------------------------------------------------- C++ calls

class Cpp:
    FLAGS = ("--dump-records", "--rewrite-records", "--fingerprint")

    def __init__(self, exe):
        """A missing or unreadable exe is an error (self.error set, every call fails), never "flag absent"."""
        self.exe = os.path.abspath(exe)
        self.error = None
        try:
            with open(self.exe, "rb") as fh:
                blob = fh.read()
        except OSError as exc:
            self.error = f"cannot read {self.exe}: {exc}"
            blob = b""
        self.has = {f: f.encode("ascii") in blob for f in self.FLAGS}

    def _run(self, argv, out):
        r = subprocess.run([self.exe] + argv, capture_output=True, cwd=os.path.dirname(self.exe))
        with open(out, "wb") as fh:
            fh.write(r.stdout)
        if r.returncode != 0:
            return "FAIL", f"exit {r.returncode}: {r.stderr.decode('utf-8', 'replace').strip()}"
        return "ok", r.stdout

    def call(self, flag, args, out):
        """Runs `diw_translate [--config-from-gcode] <flag> <args...>` (result on stdout, section 7) and keeps a
        copy of stdout in <out>. Returns (status, bytes or detail): 'ok' or 'FAIL' (also for a missing exe or flag)."""
        if self.error:
            return "FAIL", self.error
        if not self.has[flag]:
            return "FAIL", f"diw_translate.exe has no {flag}"
        pre = [a for a in args if a == "--config-from-gcode"]
        rest = [a for a in args if a != "--config-from-gcode"]
        return self._run(pre + [flag] + rest, out)

    def translate(self, src, dst, flags, overrides, log):
        """Runs `diw_translate <flags> <src> <dst> <overrides>`; returns ('ok', output bytes) or ('FAIL', detail)."""
        if self.error:
            return "FAIL", self.error
        s, t = self._run(flags + [src, dst] + overrides, log)
        if s != "ok":
            return s, t
        with open(dst, "rb") as fh:
            return "ok", fh.read()


def harness_selftest(work):
    """Regression for the harness itself: a mistyped exe path must fail, never skip."""
    bad = Cpp(os.path.join(work, "no_such_dir", "diw_translat.exe"))
    s, t = bad.call("--dump-records", [os.path.join(work, "x")], os.path.join(work, "harness.missing_exe.out"))
    return [] if (bad.error and s == "FAIL") else [f"missing exe gave {s!r} {t!r}"]


def first_diff(a: bytes, b: bytes):
    la, lb = a.split(b"\n"), b.split(b"\n")
    i = next((k for k in range(min(len(la), len(lb))) if la[k] != lb[k]), min(len(la), len(lb)))
    return (f"first difference at line {i + 1}\n       py : {la[i] if i < len(la) else '<end>'!r}\n"
            f"       c++: {lb[i] if i < len(lb) else '<end>'!r}")


# ----------------------------------------------------------------------------- main

def selftest():
    bad = []
    if B.crc32(b"123456789") != 0xCBF43926:
        bad.append("crc32")
    if B.fnv1a32(b"") != 0x811C9DC5 or B.fnv1a32(b"a") != 0xE40C292C or B.fnv1a32(b"foobar") != 0xBF9CF968:
        bad.append("fnv1a32")
    for s in ("internal solid infill", "sparse infill", "a_b%C", "first layer", "LAYER x", "", "layerlayer"):
        e = B.encode_ty(s)
        if B.decode_ty(e) != s or "layer" in e.lower() or any(ch in e for ch in " []()"):
            bad.append(f"ty {s!r} -> {e!r}")
    if B.encode_ty("internal solid infill") != "internal_solid_infill":
        bad.append("ty example")
    # lines split at \n only: a lone \r neither ends a record line nor starts one (sections 1 and 4)
    if B.strip_records(LONE_CR) != b"G1 X1\r;DIW:BEGIN v=1 k=1\n":
        bad.append(f"strip lone CR -> {B.strip_records(LONE_CR)!r}")
    return bad


def main(argv):
    allow_drift = "--allow-version-drift" in argv
    argv = [a for a in argv if a != "--allow-version-drift"]
    if len(argv) < 5:
        print(__doc__)
        return 2
    engine_dir, exe, work = argv[1], argv[2], os.path.abspath(argv[3])
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
    cpp = Cpp(exe)
    if cpp.error:
        print(cpp.error)
        return 2
    print(f"diw_translate flags present: {cpp.has}")
    report = {"versions": versions, "cpp_flags": cpp.has, "cases": {}}
    counts = {"ok": 0, "FAIL": 0, "SKIP": 0}

    def record(name, status, detail):
        status = "SKIP" if status is None else status
        counts[status] += 1
        report["cases"][name] = {"status": status, "detail": detail if isinstance(detail, str) else ""}
        print(f"{status:4} {name}: {detail if isinstance(detail, str) else ''}")

    st = selftest()
    record("selftest", "ok" if not st else "FAIL", "crc32 / fnv1a32 / ty encoding / strip" + (f": {st}" if st else ""))
    st = harness_selftest(work)
    record("harness.missing_exe", "ok" if not st else "FAIL", "a missing exe fails" + (f": {st}" if st else ""))

    inputs = []
    for path in corpus:
        with open(path, "rb") as fh:
            inputs.append((os.path.splitext(os.path.basename(path))[0], fh.read(), VARIANTS))
    if inputs:
        inputs.append(("notch", make_notch(inputs[0][1]), NOTCH_VARIANTS))
    for ci, (cname, data, variants) in enumerate(inputs):
        base_name = next(iter(variants))          # default / notch: what the other variants are judged against
        base_layers, base_out = None, None
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
            P = E.UMexParams.from_config(cfg)[0]
            fp = B.fingerprint(P, E)

            plain = os.path.join(work, f"{name}.v2.gcode")
            E.DIWPostProcessor(cfg).process_file(src, plain)
            cap_out = os.path.join(work, f"{name}.v2cap.gcode")
            rec_gcode, layers, _ = H.capture(E, cfg, src, cap_out)
            rec_path = os.path.join(work, f"{name}.rec.gcode")
            with open(rec_path, "wb") as fh:
                fh.write(rec_gcode)
            plain_b, cap_b = open(plain, "rb").read(), open(cap_out, "rb").read()
            sts = {s: sum(1 for l in layers if l.st == s) for s in ("ok", "open", "small")}
            n_line = sum(1 for l in layers if l.plan for r in l.plan.runs if r.kind == "line" and l.st == "ok")
            n_pt = sum(1 for l in layers if l.plan for r in l.plan.runs if r.kind == "point" and l.st == "ok")
            n_fb = sum(len(l.plan.fallback_paths) for l in layers if l.st == "ok")
            stats = f"{len(layers)} blocks {sts}, runs line/point {n_line}/{n_pt}, fb {n_fb}, fp {fp:08x}"
            record(f"{name}.capture", "ok" if cap_b == plain_b else "FAIL",
                   stats if cap_b == plain_b else "capture changed v2's output: " + first_diff(plain_b, cap_b))
            if vname == base_name:
                base_layers, base_out = layers, plain_b
            if vname in EXPECT:
                record(f"{name}.exercises", "ok" if EXPECT[vname](layers, base_layers, P, plain_b, base_out) else "FAIL",
                       stats)
                if vname != base_name:
                    ctl = EXPECT[vname](base_layers, base_layers, P, base_out, base_out)
                    record(f"{name}.exercises_control", "FAIL" if ctl else "ok",
                           f"predicate on the {base_name} plans is " + str(ctl))

            record(f"{name}.strip", "ok" if B.strip_records(rec_gcode) == data else "FAIL", "")

            blocks, errors = B.parse_records(rec_gcode)
            py_blocks = "".join(l.block for l in layers)
            rewritten = "".join(B.write_rec(b) for b in blocks)
            py_dump = B.dump(blocks, errors)
            ok = (not errors and len(blocks) == len(layers) and rewritten == py_blocks and
                  [b.k for b in blocks] == [l.k for l in layers])
            record(f"{name}.roundtrip", "ok" if ok else "FAIL",
                   f"{len(blocks)} blocks, {len(errors)} errors, {len(py_blocks)} bytes" + ("" if ok else f" {errors[:5]}"))
            with open(os.path.join(work, f"{name}.py.dump"), "w", encoding="ascii", newline="") as fh:
                fh.write(py_dump)

            rep_out = os.path.join(work, f"{name}.v2replay.gcode")
            _, problems = H.replay(E, cfg, src, rec_gcode, rep_out)
            rep_b = open(rep_out, "rb").read()
            ok = rep_b == plain_b and not problems
            record(f"{name}.replay", "ok" if ok else "FAIL",
                   f"{len(plain_b.splitlines())} lines identical" if ok else
                   (f"{problems[:5]} " + ("" if rep_b == plain_b else first_diff(plain_b, rep_b))))

            if vname in ("default", "notch") and blocks and any(b.runs for b in blocks):
                # negative control: a moved record point must change v2's replayed output
                bk = next(b for b in blocks if b.runs)
                x, y = bk.runs[0].pts[0]
                bk.runs[0].pts[0] = (x + 0.5, y)
                neg = "".join(B.write_rec(b) for b in blocks)
                _, pr = H.replay(E, cfg, src, neg, rep_out + ".neg")
                differs = open(rep_out + ".neg", "rb").read() != plain_b
                record(f"{name}.replay_sensitive", "ok" if differs and not pr else "FAIL",
                       "moved point changes the output" if differs else f"output unchanged {pr[:3]}")

            out = os.path.join(work, f"{name}.cpp.dump")
            s, t = cpp.call("--dump-records", [rec_path], out)
            if s == "ok":
                s, t = ("ok", "identical") if t == py_dump.encode("ascii") else ("FAIL", first_diff(py_dump.encode("ascii"), t))
            record(f"{name}.cpp_dump", s, t)
            out = os.path.join(work, f"{name}.cpp.blocks")
            s, t = cpp.call("--rewrite-records", [rec_path], out)
            if s == "ok":
                s, t = ("ok", "identical") if t == py_blocks.encode("ascii") else ("FAIL", first_diff(py_blocks.encode("ascii"), t))
            record(f"{name}.cpp_rewrite", s, t)
            out = os.path.join(work, f"{name}.cpp.fp")
            s, t = cpp.call("--fingerprint", ["--config-from-gcode", src, "diw_powder_infill=1"] + ov, out)
            if s == "ok":
                want = f"{B.fingerprint_string(P, E)}\n{fp:08x}\n".encode("ascii")
                s, t = ("ok", f"{fp:08x}") if t == want else ("FAIL", first_diff(want, t))
            record(f"{name}.cpp_fingerprint", s, t)

            if ci == 0 and vname == base_name:
                # section 4: translate(F) == translate(strip_records(F)), with lone-CR lines injected (paste mode)
                cut = rec_gcode.find(b";LAYER_CHANGE", rec_gcode.find(b";LAYER_CHANGE") + 1)
                f_in = rec_gcode[:cut] + LONE_CR + rec_gcode[cut:]
                f_strip = B.strip_records(f_in)
                # Powder mode reads the records since increment 3a (they drive the core layers), so there:
                # - the first lone-CR piece is a record line holding a \r: the reader's error stops the export;
                # - the second piece alone (";DIW:" after a lone \r, not a record) is kept as a comment, and the
                #   blocks after it are still found in the right layers (positions count the lone \r as a line end).
                errs = B.parse_records(f_in)[1]
                p_in = os.path.join(work, f"{name}.lonecr.powder.error.gcode")
                with open(p_in, "wb") as fh:
                    fh.write(f_in)
                s, t = cpp.translate(p_in, p_in.replace(".gcode", ".out.gcode"),
                                     ["--config-from-gcode", "--stub-powder-stage"], ["diw_powder_infill=1"] + ov,
                                     p_in.replace(".gcode", ".log"))
                want = [f"record at line {errs[0][0]} ", f"({errs[0][1]})"] if errs else ["<no reader error>"]
                if s == "FAIL" and t.startswith("exit 3:") and all(w in t for w in want):
                    s, t = "ok", t
                else:
                    s, t = "FAIL", f"want exit 3 with {want}, got {s} {t if isinstance(t, str) else '(output)'}"
                record(f"{name}.cpp_translate_record_error.powder", s, t)
                second = LONE_CR[LONE_CR.index(b"\n") + 1:]
                f_in2 = rec_gcode[:cut] + second + rec_gcode[cut:]
                p_in = os.path.join(work, f"{name}.lonecr.powder.kept.gcode")
                with open(p_in, "wb") as fh:
                    fh.write(f_in2)
                s, t = cpp.translate(p_in, p_in.replace(".gcode", ".out.gcode"),
                                     ["--config-from-gcode", "--stub-powder-stage"], ["diw_powder_infill=1"] + ov,
                                     p_in.replace(".gcode", ".log"))
                if s == "ok":
                    n_s1 = t.count(b" S1 paste contour ")
                    ok = (t.count(b";DIW:") == 1 and b"\n;DIW:BEGIN v=1 k=1\n" in t and
                          n_s1 == sum(1 for l in layers if l.st == "ok"))
                    s, t = ("ok" if ok else "FAIL"), f"lone-CR ;DIW: kept as a comment, {n_s1} core layer(s) sequenced"
                record(f"{name}.cpp_translate_lonecr.powder", s, t)
                for mode, fl, mo in (("paste", [], ["diw_powder_infill=0"]),):
                    outs = []
                    for tag, body in (("with", f_in), ("stripped", f_strip)):
                        p_in = os.path.join(work, f"{name}.lonecr.{mode}.{tag}.gcode")
                        with open(p_in, "wb") as fh:
                            fh.write(body)
                        outs.append(cpp.translate(p_in, p_in.replace(".gcode", ".out.gcode"),
                                                  ["--config-from-gcode"] + fl, mo + ov,
                                                  p_in.replace(".gcode", ".log")))
                    (s1, a), (s2, b) = outs
                    if s1 != "ok" or s2 != "ok":
                        s, t = "FAIL", a if s1 != "ok" else b
                    elif a != b:
                        s, t = "FAIL", "translate(F) != translate(strip_records(F)): " + first_diff(b, a)
                    elif b"G1 X5 Y5" in a or b";DIW:BEGIN" not in a:
                        s, t = "FAIL", "the lone-CR lines were not handled as POWDER_RECORDS.md section 4 says"
                    else:
                        s, t = "ok", f"identical, {len(a)} bytes"
                    record(f"{name}.cpp_translate_strip.{mode}", s, t)

    for sname, (text, expected) in synthetic_cases().items():
        raw = text.encode("utf-8") if isinstance(text, str) else text
        path = os.path.join(work, f"synth.{sname}.gcode")
        with open(path, "wb") as fh:
            fh.write(raw)
        blocks, errors = B.parse_records(raw)
        py_dump = B.dump(blocks, errors)
        with open(path.replace(".gcode", ".py.dump"), "w", encoding="ascii", newline="") as fh:
            fh.write(py_dump)
        if expected is not None:
            ok = errors == expected
            record(f"synth.{sname}.python", "ok" if ok else "FAIL",
                   f"{len(blocks)} blocks, errors {errors}" + ("" if ok else f" (expected {expected})"))
        out = path.replace(".gcode", ".cpp.dump")
        s, t = cpp.call("--dump-records", [path], out)
        if s == "ok":
            s, t = ("ok", "identical") if t == py_dump.encode("ascii") else ("FAIL", first_diff(py_dump.encode("ascii"), t))
        record(f"synth.{sname}.cpp_dump", s, t)

    with open(os.path.join(work, "compare_records_report.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"\n{counts['ok']} ok, {counts['FAIL']} failure(s), {counts['SKIP']} skipped")
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
