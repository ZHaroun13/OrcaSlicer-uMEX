"""Increment-5 product-path gates (diw/PRODUCT_DESIGN.md section 8).

usage:
  py -3.15 -E -s product_gate.py g9 <v2 engine dir> <diw_translate.exe> <work dir> <set>=<slices dir> [<set>=<slices dir> ...]
         [--only SUBSTR] [--jobs N] [--coverage-sets cov]

  g9   offline product replay (stage 2b): for every slice directory (holding the increment-4 shadow-mode
       keep/plate_1.gcode with its ;DIWP placeholders and dump/k<k>.json):
         S0   diw_translate --plan-records <keep> (no --product, ORCA_DIW_POWDER_MODE unset, placeholders kept, dump
              reloaded) reproduces the keep file byte for byte (increment-4 T2(d));
         ref  diw_translate --plan-records <keep> with ORCA_DIW_POWDER_MODE=product in the environment is still shadow
              mode (summary "DIW shadow planner:") and gives the keep file without its placeholders;
         G9a  diw_translate --plan-records --product <keep> succeeds, summary fallback=0;
         G9b  its ;DIW: lines equal the keep file's byte for byte;
         G9c  translate(product replay) == translate(ref) byte for byte (or both abort with the same message);
         G3   the structural check of the edit (structural_check below) on (ref, product replay);
         dump the product dumps say suppressed=true exactly for the edited chunks, with chunk_sha_pre;
       and over the coverage sets (default: the set named "cov"): fb_lines > 0, small_blocks > 0,
       edited_chunks_with_fb >= 2 on >= 2 distinct slices, small_chunks_unedited == small_blocks.

  py -3.15 -E -s product_gate.py g5 <v2 engine dir> <diw_translate.exe> <work dir> --orca <orca-slicer.exe>
         --datadir <orca datadir> --t5 <increment-4 fx_t5\t5 dir> [--jobs N]

  g5   product refusals and safety nets (stage 3, PRODUCT_DESIGN.md 8 G5), CLI slices of the 25 mm cube with the flat
       DIW powder profiles (product mode unless named shadow):
         every refusal fails with its exact message (the failure list of 5.1 or "u-MEX powder infill cannot be used:
         <reason>"), rc != 0, no plate_1.gcode and no .tmp file left: G91 in the layer change G-code (arc_g91, every
         core layer); an extrusion outside the part in the layer change G-code typed ;TYPE:Sparse infill (orphan,
         every core layer) and without a ;TYPE: (orphan on the core layers that follow a layer ending in sparse
         infill, which it inherits); with ORCA_DIW_TEST_SKIP_VALIDATE=1: density 0 (no_core), wall_loops 0 (open),
         spiral vase, belt printer (refused as "G-code axis remap": the fork's axis remap is on exactly for belt
         printers), differing extruder offsets; faults capture / plan / binder (internal error lines); fault cancel
         in both modes (cancel message, no failure list);
         fault suppress (product): a successful export with suppressed=0, fallback = st=ok blocks, the fallback
         logged, the export equal to the shadow export (norm) and its strict translation equal to the shadow export's
         --v2-parity translation (norm + the frozen G1 mask);
         shadow-mode faults capture / plan / binder as increment-4 T6 (rc 0, warning, no ;DIWP, strip == kill);
         translator: the kill-switch export is refused with the no-plan message (--allow-unconverted-powder passes);
         v2's records on the T5 cube25 seam-gap 15 % text (st=open) are refused at the first open layer by default
         and equal v2's own output with --v2-parity; the coverage st=small export (cone, clearance 3.0) and an h <= 0
         carrier (the binder's --plan-records replay of the kill export with one core layer's ;HEIGHT: set to -0.7)
         translate with the strict default and carry v2's warnings.
       Also a self-test of the frozen G1 mask (g1_mask).

  py -3.15 -E -s product_gate.py g1|g10 <v2 engine dir> <diw_translate.exe> <work dir> matrix=<S0 work> stress=<S0 work>
         t5=<S0 work> cov=<S0 work> --orca <orca-slicer.exe> --datadir <orca datadir> [--only SUBSTR] [--jobs N]
         [--reuse-product] [--g10 set/slice,...]

  The S0 work dirs are the stage-4a shadow_export.py --matrix --powder-mode shadow runs (T2 --reuse, --stress, --t5,
  --extra coverage_slices.json). Each slice is exported again in product mode (the slicer default) with the S0
  slice's own profiles, meshes, CLI arguments and test environment.
  g1   G1 (a) the product export succeeds; (b) its ;DIW: lines equal the S0 shadow export's; (c) translate(shadow,
       --v2-parity) and translate(product, strict) both exit 0 and are equal after norm + the frozen mask, or both
       abort with the same message; G3 the structural check (on norm + mask of both texts), fallback = 0, every
       st=ok chunk edited; coverage counters over the cov set. Expected slice counts: 191 / 16 / 16 (8 T5 slices at
       seam gap 0 and 15 %) / 5.
  g10  for each of 4 slices (default 3 matrix + 1 coverage FB) and each ORCA_DIW_TEST_SUPPRESS_MUTATE value:
       (a) verification on: summary suppressed=0 fallback=<st=ok blocks>, every st=ok dump says suppressed=false with a
       v1_* / v2_* fallback, the export equals the shadow export (norm), G1 (b) and (c) pass;
       (b) ORCA_DIW_TEST_SKIP_VERIFY=1: every st=ok chunk released edited, G3 fails, and G1 fails for drop_restore and
       wrong_g92 (asserted); for extra_delete whether the translation changes is reported.

  Stage 4b (same arguments as g1; g8 also --g1-work <G1 work dir>):
  g2   G2 every S0 slice exported with ORCA_DIW_TEST_NO_SUPPRESS=1 (product): identical to the S0 shadow export after
       norm, the disagreement counters (text_only, native_only, native_only_nosparse) 0, nothing suppressed; G8 on it.
  g1d  G1 (d) the 13 classic.base matrix slices, the 16 stress slices and the coverage set exported in shadow and in
       product mode with disable_m73 = 1 (profile copies): no M73 line, records equal, translations equal after norm +
       the statistics-line mask (ii) only (or both abort with the same message); G3 structural check; G8 on the pair.
  g8   G8 on every stage-4a G1 product export: filament used [mm] == shadow - removed_de (summary) within one unit of
       the printed last digit; every estimated time line not larger.
  Stage 5:
  py -3.15 -E -s product_gate.py g4 <v2 engine dir> <diw_translate.exe> <work dir> --orca <orca-slicer.exe>
         --datadir <orca datadir> [--catch2 <diw_planner_tests.exe>] [--jobs N]
  g4   G4 disagreement classes, CLI exports in product and shadow mode: text_only ({if layer_num == 1} extrusion
       outside the part typed ;TYPE:Sparse infill on the internal-solid-only k 2: no block, the product warning,
       shadow disagree_text_only, other blocks byte-equal to the plain plate's); native_only (a) (;TYPE:Top surface
       extrusion on every layer: product blocks byte-equal to the plain plate's, native_only counted, no warning;
       shadow disagree_native_only on every core layer); tall_short (cube25 + cube10, precise_z_height 0, and the
       profile's precise variant) and final_core (top_shell_layers 0): no block on skin / final layers in both modes,
       the final warning, and v2 agreement (text-core k set, planned (k, st), BEGIN lines and RUN / FB counts; the
       T2 tolerance checks of these plates are shadow_export.py --extra g4_slices.json); native_only (b) by the
       Catch2 T12 stream; the profile check (no shipped DIW profile, no untyped in-layer extrusion in the test
       profiles).
  py -3.15 -E -s product_gate.py g7 <work dir> --orca <orca-slicer.exe> --datadir <orca datadir> [--runs 5]
  g7   G7 cube25 and cone: product CLI export time <= 1.2 x the kill-switch time (best of 5, alternating, as T10),
       max_held = 1 in every product export.

Export manifest (4.5): every G-code file this harness writes is recorded in <work dir>/manifest.json with its mode
("shadow" or "product"). oracle_input() refuses a product file for any oracle use (v2, compare_*, a --plan-records
replay); the run checks that refusal once on a product replay. Inputs taken from the increment-4 references are
registered as shadow.

Every check prints one line; the run ends with "N checks, F failures" and exits 1 on a failure. A missing exe, input
or flag is a failure, never a skip.
"""
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

E = None   # v2 engine module (diw_engine), loaded in main()

SUMMARY_SHADOW_RE = re.compile(r"DIW shadow planner: (\d+) block\(s\), (\d+) carrier\(s\)")
SUMMARY_PRODUCT_RE = re.compile(r"DIW powder planner \(product\): (\d+) block\(s\), (\d+) carrier\(s\)([^;\r\n]*);([^\r\n]*)")
R1_RE = re.compile(rb"^G1 X(\S+) Y(\S+)(?: Z(\S+))? F(\S+)\n$")
R2_RE = re.compile(rb"^G1 F(\S+)\n$")
R3_RE = re.compile(rb"^G92 E(\S+)\n$")
R5_RE = re.compile(rb"^G1 F(\S+) ;WIDTH:(\S+)\n$")
R4_RE = re.compile(rb"^G1 F(\S+) ;TYPE:([^\r\n]+)\n$")


# ----------------------------------------------------------------------------------------------- manifest

class Manifest:
    """Mode of every G-code file the harness made or consumed (PRODUCT_DESIGN.md 4.5)."""

    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.files = {}
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                self.files = json.load(f)

    def register(self, path, mode, what=""):
        assert mode in ("shadow", "product")
        with self.lock:
            self.files[os.path.normcase(os.path.abspath(path))] = {"mode": mode, "what": what}

    def mode(self, path):
        with self.lock:
            e = self.files.get(os.path.normcase(os.path.abspath(path)))
        return e["mode"] if e else None

    def oracle_input(self, path):
        """The path, if it may feed v2, a compare_* harness or a --plan-records replay; else ValueError."""
        m = self.mode(path)
        if m is None:
            raise ValueError(f"{path}: not in the export manifest (mode unknown); refusing it as an oracle input")
        if m != "shadow":
            raise ValueError(f"{path}: a product-mode export is not a valid v2 / replay input (PRODUCT_DESIGN.md 4.5)")
        return path

    def save(self):
        with self.lock:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self.files, f, indent=1, sort_keys=True)


# ----------------------------------------------------------------------------------------------- helpers

class Checks:
    def __init__(self, name):
        self.name = name
        self.lines = []
        self.n = 0
        self.fails = []

    def check(self, gate, what, ok, detail=""):
        self.n += 1
        self.lines.append(f"{'ok  ' if ok else 'FAIL'} {gate} {self.name}.{what}{': ' + detail if detail else ''}")
        if not ok:
            self.fails.append(f"{gate} {self.name}.{what}: {detail}")
        return ok


def run_tr(tr, argv, env_extra=None):
    env = {k: v for k, v in os.environ.items() if not k.startswith("ORCA_DIW_")}
    env.update(env_extra or {})
    r = subprocess.run([tr] + argv, capture_output=True, cwd=os.path.dirname(tr), env=env)
    return r.returncode, r.stderr.decode("utf-8", "replace")


def read(p):
    with open(p, "rb") as f:
        return f.read()


def lines_of(data):
    return data.splitlines(keepends=True)


def strip_placeholders(data):
    return b"".join(l for l in lines_of(data) if not l.startswith(b";DIWP "))


def strip_records(data):
    return b"".join(l for l in lines_of(data) if not l.startswith(b";DIW:"))


def record_lines(data):
    return [l for l in lines_of(data) if l.startswith(b";DIW:")]


def blocks_of(data):
    """k -> {"st": ..., "fb": number of ;DIW:FB lines} from the record lines."""
    out, cur = {}, None
    for l in record_lines(data):
        if l.startswith(b";DIW:BEGIN"):
            f = dict(t.split("=", 1) for t in l.decode().split()[1:])
            cur = int(f["k"])
            out[cur] = {"st": f["st"], "fb": 0}
        elif l.startswith(b";DIW:FB") and cur is not None:
            out[cur]["fb"] += 1
        elif l.startswith(b";DIW:END"):
            cur = None
    return out


def first_diff(a, b):
    la, lb = a.splitlines(), b.splitlines()
    for i, (x, y) in enumerate(zip(la, lb)):
        if x != y:
            return f"line {i + 1}: {x[:90]!r} != {y[:90]!r}"
    return f"line count {len(la)} != {len(lb)}"


def parse_product_summary(err):
    m = SUMMARY_PRODUCT_RE.search(err)
    if not m:
        return None
    out = {"blocks": int(m.group(1)), "carriers": int(m.group(2)), "reasons": {}}
    for tok in m.group(3).replace(",", " ").split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
        elif tok:
            out[tok] = True
    for tok in m.group(4).split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


# ----------------------------------------------------------------------------------------------- G3

class BBox:
    """Port of the translator's extruded-bbox rule (v2 DIWPostProcessor.compute_geometric_center, streaming)."""

    def __init__(self):
        self.found = False
        self.rel = False
        self.stopped = False
        self.prev_e = 0.0
        self.tx = self.ty = 0.0
        self.box = [float("inf"), float("-inf"), float("inf"), float("-inf")]

    def state(self):
        return (self.found, self.rel, self.stopped, self.prev_e, self.tx, self.ty)

    def reset_box(self):
        self.box = [float("inf"), float("-inf"), float("inf"), float("-inf")]

    def feed(self, line):
        P = E.DIWPostProcessor
        if self.stopped:
            return
        if "; CONFIG_BLOCK_START" in line:
            self.stopped = True
            return
        s = line.strip()
        if not self.found and (P.RE_LAYER_MARKER.search(s) or ";LAYER:0" in s):
            self.found = True
        if re.search(r'^(?:N\d+\s+)?M83\b', s, re.I):
            self.rel = True
        elif re.search(r'^(?:N\d+\s+)?M82\b', s, re.I):
            self.rel = False
        if re.search(r'^(?:N\d+\s+)?G92\b', s, re.I):
            em = P.RE_EXTRUSION.search(s)
            if em:
                self.prev_e = float(em.group(1))
        if P.RE_MOTION.search(s):
            xm, ym, em = P.RE_COORD_X.search(s), P.RE_COORD_Y.search(s), P.RE_EXTRUSION.search(s)
            lx = float(xm.group(1)) if xm else self.tx
            ly = float(ym.group(1)) if ym else self.ty
            if em:
                ev = float(em.group(1))
                if self.rel:
                    de = ev
                else:
                    de = ev - self.prev_e
                    self.prev_e = ev
                if self.found and de > 1e-7:
                    b = self.box
                    b[0] = min(b[0], self.tx, lx)
                    b[1] = max(b[1], self.tx, lx)
                    b[2] = min(b[2], self.ty, ly)
                    b[3] = max(b[3], self.ty, ly)
            self.tx, self.ty = lx, ly


def scan_state(li, with_type):
    s = (li.x1, li.y1, li.z1, li.e_after, li.rel_after, li.relxyz, li.feed, li.width)
    return s + (li.ftype,) if with_type else s


def removal_set(lines, infos, chunk, width_fallback):
    """v2 parse_core_layer's powder paths: their extruding lines and pre_lines (PRODUCT_DESIGN.md 4.3 step 1)."""
    model = E.parse_core_layer(lines, infos, chunk, width_fallback)
    R = set()
    for p in model.paths:
        if p.kind == "powder":
            R.update(p.pre_lines)
    for i in range(chunk.start, chunk.end):
        li = infos[i]
        if li.is_motion and li.is_extruding_move and E._path_kind(li.ftype) == "powder":
            R.add(i)
    return R


def structural_check(chk, gate, shadow, product, sts, width_fallback):
    """G3: product vs shadow text (records stripped). sts: k -> block status of the shadow text. Returns
    {"edited": set of k whose chunk changed, "ok_chunks": n} for the coverage counters."""
    a = [l.decode("utf-8", "surrogateescape") for l in lines_of(strip_records(shadow))]
    b = [l.decode("utf-8", "surrogateescape") for l in lines_of(strip_records(product))]
    ia, ib = E.scan_gcode(a), E.scan_gcode(b)
    ca, cb = E.split_layers(a, ia), E.split_layers(b, ib)
    res = {"edited": set(), "ok_chunks": 0}
    if not chk.check(gate, "chunks", len(ca) == len(cb) and [c.index for c in ca] == [c.index for c in cb],
                     f"{len(ca)} vs {len(cb)} chunks"):
        return res
    if not ca:
        chk.check(gate, "has_chunks", False, "no layer chunk")
        return res
    ok_outside = a[:ca[0].start] == b[:cb[0].start] and a[ca[-1].end:] == b[cb[-1].end:]
    chk.check(gate, "outside_chunks", ok_outside, "text before the first / after the last chunk identical")
    bad = []
    bba, bbb = BBox(), BBox()
    for l in a[:ca[0].start]:
        bba.feed(l)
    for l in b[:cb[0].start]:
        bbb.feed(l)
    for x, y in zip(ca, cb):
        k = x.index
        A, B = a[x.start:x.end], b[y.start:y.end]
        # the extruded bbox of the chunk (translator rule) and the bbox scan state after it
        bba.reset_box()
        bbb.reset_box()
        for l in A:
            bba.feed(l)
        for l in B:
            bbb.feed(l)
        if bba.box != bbb.box or bba.state() != bbb.state():
            bad.append(f"k={k}: extruded bbox {bba.box} vs {bbb.box}" if bba.box != bbb.box else
                       f"k={k}: bbox scan state after the chunk (found, rel, stopped, prev_e, x, y) {bba.state()} vs {bbb.state()}")
            continue
        # the scanner state at the chunk's end (every later chunk reads it)
        if scan_state(ia[x.end - 1], True) != scan_state(ib[y.end - 1], True):
            bad.append(f"k={k}: end-of-chunk scan state differs")
            continue
        if sts.get(k) != "ok":
            if A != B:
                bad.append(f"k={k}: st={sts.get(k)} chunk changed")
            continue
        res["ok_chunks"] += 1
        if A != B:
            res["edited"].add(k)
        R = removal_set(a, ia, x, width_fallback)
        kept = [i for i in range(x.start, x.end) if i not in R]
        j, groups, cur, err = 0, [], [], None
        for pi in range(y.start, y.end):
            if j < len(kept) and b[pi] == a[kept[j]]:
                if cur:
                    groups.append((cur, j))
                    cur = []
                j += 1
            else:
                cur.append(pi)
        if cur:
            groups.append((cur, j))
        if j != len(kept):
            bad.append(f"k={k}: kept line {kept[j] - x.start} of the chunk ({a[kept[j]].rstrip()!r}) is missing")
            continue
        if b[y.start] != a[x.start]:
            bad.append(f"k={k}: start marker differs")
            continue
        for grp, jn in groups:
            end_group = grp[-1] == y.end - 1
            for gi, pi in enumerate(grp):
                l = b[pi].encode("utf-8", "surrogateescape")
                last_line = end_group and gi == len(grp) - 1
                if not (R1_RE.match(l) or R2_RE.match(l) or R3_RE.match(l) or R5_RE.match(l) or (last_line and R4_RE.match(l))):
                    err = f"k={k}: inserted line {pi - y.start} {b[pi].rstrip()!r} is not a restore line"
                    break
            if err:
                break
            # the values the original text implies at that point: the state before the next kept line (or at the
            # end of the chunk); the modal type is restored only at the end of the chunk (R4)
            ref_i = (kept[jn] - 1) if jn < len(kept) else x.end - 1
            if scan_state(ib[grp[-1]], end_group) != scan_state(ia[ref_i], end_group):
                err = (f"k={k}: state after the restore group at line {grp[0] - y.start} is "
                       f"{scan_state(ib[grp[-1]], end_group)}, the original implies {scan_state(ia[ref_i], end_group)}")
                break
        if err:
            bad.append(err)
            continue
        for pi in range(y.start, y.end):
            li = ib[pi]
            if li.is_motion and li.is_extruding_move and E._path_kind(li.ftype) == "powder":
                err = f"k={k}: powder-kind extruding move left: {b[pi].rstrip()!r}"
                break
        if err:
            bad.append(err)
    chk.check(gate, "structure", not bad, f"{res['ok_chunks']} st=ok chunk(s), {len(res['edited'])} edited" +
              (f"; {len(bad)} problem(s): " + "; ".join(bad[:3]) if bad else ""))
    return res


# ----------------------------------------------------------------------------------------------- G9

def g9_slice(args, manifest, set_name, sdir):
    name = f"{set_name}/{os.path.basename(sdir)}"
    chk = Checks(name)
    info = {"set": set_name, "name": name, "sdir": sdir}
    tr = args.tr
    keep_path = os.path.join(sdir, "keep", "plate_1.gcode")
    dump_dir = os.path.join(sdir, "dump")
    if not chk.check("G9", "inputs", os.path.isfile(keep_path) and os.path.isdir(dump_dir), keep_path):
        return chk, info
    manifest.register(keep_path, "shadow", "increment-4 shadow keep export")
    keep = read(keep_path)
    w = os.path.join(args.work, set_name, os.path.basename(sdir))
    if os.path.isdir(w):
        shutil.rmtree(w)
    os.makedirs(w)

    def dump_copy(tag):
        d = os.path.join(w, tag + "_dump")
        shutil.copytree(dump_dir, d)
        return d

    src = manifest.oracle_input(keep_path)
    # S0: increment-4 T2(d)
    s0 = os.path.join(w, "s0.gcode")
    rc, err = run_tr(tr, ["--config-from-gcode", "--plan-records", src, s0],
                     {"ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS": "1", "ORCA_DIW_SHADOW_DUMP": dump_copy("s0")})
    manifest.register(s0, "shadow", "S0 replay")
    same = rc == 0 and os.path.isfile(s0) and read(s0) == keep
    chk.check("S0", "replay", same, "" if same else (f"rc={rc} {err[-300:]}" if rc else first_diff(read(s0), keep)))
    # reference: shadow replay although ORCA_DIW_POWDER_MODE=product is set
    ref = os.path.join(w, "ref.gcode")
    rc, err = run_tr(tr, ["--config-from-gcode", "--plan-records", src, ref],
                     {"ORCA_DIW_SHADOW_DUMP": dump_copy("ref"), "ORCA_DIW_POWDER_MODE": "product"})
    manifest.register(ref, "shadow", "G9 reference replay")
    chk.check("G9", "ref.rc", rc == 0, f"rc={rc} {err[-300:]}" if rc else "")
    chk.check("G9", "ref.shadow_mode", SUMMARY_SHADOW_RE.search(err) is not None and "powder planner (product)" not in err,
              err.strip().splitlines()[-1][:120] if err.strip() else "no summary")
    refd = read(ref) if os.path.isfile(ref) else b""
    chk.check("G9", "ref.equals_keep", refd == strip_placeholders(keep),
              "" if refd == strip_placeholders(keep) else first_diff(refd, strip_placeholders(keep)))
    # product replay
    prod = os.path.join(w, "product.gcode")
    pdump = dump_copy("product")
    rc, err = run_tr(tr, ["--config-from-gcode", "--plan-records", "--product", src, prod], {"ORCA_DIW_SHADOW_DUMP": pdump})
    manifest.register(prod, "product", "G9 product replay")
    summ = parse_product_summary(err)
    chk.check("G9a", "product.rc", rc == 0 and os.path.isfile(prod), f"rc={rc} {err[-400:]}" if rc else "")
    if not chk.check("G9a", "product.summary", summ is not None, err.strip()[-200:]):
        return chk, info
    nfb = int(summ.get("fallback", "-1"))
    chk.check("G9a", "product.fallback", nfb == 0, f"fallback={nfb}" + ("; " + " | ".join(
        l for l in err.splitlines() if "suppression fallback" in l)[:400] if nfb else ""))
    info["summary"] = summ
    prodd = read(prod) if os.path.isfile(prod) else b""
    # G9b: blocks byte-equal to the keep file's
    same = record_lines(prodd) == record_lines(keep)
    chk.check("G9b", "blocks", same, f"{len(record_lines(keep))} record lines" if same else
              f"{len(record_lines(prodd))} vs {len(record_lines(keep))} record lines")
    # G9c: translate(product) == translate(ref)
    trr, trp = os.path.join(w, "ref.tr.gcode"), os.path.join(w, "product.tr.gcode")
    rc1, e1 = run_tr(tr, ["--config-from-gcode", manifest.oracle_input(ref), trr])
    rc2, e2 = run_tr(tr, ["--config-from-gcode", prod, trp])   # the translator itself, not an oracle
    if rc1 == 0 and rc2 == 0:
        a, b = read(trr), read(trp)
        chk.check("G9c", "translate", a == b, f"{len(a)} bytes" if a == b else first_diff(b, a))
    else:
        m1 = [l for l in e1.splitlines() if l.startswith("error:")]
        m2 = [l for l in e2.splitlines() if l.startswith("error:")]
        chk.check("G9c", "translate", rc1 == rc2 and m1 == m2 and bool(m1),
                  f"both abort rc={rc1}: {m1[:1]}" if rc1 == rc2 and m1 == m2 else f"rc {rc1} vs {rc2}: {m1[:1]} vs {m2[:1]}")
    info["translate_rc"] = rc1
    # G3 structural check
    blocks = blocks_of(keep)
    sts = {k: v["st"] for k, v in blocks.items()}
    wf = width_fallback(keep)
    res = structural_check(chk, "G3", refd, prodd, sts, wf)
    n_ok = sum(1 for v in sts.values() if v == "ok")
    sup = int(summ.get("suppressed", "-1"))
    chk.check("G3", "suppressed", sup == n_ok and len(res["edited"]) == n_ok,
              f"summary suppressed={sup}, st=ok blocks {n_ok}, edited chunks {len(res['edited'])}")
    # pre-edit dump fields
    dumps = {}
    for fn in os.listdir(pdump):
        if fn.startswith("k") and fn.endswith(".json"):
            with open(os.path.join(pdump, fn), encoding="utf-8") as f:
                j = json.load(f)
            dumps[j["k"]] = j
    dbad = [k for k, v in sts.items() if v == "ok" and not (k in dumps and dumps[k].get("suppressed") is True
                                                             and str(dumps[k].get("chunk_sha_pre", "")).startswith("fnv1a64:")
                                                             and dumps[k].get("mode") == "product")]
    dbad += [k for k, v in sts.items() if v != "ok" and k in dumps and dumps[k].get("suppressed") is not False]
    chk.check("G9", "dump_fields", not dbad, f"{len(dumps)} dumps" if not dbad else f"bad k {sorted(dbad)[:10]}")
    # coverage counters of this slice
    info["fb_lines"] = sum(v["fb"] for v in blocks.values())
    info["small_blocks"] = sum(1 for v in blocks.values() if v["st"] == "small")
    info["edited_with_fb"] = sorted(k for k in res["edited"] if blocks.get(k, {}).get("fb", 0) > 0)
    info["small_unedited"] = info["small_blocks"]   # structural_check failed any changed non-ok chunk
    info["small_changed"] = [k for k, v in blocks.items() if v["st"] == "small" and k in res["edited"]]
    return chk, info


def width_fallback(gcode):
    # DIW::UMexParams::width_fallback = the first nozzle_diameter (DIWUMexParams.cpp); it changes no removal set
    m = re.search(rb"^; nozzle_diameter = ([0-9.]+)", gcode, re.M)
    return float(m.group(1)) if m else 1.4


# ----------------------------------------------------------------------------------------------- frozen G1 mask

# PRODUCT_DESIGN.md 8, G1 (c): the list is frozen; it cannot grow without a design-doc change.
# (i) also covers the translator's form of the same line: it comments every M73 out as "; (FILTERED) M73 P<n> R<n>"
# (DIWTranslator.cpp, prohibited commands), so a translated file never holds a bare M73 (PRODUCT_DESIGN.md
# Deviations, stage 4a)
M73_RE = re.compile(rb"^(?:; \(FILTERED\) )?M73 P\d+ R\d+\s*$")
STAT_KEYS = (b"; estimated printing time", b"; total estimated time", b"; model printing time",
             b"; filament used [mm]", b"; filament used [cm3]", b"; filament used [g]", b"; total filament used [g]",
             b"; filament cost", b"; total filament cost")
# the same 9 keys as the translator writes them: it makes every comment bracket-free ([ ] -> ( ), PlanetCNC
# sanitation in Translator::process_line), so "; filament used [mm]" is "; filament used (mm)" in a translated file
# (PRODUCT_DESIGN.md Deviations, stage 4a)
STAT_KEYS = STAT_KEYS + tuple(k.replace(b"[", b"(").replace(b"]", b")") for k in STAT_KEYS if b"[" in k)
LAYER_MARKER_RE = re.compile(rb"^\s*;\s*(layer_change\b|layer\s*:\s*\d)", re.I)


def g1_mask(data):
    """(i) delete every line that is exactly 'M73 P<n> R<n>' (anywhere); (ii) blank the value of the 9 statistics
    comment lines only outside layer chunks (before the first and after the last layer marker). Anything else is kept,
    so a statistics line inside a layer chunk or an M73 of another form still differs."""
    ls = lines_of(data)
    marks = [i for i, l in enumerate(ls) if LAYER_MARKER_RE.match(l)]
    first, last = (marks[0], marks[-1]) if marks else (len(ls), -1)
    out = []
    for i, l in enumerate(ls):
        if M73_RE.match(l.rstrip(b"\r\n")):
            continue
        if i < first or i > last:
            for key in STAT_KEYS:
                if l.startswith(key):
                    l = key + b" <masked>\n"
                    break
        out.append(l)
    return b"".join(out)


def g1_mask_selftest(chk):
    body = b";LAYER_CHANGE\nG1 X1\nM73 P5 R3\nM73 P5\n; filament used [mm] = 9\n;LAYER_CHANGE\nG1 X2\n"
    a = b"; estimated printing time (normal mode) = 1m\nM73 P0 R2\n" + body + b"; filament used [mm] = 1.5\n"
    b = b"; estimated printing time (normal mode) = 2m\nM73 P0 R9\n" + body + b"; filament used [mm] = 2.5\n"
    chk.check("MASK", "equal_after_mask", g1_mask(a) == g1_mask(b), "header / trailer statistics and M73 P R masked")
    chk.check("MASK", "keeps_other_m73", b"M73 P5\n" in g1_mask(a), "an M73 without R is kept")
    fa, fb = (x.replace(b"M73 P0 R", b"; (FILTERED) M73 P0 R") for x in (a, b))
    chk.check("MASK", "filtered_m73", g1_mask(fa) == g1_mask(fb) and b"FILTERED" not in g1_mask(fa),
              "the translator's '; (FILTERED) M73 P<n> R<n>' form is masked too")
    fo = a.replace(b"\nM73 P5\n", b"\n; (FILTERED) M73 P5\n")
    ta, tb = (x.replace(b"[mm]", b"(mm)") for x in (a, b))
    chk.check("MASK", "translated_stat_keys", g1_mask(ta) == g1_mask(tb), "'; filament used (mm)' (translator spelling) masked")
    chk.check("MASK", "keeps_other_filtered", b"; (FILTERED) M73 P5\n" in g1_mask(fo), "a filtered M73 without R is kept")
    inner = body.replace(b"; filament used [mm] = 9", b"; filament used [mm] = 8")
    chk.check("MASK", "inner_stat_differs", g1_mask(a) != g1_mask(a.replace(body, inner)),
              "a statistics line inside a layer chunk is not masked")


# ----------------------------------------------------------------------------------------------- G5

FAIL_PREFIX = "u-MEX powder: the G-code export was stopped because these core layers would be printed without powder:"
REFUSE_PREFIX = "u-MEX powder infill cannot be used: "
NO_PLAN = ("u-MEX powder infill: this G-code has no powder plan (;DIW: records). Export it again from OrcaSlicer with "
           "u-MEX powder infill on; the plan is written during the export.")
CANCEL_MSG = "Background processing has been canceled"
PLANNER_WARN = b"u-MEX native powder planner"
FAIL_LINE_RE = re.compile(r"^(layer\(s\) |internal error \(|no layer of this plate)")
REL_E = {"use_relative_e_distances": "1"}
LC_BASE = ";AFTER_LAYER_CHANGE\\n;[layer_z]"
# custom extrusion outside the part (the 25 mm cube is arranged at the bed centre): a travel to (5, 5), then 1 mm
LC_EXTRUDE = "\\nG92 E0\\nG1 X5 Y5 F3000\\nG1 X6 Y5 E0.3 F600"


def k_list(ks):
    s = ", ".join(str(k) for k in ks[:10])
    if len(ks) > 10:
        s += f", ... ({len(ks) - 10} more)"
    return s


def failure_list(log):
    """The failure-list message the export printed (prefix + its reason lines), or None."""
    t = log.decode("utf-8", "replace").replace("\r\n", "\n")
    i = t.find(FAIL_PREFIX)
    if i < 0:
        return None
    out = [FAIL_PREFIX]
    for l in t[i + len(FAIL_PREFIX):].split("\n")[1:]:
        if not FAIL_LINE_RE.match(l):
            break
        out.append(l)
    return "\n".join(out)


def refusals(log):
    t = log.decode("utf-8", "replace").replace("\r\n", "\n")
    return sorted({l.strip() for l in t.split("\n") if l.strip().startswith(REFUSE_PREFIX)})


def g5_slice(job):
    import shadow_export as S
    sd = os.path.join(job["work"], job["name"])
    os.makedirs(sd, exist_ok=True)
    settings = S.write_profiles(os.path.join(sd, "profiles"), "p", job.get("overrides", {}))
    out = os.path.join(sd, "out")
    rc, data, log, _ = S.slice_part(job["orca"], job["datadir"], [job["mesh"]], out, dict(job.get("env") or {}), settings,
                                    untranslated=job.get("untranslated", True))
    files = sorted(os.listdir(out)) if os.path.isdir(out) else []
    return {"name": job["name"], "rc": rc, "data": data, "log": log, "plate": os.path.join(out, "plate_1.gcode"),
            "files": files, "dir": out}


def g5(args):
    import shadow_export as S
    total = Checks("g5")
    for nm, p in (("exe.orca", args.orca), ("exe.translate", args.tr)):
        total.check("G5", nm, os.path.isfile(p), p)
    total.check("G5", "datadir", os.path.isdir(args.datadir), args.datadir)
    t5src = os.path.join(args.t5, "cube25.classic", "gap15.stripped.gcode")
    total.check("G5", "t5_input", os.path.isfile(t5src), t5src)
    if total.fails:
        return total
    g1_mask_selftest(total)
    os.makedirs(args.work, exist_ok=True)
    manifest = Manifest(os.path.join(args.work, "manifest.json"))
    mdir = S.make_meshes(args.work)
    mcov = S.make_meshes(args.work, 0.85)
    cube = os.path.join(mdir, "cube25.stl")
    base = {"work": args.work, "orca": args.orca, "datadir": args.datadir, "mesh": cube}
    SH = {"ORCA_DIW_POWDER_MODE": "shadow"}
    SKIP = {"ORCA_DIW_TEST_SKIP_VALIDATE": "1"}
    jobs = [
        dict(base, name="plain"),
        dict(base, name="shadow", env=SH),
        dict(base, name="kill", env={"ORCA_DIW_NO_NATIVE_PLAN": "1"}),
        dict(base, name="arc_g91", overrides={"layer_change_gcode": LC_BASE + "\\nG91\\nG1 Z0 F3000\\nG90"}),
        dict(base, name="orphan_typed", overrides=dict(REL_E, layer_change_gcode=LC_BASE + LC_EXTRUDE.replace(
            "\\nG1 X6", "\\n;TYPE:Sparse infill\\nG1 X6"))),
        dict(base, name="orphan_untyped", overrides=dict(REL_E, layer_change_gcode=LC_BASE + LC_EXTRUDE)),
        dict(base, name="skip.density0", overrides={"sparse_infill_density": "0%"}, env=SKIP),
        dict(base, name="skip.wall_loops0", overrides={"wall_loops": "0"}, env=SKIP),
        dict(base, name="skip.spiral", overrides={"spiral_mode": "1"}, env=SKIP),
        dict(base, name="skip.belt", overrides={"belt_printer": "1"}, env=SKIP),
        dict(base, name="skip.extruder_offset", overrides={"extruder_offset": "0x0,1x0"}, env=SKIP),
        dict(base, name="small", mesh=os.path.join(mcov, "cone.stl"), overrides={"diw_shell_powder_clearance": "3.0"}),
    ]
    jobs += [dict(base, name=f"fault.{f}", env={"ORCA_DIW_SHADOW_FAULT": f}) for f in ("capture", "plan", "binder", "cancel", "suppress")]
    jobs += [dict(base, name=f"shadow_fault.{f}", env=dict(SH, ORCA_DIW_SHADOW_FAULT=f)) for f in ("capture", "plan", "binder", "cancel")]
    print(f"G5: {len(jobs)} CLI slices, {args.jobs} workers", flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        res = {r["name"]: r for r in ex.map(g5_slice, jobs)}
    for r in res.values():
        mode = "shadow" if r["name"].startswith("shadow") or r["name"] == "kill" else "product"
        if r["data"] is not None:
            manifest.register(r["plate"], mode, f"G5 {r['name']}")
    manifest.save()

    def ok_export(name):
        r = res[name]
        return total.check("G5", f"{name}.rc", r["rc"] == 0 and r["data"] is not None,
                           f"rc={r['rc']} files={r['files']} {(failure_list(r['log']) or '')[:300]}")

    def refused(name, want, cancel=False):
        """the export failed with exactly `want` (failure list or refusal line), no plate and no .tmp file left"""
        r = res[name]
        tmp = [f for f in r["files"] if f.lower().endswith(".tmp")]
        total.check("G5", f"{name}.no_output", r["rc"] != 0 and "plate_1.gcode" not in r["files"] and not tmp,
                    f"rc={r['rc']} files={r['files']}")
        fl, rf = failure_list(r["log"]), refusals(r["log"])
        txt = r["log"].decode("utf-8", "replace").replace("\r\n", "\n")
        if cancel:
            total.check("G5", f"{name}.message", CANCEL_MSG in txt and fl is None and FAIL_PREFIX[:30] not in txt,
                        f"cancel message {'present' if CANCEL_MSG in txt else 'MISSING'}, failure list {'absent' if fl is None else 'PRESENT'}")
        elif want.startswith(REFUSE_PREFIX):
            total.check("G5", f"{name}.message", rf == [want] and fl is None, f"want {want!r}; got {rf} / list {fl!r}"[:600])
        else:
            total.check("G5", f"{name}.message", fl == want and not rf,
                        "exact" if fl == want else f"want {want!r}\n      got  {fl!r}"[:1200])

    if not ok_export("plain"):
        return total
    plain = res["plain"]["data"]
    blocks = blocks_of(plain)
    ks = sorted(blocks)
    n_ok = sum(1 for b in blocks.values() if b["st"] == "ok")
    total.check("G5", "plain.blocks", bool(ks) and n_ok == len(ks), f"{len(ks)} blocks, {n_ok} st=ok: k {k_list(ks)}")
    ps = parse_product_summary(res["plain"]["log"].decode("utf-8", "replace"))
    total.check("G5", "plain.summary", ps is not None and int(ps.get("suppressed", -1)) == n_ok and ps.get("fallback") == "0",
                f"{ps}")
    # the last ;TYPE: before each block (a custom extrusion at the layer change inherits it), read from the shadow
    # export: in the product export the sparse paths' ;TYPE: lines are removed with them
    last_type, inherit = None, {}
    shadow_ok = res["shadow"]["data"] is not None
    for l in lines_of(res["shadow"]["data"] or b""):
        if l.startswith(b";TYPE:"):
            last_type = l[6:].strip().decode()
        elif l.startswith(b";DIW:BEGIN"):
            inherit[int(dict(t.split("=", 1) for t in l.decode().split()[1:])["k"])] = last_type
    sparse_after = [k for k in ks if inherit.get(k) == "Sparse infill"]
    total.check("G5", "shadow.blocks_equal", shadow_ok and sorted(inherit) == ks, f"shadow blocks k {k_list(sorted(inherit))}")

    # ---- product refusals
    refused("arc_g91", FAIL_PREFIX + "\nlayer(s) " + k_list(ks) + ": arc moves (G2/G3) or relative positioning (G91) in "
            "the layer; arc fitting is off on DIW printers, so check the custom G-code of these layers.")
    orphan_tail = (": extrusions typed sparse infill lie outside the part's outer walls (first at X5.0 Y5.0); sparse infill "
                   "must lie inside the part's outer walls (tolerance: half an outer-wall width). Possible causes: an extrusion "
                   "typed sparse infill outside the part, such as custom G-code without its own ;TYPE: line (for example the "
                   "layer change G-code) or a prime tower, or an island whose outer wall is missing.")
    refused("orphan_typed", FAIL_PREFIX + "\nlayer(s) " + k_list(ks) + orphan_tail)
    total.check("G5", "orphan_untyped.inherit_set", bool(sparse_after), f"blocks after a sparse-typed layer end: {k_list(sparse_after)}")
    refused("orphan_untyped", FAIL_PREFIX + "\nlayer(s) " + k_list(sparse_after) + orphan_tail)
    refused("skip.density0", FAIL_PREFIX + "\nno layer of this plate is a powder core layer (sparse infill without top or "
            "bottom surface). Turn u-MEX powder infill off, or give the part sparse infill.")
    refused("skip.wall_loops0", FAIL_PREFIX + "\nlayer(s) " + k_list(ks) + ": the outer walls do not enclose a cavity.")
    refused("skip.spiral", REFUSE_PREFIX + "spiral vase mode")
    refused("skip.belt", REFUSE_PREFIX + "G-code axis remap")
    refused("skip.extruder_offset", REFUSE_PREFIX + "extruders that have different extruder_offset values")
    for f in ("capture", "plan"):
        refused(f"fault.{f}", FAIL_PREFIX + "\nlayer(s) " + k_list(ks) + f": internal error ({f}: fault injection: {f}). "
                "Please report this with the 3MF.")
    refused("fault.binder", FAIL_PREFIX + "\ninternal error (binder: fault injection: binder). Please report this with the 3MF.")
    refused("fault.cancel", None, cancel=True)
    refused("shadow_fault.cancel", None, cancel=True)

    # ---- suppress fault: a successful, unedited export equal to the shadow one (and after translation)
    if ok_export("shadow") and ok_export("fault.suppress"):
        sh, su = res["shadow"]["data"], res["fault.suppress"]["data"]
        ss = parse_product_summary(res["fault.suppress"]["log"].decode("utf-8", "replace"))
        total.check("G5", "fault.suppress.summary", ss is not None and ss.get("suppressed") == "0" and
                    int(ss.get("fallback", -1)) == n_ok, f"{ss}")
        fl_want = (f"u-MEX powder: sparse infill kept in the G-code text of layer(s) {k_list(ks)} (suppression fallback "
                   f"exception: k={ks[0]}: fault injection: suppress)")
        total.check("G5", "fault.suppress.fallback_log", fl_want.encode() in res["fault.suppress"]["log"],
                    f"logged: {fl_want!r}" if fl_want.encode() in res["fault.suppress"]["log"] else f"MISSING {fl_want!r}")
        total.check("G5", "fault.suppress.no_failure_list", failure_list(res["fault.suppress"]["log"]) is None,
                    "a suppression fallback is never a failure")
        total.check("G5", "fault.suppress.unedited", S.norm(su) == S.norm(sh),
                    "export == shadow export (norm)" if S.norm(su) == S.norm(sh) else first_diff(S.norm(su), S.norm(sh)))
        t_sh = os.path.join(args.work, "shadow", "translated.gcode")
        t_su = os.path.join(args.work, "fault.suppress", "translated.gcode")
        rc1, e1 = run_tr(args.tr, ["--config-from-gcode", "--v2-parity", res["shadow"]["plate"], t_sh])
        rc2, e2 = run_tr(args.tr, ["--config-from-gcode", res["fault.suppress"]["plate"], t_su])
        if total.check("G5", "fault.suppress.translate_rc", rc1 == 0 and rc2 == 0, f"shadow rc={rc1} {e1[-200:]} product rc={rc2} {e2[-200:]}"):
            # norm (the generated-at line) + the frozen G1 mask
            a, b = g1_mask(S.norm(read(t_su))), g1_mask(S.norm(read(t_sh)))
            total.check("G5", "fault.suppress.translate_equal", a == b, f"{len(a)} bytes after norm + the G1 mask" if a == b
                        else first_diff(a, b))
            total.check("G5", "fault.suppress.translate_bytes", S.norm(read(t_su)) == S.norm(read(t_sh)),
                        "byte-identical after norm alone (the export itself is unedited)")

    # ---- shadow-mode faults degrade as in increment 4 (T6)
    if ok_export("kill"):
        kill = res["kill"]["data"]
        for f in ("capture", "plan", "binder"):
            n = f"shadow_fault.{f}"
            if not ok_export(n):
                continue
            r = res[n]
            wl = [l.decode(errors="replace").strip()[:160] for l in r["log"].splitlines() if PLANNER_WARN in l]
            total.check("G5", f"{n}.T6.warning", bool(wl), wl[0] if wl else "no warning")
            total.check("G5", f"{n}.T6.no_placeholder", b";DIWP" not in r["data"], "no ;DIWP line")
            same = S.norm(strip_records(r["data"])) == S.norm(kill)
            total.check("G5", f"{n}.T6.strip_equals_kill", same, "identical" if same else
                        first_diff(S.norm(strip_records(r["data"])), S.norm(kill)))
            total.check("G5", f"{n}.T6.no_failure_list", failure_list(r["log"]) is None, "no failure list in shadow mode")

        # ---- translator: no plan
        out = os.path.join(args.work, "kill", "translated.gcode")
        rc, err = run_tr(args.tr, ["--config-from-gcode", res["kill"]["plate"], out])
        total.check("G5", "translate.no_plan", rc == 3 and err.replace("\r\n", "\n").strip() == "error: " + NO_PLAN,
                    f"rc={rc} {err.strip()[:300]!r}")
        rc, err = run_tr(args.tr, ["--config-from-gcode", "--allow-unconverted-powder", res["kill"]["plate"], out])
        total.check("G5", "translate.no_plan_allowed", rc == 0 and os.path.isfile(out), f"rc={rc} {err.strip()[:200]}")

        # ---- translator: an h <= 0 carrier (made by the binder from a layer whose ;HEIGHT: tag is negative)
        kls = lines_of(kill)
        marks = [i for i, l in enumerate(kls) if LAYER_MARKER_RE.match(l)]
        kh = ks[len(ks) // 2]
        hi = next((i for i in range(marks[kh - 1], marks[kh] if kh < len(marks) else len(kls)) if kls[i].startswith(b";HEIGHT:")), None)
        if total.check("G5", "h0.height_tag", hi is not None, f"layer {kh}"):
            kls[hi] = b";HEIGHT:-0.7\n"
            src = os.path.join(args.work, "h0", "kill_h0.gcode")
            os.makedirs(os.path.dirname(src), exist_ok=True)
            with open(src, "wb") as fh:
                fh.write(b"".join(kls))
            rep = os.path.join(args.work, "h0", "replay.gcode")
            rc, err = run_tr(args.tr, ["--config-from-gcode", "--plan-records", src, rep])
            manifest.register(rep, "shadow", "G5 h<=0 carrier replay")
            rb = blocks_of(read(rep)) if rc == 0 and os.path.isfile(rep) else {}
            beg = [l for l in record_lines(read(rep)) if l.startswith(b";DIW:BEGIN") and f" k={kh} ".encode() in l] if rb else []
            if total.check("G5", "h0.carrier", rc == 0 and rb.get(kh, {}).get("st") == "open" and beg and b" h=-0.7 " in beg[0],
                           f"rc={rc} {beg[0].decode().strip() if beg else err[-200:]}"):
                out = os.path.join(args.work, "h0", "translated.gcode")
                rc, err = run_tr(args.tr, ["--config-from-gcode", rep, out])
                w = f"layer {kh}: non-positive layer height; left as paste.".encode()
                total.check("G5", "translate.h0_strict", rc == 0 and os.path.isfile(out) and w in read(out),
                            f"rc={rc} warning {'present' if rc == 0 and w in read(out) else 'MISSING'} {err.strip()[-200:]}")

    # ---- translator: the coverage st=small export (product) is accepted with v2's warning
    if ok_export("small"):
        sb = blocks_of(res["small"]["data"])
        small = sorted(k for k, b in sb.items() if b["st"] == "small")
        total.check("G5", "small.blocks", bool(small), f"st=small at k {k_list(small)} of {len(sb)} blocks")
        out = os.path.join(args.work, "small", "translated.gcode")
        rc, err = run_tr(args.tr, ["--config-from-gcode", res["small"]["plate"], out])
        tr = read(out) if rc == 0 and os.path.isfile(out) else b""
        m = re.search(rb"; WARNING \(uMEX\): (\d+) core layer\(s\) \(([0-9, -]+)\) have no cavity region of inscribed radius "
                      rb">= shell_powder_clearance; the slicer's paste infill is kept unchanged there\.", tr)
        listed = []
        for part in (m.group(2).decode().split(", ") if m else []):
            a, _, b = part.partition("-")
            listed += list(range(int(a), int(b or a) + 1))
        total.check("G5", "translate.small_strict", rc == 0 and m is not None and int(m.group(1)) == len(small) and listed == small,
                    f"rc={rc} warning {m.group(0).decode()[:140] if m else 'MISSING'} {err.strip()[-200:]}")

    # ---- translator: a v2-made st=open block (T5 cube25 classic, seam gap 15 %)
    import v2_hooks as H
    from compare_powder import parse_config_block, v2_config
    src = os.path.join(args.work, "v2open", "gap15.stripped.gcode")
    os.makedirs(os.path.dirname(src), exist_ok=True)
    shutil.copyfile(t5src, src)
    cfg = v2_config(parse_config_block(read(src)), E)
    cfg["enable_u_mex_powder_infill"] = True
    v2out = os.path.join(args.work, "v2open", "v2.gcode")
    rec, layers, _ = H.capture(E, cfg, src, v2out)
    recp = os.path.join(args.work, "v2open", "v2rec.gcode")
    with open(recp, "wb") as fh:
        fh.write(rec)
    manifest.register(recp, "shadow", "G5 v2 records (st=open)")
    opn = sorted(L.k for L in layers if L.st == "open" and L.h > 0.0)
    if total.check("G5", "v2open.has_open", bool(opn), f"v2 st=open at k {k_list(opn)} of {len(layers)} planned layers"):
        out = os.path.join(args.work, "v2open", "strict.gcode")
        rc, err = run_tr(args.tr, ["--config-from-gcode", recp, out])
        want = (f"error: u-MEX powder records: layer {opn[0]} has no closed wall loop (st=open); this powder plan cannot be "
                "printed. Export the G-code again from OrcaSlicer.")
        total.check("G5", "translate.v2open_strict", rc == 3 and err.replace("\r\n", "\n").strip() == want, f"rc={rc} {err.strip()[:300]!r}")
        out = os.path.join(args.work, "v2open", "parity.gcode")
        rc, err = run_tr(args.tr, ["--config-from-gcode", "--v2-parity", recp, out])
        if total.check("G5", "translate.v2open_parity_rc", rc == 0 and os.path.isfile(out), f"rc={rc} {err.strip()[-200:]}"):
            from compare_powder import norm as pnorm
            a, b = pnorm(out), pnorm(v2out)
            total.check("G5", "translate.v2open_parity_equals_v2", a == b, f"{len(a)} lines equal to v2 after the header line"
                        if a == b else first_diff(b"\n".join(a), b"\n".join(b)))
    manifest.save()
    return total


def main_g5(argv):
    global E
    import argparse
    ap = argparse.ArgumentParser(prog="product_gate.py g5")
    ap.add_argument("engine")
    ap.add_argument("tr")
    ap.add_argument("work")
    ap.add_argument("--orca", required=True)
    ap.add_argument("--datadir", required=True)
    ap.add_argument("--t5", required=True, help="the increment-4 T5 directory (fx_t5\\t5)")
    ap.add_argument("--jobs", type=int, default=6)
    args = ap.parse_args(argv)
    args.tr, args.orca = os.path.abspath(args.tr), os.path.abspath(args.orca)
    if not os.path.isfile(os.path.join(args.engine, "diw_engine.py")):
        print(f"FAIL G5 engine: {args.engine}\n1 checks, 1 failures")
        return 1
    if args.engine not in sys.path:
        sys.path.insert(0, args.engine)
    import diw_engine
    E = diw_engine
    t0 = time.time()
    total = g5(args)
    for l in total.lines:
        print(l)
    for f in total.fails[:40]:
        print("FAILED:", f)
    print(f"G5 in {time.time() - t0:.0f} s")
    print(f"{total.n} checks, {len(total.fails)} failures")
    return 0 if not total.fails else 1


# ----------------------------------------------------------------------------------------------- G4 (stage 5)

# relative E (the custom extrusions reset E with G92 E0, which relative E also requires in the layer change G-code;
# Print::validate wants "G92 E0" on a line of its own, hence the newline before a following {if})
G4_LC = LC_BASE + "\\nG92 E0\\n"
G4_TEXT_ONLY_LAYER_NUM = 1      # layer_num is 0-based: k = layer_num + 1 = 2, the internal-solid-only layer of cube25
# the custom extrusion moves to the new layer's z first: a record's z is the chunk's first extruding move's z (v2
# rule), so an extrusion left at the previous layer's height would change every block's z
G4_OUTSIDE = ";TYPE:{t}\\nG1 X5 Y5 Z[layer_z] F3000\\nG1 X6 Y5 E0.3 F600\\n"
WARN_TEXT_ONLY = ("u-MEX powder: layer(s) {ks} have moves typed sparse infill but no sparse infill of the part (custom "
                  "G-code or a prime tower inherits ;TYPE:); they are printed as paste.")
WARN_FINAL = ("u-MEX powder: the last printed layer (k={k}) has sparse infill without a top surface; the last layer is "
              "never filled with powder, it is printed as paste.")
G4_CATCH2_CASE = "T12 product binder: text_only\\, native_only (a) and (b)"   # Catch2: an escaped comma
SKIN_TYPES = {"Top surface", "Bottom surface"}


def block_texts(data):
    """k -> the bytes of its record block (;DIW:BEGIN .. ;DIW:END lines)."""
    out, cur, buf = {}, None, []
    for l in record_lines(data):
        if l.startswith(b";DIW:BEGIN"):
            cur, buf = int(dict(t.split("=", 1) for t in l.decode().split()[1:])["k"]), [l]
        elif cur is not None:
            buf.append(l)
            if l.startswith(b";DIW:END"):
                out[cur], cur = b"".join(buf), None
    return out


def product_warnings(log):
    """the product-mode warning texts (3.2) in an export log, without the failure list"""
    t = log.decode("utf-8", "replace").replace("\r\n", "\n")
    return sorted({l[l.find("u-MEX powder: "):].strip() for l in t.split("\n")
                   if "u-MEX powder: " in l and FAIL_PREFIX not in l})


def is_layer_change(l):
    return l.lstrip().lower().startswith(b";layer_change")


def layer_types(data):
    """k (1-based ;LAYER_CHANGE count) -> {object label -> set of ;TYPE: names} of a records-free export."""
    out, k, obj = {}, 0, None
    for l in lines_of(data):
        if is_layer_change(l):
            k += 1
        elif l.startswith(b"; printing object "):
            obj = l[len(b"; printing object "):].split(b" id:")[0].decode()
        elif l.startswith(b"; stop printing object"):
            obj = None
        elif l.startswith(b";TYPE:") and k:
            out.setdefault(k, {}).setdefault(obj, set()).add(l[6:].strip().decode())
    return out


def g4_v2(chk, manifest, name, shadow_plate, nat_blocks):
    """v2 agreement on a shadow export (the only valid oracle input, 4.5): v2's own text classification and v2's
    planned layers (k, st) equal the native blocks, and every block has the same ;DIW:BEGIN line (k st z h s F fu
    fp) and the same ;DIW:RUN / ;DIW:FB line counts. The geometry inside the blocks agrees within the increment-4
    T2(c) tolerances, not bit for bit (nm grid vs shapely): that comparison is the T2 run of g4_slices.json."""
    import shadow_export as S
    import v2_hooks as H
    from compare_powder import parse_config_block, v2_config
    try:
        manifest.oracle_input(shadow_plate)
    except ValueError as ex:
        chk.check("G4", f"{name}.v2.oracle_input", False, str(ex))
        return
    data = read(shadow_plate)
    src = os.path.join(os.path.dirname(shadow_plate), "stripped.gcode")
    with open(src, "wb") as fh:
        fh.write(strip_records(data))
    manifest.register(src, "shadow", f"G4 {name} stripped")
    core, _, _ = S.v2_text_core(E, strip_records(data))
    chk.check("G4", f"{name}.v2.text_core", sorted(core) == sorted(nat_blocks),
              f"v2 text core k {k_list(sorted(core))} == native blocks" if sorted(core) == sorted(nat_blocks) else
              f"v2 text core k {k_list(sorted(core))} != native blocks k {k_list(sorted(nat_blocks))}")
    cfg = v2_config(parse_config_block(data), E)
    cfg["enable_u_mex_powder_infill"] = True
    v2out = os.path.join(os.path.dirname(shadow_plate), "v2.gcode")
    _, layers, _ = H.capture(E, cfg, src, v2out)
    v2st = {L.k: L.st for L in layers}
    nst = {k: b["st"] for k, b in blocks_of(data).items()}
    chk.check("G4", f"{name}.v2.planned", v2st == nst, f"{len(v2st)} layers planned by v2, (k, st) == native"
              if v2st == nst else f"v2 {sorted(v2st.items())[:12]} != native {sorted(nst.items())[:12]}")
    texts = block_texts(data)

    def head(b):
        ls = b.split(b"\n")
        return (ls[0].strip(), sum(1 for l in ls if l.startswith(b";DIW:RUN")),
                sum(1 for l in ls if l.startswith(b";DIW:FB")))
    same = [L.k for L in layers if L.k in texts and head(texts[L.k]) == head(L.block.encode())]
    chk.check("G4", f"{name}.v2.block_heads", len(same) == len(layers) == len(texts),
              f"{len(same)} of {len(layers)} v2 blocks have the native block's BEGIN line and RUN / FB counts "
              f"({len(texts)} native)")


EXTRUDING_RE = re.compile(r"^\s*G[0-3](?=[\s;]|$)[^;]*\sE-?[0-9.]", re.I)
# custom G-code printed outside the layer chunks (before the first / after the last layer marker)
OUTSIDE_LAYERS = ("machine_start_gcode", "machine_end_gcode")


def g4_profiles(chk, plain_export):
    import shadow_export as S
    repo = os.path.dirname(os.path.dirname(HERE))
    diw = []
    for root, _, files in os.walk(os.path.join(repo, "resources", "profiles")):
        for f in files:
            if f.endswith(".json"):
                with open(os.path.join(root, f), "rb") as fh:
                    if b'"diw_' in fh.read():
                        diw.append(os.path.relpath(os.path.join(root, f), repo))
    chk.check("G4", "profiles.shipped_diw", not diw, f"resources/profiles files with diw_* keys: {diw[:10]}")
    bad, n = [], 0
    for fn in PROFILE_FILES:
        with open(os.path.join(HERE, "profiles", fn), encoding="utf-8-sig") as fh:
            d = json.load(fh)
        for key, v in d.items():
            if not isinstance(v, str) or not key.endswith("_gcode") or key in OUTSIDE_LAYERS:
                continue
            n += 1
            typed = False
            for l in v.replace("\\n", "\n").split("\n"):
                typed = typed or l.strip().startswith(";TYPE:")
                if EXTRUDING_RE.match(l) and not typed:
                    bad.append(f"{fn}:{key}: {l.strip()}")
    chk.check("G4", "profiles.test_custom_gcode", not bad, f"{n} in-layer custom G-code values of {len(PROFILE_FILES)} "
              f"test profiles, none extrudes without ;TYPE:" if not bad else f"untyped extrusion: {bad[:5]}")
    # the start G-code extrudes (initial load, intro line): Orca prints it under ;TYPE:Custom before the first layer
    ls = lines_of(plain_export)
    first_e = next((i for i, l in enumerate(ls) if EXTRUDING_RE.match(l.decode("utf-8", "replace"))), None)
    first_layer = next((i for i, l in enumerate(ls) if is_layer_change(l)), None)
    custom = next((i for i, l in enumerate(ls) if l.startswith(b";TYPE:Custom")), None)
    chk.check("G4", "profiles.start_gcode", None not in (first_e, first_layer, custom) and custom < first_e < first_layer,
              f";TYPE:Custom at line {custom}, first extruding move {first_e}, first layer marker {first_layer}")


def g4(args):
    import shadow_export as S
    total = Checks("g4")
    for nm, p in (("exe.orca", args.orca), ("exe.translate", args.tr), ("exe.catch2", args.catch2)):
        total.check("G4", nm, os.path.isfile(p), p)
    total.check("G4", "datadir", os.path.isdir(args.datadir), args.datadir)
    if total.fails:
        return total
    os.makedirs(args.work, exist_ok=True)
    manifest = Manifest(os.path.join(args.work, "manifest.json"))
    mdir = S.make_meshes(args.work)
    cube, cube10 = os.path.join(mdir, "cube25.stl"), os.path.join(mdir, "cube10.stl")
    if not total.check("G4", "mesh.cube10", os.path.isfile(cube10), cube10):
        return total
    cases = {
        # name: (meshes, overrides)
        "plain": ([cube], dict(REL_E, layer_change_gcode=G4_LC)),
        "text_only": ([cube], dict(REL_E, layer_change_gcode=G4_LC + "{if layer_num == %d}" % G4_TEXT_ONLY_LAYER_NUM
                                   + G4_OUTSIDE.format(t="Sparse infill") + "{endif}")),
        "native_only_a": ([cube], dict(REL_E, layer_change_gcode=G4_LC + G4_OUTSIDE.format(t="Top surface"))),
        # precise_z_height = 0: both cubes are sliced on the same 0.7 mm grid, so the short cube's top surface and
        # the tall cube's sparse infill share print layers. The test profile's precise_z_height = 1 gives the short
        # cube its own z values near its top (no shared layer there); that plate runs as tall_short_precise.
        "tall_short": ([cube, cube10], {"precise_z_height": "0"}),
        "tall_short_precise": ([cube, cube10], {}),
        "final_core": ([cube], {"top_shell_layers": "0"}),
    }
    jobs = []
    for nm, (meshes, ov) in cases.items():
        for mode in ("product", "shadow"):
            jobs.append({"work": args.work, "name": f"{nm}.{mode}", "orca": args.orca, "datadir": args.datadir,
                         "meshes": meshes, "overrides": ov, "env": {"ORCA_DIW_POWDER_MODE": mode}})

    def run(job):
        sd = os.path.join(job["work"], job["name"])
        os.makedirs(sd, exist_ok=True)
        settings = S.write_profiles(os.path.join(sd, "profiles"), "p", job["overrides"])
        out = os.path.join(sd, "out")
        rc, data, log, _ = S.slice_part(job["orca"], job["datadir"], job["meshes"], out, job["env"], settings)
        return {"name": job["name"], "rc": rc, "data": data, "log": log, "plate": os.path.join(out, "plate_1.gcode")}

    print(f"G4: {len(jobs)} CLI slices, {args.jobs} workers", flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        res = {r["name"]: r for r in ex.map(run, jobs)}
    for r in res.values():
        if r["data"] is not None:
            manifest.register(r["plate"], r["name"].rsplit(".", 1)[1], f"G4 {r['name']}")
    manifest.save()
    ok = [total.check("G4", f"{n}.rc", r["rc"] == 0 and r["data"] is not None,
                      f"rc={r['rc']} {(failure_list(r['log']) or '')[:300]}") for n, r in sorted(res.items())]
    if not all(ok):
        return total
    D = {n: r["data"] for n, r in res.items()}
    L = {n: r["log"] for n, r in res.items()}
    PS = {n: parse_product_summary(L[n].decode("utf-8", "replace")) for n in res if n.endswith(".product")}
    SS = {n: S.parse_summary(L[n]) for n in res if n.endswith(".shadow")}
    for n in sorted(PS):
        n_ok = sum(1 for b in blocks_of(D[n]).values() if b["st"] == "ok")
        total.check("G4", f"{n}.summary", PS[n] is not None and PS[n].get("fallback") == "0" and
                    int(PS[n].get("suppressed", -1)) == n_ok and PS[n].get("orphan") == "0" and
                    failure_list(L[n]) is None, f"{PS[n]}")
    for n in sorted(SS):
        total.check("G4", f"{n}.summary", SS[n] is not None and SS[n]["max_held"] <= 1 and not SS[n]["disabled"],
                    f"{SS[n]}")
    # translations: shadow exports with --v2-parity, product exports with the strict default
    for n, r in sorted(res.items()):
        out = os.path.join(args.work, n, "translated.gcode")
        argv = ["--config-from-gcode"] + (["--v2-parity"] if n.endswith(".shadow") else []) + [r["plate"], out]
        if n == "native_only_a.shadow":
            # shadow mode leaves every disagreeing layer without a block, so this export has no plan at all
            argv.insert(1, "--allow-unconverted-powder")
        rc, err = run_tr(args.tr, argv)
        total.check("G4", f"{n}.translate", rc == 0 and os.path.isfile(out), f"rc={rc} {err.strip()[-200:]}")

    plainB = block_texts(D["plain.product"])
    total.check("G4", "plain.blocks", bool(plainB) and block_texts(D["plain.shadow"]) == plainB and
                not product_warnings(L["plain.product"]),
                f"{len(plainB)} blocks k {k_list(sorted(plainB))}, shadow blocks byte-equal, no product warning")

    # ---- text_only: the internal-solid-only layer k = layer_num + 1 is text core (v2) but not native core
    kt = G4_TEXT_ONLY_LAYER_NUM + 1
    types = layer_types(strip_records(D["text_only.shadow"]))
    tk = sorted(t for o in types.get(kt, {}).values() for t in o)
    total.check("G4", "text_only.layer_types", "Sparse infill" in tk and "Internal solid infill" in tk and
                not (SKIN_TYPES & set(tk)), f"k {kt} types {tk}")
    core, _, _ = S.v2_text_core(E, strip_records(D["text_only.shadow"]))
    total.check("G4", "text_only.v2_text_core", kt in core, f"v2 text core includes k {kt}: {kt in core}")
    for mode in ("product", "shadow"):
        b, pl = block_texts(D[f"text_only.{mode}"]), block_texts(D[f"plain.{mode}"])
        total.check("G4", f"text_only.{mode}.no_block", kt not in b, f"no block at k {kt}")
        total.check("G4", f"text_only.{mode}.blocks_equal_plain", b == pl, f"{len(b)} blocks byte-equal to the plain "
                    f"plate's" if b == pl else f"k {k_list(sorted(b))} vs plain k {k_list(sorted(pl))}")
    pw = product_warnings(L["text_only.product"])
    total.check("G4", "text_only.product.warning", pw == [WARN_TEXT_ONLY.format(ks=kt)], f"{pw}")
    ps = PS["text_only.product"]
    total.check("G4", "text_only.product.counted", ps.get("text_only") == "1" and ps.get("native_only") == "0",
                f"text_only={ps.get('text_only')} native_only={ps.get('native_only')}")
    sr = SS["text_only.shadow"]["reasons"]
    total.check("G4", "text_only.shadow.disagree", sr.get("disagree_text_only") == 1 and not sr.get("disagree_native_only"),
                f"{sr}")
    sw = (f"u-MEX native powder planner: 1 core layer(s) ({kt}) left as paste: the G-code text makes them core layers "
          "but the native classification does not").encode()
    total.check("G4", "text_only.shadow.warning", sw in L["text_only.shadow"], sw.decode()[:120])

    # ---- native_only (a): every layer carries a ;TYPE:Top surface extrusion outside the part
    core, _, _ = S.v2_text_core(E, strip_records(D["native_only_a.shadow"]))
    total.check("G4", "native_only_a.v2_text_core", not core, f"v2 text core k {k_list(sorted(core))} (none expected)")
    b = block_texts(D["native_only_a.product"])
    total.check("G4", "native_only_a.product.blocks_equal_plain", b == plainB,
                f"{len(b)} blocks byte-equal to the plain plate's" if b == plainB else
                f"k {k_list(sorted(b))} vs plain k {k_list(sorted(plainB))}")
    ps = PS["native_only_a.product"]
    pw = product_warnings(L["native_only_a.product"])
    total.check("G4", "native_only_a.product.counted", ps.get("native_only") == str(len(plainB)) and
                ps.get("native_only_nosparse") == "0" and not pw,
                f"native_only={ps.get('native_only')} (want {len(plainB)}), no warning {pw}")
    sb = block_texts(D["native_only_a.shadow"])
    sr = SS["native_only_a.shadow"]["reasons"]
    total.check("G4", "native_only_a.shadow.disagree", not sb and sr.get("disagree_native_only") == len(plainB) and
                not sr.get("disagree_text_only"), f"{len(sb)} blocks, {sr}")
    sw = (f"u-MEX native powder planner: {len(plainB)} core layer(s) ({k_list(sorted(plainB))}) left as paste: the "
          "native classification makes them core layers but the G-code text does not").encode()
    total.check("G4", "native_only_a.shadow.warning", sw in L["native_only_a.shadow"], sw.decode()[:160])

    # ---- tall_short: no block on a layer where an object has a top or bottom surface, in both modes; v2 agrees
    for nm in ("tall_short", "tall_short_precise"):
        pb, sb = block_texts(D[f"{nm}.product"]), block_texts(D[f"{nm}.shadow"])
        total.check("G4", f"{nm}.blocks_equal", pb == sb and bool(pb), f"{len(pb)} product blocks byte-equal to shadow")
        types = layer_types(strip_records(D[f"{nm}.shadow"]))
        skin = sorted(k for k, objs in types.items() if any(SKIN_TYPES & t for t in objs.values()))
        mixed = sorted(k for k in skin if any("Sparse infill" in t and not SKIN_TYPES & t for t in types[k].values()))
        bad = [k for k in skin if k in pb or k in sb]
        total.check("G4", f"{nm}.skin_layers_no_block", bool(skin) and not bad,
                    f"skin layers k {k_list(skin)}; mixed (one object sparse without skin, another skin) k "
                    f"{k_list(mixed)}" + (f"; BLOCK at {bad}" if bad else ""))
        if nm == "tall_short":
            total.check("G4", f"{nm}.mixed_layer", bool(mixed), f"mixed layers k {k_list(mixed)} (>= 1 required)")
        pw = product_warnings(L[f"{nm}.product"])
        total.check("G4", f"{nm}.no_warning", not pw, f"{pw}")
        g4_v2(total, manifest, nm, res[f"{nm}.shadow"]["plate"], sb)

    # ---- final_core: top_shell_layers = 0, the last layer has sparse infill and no block; v2 agrees
    nl = sum(1 for l in lines_of(D["final_core.shadow"]) if is_layer_change(l))
    types = layer_types(strip_records(D["final_core.shadow"]))
    tl = sorted(t for o in types.get(nl, {}).values() for t in o)
    total.check("G4", "final_core.last_layer", "Sparse infill" in tl and not (SKIN_TYPES & set(tl)), f"k {nl} types {tl}")
    pb, sb = block_texts(D["final_core.product"]), block_texts(D["final_core.shadow"])
    total.check("G4", "final_core.no_block", nl not in pb and nl not in sb and pb == sb and bool(pb),
                f"{len(pb)} blocks (k {k_list(sorted(pb))}), none at k {nl}, product == shadow")
    pw = product_warnings(L["final_core.product"])
    total.check("G4", "final_core.product.warning", pw == [WARN_FINAL.format(k=nl)] and
                PS["final_core.product"].get("final") == "1", f"{pw}")
    total.check("G4", "final_core.shadow.final", SS["final_core.shadow"]["reasons"].get("final") == 1,
                f"{SS['final_core.shadow']['reasons']}")
    g4_v2(total, manifest, "final_core", res["final_core.shadow"]["plate"], sb)
    # the plain plate's last layer has a top surface: not native core, no final warning
    total.check("G4", "plain.no_final_warning", PS["plain.product"].get("final") is None, f"{PS['plain.product']}")

    # ---- profile check (3.2 "untyped extrusion after sparse infill"): no shipped DIW printer profile, and no test
    # profile custom G-code that runs inside the layers extrudes without its own ;TYPE: line
    g4_profiles(total, D["plain.shadow"])

    # ---- native_only (b): no slice reaches it (stage-5 search, PRODUCT_DESIGN.md Deviations); the T12 stream does
    r = subprocess.run([args.catch2, G4_CATCH2_CASE], capture_output=True, cwd=os.path.dirname(args.catch2))
    out = (r.stdout + r.stderr).decode("utf-8", "replace")
    m = re.search(r"All tests passed \((\d+) assertions? in 1 test case\)", out)
    total.check("G4", "native_only_b.catch2", r.returncode == 0 and m is not None, m.group(0) if m else out.strip()[-300:])
    manifest.save()
    return total


def main_g4(argv):
    global E
    import argparse
    ap = argparse.ArgumentParser(prog="product_gate.py g4")
    ap.add_argument("engine")
    ap.add_argument("tr")
    ap.add_argument("work")
    ap.add_argument("--orca", required=True)
    ap.add_argument("--datadir", required=True)
    ap.add_argument("--catch2", default=None, help="diw_planner_tests.exe (default: next to diw_translate.exe)")
    ap.add_argument("--jobs", type=int, default=6)
    args = ap.parse_args(argv)
    args.tr, args.orca = os.path.abspath(args.tr), os.path.abspath(args.orca)
    args.catch2 = os.path.abspath(args.catch2 or os.path.join(os.path.dirname(args.tr), "diw_planner_tests.exe"))
    if not os.path.isfile(os.path.join(args.engine, "diw_engine.py")):
        print(f"FAIL G4 engine: {args.engine}\n1 checks, 1 failures")
        return 1
    if args.engine not in sys.path:
        sys.path.insert(0, args.engine)
    import diw_engine
    E = diw_engine
    t0 = time.time()
    total = g4(args)
    for l in total.lines:
        print(l)
    for f in total.fails[:40]:
        print("FAILED:", f)
    print(f"G4 in {time.time() - t0:.0f} s")
    print(f"{total.n} checks, {len(total.fails)} failures")
    return 0 if not total.fails else 1


# ----------------------------------------------------------------------------------------------- G1 / G3 (stage 4a)

PROFILE_FILES = ("flat_machine_diw.json", "flat_process_powder.json")
G10_MUTATIONS = ("drop_restore", "extra_delete", "wrong_g92")
G10_CHANGES_OUTPUT = ("drop_restore", "wrong_g92")   # PRODUCT_DESIGN.md 8 G10 (b): these must change the translation


def s0_jobs(sets, only=None):
    """The slices of the stage-4a S0 shadow runs (shadow_export.py --matrix --powder-mode shadow work dirs), with what
    is needed to export the same slice again in product mode: the S0 profiles of the slice, its meshes (the S0 work's
    mesh directories), CLI arguments and extra environment. sets: {"matrix" | "stress" | "t5" | "cov": work dir}."""
    import shadow_export as S
    jobs = []

    def mdir(work, d):
        return os.path.join(work, f"meshes_d{d}")

    for set_name, work in sets.items():
        if set_name == "t5":
            for part in S.T5_PARTS:
                for wname in S.WALLS:
                    sd = os.path.join(work, "t5", f"{part}.{wname}")
                    stls, a = S.mesh_files(mdir(work, S.DEFAULT_DELTA), part)
                    for tag in ("gap0", "gap15"):
                        jobs.append({"set": set_name, "name": f"T5.{part}.{wname}.{tag}", "sdir": sd,
                                     "shadow": os.path.join(sd, tag, "plate_1.gcode"),
                                     "settings": ";".join(os.path.join(sd, "profiles", f"{tag}.{fn}") for fn in PROFILE_FILES),
                                     "stls": stls, "args": a, "env": {"ORCA_DIW_TEST_KEEP_SEAM_GAP": "1"}})
            continue
        if set_name == "matrix":
            entries = [(p, v, ov, {}) for p, v, ov in S.matrix_jobs()]
        elif set_name == "stress":
            entries = list(S.STRESS)
        elif set_name == "cov":
            with open(os.path.join(HERE, "coverage_slices.json"), encoding="utf-8") as f:
                cov = json.load(f)
            entries = [(s["part"], s["variant"], dict(s["overrides"]), {}) for s in cov["slices"]]
        else:
            raise ValueError(f"unknown S0 set {set_name!r}")
        for part, v, ov, env in entries:
            name = f"{part}.{v}"
            sd = os.path.join(work, "slices", name)
            d = float(ov["diw_shell_powder_clearance"]) if set_name == "matrix" and "diw_shell_powder_clearance" in ov \
                else S.DEFAULT_DELTA
            if part in S.ASSEMBLE:
                stls, a = [], ["--load-assemble-list", os.path.join(sd, "assemble.json")]
            else:
                stls, a = S.mesh_files(mdir(work, d), part)
            jobs.append({"set": set_name, "name": name, "sdir": sd, "shadow": os.path.join(sd, "on", "plate_1.gcode"),
                         "settings": ";".join(os.path.join(sd, "profiles", f"p.{fn}") for fn in PROFILE_FILES),
                         "stls": stls, "args": a, "env": dict(env)})
    if only:
        jobs = [j for j in jobs if only in f"{j['set']}/{j['name']}"]
    return jobs


def product_export(args, job, out_dir, env_extra=None, dump=None, untranslated=True):
    """Exports the slice of `job` again with the slicer default mode (product), the S0 profiles and meshes."""
    import shadow_export as S
    env = dict(job["env"])
    env["ORCA_DIW_POWDER_MODE"] = "product"
    env.update(env_extra or {})
    if dump:
        if os.path.isdir(dump):
            shutil.rmtree(dump)
        os.makedirs(dump)
        env["ORCA_DIW_SHADOW_DUMP"] = dump
    missing = [p for p in job["stls"] + job["settings"].split(";") if not os.path.isfile(p)]
    if missing:
        return None, None, f"missing inputs {missing[:3]}".encode()
    rc, data, log, _ = S.slice_part(args.orca, args.datadir, job["stls"], out_dir, env, job["settings"], job["args"],
                                    untranslated=untranslated)
    return rc, data, log


def translate_pair(args, chk, gate, shadow_path, product_path, wdir, tag=""):
    """G1 (c): translate(shadow, --v2-parity as every S0 translation) vs translate(product, strict default); both exit 0
    and are equal after norm (the generated-at line) + the frozen mask, or both abort with the same message.
    Returns (equal_after_mask, byte_equal_after_norm, rc)."""
    import shadow_export as S
    ts, tp = os.path.join(wdir, f"shadow{tag}.tr.gcode"), os.path.join(wdir, f"product{tag}.tr.gcode")
    rc1, e1 = run_tr(args.tr, ["--config-from-gcode", "--v2-parity", shadow_path, ts])
    rc2, e2 = run_tr(args.tr, ["--config-from-gcode", product_path, tp])
    if rc1 == 0 and rc2 == 0:
        a, b = S.norm(read(ts)), S.norm(read(tp))
        ma, mb = g1_mask(a), g1_mask(b)
        eq = ma == mb
        chk.check(gate, f"translate{tag}", eq, f"{len(mb)} bytes equal after norm + mask" + (" (byte-identical after norm)"
                  if a == b else " (M73 / statistics differ before the mask)") if eq else first_diff(mb, ma))
        return eq, a == b, 0
    m1 = [l for l in e1.splitlines() if l.startswith("error:")]
    m2 = [l for l in e2.splitlines() if l.startswith("error:")]
    eq = rc1 == rc2 and m1 == m2 and bool(m1)
    chk.check(gate, f"translate{tag}", eq, f"both abort rc={rc1}: {m1[:1]}" if eq else f"rc {rc1} vs {rc2}: {m1[:1]} vs {m2[:1]}")
    return eq, eq, rc1


def g1_slice(args, manifest, job):
    name = f"{job['set']}/{job['name']}"
    chk = Checks(name)
    info = {"set": job["set"], "name": name, "job": job}
    if not chk.check("G1", "shadow_input", os.path.isfile(job["shadow"]), job["shadow"]):
        return chk, info
    manifest.register(job["shadow"], "shadow", "stage-4a S0 shadow export")
    shadow = read(job["shadow"])
    w = os.path.join(args.work, job["set"], job["name"])
    os.makedirs(w, exist_ok=True)
    out = os.path.join(w, "product")
    prod_path = os.path.join(out, "plate_1.gcode")
    if args.reuse_product and os.path.isfile(prod_path):
        rc, data, log = 0, read(prod_path), read(os.path.join(out, "log.txt"))
    else:
        rc, data, log = product_export(args, job, out)
    if data is not None:
        manifest.register(prod_path, "product", "G1 product export")
    # G1 (a)
    if not chk.check("G1a", "export", rc == 0 and data is not None,
                     f"rc={rc} {(failure_list(log or b'') or (log or b'')[-300:].decode('utf-8', 'replace'))[:400]}" if rc or data is None else ""):
        return chk, info
    summ = parse_product_summary(log.decode("utf-8", "replace"))
    if not chk.check("G1a", "summary", summ is not None, "product summary line" if summ else "no product summary line"):
        return chk, info
    info["summary"] = summ
    # G1 (b)
    same = record_lines(data) == record_lines(shadow)
    chk.check("G1b", "records", same, f"{len(record_lines(shadow))} record lines byte-equal" if same else
              f"{len(record_lines(data))} vs {len(record_lines(shadow))} record lines; " +
              first_diff(b"".join(record_lines(data)), b"".join(record_lines(shadow))))
    # G1 (c)
    eq, beq, rc_t = translate_pair(args, chk, "G1c", job["shadow"], prod_path, w)
    info["translate_rc"], info["translate_bytes_equal"] = rc_t, beq
    # G3
    blocks = blocks_of(shadow)
    sts = {k: v["st"] for k, v in blocks.items()}
    # with the G1 mask (PRODUCT_DESIGN.md 8 G3): the processor places M73 lines by the (smaller) time estimate
    # (and norm: the generated-at line)
    import shadow_export as S
    res = structural_check(chk, "G3", g1_mask(S.norm(shadow)), g1_mask(S.norm(data)), sts, width_fallback(shadow))
    n_ok = sum(1 for v in sts.values() if v == "ok")
    sup, nfb = int(summ.get("suppressed", "-1")), int(summ.get("fallback", "-1"))
    flog = [l.strip() for l in log.decode("utf-8", "replace").splitlines() if "suppression fallback" in l]
    chk.check("G3", "fallback", nfb == 0, f"fallback={nfb}" + (f"; {' | '.join(flog)[:400]}" if nfb else ""))
    chk.check("G3", "suppressed", sup == n_ok and len(res["edited"]) == n_ok,
              f"summary suppressed={sup}, st=ok blocks {n_ok}, edited chunks {len(res['edited'])}")
    info["fallback"], info["suppressed"], info["n_ok"] = nfb, sup, n_ok
    info["removed_moves"] = int(summ.get("removed_moves", 0))
    info["fb_lines"] = sum(v["fb"] for v in blocks.values())
    info["small_blocks"] = sum(1 for v in blocks.values() if v["st"] == "small")
    info["edited_with_fb"] = sorted(k for k in res["edited"] if blocks.get(k, {}).get("fb", 0) > 0)
    info["small_changed"] = [k for k, v in blocks.items() if v["st"] == "small" and k in res["edited"]]
    info["st"] = {s: sum(1 for v in sts.values() if v == s) for s in sorted(set(sts.values()))}
    return chk, info


def coverage_counters(total, gate, cov):
    if not cov:
        total.check("COV", f"{gate}.coverage_set", False, "no coverage slice")
        return
    fb = sum(i.get("fb_lines", 0) for i in cov)
    small = sum(i.get("small_blocks", 0) for i in cov)
    ed = {i["name"]: i["edited_with_fb"] for i in cov if i.get("edited_with_fb")}
    n_ed = sum(len(v) for v in ed.values())
    small_unedited = sum(i.get("small_blocks", 0) - len(i.get("small_changed", [])) for i in cov)
    total.check("COV", f"{gate}.fb_lines", fb > 0, f"fb_lines={fb}")
    total.check("COV", f"{gate}.small_blocks", small > 0, f"small_blocks={small}")
    total.check("COV", f"{gate}.edited_chunks_with_fb", n_ed >= 2 and len(ed) >= 2,
                f"edited_chunks_with_fb={n_ed} on {len(ed)} slice(s): " + ", ".join(f"{k} {v}" for k, v in sorted(ed.items())))
    total.check("COV", f"{gate}.small_chunks_unedited", small > 0 and small_unedited == small,
                f"small_chunks_unedited={small_unedited} of {small}")


def g10_slice(args, manifest, job, mut):
    """G10 for one slice and one ORCA_DIW_TEST_SUPPRESS_MUTATE value: (a) verification on, (b) SKIP_VERIFY."""
    name = f"{job['set']}/{job['name']}/{mut}"
    chk = Checks(name)
    info = {"name": name, "mut": mut}
    shadow = read(job["shadow"])
    manifest.register(job["shadow"], "shadow", "stage-4a S0 shadow export")
    blocks = blocks_of(shadow)
    sts = {k: v["st"] for k, v in blocks.items()}
    n_ok = sum(1 for v in sts.values() if v == "ok")
    chk.check("G10", "has_ok_blocks", n_ok > 0, f"{n_ok} st=ok blocks")
    base = os.path.join(args.work, "g10", job["set"], job["name"], mut)
    os.makedirs(base, exist_ok=True)
    import shadow_export as S
    # (a) verification on: every st=ok chunk falls back with a V1 / V2 reason, the export is the unedited (shadow) text
    out, dump = os.path.join(base, "verify"), os.path.join(base, "verify.dump")
    rc, data, log = product_export(args, job, out, {"ORCA_DIW_TEST_SUPPRESS_MUTATE": mut}, dump=dump)
    if chk.check("G10a", "export", rc == 0 and data is not None, f"rc={rc} {(log or b'')[-300:]!r}" if rc or data is None else ""):
        manifest.register(os.path.join(out, "plate_1.gcode"), "product", f"G10a {mut}")
        summ = parse_product_summary(log.decode("utf-8", "replace")) or {}
        chk.check("G10a", "summary", summ.get("suppressed") == "0" and int(summ.get("fallback", -1)) == n_ok,
                  f"suppressed={summ.get('suppressed')} fallback={summ.get('fallback')} st=ok blocks {n_ok}")
        reasons, bad = {}, []
        for fn in os.listdir(dump):
            if fn.startswith("k") and fn.endswith(".json"):
                with open(os.path.join(dump, fn), encoding="utf-8") as f:
                    j = json.load(f)
                if sts.get(j["k"]) != "ok":
                    continue
                r = j.get("fallback") or ""
                reasons[r] = reasons.get(r, 0) + 1
                if j.get("suppressed") is not False or not r.startswith(("v1_", "v2_")):
                    bad.append(j["k"])
        chk.check("G10a", "fallback_reasons", not bad and sum(reasons.values()) == n_ok,
                  f"per reason {reasons}" + (f"; bad k {sorted(bad)[:10]}" if bad else ""))
        info["reasons_a"] = reasons
        flog = [l.strip() for l in log.decode("utf-8", "replace").splitlines() if "suppression fallback" in l]
        chk.check("G10a", "fallback_logged", bool(flog), flog[0][:200] if flog else "no fallback line in the log")
        same = S.norm(data) == S.norm(shadow)
        chk.check("G10a", "unedited", same, "export == shadow export (norm)" if same else first_diff(S.norm(data), S.norm(shadow)))
        chk.check("G10a", "records", record_lines(data) == record_lines(shadow), "G1 (b)")
        translate_pair(args, chk, "G10a.G1c", job["shadow"], os.path.join(out, "plate_1.gcode"), base, ".verify")
    # (b) SKIP_VERIFY: the mutated edit is released; G3 must fail; G1 must fail where the translation changes
    out = os.path.join(base, "skip")
    rc, data, log = product_export(args, job, out, {"ORCA_DIW_TEST_SUPPRESS_MUTATE": mut, "ORCA_DIW_TEST_SKIP_VERIFY": "1"})
    if chk.check("G10b", "export", rc == 0 and data is not None, f"rc={rc} {(log or b'')[-300:]!r}" if rc or data is None else ""):
        manifest.register(os.path.join(out, "plate_1.gcode"), "product", f"G10b {mut}")
        summ = parse_product_summary(log.decode("utf-8", "replace")) or {}
        chk.check("G10b", "summary", summ.get("suppressed") == str(n_ok) and summ.get("fallback") == "0",
                  f"suppressed={summ.get('suppressed')} fallback={summ.get('fallback')} st=ok blocks {n_ok}")
        probe = Checks(f"{name}.probe")
        structural_check(probe, "G3", g1_mask(S.norm(shadow)), g1_mask(S.norm(data)), sts, width_fallback(shadow))
        chk.check("G10b", "G3_fails", len(probe.fails) > 0, "G3 reports: " + (probe.fails[0][:260] if probe.fails else "nothing"))
        info["g3_b"] = probe.fails[0][:200] if probe.fails else ""
        tprobe = Checks(f"{name}.tprobe")
        eq, beq, rc_t = translate_pair(args, tprobe, "G1c", job["shadow"], os.path.join(out, "plate_1.gcode"), base, ".skip")
        g1_fails = (not eq) or record_lines(data) != record_lines(shadow)
        info["g1_b_fails"] = g1_fails
        info["g1_b_detail"] = tprobe.lines[-1][:240] if tprobe.lines else ""
        if mut in G10_CHANGES_OUTPUT:
            chk.check("G10b", "G1_fails", g1_fails, "G1 (c) reports: " + info["g1_b_detail"])
        else:
            # extra_delete: G1 must fail exactly when the translated output changes; G1 is that comparison, so this is
            # reported (the mutation is caught by G3 either way)
            chk.check("G10b", "G1_reported", True, f"translated output {'changes (G1 fails)' if g1_fails else 'unchanged (G1 passes)'}: "
                      + info["g1_b_detail"])
    return chk, info


# ----------------------------------------------------------------------------------------------- stage 4b: G2, G1 (d), G8

FIL_MM_RE = re.compile(rb"^; filament used \[mm\] = ([^\r\n]+)$", re.M)
TIME_RE = re.compile(rb"^; (estimated printing time \([^)]*\)|total estimated time|model printing time)[^=\r\n]*= ([^\r\n]+)$", re.M)
M73_ANY_RE = re.compile(rb"^(?:; \(FILTERED\) )?M73\b", re.M)


def disagreement(summ):
    """The product summary's disagreement counters (PRODUCT_DESIGN.md 3.2): text_only, native_only, native_only_nosparse."""
    return {"text_only": int(summ.get("text_only", 0)), "native_only": int(summ.get("native_only", 0)),
            "native_only_nosparse": int(summ.get("native_only_nosparse", 0))}


def parse_duration(s):
    """'1d 2h 3m 4s' -> seconds (GCodeProcessor's get_time_dhms form)."""
    tot, ok = 0.0, False
    for v, u in re.findall(r"([0-9.]+)\s*([dhms])", s):
        tot += float(v) * {"d": 86400, "h": 3600, "m": 60, "s": 1}[u]
        ok = True
    return tot if ok else None


def g8_check(chk, shadow, product, log, gate="G8"):
    """G8: the processor's filament used [mm] of the product export == the shadow export's minus the summed removed_de
    of the product summary, within one unit of the printed last digit; every estimated time line not larger."""
    summ = parse_product_summary(log.decode("utf-8", "replace")) if isinstance(log, bytes) else log
    if not chk.check(gate, "summary", summ is not None and "removed_de" in summ, "product summary with removed_de"):
        return None
    rde = float(summ["removed_de"])
    fs, fp = FIL_MM_RE.findall(shadow), FIL_MM_RE.findall(product)
    if not chk.check(gate, "filament_lines", len(fs) == 1 and len(fp) == 1, f"{len(fs)} / {len(fp)} '; filament used [mm]' lines"):
        return None
    vs, vp = [x.strip() for x in fs[0].decode().split(",")], [x.strip() for x in fp[0].decode().split(",")]
    ok, det = len(vs) == len(vp) and len(vs) > 0, []
    # the removed paste is extruder 0's (the core material; DIW single paste extruder)
    for i, (a, b) in enumerate(zip(vs, vp)):
        dec = len(b.split(".")[1]) if "." in b else 0
        unit = 10.0 ** -dec
        want = float(a) - (rde if i == 0 else 0.0)
        d = abs(float(b) - want)
        ok &= d <= unit + 1e-9
        det.append(f"[{i}] shadow {a} - removed_de {rde if i == 0 else 0:.6g} = {want:.4f} vs product {b} (|d|={d:.4f}, unit {unit:g})")
    chk.check(gate, "filament_mm", ok, "; ".join(det))
    ts, tp = TIME_RE.findall(shadow), TIME_RE.findall(product)
    ok = len(ts) > 0 and [k for k, _ in ts] == [k for k, _ in tp]
    det = []
    for (k, a), (_, b) in zip(ts, tp):
        sa, sb = parse_duration(a.decode()), parse_duration(b.decode())
        ok &= sa is not None and sb is not None and sb <= sa
        det.append(f"{k.decode()}: {b.decode()} <= {a.decode()}")
    chk.check(gate, "time_not_larger", ok, "; ".join(det) if det else f"{len(ts)} / {len(tp)} time lines")
    return {"removed_de": rde, "fil_shadow": vs, "fil_product": vp}


def g2_slice(args, manifest, job):
    """G2: ORCA_DIW_TEST_NO_SUPPRESS=1 product export == S0 shadow export byte for byte (after norm, the generated-at
    line); the disagreement counters are 0, nothing is suppressed."""
    import shadow_export as S
    name = f"{job['set']}/{job['name']}"
    chk = Checks(name)
    info = {"set": job["set"], "name": name}
    if not chk.check("G2", "shadow_input", os.path.isfile(job["shadow"]), job["shadow"]):
        return chk, info
    manifest.register(job["shadow"], "shadow", "stage-4a S0 shadow export")
    shadow = read(job["shadow"])
    out = os.path.join(args.work, job["set"], job["name"], "nosup")
    rc, data, log = product_export(args, job, out, {"ORCA_DIW_TEST_NO_SUPPRESS": "1"})
    if not chk.check("G2", "export", rc == 0 and data is not None,
                     f"rc={rc} {(log or b'')[-300:].decode('utf-8', 'replace')}" if rc or data is None else ""):
        return chk, info
    manifest.register(os.path.join(out, "plate_1.gcode"), "product", "G2 no-suppress export")
    summ = parse_product_summary(log.decode("utf-8", "replace"))
    if not chk.check("G2", "summary", summ is not None, "product summary line" if summ else "no product summary line"):
        return chk, info
    dis = disagreement(summ)
    chk.check("G2", "disagreement", sum(dis.values()) == 0, " ".join(f"{k}={v}" for k, v in dis.items()))
    chk.check("G2", "not_suppressed", summ.get("suppressed") == "0" and summ.get("removed_moves") == "0"
              and float(summ.get("removed_de", "nan")) == 0.0,
              f"suppressed={summ.get('suppressed')} removed_moves={summ.get('removed_moves')} removed_de={summ.get('removed_de')}")
    same = S.norm(data) == S.norm(shadow)
    chk.check("G2", "identical", same, f"{len(data)} bytes, identical after norm" if same else first_diff(S.norm(data), S.norm(shadow)))
    info["identical"] = same
    info["disagreement"] = sum(dis.values())
    g8 = g8_check(chk, shadow, data, summ, "G8.nosup")
    return chk, info


def m73_settings(job, wdir):
    """Copies of the slice's S0 profiles with disable_m73 = 1 (G1 (d)); the key is in both flattened profiles."""
    os.makedirs(wdir, exist_ok=True)
    out = []
    for p in job["settings"].split(";"):
        t = read(p)
        n = len(re.findall(rb'"disable_m73": "0"', t))
        if n != 1:
            raise ValueError(f"{p}: {n} '\"disable_m73\": \"0\"' entries")
        q = os.path.join(wdir, "m73." + os.path.basename(p))
        with open(q, "wb") as f:
            f.write(t.replace(b'"disable_m73": "0"', b'"disable_m73": "1"'))
        out.append(q)
    return ";".join(out)


def g1_mask_stats(data):
    """G1 (d): only (ii) of the frozen mask: the 9 statistics lines outside layer chunks (no M73 deletion)."""
    ls = lines_of(data)
    marks = [i for i, l in enumerate(ls) if LAYER_MARKER_RE.match(l)]
    first, last = (marks[0], marks[-1]) if marks else (len(ls), -1)
    out = []
    for i, l in enumerate(ls):
        if i < first or i > last:
            for key in STAT_KEYS:
                if l.startswith(key):
                    l = key + b" <masked>\n"
                    break
        out.append(l)
    return b"".join(out)


def g1d_slice(args, manifest, job):
    """G1 (d): shadow and product exports with disable_m73 = 1; records equal; translate(shadow, --v2-parity) and
    translate(product, strict) equal after norm + the statistics-line mask only (or both abort with the same message);
    no M73 anywhere; G3 structural check; G8 on the pair."""
    import shadow_export as S
    name = f"{job['set']}/{job['name']}"
    chk = Checks(name)
    info = {"set": job["set"], "name": name}
    w = os.path.join(args.work, job["set"], job["name"])
    try:
        settings = m73_settings(job, os.path.join(w, "profiles"))
    except (OSError, ValueError) as e:
        chk.check("G1d", "profiles", False, repr(e))
        return chk, info
    j2 = dict(job, settings=settings)
    res = {}
    for mode in ("shadow", "product"):
        out = os.path.join(w, mode)
        rc, data, log = product_export(args, j2, out, {"ORCA_DIW_POWDER_MODE": mode})
        ok = chk.check("G1d", f"{mode}.export", rc == 0 and data is not None,
                       f"rc={rc} {(log or b'')[-300:].decode('utf-8', 'replace')}" if rc or data is None else "")
        if not ok:
            return chk, info
        manifest.register(os.path.join(out, "plate_1.gcode"), mode, f"G1 (d) disable_m73 {mode} export")
        chk.check("G1d", f"{mode}.disable_m73", b"\n; disable_m73 = 1\n" in data, "config block says disable_m73 = 1")
        n73 = len(M73_ANY_RE.findall(data))
        chk.check("G1d", f"{mode}.no_m73", n73 == 0, f"{n73} M73 line(s)")
        res[mode] = (data, log)
    shadow, product = res["shadow"][0], res["product"][0]
    summ = parse_product_summary(res["product"][1].decode("utf-8", "replace"))
    chk.check("G1d", "summary", summ is not None and summ.get("fallback") == "0",
              f"suppressed={summ.get('suppressed')} fallback={summ.get('fallback')}" if summ else "no product summary")
    same = record_lines(product) == record_lines(shadow)
    chk.check("G1d", "records", same, f"{len(record_lines(shadow))} record lines byte-equal" if same else
              first_diff(b"".join(record_lines(product)), b"".join(record_lines(shadow))))
    ts, tp = os.path.join(w, "shadow.tr.gcode"), os.path.join(w, "product.tr.gcode")
    rc1, e1 = run_tr(args.tr, ["--config-from-gcode", "--v2-parity", os.path.join(w, "shadow", "plate_1.gcode"), ts])
    rc2, e2 = run_tr(args.tr, ["--config-from-gcode", os.path.join(w, "product", "plate_1.gcode"), tp])
    info["translate_rc"] = rc1
    if rc1 == 0 and rc2 == 0:
        a, b = S.norm(read(ts)), S.norm(read(tp))
        ma, mb = g1_mask_stats(a), g1_mask_stats(b)
        eq = ma == mb
        nd = sum(1 for x, y in zip(lines_of(a), lines_of(b)) if x != y) if len(lines_of(a)) == len(lines_of(b)) else -1
        chk.check("G1d", "translate", eq, (f"{len(mb)} bytes equal after norm + statistics mask; {nd} statistics line(s) "
                                           f"differ before it" if eq else first_diff(mb, ma)))
        info["stat_lines_differ"] = nd
        info["byte_equal"] = a == b
    else:
        m1 = [l for l in e1.splitlines() if l.startswith("error:")]
        m2 = [l for l in e2.splitlines() if l.startswith("error:")]
        eq = rc1 == rc2 and m1 == m2 and bool(m1)
        chk.check("G1d", "translate", eq, f"both abort rc={rc1}: {m1[:1]}" if eq else f"rc {rc1} vs {rc2}: {m1[:1]} vs {m2[:1]}")
    sts = {k: v["st"] for k, v in blocks_of(shadow).items()}
    r = structural_check(chk, "G3d", g1_mask(S.norm(shadow)), g1_mask(S.norm(product)), sts, width_fallback(shadow))
    n_ok = sum(1 for v in sts.values() if v == "ok")
    chk.check("G3d", "suppressed", summ is not None and int(summ.get("suppressed", -1)) == n_ok and len(r["edited"]) == n_ok,
              f"st=ok blocks {n_ok}, edited chunks {len(r['edited'])}")
    info["n_ok"] = n_ok
    g8_check(chk, shadow, product, res["product"][1], "G8.m73")
    return chk, info


def g8_slice(args, manifest, job):
    """G8 on the stage-4a G1 product export of the slice (--g1-work) against its S0 shadow export."""
    name = f"{job['set']}/{job['name']}"
    chk = Checks(name)
    info = {"set": job["set"], "name": name}
    d = os.path.join(args.g1_work, job["set"], job["name"], "product")
    pp = os.path.join(d, "plate_1.gcode")
    if not chk.check("G8", "inputs", os.path.isfile(pp) and os.path.isfile(job["shadow"]), pp):
        return chk, info
    manifest.register(job["shadow"], "shadow", "stage-4a S0 shadow export")
    manifest.register(pp, "product", "stage-4a G1 product export")
    r = g8_check(chk, read(job["shadow"]), read(pp), read(os.path.join(d, "log.txt")))
    if r:
        info.update(r)
    return chk, info


def main_g7(argv):
    """G7: product export time <= 1.2 x kill-switch time on cube25 and cone (best of 5, alternating, the T10 rule);
    max_held = 1 in every product export."""
    import argparse
    import math
    import shadow_export as S
    ap = argparse.ArgumentParser(prog="product_gate.py g7")
    ap.add_argument("work")
    ap.add_argument("--orca", required=True)
    ap.add_argument("--datadir", required=True)
    ap.add_argument("--runs", type=int, default=5)
    args = ap.parse_args(argv)
    os.environ.pop("DIW_HARNESS_POWDER_MODE", None)
    total = Checks("run")
    total.check("G7", "exe.orca", os.path.isfile(args.orca), args.orca)
    total.check("G7", "datadir", os.path.isdir(args.datadir), args.datadir)
    if total.fails:
        print(f"{total.n} checks, {len(total.fails)} failures")
        return 1
    os.makedirs(args.work, exist_ok=True)
    mdir = S.make_meshes(args.work)
    for part in S.T10_PARTS:
        stls, a = S.mesh_files(mdir, part)
        times, exp, held, binder, modes = {"kill": [], "product": []}, {"kill": [], "product": []}, [], [], set()
        for it in range(args.runs):
            for rn, env in (("kill", {"ORCA_DIW_NO_NATIVE_PLAN": "1"}), ("product", {"ORCA_DIW_POWDER_MODE": "product"})):
                rc, data, log, dt = S.slice_part(args.orca, args.datadir, stls, os.path.join(args.work, f"{part}.{rn}"), env, None, a)
                if rc == 0 and data is not None:
                    times[rn].append(dt)
                m = S.EXPORT_MS_RE.search(log)
                if m:
                    exp[rn].append(float(m.group(1)))
                if rn == "product":
                    s = parse_product_summary(log.decode("utf-8", "replace"))
                    held.append(int(s["max_held"]) if s and "max_held" in s else -1)
                    if s:
                        binder.append(float(s["binder_ms"]) + float(s["capture_ms"]))
                        modes.add(f"suppressed={s.get('suppressed')} fallback={s.get('fallback')}")
        ok = len(times["kill"]) == args.runs and len(times["product"]) == args.runs
        ratio = min(times["product"]) / min(times["kill"]) if ok else math.inf
        eratio = min(exp["product"]) / min(exp["kill"]) if exp["product"] and exp["kill"] else math.inf
        total.check("G7", f"{part}.time", ok and ratio <= 1.2,
                    f"CLI export product {min(times['product']) if times['product'] else 0:.3f} s / kill "
                    f"{min(times['kill']) if times['kill'] else 0:.3f} s = {ratio:.3f} (best of {args.runs}); "
                    f"G-code export step alone {min(exp['product']) if exp['product'] else 0:.1f} / "
                    f"{min(exp['kill']) if exp['kill'] else 0:.1f} ms = {eratio:.2f}; planner (capture + binder) "
                    f"{min(binder) if binder else 0:.1f} ms; all runs: product {[round(t, 3) for t in times['product']]} "
                    f"kill {[round(t, 3) for t in times['kill']]}")
        total.check("G7", f"{part}.max_held", bool(held) and all(h == 1 for h in held), f"max_held per export {held}")
        total.check("G7", f"{part}.product", len(modes) == 1 and "fallback=0" in next(iter(modes)) and "suppressed=0" not in next(iter(modes)),
                    f"{sorted(modes)}")
    for l in total.lines:
        print(l)
    print(f"{total.n} checks, {len(total.fails)} failures")
    return 0 if not total.fails else 1


def run_parallel(jobs_fn, n):
    out = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=n) as ex:
        futs = [ex.submit(*j) for j in jobs_fn]
        for f in futs:
            try:
                out.append(f.result())
            except Exception as exc:   # a crash of the harness is a failure
                import traceback
                chk = Checks("crash")
                chk.check("ERR", "exception", False, f"{exc!r} {traceback.format_exc()[-600:]}")
                out.append((chk, {}))
    return out


G11_SET_PICK = {"matrix": "cube25.classic.base", "cov": "slot.classic.cov_fb_d1.25"}


def g11_slice(args, manifest, job):
    """G11 (PRODUCT_DESIGN.md 8): the shipped in-slicer translation path (OrcaSlicer.cpp, PostProcessor.cpp: no
    ORCA_DIW_EXPORT_UNTRANSLATED) equals translate(product export) after norm. Both exports are product mode."""
    import shadow_export as S
    name = f"{job['set']}/{job['name']}"
    chk = Checks(name)
    info = {"set": job["set"], "name": name, "ok": False, "skipped": False}
    w = os.path.join(args.work, job["set"], job["name"])
    os.makedirs(w, exist_ok=True)
    rc, raw, log = product_export(args, job, os.path.join(w, "untranslated"), untranslated=True)
    if not chk.check("G11", "untranslated_export", rc == 0 and raw is not None, f"rc={rc}"):
        return chk, info
    raw_path = os.path.join(w, "untranslated", "plate_1.gcode")
    manifest.register(raw_path, "product", "G11 untranslated product export")
    tp = os.path.join(w, "untranslated.tr.gcode")
    rc_t, err = run_tr(args.tr, ["--config-from-gcode", raw_path, tp])
    if rc_t != 0:
        info["skipped"] = True   # a policy=error slice: no translated reference to compare with
        chk.check("G11", "reference_translation", True, f"diw_translate aborts rc={rc_t}; candidate skipped")
        return chk, info
    rc2, shipped, log2 = product_export(args, job, os.path.join(w, "shipped"), untranslated=False)
    if not chk.check("G11", "shipped_export", rc2 == 0 and shipped is not None,
                     f"rc={rc2} {(failure_list(log2 or b'') or (log2 or b'')[-300:].decode('utf-8', 'replace'))[:400]}"):
        return chk, info
    ref = S.norm(read(tp))
    got = S.norm(shipped)
    chk.check("G11", "raw_differs_from_shipped", S.norm(raw) != got and b";DIW:BEGIN" not in got,
              "the shipped file is translated (no ;DIW:BEGIN record left)" if b";DIW:BEGIN" not in got else "records remain")
    eq = got == ref
    chk.check("G11", "shipped_equals_translate", eq, f"{len(got)} bytes equal after norm" if eq else first_diff(got, ref))
    info["ok"] = eq
    info["bytes"] = len(got)
    return chk, info


def g11_refusals(args, total):
    """G11: the G5 refusals without the untranslated switch give the same messages and leave no plate / .tmp file."""
    import shadow_export as S
    mdir = S.make_meshes(args.work)
    base = {"work": os.path.join(args.work, "refusals"), "orca": args.orca, "datadir": args.datadir,
            "mesh": os.path.join(mdir, "cube25.stl")}
    SKIP = {"ORCA_DIW_TEST_SKIP_VALIDATE": "1"}
    jobs = [
        dict(base, name="arc_g91", overrides={"layer_change_gcode": LC_BASE + "\\nG91\\nG1 Z0 F3000\\nG90"}),
        dict(base, name="orphan_typed", overrides=dict(REL_E, layer_change_gcode=LC_BASE + LC_EXTRUDE.replace(
            "\\nG1 X6", "\\n;TYPE:Sparse infill\\nG1 X6"))),
        dict(base, name="skip.density0", overrides={"sparse_infill_density": "0%"}, env=SKIP),
        dict(base, name="skip.spiral", overrides={"spiral_mode": "1"}, env=SKIP),
        dict(base, name="fault.binder", env={"ORCA_DIW_SHADOW_FAULT": "binder"}),
        dict(base, name="fault.cancel", env={"ORCA_DIW_SHADOW_FAULT": "cancel"}),
    ]
    both = []
    for j in jobs:
        both.append(dict(j, name=j["name"] + ".raw", untranslated=True))
        both.append(dict(j, name=j["name"] + ".shipped", untranslated=False))
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        res = {r["name"]: r for r in ex.map(g5_slice, both)}
    for j in jobs:
        a, b = res[j["name"] + ".raw"], res[j["name"] + ".shipped"]
        tmp = [f for f in b["files"] if f.lower().endswith(".tmp")]
        total.check("G11", f"refusal.{j['name']}.no_output", b["rc"] != 0 and "plate_1.gcode" not in b["files"] and not tmp,
                    f"rc={b['rc']} files={b['files']}")
        fa, fb = failure_list(a["log"]), failure_list(b["log"])
        ra, rb = refusals(a["log"]), refusals(b["log"])
        total.check("G11", f"refusal.{j['name']}.same_message", a["rc"] != 0 and (fa, ra) == (fb, rb) and bool(fa or ra or j["name"] == "fault.cancel"),
                    f"failure list {str(fb)[:160]!r} refusals {rb}")


def main_g1(argv, mode):
    global E
    import argparse
    ap = argparse.ArgumentParser(prog=f"product_gate.py {mode}")
    ap.add_argument("engine")
    ap.add_argument("tr")
    ap.add_argument("work")
    ap.add_argument("sets", nargs="+", help="<set>=<S0 shadow_export work dir>, set in matrix, stress, t5, cov")
    ap.add_argument("--orca", required=True)
    ap.add_argument("--datadir", required=True)
    ap.add_argument("--only", default=None)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--reuse-product", action="store_true", help="g1: reuse product exports already in the work dir")
    ap.add_argument("--g10", default="matrix/cube25.classic.base,matrix/ring.arachne.angle45,"
                                      "matrix/two_objects.classic.base,cov/slot.classic.cov_fb_d1.25")
    ap.add_argument("--g1-work", default=None, help="g8: the G1 work dir holding <set>/<slice>/product/plate_1.gcode")
    args = ap.parse_args(argv)
    args.tr, args.orca = os.path.abspath(args.tr), os.path.abspath(args.orca)
    os.environ.pop("DIW_HARNESS_POWDER_MODE", None)   # slice_part: the slicer default (product) unless env says so
    total = Checks("run")
    for nm, p in (("exe.orca", args.orca), ("exe.translate", args.tr)):
        total.check(mode.upper(), nm, os.path.isfile(p), p)
    total.check(mode.upper(), "datadir", os.path.isdir(args.datadir), args.datadir)
    total.check(mode.upper(), "engine", os.path.isfile(os.path.join(args.engine, "diw_engine.py")), args.engine)
    sets = {}
    for s in args.sets:
        k, d = s.split("=", 1)
        if total.check(mode.upper(), f"set.{k}", os.path.isdir(d), d):
            sets[k] = d
    if total.fails:
        print(f"{total.n} checks, {len(total.fails)} failures")
        return 1
    if args.engine not in sys.path:
        sys.path.insert(0, args.engine)
    import diw_engine
    E = diw_engine
    os.makedirs(args.work, exist_ok=True)
    manifest = Manifest(os.path.join(args.work, "manifest.json"))
    g1_mask_selftest(total)
    t0 = time.time()
    jobs = s0_jobs(sets, args.only if mode != "g10" else None)
    infos = []
    if mode in ("g2", "g1d", "g8"):
        if mode == "g1d":
            # PRODUCT_DESIGN.md 8 G1 (d): the 13 classic.base matrix slices, the 16 stress slices and the coverage set
            jobs = [j for j in jobs if j["set"] in ("stress", "cov") or (j["set"] == "matrix" and j["name"].endswith(".classic.base"))]
            expect = {"matrix": 13, "stress": 16, "cov": 5}
        else:
            expect = {"matrix": 191, "stress": 16, "t5": 16, "cov": 5}
        if mode == "g8" and not total.check("G8", "g1_work", bool(args.g1_work) and os.path.isdir(args.g1_work), str(args.g1_work)):
            print(f"{total.n} checks, {len(total.fails)} failures")
            return 1
        for k in expect:
            n = sum(1 for j in jobs if j["set"] == k)
            total.check(mode.upper(), f"set.{k}.slices", args.only is not None or n == expect[k], f"{n} slice(s) (expected {expect[k]})")
        fn = {"g2": g2_slice, "g1d": g1d_slice, "g8": g8_slice}[mode]
        print(f"{mode}: {len(jobs)} slices, {args.jobs} workers", flush=True)
        for chk, info in run_parallel([(fn, args, manifest, j) for j in jobs], args.jobs):
            for l in chk.lines:
                print(l, flush=True)
            total.n += chk.n
            total.fails += chk.fails
            infos.append(info)
        manifest.save()
        for set_name in sorted({i.get("set") for i in infos if i.get("set")}):
            si = [i for i in infos if i.get("set") == set_name]
            if mode == "g2":
                d = f"identical {sum(1 for i in si if i.get('identical'))}, disagreement total {sum(i.get('disagreement', 0) for i in si)}"
            elif mode == "g1d":
                d = (f"st=ok blocks {sum(i.get('n_ok', 0) for i in si)}, translations aborted (same message) "
                     f"{sum(1 for i in si if i.get('translate_rc'))}, byte-identical before the mask "
                     f"{sum(1 for i in si if i.get('byte_equal'))}")
            else:
                d = f"summed removed_de {sum(i.get('removed_de', 0) for i in si):.3f}"
            total.check(mode.upper(), f"set.{set_name}.totals", True, f"{len(si)} slice(s), {d}")
    elif mode == "g11":
        # one slice per set (a preferred slice, else the first whose reference translation works) + the G5 refusals
        picked = []
        for k in sets:
            cands = [j for j in jobs if j["set"] == k]
            pref = G11_SET_PICK.get(k)
            cands.sort(key=lambda j: (j["name"] != pref, 0))
            picked.append((k, cands[:4]))
        for k, cands in picked:
            done = False
            for j in cands:
                chk, info = g11_slice(args, manifest, j)
                for l in chk.lines:
                    print(l, flush=True)
                total.n += chk.n
                total.fails += chk.fails
                infos.append(info)
                if not info.get("skipped"):
                    done = True
                    break
            total.check("G11", f"set.{k}.compared", done and any(i.get("ok") and i["name"].startswith(k + "/") for i in infos),
                        f"{sum(1 for i in infos if i['name'].startswith(k + '/') and i.get('ok'))} slice(s) compared")
        g11_refusals(args, total)
        manifest.save()
    elif mode == "g1":
        expect = {"matrix": 191, "stress": 16, "t5": 16, "cov": 5}
        for k in sets:
            n = sum(1 for j in jobs if j["set"] == k)
            total.check("G1", f"set.{k}.slices", args.only is not None or n == expect[k], f"{n} slice(s) (expected {expect[k]})")
        print(f"G1/G3: {len(jobs)} slices, {args.jobs} workers", flush=True)
        for chk, info in run_parallel([(g1_slice, args, manifest, j) for j in jobs], args.jobs):
            for l in chk.lines:
                print(l, flush=True)
            total.n += chk.n
            total.fails += chk.fails
            infos.append(info)
        manifest.save()
        for set_name in sorted({i.get("set") for i in infos if i.get("set")}):
            si = [i for i in infos if i.get("set") == set_name]
            total.check("G1", f"set.{set_name}.totals", True,
                        f"{len(si)} slice(s), st=ok blocks {sum(i.get('n_ok', 0) for i in si)}, suppressed chunks "
                        f"{sum(i.get('suppressed', 0) for i in si)}, removed_moves {sum(i.get('removed_moves', 0) for i in si)}, "
                        f"fallbacks {sum(i.get('fallback', 0) for i in si)}, translations aborted (same message) "
                        f"{sum(1 for i in si if i.get('translate_rc'))}, translations byte-identical after norm "
                        f"{sum(1 for i in si if i.get('translate_bytes_equal'))}")
        coverage_counters(total, "G1G3", [i for i in infos if i.get("set") == "cov"])
    else:
        by = {f"{j['set']}/{j['name']}": j for j in jobs}
        want = args.g10.split(",")
        missing = [w for w in want if w not in by]
        total.check("G10", "slices", not missing and len(want) == 4, f"{want}" + (f"; missing {missing}" if missing else ""))
        todo = [(g10_slice, args, manifest, by[w], m) for w in want if w in by for m in G10_MUTATIONS]
        print(f"G10: {len(todo)} runs, {args.jobs} workers", flush=True)
        for chk, info in run_parallel(todo, args.jobs):
            for l in chk.lines:
                print(l, flush=True)
            total.n += chk.n
            total.fails += chk.fails
            infos.append(info)
        manifest.save()
        print("\nG10 summary (slice / mutation: (a) fallback reasons; (b) G3 finding; G1 (c)):")
        for i in infos:
            if i.get("mut"):
                print(f"  {i['name']}: (a) {i.get('reasons_a')}; (b) G3: {i.get('g3_b', '')[:150]}; G1 fails: {i.get('g1_b_fails')}")
    # the manifest refuses a product file as an oracle input
    prods = [p for p, e in manifest.files.items() if e["mode"] == "product"]
    try:
        manifest.oracle_input(prods[0]) if prods else None
        refused = False
    except ValueError:
        refused = True
    total.check(mode.upper(), "manifest.refuses_product", bool(prods) and refused, f"{len(prods)} product file(s) registered")
    for l in total.lines:
        print(l)
    for f in total.fails[:60]:
        print("FAILED:", f[:500])
    print(f"{len(jobs)} S0 slices known, {mode} in {time.time() - t0:.0f} s")
    print(f"{total.n} checks, {len(total.fails)} failures")
    return 0 if not total.fails else 1


def main():
    global E
    argv = sys.argv[1:]
    if argv and argv[0] == "g5":
        return main_g5(argv[1:])
    if argv and argv[0] == "g4":
        return main_g4(argv[1:])
    if argv and argv[0] in ("g1", "g10", "g2", "g1d", "g8", "g11"):
        return main_g1(argv[1:], argv[0])
    if argv and argv[0] == "g7":
        return main_g7(argv[1:])
    if len(argv) < 5 or argv[0] != "g9":
        print(__doc__)
        return 2
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("mode")
    ap.add_argument("engine")
    ap.add_argument("tr")
    ap.add_argument("work")
    ap.add_argument("sets", nargs="+")
    ap.add_argument("--only", default=None)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--coverage-sets", default="cov")
    ap.add_argument("--no-selftest", action="store_true")
    args = ap.parse_args(argv)
    args.tr = os.path.abspath(args.tr)
    total = Checks("run")
    if not total.check("G9", "exe", os.path.isfile(args.tr), args.tr) or \
            not total.check("G9", "engine", os.path.isfile(os.path.join(args.engine, "diw_engine.py")), args.engine):
        print(f"{total.n} checks, {len(total.fails)} failures")
        return 1
    with open(args.tr, "rb") as f:
        exe = f.read()
    total.check("G9", "exe.flags", b"--product" in exe and b"--no-suppress" in exe, "--product / --no-suppress in the exe")
    if args.engine not in sys.path:
        sys.path.insert(0, args.engine)
    import diw_engine
    E = diw_engine
    os.makedirs(args.work, exist_ok=True)
    manifest = Manifest(os.path.join(args.work, "manifest.json"))
    jobs = []
    for s in args.sets:
        set_name, d = s.split("=", 1)
        if not total.check("G9", f"set.{set_name}", os.path.isdir(d), d):
            continue
        for n in sorted(os.listdir(d)):
            if args.only and args.only not in n:
                continue
            jobs.append((set_name, os.path.join(d, n)))
    t0 = time.time()
    infos = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as ex:
        futs = [ex.submit(g9_slice, args, manifest, s, d) for s, d in jobs]
        for f in futs:
            try:
                chk, info = f.result()
            except Exception as exc:   # a crash of the harness is a failure
                chk, info = Checks("crash"), {}
                chk.check("G9", "exception", False, repr(exc))
            for l in chk.lines:
                print(l, flush=True)
            total.n += chk.n
            total.fails += chk.fails
            infos.append(info)
    manifest.save()
    # the manifest refuses a product file as an oracle input
    prods = [p for p, e in manifest.files.items() if e["mode"] == "product"]
    if prods:
        try:
            manifest.oracle_input(prods[0])
            refused = False
        except ValueError:
            refused = True
        total.check("G9", "manifest.refuses_product", refused, f"{len(prods)} product file(s) registered")
    # per set
    for set_name in sorted({i.get("set") for i in infos if i.get("set")}):
        si = [i for i in infos if i.get("set") == set_name]
        total.check("G9", f"set.{set_name}.count", len(si) > 0,
                    f"{len(si)} slice(s), suppressed chunks {sum(int(i['summary']['suppressed']) for i in si if i.get('summary'))}, "
                    f"removed_moves {sum(int(i['summary']['removed_moves']) for i in si if i.get('summary'))}, "
                    f"fallbacks {sum(int(i['summary']['fallback']) for i in si if i.get('summary'))}, "
                    f"translations aborted (same message) {sum(1 for i in si if i.get('translate_rc'))}")
    # coverage counters
    cov = [i for i in infos if i.get("set") in args.coverage_sets.split(",")]
    if cov:
        fb = sum(i.get("fb_lines", 0) for i in cov)
        small = sum(i.get("small_blocks", 0) for i in cov)
        ed = {i["name"]: i["edited_with_fb"] for i in cov if i.get("edited_with_fb")}
        n_ed = sum(len(v) for v in ed.values())
        small_unedited = sum(i.get("small_blocks", 0) - len(i.get("small_changed", [])) for i in cov)
        total.check("COV", "fb_lines", fb > 0, f"fb_lines={fb}")
        total.check("COV", "small_blocks", small > 0, f"small_blocks={small}")
        total.check("COV", "edited_chunks_with_fb", n_ed >= 2 and len(ed) >= 2,
                    f"edited_chunks_with_fb={n_ed} on {len(ed)} slice(s): " + ", ".join(f"{k} {v}" for k, v in sorted(ed.items())))
        total.check("COV", "small_chunks_unedited", small_unedited == small, f"small_chunks_unedited={small_unedited} of {small}")
    else:
        total.check("COV", "coverage_set", False, f"no slice of set(s) {args.coverage_sets}")
    # the G3 checker itself: a mutated edit released without V0-V3 must fail it (one coverage FB slice)
    fbs = [i for i in cov if i.get("edited_with_fb")]
    if fbs and not args.no_selftest:
        sdir = fbs[0]["sdir"]
        for mut in ("drop_restore", "extra_delete", "wrong_g92"):
            w = os.path.join(args.work, "g3_selftest", mut)
            if os.path.isdir(w):
                shutil.rmtree(w)
            os.makedirs(w)
            d = os.path.join(w, "dump")
            shutil.copytree(os.path.join(sdir, "dump"), d)
            out = os.path.join(w, "product.gcode")
            keep_path = os.path.join(sdir, "keep", "plate_1.gcode")
            rc, err = run_tr(args.tr, ["--config-from-gcode", "--plan-records", "--product", keep_path, out],
                             {"ORCA_DIW_SHADOW_DUMP": d, "ORCA_DIW_TEST_SUPPRESS_MUTATE": mut, "ORCA_DIW_TEST_SKIP_VERIFY": "1"})
            manifest.register(out, "product", f"G3 self-test {mut}")
            keep = read(keep_path)
            sts = {k: v["st"] for k, v in blocks_of(keep).items()}
            probe = Checks(f"selftest.{mut}")
            if rc == 0:
                structural_check(probe, "G3", strip_placeholders(keep), read(out), sts, width_fallback(keep))
            total.check("G3", f"selftest.{mut}", rc == 0 and len(probe.fails) > 0,
                        f"rc={rc}; the structural check reports: " + (probe.fails[0][:220] if probe.fails else "nothing"))
        manifest.save()
    for l in total.lines:
        print(l)
    for f in total.fails[:40]:
        print("FAILED:", f)
    print(f"{len(jobs)} slices in {time.time() - t0:.0f} s")
    print(f"{total.n} checks, {len(total.fails)} failures")
    return 0 if not total.fails else 1


if __name__ == "__main__":
    sys.exit(main())
