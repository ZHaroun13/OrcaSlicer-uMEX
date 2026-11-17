"""Hooks into the v2 post-processor for the `;DIW:` powder records (diw/POWDER_RECORDS.md).

  capture(E, cfg, src_path, out_path)  run v2 in real powder mode and record, for every core layer that reaches
                                       plan_powder_layer, its plan as a block. Returns
                                       (records_gcode: bytes, layers: [CapturedLayer], out_path) where
                                       records_gcode is the source with each block inserted right after its
                                       layer's start-marker line, and out_path holds v2's own (unpatched-
                                       equivalent) output.
  replay(E, cfg, src_path, records, out_path)
                                       run v2 on src_path (the file WITHOUT records) with plan_powder_layer and
                                       resolve_powder_feed answered from the parsed records, as the C++
                                       sequencer will be driven. Returns (out_path, problems); problems lists
                                       every mismatch between v2's own z / h / s / F / fu and the records, and
                                       every layer v2 planned that has no block.

E is the imported v2 diw_engine module; cfg a v2 config dict with enable_u_mex_powder_infill set.
Every patch is undone in a finally block. v2 itself is never modified on disk.
"""
import contextlib
from dataclasses import dataclass
from typing import Any, List, Optional

import v2_blocks as B


@dataclass
class CapturedLayer:
    k: int
    start: int                  # 0-based index of the layer's start-marker line (LayerChunk.start)
    z: float
    h: float
    s: float
    F: float
    fu: float
    st: str
    plan: Any                   # v2 PowderPlan or None
    block: str                  # the written block


def _owner(cls, name):
    for c in cls.__mro__:
        if name in c.__dict__:
            return c
    raise AttributeError(name)


@contextlib.contextmanager
def _patched(E, convert_wrap, feed_fn, plan_fn):
    owner = _owner(E.DIWPostProcessor, "_convert_core_layer")
    orig_convert = owner.__dict__["_convert_core_layer"]
    orig_feed, orig_plan = E.resolve_powder_feed, E.plan_powder_layer
    try:
        owner._convert_core_layer = convert_wrap(orig_convert)
        E.resolve_powder_feed = feed_fn(orig_feed)
        E.plan_powder_layer = plan_fn(orig_plan)
        yield
    finally:
        owner._convert_core_layer = orig_convert
        E.resolve_powder_feed, E.plan_powder_layer = orig_feed, orig_plan


def _params(E, cfg):
    return E.UMexParams.from_config(cfg)[0]


def capture(E, cfg, src_path, out_path):
    fp = B.fingerprint(_params(E, cfg), E)
    layers: List[CapturedLayer] = []
    ctx = {}

    def convert_wrap(orig):
        def conv(self, raw_lines, infos, chunk, *a, **kw):
            ctx.clear()
            ctx["chunk"] = chunk
            try:
                return orig(self, raw_lines, infos, chunk, *a, **kw)
            finally:
                ctx.clear()
        return conv

    def feed_fn(orig):
        def feed(*a, **kw):
            res = orig(*a, **kw)
            ctx["feed"] = res
            return res
        return feed

    def plan_fn(orig):
        def plan(model, prm, h, s_eff, sg):
            p = orig(model, prm, h, s_eff, sg)
            chunk = ctx["chunk"]
            F, s2, fu, _ = ctx["feed"]
            assert s2 == s_eff
            st = "open" if p is None else ("small" if not p.runs else "ok")
            blk = B.write_block(chunk.index, st, model.z, h, s_eff, F, fu, fp, p)
            layers.append(CapturedLayer(chunk.index, chunk.start, model.z, h, s_eff, F, fu, st, p, blk))
            return p
        return plan

    with _patched(E, convert_wrap, feed_fn, plan_fn):
        E.DIWPostProcessor(cfg).process_file(src_path, out_path)

    with open(src_path, "rb") as fh:
        src = fh.read()
    lines = src.splitlines(keepends=True)     # same split as v2's text-mode readlines()
    after = {}
    for L in layers:
        assert L.start not in after and L.start < len(lines)
        after[L.start] = L.block.encode("ascii")
    out = []
    for i, l in enumerate(lines):
        out.append(l)
        if i in after:
            if not l.endswith((b"\n", b"\r")):
                out.append(b"\n")
            out.append(after[i])
    return b"".join(out), layers, out_path


def plan_from_rec(E, rec: B.PowderLayerRec):
    """A v2 PowderPlan rebuilt from a parsed block (None for st=open)."""
    if rec.st == "open":
        return None
    plan = E.PowderPlan(runs=[], fallback_paths=[], cavity_area=rec.ca, omega_area=rec.oa, revert_area=rec.ra,
                        hatch_length=rec.hl, s_eff=rec.s, n_components=rec.nc)
    for r in rec.runs:
        plan.runs.append(E.PowderRun(list(r.pts), r.kind, r.d))
    for f in rec.fbs:
        plan.fallback_paths.append(E.PathRec(ptype=f.ty, kind="paste", points=list(f.pts),
                                             seg_de=[a for a, _ in f.segs], seg_feed=[b for _, b in f.segs],
                                             seg_raw=[], width=f.w, z=f.z, pre_lines=[], travel_feed=f.tf))
    return plan


def replay(E, cfg, src_path, records, out_path):
    blocks, errors = B.parse_records(records)
    problems = [f"record error line {ln}: {reason}" for ln, reason in errors]
    fp = B.fingerprint(_params(E, cfg), E)
    by_k = {b.k: b for b in blocks}
    ctx = {}

    def convert_wrap(orig):
        def conv(self, raw_lines, infos, chunk, *a, **kw):
            ctx.clear()
            ctx["k"] = chunk.index
            try:
                return orig(self, raw_lines, infos, chunk, *a, **kw)
            finally:
                ctx.clear()
        return conv

    def feed_fn(orig):
        def feed(*a, **kw):
            res = orig(*a, **kw)
            rec = by_k.get(ctx.get("k"))
            if rec is None:
                return res              # plan_powder_layer reports the missing block
            if (res[0], res[1], res[2]) != (rec.F, rec.s, rec.fu):
                problems.append(f"layer {rec.k}: v2 feed {res[:3]} != record F/s/fu {(rec.F, rec.s, rec.fu)}")
            return rec.F, rec.s, rec.fu, rec.fu != rec.F
        return feed

    def plan_fn(orig):
        def plan(model, prm, h, s_eff, sg):
            k = ctx.get("k")
            rec = by_k.get(k)
            if rec is None:
                problems.append(f"layer {k}: planned by v2 but no record block")
                raise RuntimeError(f"no record block for layer {k}")
            if (model.z, h, s_eff) != (rec.z, rec.h, rec.s):
                problems.append(f"layer {k}: v2 z/h/s {(model.z, h, s_eff)} != record {(rec.z, rec.h, rec.s)}")
            if rec.fp != fp:
                problems.append(f"layer {k}: fingerprint {rec.fp:08x} != settings {fp:08x}")
            rec_used.add(k)
            return plan_from_rec(E, rec)
        return plan

    rec_used = set()
    with _patched(E, convert_wrap, feed_fn, plan_fn):
        E.DIWPostProcessor(cfg).process_file(src_path, out_path)
    for k in sorted(set(by_k) - rec_used):
        problems.append(f"layer {k}: record block never used")
    return out_path, problems
