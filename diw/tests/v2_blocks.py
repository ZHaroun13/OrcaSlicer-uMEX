"""Python reference of the `;DIW:` powder records, format v1 (diw/POWDER_RECORDS.md).

The C++ side is src/libslic3r/GCode/DIWPowderBlock.{hpp,cpp}; both must produce byte-identical blocks and dumps.

  crc32(data)                    CRC-32 (zlib)
  fnv1a32(data)                  FNV-1a 32-bit
  fingerprint_string(P, E)       the section 5 string for a v2 UMexParams P (E = the v2 diw_engine module)
  fingerprint(P, E)              its FNV-1a value (int)
  write_block(k, st, z, h, s, F, fu, fp, plan)   one block (str) from a v2 PowderPlan (None for st=open)
  write_rec(rec)                 one block (str) from a PowderLayerRec
  parse_records(text)            -> (blocks, errors); errors are (1-based line, reason) in the order found
  dump(blocks, errors)           the section 6 dump (str)

Only the standard library is used; nothing here imports v2 (E is passed in where v2 constants are needed).
"""
import math
import re
import zlib
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

FORMAT_VERSION = 1
MAX_PAIRS = 8
INT_MAX = 2 ** 31 - 1

# Error reasons (spec section 4, exact strings shared with the C++ reader).
E_UNKNOWN_TAG = "unknown tag"
E_UNKNOWN_KEY = "unknown key"
E_KEY_ORDER = "key order"
E_BAD_NUMBER = "bad number"
E_COUNT = "count mismatch"
E_CRC = "crc mismatch"
E_NESTED = "nested begin"
E_END_NO_BEGIN = "end without begin"
E_UNTERMINATED = "unterminated block"
E_DUP_K = "duplicate k"
E_VERSION = "unsupported version"
E_MISSING_KEY = "missing key"
E_BAD_LINE = "bad line"
ERROR_REASONS = (E_UNKNOWN_TAG, E_UNKNOWN_KEY, E_KEY_ORDER, E_BAD_NUMBER, E_COUNT, E_CRC, E_NESTED,
                 E_END_NO_BEGIN, E_UNTERMINATED, E_DUP_K, E_VERSION, E_MISSING_KEY, E_BAD_LINE)

PREFIX = ";DIW:"
TAGS = ("BEGIN", "STAT", "RUN", "FB", "XY", "DE", "END")


# ----------------------------------------------------------------------------- data

@dataclass
class RunRec:
    kind: str                                   # "line" | "point"
    pts: List[Tuple[float, float]]
    d: float = 0.0                              # point dwell feed time (s); 0.0 for a line run


@dataclass
class FbRec:
    w: float
    tf: Optional[float]
    z: float
    ty: str                                     # decoded ptype
    pts: List[Tuple[float, float]]
    segs: List[Tuple[float, float]]             # (de, feed) per segment, len(pts) - 1


@dataclass
class PowderLayerRec:
    k: int
    st: str                                     # "ok" | "open" | "small"
    z: float
    h: float
    s: float
    F: float
    fu: float
    fp: int
    ca: float = 0.0
    oa: float = 0.0
    ra: float = 0.0
    hl: float = 0.0
    nc: int = 0
    runs: List[RunRec] = field(default_factory=list)
    fbs: List[FbRec] = field(default_factory=list)
    line: int = 0                               # 1-based line of BEGIN when parsed; 0 when built


# ----------------------------------------------------------------------------- hashes

def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def fnv1a32(data: bytes) -> int:
    h = 0x811C9DC5
    for b in data:
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


def R(v) -> str:
    """py_repr of a finite real (numpy scalars are converted first)."""
    v = float(v)
    if not math.isfinite(v):
        raise ValueError(f"non-finite value {v!r} cannot be written to a record")
    return repr(v)


def fingerprint_string(P, E) -> str:
    return (f"v={FORMAT_VERSION};delta={R(P.delta)};guard={R(E.POWDER_GUARD_MM)};s_hatch={R(P.s_hatch)};"
            f"hatch_angle={R(P.hatch_angle)};point_len={R(P.point_len)};rho_tap={R(P.rho_tap)};eta={R(P.eta)};"
            f"m_dot={R(P.m_dot)};f_max={R(P.f_max)};policy={P.policy};paste_min_area={R(P.paste_min_area)};"
            f"width_fallback={R(P.width_fallback)};quad_segs={int(E.UMEX_QUAD_SEGS)};")


def fingerprint(P, E) -> int:
    return fnv1a32(fingerprint_string(P, E).encode("ascii"))


# ----------------------------------------------------------------------------- ty encoding

_TY_KEEP = frozenset(b"abcdefghijklmnopqrstuvwxyz0123456789.-")


def encode_ty(ptype: str) -> str:
    """Bytes in [a-z0-9.-] kept, space -> '_', any other byte -> %XX (upper-case hex); then every 'layer'
    becomes 'la%79er' so the word never appears. The ptype must be printable ASCII (0x20-0x7E)."""
    raw = ptype.encode("ascii")
    if any(b < 0x20 or b > 0x7E for b in raw):
        raise ValueError(f"ptype {ptype!r} is not printable ASCII")
    out = []
    for b in raw:
        if b in _TY_KEEP:
            out.append(chr(b))
        elif b == 0x20:
            out.append("_")
        else:
            out.append(f"%{b:02X}")
    return "".join(out).replace("layer", "la%79er")


_RE_TY = re.compile(r"(?:[a-z0-9.\-_]|%[0-9A-F]{2})*")


def decode_ty(tok: str) -> Optional[str]:
    """Inverse of encode_ty; None unless tok is the canonical encoding of a printable-ASCII string."""
    if not _RE_TY.fullmatch(tok):
        return None
    out, i = [], 0
    while i < len(tok):
        c = tok[i]
        if c == "%":
            b = int(tok[i + 1:i + 3], 16)
            if b < 0x20 or b > 0x7E:
                return None
            out.append(chr(b))
            i += 3
        else:
            out.append(" " if c == "_" else c)
            i += 1
    s = "".join(out)
    return s if encode_ty(s) == tok else None


# ----------------------------------------------------------------------------- writer

def _pair_lines(tag: str, pairs) -> List[str]:
    lines = []
    for i in range(0, len(pairs), MAX_PAIRS):
        chunk = pairs[i:i + MAX_PAIRS]
        lines.append(PREFIX + tag + " " + " ".join(f"{R(a)} {R(b)}" for a, b in chunk))
    return lines


def write_rec(r: PowderLayerRec) -> str:
    if r.st not in ("ok", "open", "small"):
        raise ValueError(f"bad st {r.st!r}")
    if r.st == "ok" and not r.runs:
        raise ValueError("st=ok needs at least one run")
    if r.st != "ok" and (r.runs or r.fbs):
        raise ValueError(f"st={r.st} carries no RUN/FB records")
    L = [f"{PREFIX}BEGIN v={FORMAT_VERSION} k={int(r.k)} st={r.st} z={R(r.z)} h={R(r.h)} s={R(r.s)} "
         f"F={R(r.F)} fu={R(r.fu)} fp={int(r.fp):08x}"]
    if r.st == "open":
        L.append(f"{PREFIX}STAT ca=0.0 oa=0.0 ra=0.0 hl=0.0 nc=0")
    else:
        L.append(f"{PREFIX}STAT ca={R(r.ca)} oa={R(r.oa)} ra={R(r.ra)} hl={R(r.hl)} nc={int(r.nc)}")
    for run in r.runs:
        if run.kind == "line":
            if len(run.pts) < 2:
                raise ValueError("line run needs >= 2 points")
            L.append(f"{PREFIX}RUN t=line n={len(run.pts)}")
        elif run.kind == "point":
            if len(run.pts) != 1:
                raise ValueError("point run needs exactly 1 point")
            L.append(f"{PREFIX}RUN t=point d={R(run.d)}")
        else:
            raise ValueError(f"bad run kind {run.kind!r}")
        L.extend(_pair_lines("XY", run.pts))
    for fb in r.fbs:
        if len(fb.pts) < 2 or len(fb.segs) != len(fb.pts) - 1:
            raise ValueError("fallback path needs n >= 2 points and n-1 segments")
        tf = "-" if fb.tf is None else R(fb.tf)
        L.append(f"{PREFIX}FB n={len(fb.pts)} w={R(fb.w)} tf={tf} z={R(fb.z)} ty={encode_ty(fb.ty)}")
        L.extend(_pair_lines("XY", fb.pts))
        L.extend(_pair_lines("DE", fb.segs))
    body = "".join(l + "\n" for l in L)
    L.append(f"{PREFIX}END n={len(L) - 1} crc={crc32(body.encode('ascii')):08x}")
    return "".join(l + "\n" for l in L)


def rec_from_plan(k, st, z, h, s, F, fu, fp, plan) -> PowderLayerRec:
    """PowderLayerRec from a v2 PowderPlan (duck-typed; None for st=open)."""
    fp = int(fp, 16) if isinstance(fp, str) else int(fp)
    r = PowderLayerRec(k=int(k), st=st, z=float(z), h=float(h), s=float(s), F=float(F), fu=float(fu), fp=fp)
    if st == "open" or plan is None:
        if st != "open" or plan is not None:
            raise ValueError("st=open exactly when plan is None")
        return r
    r.ca, r.oa, r.ra, r.hl = float(plan.cavity_area), float(plan.omega_area), float(plan.revert_area), float(plan.hatch_length)
    r.nc = int(plan.n_components)
    if st == "small":
        return r
    for run in plan.runs:
        pts = [(float(x), float(y)) for x, y in run.pts]
        r.runs.append(RunRec(run.kind, pts, float(run.t_feed) if run.kind == "point" else 0.0))
    for p in plan.fallback_paths:
        r.fbs.append(FbRec(w=float(p.width), tf=None if p.travel_feed is None else float(p.travel_feed),
                           z=float(p.z), ty=p.ptype, pts=[(float(x), float(y)) for x, y in p.points],
                           segs=[(float(a), float(b)) for a, b in zip(p.seg_de, p.seg_feed)]))
    return r


def write_block(k, st, z, h, s, F, fu, fp, plan) -> str:
    return write_rec(rec_from_plan(k, st, z, h, s, F, fu, fp, plan))


# ----------------------------------------------------------------------------- reader

class _Err(Exception):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


_RE_INT = re.compile(r"0|[1-9][0-9]*")
_RE_HEX = re.compile(r"[0-9a-f]{8}")


def _p_int(v):
    if not _RE_INT.fullmatch(v) or len(v) > 10 or int(v) > INT_MAX:
        raise _Err(E_BAD_NUMBER)
    return int(v)


def _p_real(v):
    try:
        x = float(v)
    except ValueError:
        raise _Err(E_BAD_NUMBER)
    if not math.isfinite(x) or repr(x) != v:
        raise _Err(E_BAD_NUMBER)
    return x


def _p_hex(v):
    if not _RE_HEX.fullmatch(v):
        raise _Err(E_BAD_NUMBER)
    return int(v, 16)


def _p_enum(*allowed):
    def p(v):
        if v not in allowed:
            raise _Err(E_BAD_NUMBER)
        return v
    return p


def _p_realdash(v):
    return None if v == "-" else _p_real(v)


def _p_ty(v):
    s = decode_ty(v)
    if s is None:
        raise _Err(E_BAD_NUMBER)
    return s


def _p_version(v):
    if _p_int(v) != FORMAT_VERSION:
        raise _Err(E_VERSION)
    return FORMAT_VERSION


_SPEC = {
    "BEGIN": [("v", _p_version), ("k", _p_int), ("st", _p_enum("ok", "open", "small")), ("z", _p_real),
              ("h", _p_real), ("s", _p_real), ("F", _p_real), ("fu", _p_real), ("fp", _p_hex)],
    "STAT": [("ca", _p_real), ("oa", _p_real), ("ra", _p_real), ("hl", _p_real), ("nc", _p_int)],
    "RUN": [("t", _p_enum("line", "point"))],
    "RUN line": [("t", _p_enum("line", "point")), ("n", _p_int)],
    "RUN point": [("t", _p_enum("line", "point")), ("d", _p_real)],
    "FB": [("n", _p_int), ("w", _p_real), ("tf", _p_realdash), ("z", _p_real), ("ty", _p_ty)],
    "END": [("n", _p_int), ("crc", _p_hex)],
}
_ALL_KEYS = {"BEGIN": {"v", "k", "st", "z", "h", "s", "F", "fu", "fp"}, "STAT": {"ca", "oa", "ra", "hl", "nc"},
             "RUN": {"t", "n", "d"}, "FB": {"n", "w", "tf", "z", "ty"}, "END": {"n", "crc"}}


def _lex(raw: bytes):
    """Line-level checks of a record line (raw starts with ;DIW:, no line ending). -> (tag, text, tokens)."""
    if any(b < 0x20 or b > 0x7E for b in raw):
        raise _Err(E_BAD_LINE)
    s = raw.decode("ascii")
    if any(c in s for c in "[]()") or "layer" in s.lower():
        raise _Err(E_BAD_LINE)
    toks = s[len(PREFIX):].split(" ")
    if any(t == "" for t in toks):
        raise _Err(E_BAD_LINE)
    if toks[0] not in TAGS:
        raise _Err(E_UNKNOWN_TAG)
    return toks[0], s, toks[1:]


def _keyed(tag, toks):
    """Parses key=value fields in the fixed order; first error wins, left to right."""
    spec = _SPEC[tag]
    out = {}
    for i, t in enumerate(toks):
        if "=" not in t:
            raise _Err(E_BAD_LINE)
        key, val = t.split("=", 1)
        if key not in _ALL_KEYS[tag]:
            raise _Err(E_UNKNOWN_KEY)
        if i >= len(spec) or key != spec[i][0]:
            raise _Err(E_KEY_ORDER)
        out[key] = spec[i][1](val)
        if tag == "RUN" and key == "t":
            spec = _SPEC["RUN " + out["t"]]
    if len(toks) < len(spec):
        raise _Err(E_MISSING_KEY)
    return out


def _pairs(toks):
    if len(toks) % 2 or not 2 <= len(toks) <= 2 * MAX_PAIRS:
        raise _Err(E_BAD_LINE)
    v = [_p_real(t) for t in toks]
    return list(zip(v[0::2], v[1::2]))


class _Block:
    def __init__(self, line, f, text):
        self.rec = PowderLayerRec(k=f["k"], st=f["st"], z=f["z"], h=f["h"], s=f["s"], F=f["F"], fu=f["fu"],
                                  fp=f["fp"], line=line)
        self.lines = [text]
        self.expect = "stat"     # stat | body | xy | de
        self.remaining = 0
        self.cur = None          # RunRec or FbRec being filled
        self.seen_fb = False


def _split_lines(data: bytes):
    pieces = data.split(b"\n")
    if pieces and pieces[-1] == b"":
        pieces.pop()
    for i, p in enumerate(pieces, 1):
        yield i, (p[:-1] if p.endswith(b"\r") else p)


def _body_line(b: _Block, tag, toks):
    """One record line inside an open block (not BEGIN / END). Raises _Err."""
    r = b.rec
    if b.expect == "stat":
        if tag != "STAT":
            raise _Err(E_BAD_LINE)
        f = _keyed(tag, toks)
        if r.st == "open" and (toks != ["ca=0.0", "oa=0.0", "ra=0.0", "hl=0.0", "nc=0"]):
            raise _Err(E_BAD_NUMBER)
        r.ca, r.oa, r.ra, r.hl, r.nc = f["ca"], f["oa"], f["ra"], f["hl"], f["nc"]
        b.expect = "body"
    elif b.expect == "body":
        if tag in ("XY", "DE"):
            raise _Err(E_COUNT)
        if tag == "STAT" or r.st != "ok":
            raise _Err(E_BAD_LINE)
        if tag == "RUN":
            if b.seen_fb:
                raise _Err(E_BAD_LINE)
            f = _keyed(tag, toks)
            if f["t"] == "line":
                if f["n"] < 2:
                    raise _Err(E_COUNT)
                b.cur, b.remaining = RunRec("line", []), f["n"]
            else:
                b.cur, b.remaining = RunRec("point", [], f["d"]), 1
            r.runs.append(b.cur)
        else:  # FB
            f = _keyed(tag, toks)
            if f["n"] < 2:
                raise _Err(E_COUNT)
            b.cur, b.remaining = FbRec(w=f["w"], tf=f["tf"], z=f["z"], ty=f["ty"], pts=[], segs=[]), f["n"]
            b.seen_fb = True
            r.fbs.append(b.cur)
        b.expect = "xy"
    else:
        want = "XY" if b.expect == "xy" else "DE"
        if tag != want:
            raise _Err(E_COUNT)
        pairs = _pairs(toks)
        if len(pairs) > b.remaining:
            raise _Err(E_COUNT)
        if want == "XY":
            b.cur.pts.extend(pairs)
        else:
            b.cur.segs.extend(pairs)
        b.remaining -= len(pairs)
        if b.remaining == 0:
            if want == "XY" and isinstance(b.cur, FbRec):
                b.expect, b.remaining = "de", len(b.cur.pts) - 1
            else:
                b.expect, b.cur = "body", None


def parse_records(data):
    """Reads every block of a text (bytes or str; a whole G-code file is fine).
    Returns (blocks, errors): accepted PowderLayerRec in file order, and (line, reason) in the order found."""
    if isinstance(data, str):
        data = data.encode("utf-8")
    blocks: List[PowderLayerRec] = []
    errors: List[Tuple[int, str]] = []
    seen_k = set()
    state, cur = "out", None          # out | in | skip

    def begin(ln, text, toks):
        nonlocal state, cur
        try:
            cur = _Block(ln, _keyed("BEGIN", toks), text)
            state = "in"
            if cur.rec.k < 1:
                raise _Err(E_BAD_NUMBER)
        except _Err as e:
            errors.append((ln, e.reason))
            state, cur = "skip", None

    def end(ln, toks):
        nonlocal state, cur
        b, state, cur = cur, "out", None
        try:
            f = _keyed("END", toks)
            if f["n"] != len(b.lines) - 1:
                raise _Err(E_COUNT)
            if crc32("".join(l + "\n" for l in b.lines).encode("ascii")) != f["crc"]:
                raise _Err(E_CRC)
            if b.expect != "body" or (b.rec.st == "ok" and not b.rec.runs):
                raise _Err(E_COUNT)
        except _Err as e:
            errors.append((ln, e.reason))
            return
        if b.rec.k in seen_k:
            errors.append((b.rec.line, E_DUP_K))
            return
        seen_k.add(b.rec.k)
        blocks.append(b.rec)

    for ln, raw in _split_lines(data):
        if not raw.startswith(PREFIX.encode("ascii")):
            if state == "in":                 # a block is contiguous: drop it, skip its remaining records
                errors.append((cur.rec.line, E_UNTERMINATED))
                state, cur = "skip", None
            continue
        try:
            tag, text, toks = _lex(raw)
        except _Err as e:
            if state != "skip":
                errors.append((ln, e.reason))
                state, cur = "skip", None
            continue
        if state == "skip":
            if tag == "END":
                state = "out"
            elif tag == "BEGIN":
                begin(ln, text, toks)
            continue
        if state == "out":
            if tag == "BEGIN":
                begin(ln, text, toks)
            elif tag == "END":
                errors.append((ln, E_END_NO_BEGIN))
            else:
                errors.append((ln, E_BAD_LINE))
                state = "skip"
            continue
        # state == "in"
        if tag == "BEGIN":
            errors.append((ln, E_NESTED))
            begin(ln, text, toks)
        elif tag == "END":
            end(ln, toks)
        else:
            try:
                _body_line(cur, tag, toks)
                cur.lines.append(text)
            except _Err as e:
                errors.append((ln, e.reason))
                state, cur = "skip", None
    if state == "in":
        errors.append((cur.rec.line, E_UNTERMINATED))
    return blocks, errors


# ----------------------------------------------------------------------------- dump

def dump(blocks, errors) -> str:
    o = [f"blocks {len(blocks)}"]
    for b in blocks:
        o.append(f"block k={b.k} st={b.st} z={R(b.z)} h={R(b.h)} s={R(b.s)} F={R(b.F)} fu={R(b.fu)} fp={b.fp:08x}")
        o.append(f"stat ca={R(b.ca)} oa={R(b.oa)} ra={R(b.ra)} hl={R(b.hl)} nc={b.nc}")
        for r in b.runs:
            o.append(f"run line {len(r.pts)}" if r.kind == "line" else f"run point d={R(r.d)}")
            o.extend(f"  p {R(x)} {R(y)}" for x, y in r.pts)
        for f in b.fbs:
            tf = "-" if f.tf is None else R(f.tf)
            o.append(f"fb n={len(f.pts)} w={R(f.w)} tf={tf} z={R(f.z)} ty={f.ty}")
            o.extend(f"  p {R(x)} {R(y)}" for x, y in f.pts)
            o.extend(f"  s {R(a)} {R(c)}" for a, c in f.segs)
    o.append(f"errors {len(errors)}")
    o.extend(f"error {ln} {reason}" for ln, reason in errors)
    return "".join(l + "\n" for l in o)


def strip_records(data: bytes) -> bytes:
    """The input with every line that starts with ;DIW: removed (as the translator does)."""
    pieces = data.split(b"\n")
    tail = pieces.pop()               # text after the last \n (usually empty)
    kept = [p + b"\n" for p in pieces if not p.startswith(PREFIX.encode("ascii"))]
    if not tail.startswith(PREFIX.encode("ascii")):
        kept.append(tail)
    return b"".join(kept)
