#include "DIWPowderBlock.hpp"

#include "DIWPyCompat.hpp"

#include <array>
#include <charconv>
#include <climits>
#include <cmath>
#include <cstdio>
#include <set>
#include <stdexcept>

namespace Slic3r {
namespace DIW {

// ------------------------------------------------------------------------------------------------
// Hashes
// ------------------------------------------------------------------------------------------------

static const std::array<uint32_t, 256> &crc_table()
{
    static const std::array<uint32_t, 256> table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int b = 0; b < 8; ++b)
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            t[i] = c;
        }
        return t;
    }();
    return table;
}

uint32_t crc32_ieee_update(uint32_t crc, std::string_view data)
{
    const auto &t = crc_table();
    uint32_t    c = crc ^ 0xFFFFFFFFu;
    for (unsigned char ch : data)
        c = t[(c ^ ch) & 0xFFu] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

uint32_t crc32_ieee(std::string_view data) { return crc32_ieee_update(0, data); }

uint32_t fnv1a32(std::string_view data)
{
    uint32_t h = 0x811c9dc5u;
    for (unsigned char ch : data) {
        h ^= ch;
        h *= 0x01000193u;
    }
    return h;
}

std::string hex8(uint32_t v)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08x", unsigned(v));
    return buf;
}

std::string fingerprint_string(const UMexParams &U)
{
    std::string s;
    s += "v=" + std::to_string(RECORD_FORMAT_VERSION) + ";";
    s += "delta=" + py_repr(U.delta) + ";";
    s += "guard=" + py_repr(POWDER_GUARD_MM) + ";";
    s += "s_hatch=" + py_repr(U.s_hatch) + ";";
    s += "hatch_angle=" + py_repr(U.hatch_angle) + ";";
    s += "point_len=" + py_repr(U.point_len) + ";";
    s += "rho_tap=" + py_repr(U.rho_tap) + ";";
    s += "eta=" + py_repr(U.eta) + ";";
    s += "m_dot=" + py_repr(U.m_dot) + ";";
    s += "f_max=" + py_repr(U.f_max) + ";";
    s += "policy=" + U.policy + ";";
    s += "paste_min_area=" + py_repr(U.paste_min_area) + ";";
    s += "width_fallback=" + py_repr(U.width_fallback) + ";";
    s += "quad_segs=" + std::to_string(UMEX_QUAD_SEGS) + ";";
    return s;
}

uint32_t plan_fingerprint(const UMexParams &U) { return fnv1a32(fingerprint_string(U)); }

// ------------------------------------------------------------------------------------------------
// ptype encoding
// ------------------------------------------------------------------------------------------------

static bool kept_byte(unsigned char c)
{
    return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '-';
}

static bool printable(unsigned char c) { return c >= 0x20 && c <= 0x7E; }

std::string encode_ptype(std::string_view ptype)
{
    static const char hexd[] = "0123456789ABCDEF";
    std::string out;
    for (unsigned char c : ptype) {
        if (! printable(c))
            throw std::invalid_argument("DIW record: ptype is not printable ASCII");
        if (kept_byte(c))
            out += char(c);
        else if (c == ' ')
            out += '_';
        else {
            out += '%';
            out += hexd[c >> 4];
            out += hexd[c & 15];
        }
    }
    // The word "layer" never appears in a record line: every "layer" becomes "la%79er" (non-overlapping, left to right).
    std::string res;
    for (size_t i = 0; i < out.size();) {
        if (out.compare(i, 5, "layer") == 0) {
            res += "la%79er";
            i += 5;
        } else
            res += out[i++];
    }
    return res;
}

static int upper_hex_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

std::optional<std::string> decode_ptype(std::string_view tok)
{
    std::string out;
    for (size_t i = 0; i < tok.size(); ++i) {
        const unsigned char c = tok[i];
        if (kept_byte(c))
            out += char(c);
        else if (c == '_')
            out += ' ';
        else if (c == '%' && i + 2 < tok.size() && upper_hex_val(tok[i + 1]) >= 0 && upper_hex_val(tok[i + 2]) >= 0) {
            const int b = upper_hex_val(tok[i + 1]) * 16 + upper_hex_val(tok[i + 2]);
            if (! printable((unsigned char) b))
                return std::nullopt;
            out += char(b);
            i += 2;
        } else
            return std::nullopt;
    }
    if (encode_ptype(out) != tok)
        return std::nullopt;
    return out;
}

const char *status_name(PowderLayerRec::Status st)
{
    switch (st) {
    case PowderLayerRec::Status::ok: return "ok";
    case PowderLayerRec::Status::open: return "open";
    case PowderLayerRec::Status::small: return "small";
    }
    return "?";
}

// ------------------------------------------------------------------------------------------------
// Writer
// ------------------------------------------------------------------------------------------------

namespace {

constexpr size_t PAIRS_PER_LINE = 8;

double fin(double v)
{
    if (! std::isfinite(v))
        throw std::invalid_argument("DIW record: non-finite number");
    return v;
}

std::string r(double v) { return py_repr(fin(v)); }

void pair_lines(std::vector<std::string> &lines, const char *tag, const std::vector<RecPoint> &pairs)
{
    for (size_t i = 0; i < pairs.size(); i += PAIRS_PER_LINE) {
        std::string l = std::string(";DIW:") + tag;
        for (size_t j = i; j < pairs.size() && j < i + PAIRS_PER_LINE; ++j) {
            l += ' ';
            l += r(pairs[j].first);
            l += ' ';
            l += r(pairs[j].second);
        }
        lines.push_back(std::move(l));
    }
}

} // namespace

std::string write_block(const PowderLayerRec &rec)
{
    using St = PowderLayerRec::Status;
    if (rec.k < 1)
        throw std::invalid_argument("DIW record: k must be >= 1");
    if (rec.nc < 0)
        throw std::invalid_argument("DIW record: nc must be >= 0");
    if (rec.status != St::ok && (! rec.runs.empty() || ! rec.fallbacks.empty()))
        throw std::invalid_argument("DIW record: an open or small block carries no runs or fallback paths");
    if (rec.status == St::ok && rec.runs.empty())
        throw std::invalid_argument("DIW record: st=ok needs at least one run");
    if (rec.status == St::open && (rec.ca != 0.0 || rec.oa != 0.0 || rec.ra != 0.0 || rec.hl != 0.0 || rec.nc != 0))
        throw std::invalid_argument("DIW record: st=open needs an all-zero STAT");

    std::vector<std::string> lines;
    lines.push_back(";DIW:BEGIN v=" + std::to_string(RECORD_FORMAT_VERSION) + " k=" + std::to_string(rec.k) +
                    " st=" + status_name(rec.status) + " z=" + r(rec.z) + " h=" + r(rec.h) + " s=" + r(rec.s) +
                    " F=" + r(rec.F) + " fu=" + r(rec.fu) + " fp=" + hex8(rec.fp));
    if (rec.status == St::open)
        lines.push_back(";DIW:STAT ca=0.0 oa=0.0 ra=0.0 hl=0.0 nc=0");   // exact text (§3), never -0.0
    else
        lines.push_back(";DIW:STAT ca=" + r(rec.ca) + " oa=" + r(rec.oa) + " ra=" + r(rec.ra) + " hl=" + r(rec.hl) +
                        " nc=" + std::to_string(rec.nc));
    for (const PowderRunRec &run : rec.runs) {
        if (run.point) {
            if (run.pts.size() != 1)
                throw std::invalid_argument("DIW record: a point run has exactly one point");
            lines.push_back(";DIW:RUN t=point d=" + r(run.t_feed));
        } else {
            if (run.pts.size() < 2)
                throw std::invalid_argument("DIW record: a line run has at least two points");
            lines.push_back(";DIW:RUN t=line n=" + std::to_string(run.pts.size()));
        }
        pair_lines(lines, "XY", run.pts);
    }
    for (const FallbackPathRec &fb : rec.fallbacks) {
        const size_t n = fb.pts.size();
        if (n < 2 || fb.de.size() != n - 1 || fb.feed.size() != n - 1)
            throw std::invalid_argument("DIW record: a fallback path needs n >= 2 points and n - 1 (de, feed) pairs");
        lines.push_back(";DIW:FB n=" + std::to_string(n) + " w=" + r(fb.width) +
                        " tf=" + (fb.travel_feed ? r(*fb.travel_feed) : std::string("-")) + " z=" + r(fb.z) +
                        " ty=" + encode_ptype(fb.ptype));
        pair_lines(lines, "XY", fb.pts);
        std::vector<RecPoint> segs;
        segs.reserve(n - 1);
        for (size_t i = 0; i + 1 < n; ++i)
            segs.emplace_back(fb.de[i], fb.feed[i]);
        pair_lines(lines, "DE", segs);
    }

    std::string out;
    uint32_t    crc = 0;
    for (const std::string &l : lines) {
        out += l;
        out += '\n';
        crc = crc32_ieee_update(crc, l);
        crc = crc32_ieee_update(crc, "\n");
    }
    out += ";DIW:END n=" + std::to_string(lines.size() - 1) + " crc=" + hex8(crc) + "\n";
    return out;
}

// ------------------------------------------------------------------------------------------------
// Reader (POWDER_RECORDS.md section 4; mirrors diw/tests/v2_blocks.py parse_records step for step)
// ------------------------------------------------------------------------------------------------

namespace {

const char *const E_UNKNOWN_TAG  = "unknown tag";
const char *const E_UNKNOWN_KEY  = "unknown key";
const char *const E_KEY_ORDER    = "key order";
const char *const E_BAD_NUMBER   = "bad number";
const char *const E_COUNT        = "count mismatch";
const char *const E_CRC          = "crc mismatch";
const char *const E_NESTED       = "nested begin";
const char *const E_END_NO_BEGIN = "end without begin";
const char *const E_UNTERMINATED = "unterminated block";
const char *const E_DUP_K        = "duplicate k";
const char *const E_VERSION      = "unsupported version";
const char *const E_MISSING_KEY  = "missing key";
const char *const E_BAD_LINE     = "bad line";

struct RecError
{
    const char *reason;
};

[[noreturn]] void fail(const char *reason) { throw RecError{reason}; }

enum class Tag { BEGIN, STAT, RUN, FB, XY, DE, END };

bool tag_of(std::string_view t, Tag &tag)
{
    if (t == "BEGIN") tag = Tag::BEGIN;
    else if (t == "STAT") tag = Tag::STAT;
    else if (t == "RUN") tag = Tag::RUN;
    else if (t == "FB") tag = Tag::FB;
    else if (t == "XY") tag = Tag::XY;
    else if (t == "DE") tag = Tag::DE;
    else if (t == "END") tag = Tag::END;
    else return false;
    return true;
}

double parse_real(std::string_view s)
{
    double v = 0;
    const auto res = std::from_chars(s.data(), s.data() + s.size(), v, std::chars_format::general);
    if (s.empty() || res.ec != std::errc() || res.ptr != s.data() + s.size() || ! std::isfinite(v) || py_repr(v) != s)
        fail(E_BAD_NUMBER);
    return v;
}

int parse_int(std::string_view s)
{
    if (s.empty() || s.size() > 10 || (s.size() > 1 && s[0] == '0'))
        fail(E_BAD_NUMBER);
    long long v = 0;
    for (char c : s) {
        if (c < '0' || c > '9')
            fail(E_BAD_NUMBER);
        v = v * 10 + (c - '0');
    }
    if (v > INT_MAX)
        fail(E_BAD_NUMBER);
    return int(v);
}

uint32_t parse_hex(std::string_view s)
{
    if (s.size() != 8)
        fail(E_BAD_NUMBER);
    uint32_t v = 0;
    for (char c : s) {
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else fail(E_BAD_NUMBER);
        v = (v << 4) | uint32_t(d);
    }
    return v;
}

// Line checks, then the tag. A record line -> (tag, fields after the tag).
struct Lexed
{
    Tag                           tag = Tag::BEGIN;
    std::vector<std::string_view> fields;
};

Lexed lex(std::string_view line)
{
    for (unsigned char c : line)
        if (c < 0x20 || c > 0x7E)
            fail(E_BAD_LINE);
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '[' || c == ']' || c == '(' || c == ')')
            fail(E_BAD_LINE);
        if (i + 5 <= line.size()) {
            static const char word[] = "layer";
            bool m = true;
            for (size_t j = 0; j < 5 && m; ++j) {
                char ch = line[i + j];
                if (ch >= 'A' && ch <= 'Z') ch = char(ch - 'A' + 'a');
                m = ch == word[j];
            }
            if (m)
                fail(E_BAD_LINE);
        }
    }
    std::vector<std::string_view> toks;
    const std::string_view        body = line.substr(5);
    size_t                        pos  = 0;
    for (;;) {
        const size_t sp = body.find(' ', pos);
        toks.push_back(body.substr(pos, sp == std::string_view::npos ? std::string_view::npos : sp - pos));
        if (toks.back().empty())
            fail(E_BAD_LINE);
        if (sp == std::string_view::npos)
            break;
        pos = sp + 1;
    }
    Lexed lx;
    if (! tag_of(toks[0], lx.tag))
        fail(E_UNKNOWN_TAG);
    lx.fields.assign(toks.begin() + 1, toks.end());
    return lx;
}

enum class VK { Int, Real, Hex, St, T, RealDash, Ty, Version };
struct KeySpec
{
    const char *key;
    VK          kind;
};
struct Val
{
    double      d    = 0.0;
    int         i    = 0;
    uint32_t    h    = 0;
    bool        dash = false;
    std::string s;
};

const std::vector<KeySpec> SPEC_BEGIN = {{"v", VK::Version}, {"k", VK::Int},  {"st", VK::St},
                                         {"z", VK::Real},    {"h", VK::Real}, {"s", VK::Real},
                                         {"F", VK::Real},    {"fu", VK::Real}, {"fp", VK::Hex}};
const std::vector<KeySpec> SPEC_STAT  = {{"ca", VK::Real}, {"oa", VK::Real}, {"ra", VK::Real}, {"hl", VK::Real}, {"nc", VK::Int}};
const std::vector<KeySpec> SPEC_RUN   = {{"t", VK::T}};
const std::vector<KeySpec> SPEC_RUN_L = {{"t", VK::T}, {"n", VK::Int}};
const std::vector<KeySpec> SPEC_RUN_P = {{"t", VK::T}, {"d", VK::Real}};
const std::vector<KeySpec> SPEC_FB    = {{"n", VK::Int}, {"w", VK::Real}, {"tf", VK::RealDash}, {"z", VK::Real}, {"ty", VK::Ty}};
const std::vector<KeySpec> SPEC_END   = {{"n", VK::Int}, {"crc", VK::Hex}};

Val parse_val(VK kind, std::string_view v)
{
    Val out;
    switch (kind) {
    case VK::Int: out.i = parse_int(v); break;
    case VK::Real: out.d = parse_real(v); break;
    case VK::Hex: out.h = parse_hex(v); break;
    case VK::St:
        if (v != "ok" && v != "open" && v != "small")
            fail(E_BAD_NUMBER);
        out.s = std::string(v);
        break;
    case VK::T:
        if (v != "line" && v != "point")
            fail(E_BAD_NUMBER);
        out.s = std::string(v);
        break;
    case VK::RealDash:
        if (v == "-")
            out.dash = true;
        else
            out.d = parse_real(v);
        break;
    case VK::Ty: {
        auto ty = decode_ptype(v);
        if (! ty)
            fail(E_BAD_NUMBER);
        out.s = std::move(*ty);
        break;
    }
    case VK::Version:
        if (parse_int(v) != RECORD_FORMAT_VERSION)
            fail(E_VERSION);
        out.i = RECORD_FORMAT_VERSION;
        break;
    }
    return out;
}

bool key_known(Tag tag, std::string_view key)
{
    const std::vector<KeySpec> *lists[2] = {nullptr, nullptr};
    switch (tag) {
    case Tag::BEGIN: lists[0] = &SPEC_BEGIN; break;
    case Tag::STAT: lists[0] = &SPEC_STAT; break;
    case Tag::RUN: lists[0] = &SPEC_RUN_L; lists[1] = &SPEC_RUN_P; break;
    case Tag::FB: lists[0] = &SPEC_FB; break;
    case Tag::END: lists[0] = &SPEC_END; break;
    default: break;
    }
    for (const auto *l : lists)
        if (l)
            for (const KeySpec &k : *l)
                if (key == k.key)
                    return true;
    return false;
}

// key=value fields in the fixed order; first error wins, left to right, then "missing key".
std::vector<Val> keyed(Tag tag, const std::vector<std::string_view> &fields)
{
    const std::vector<KeySpec> *spec = nullptr;
    switch (tag) {
    case Tag::BEGIN: spec = &SPEC_BEGIN; break;
    case Tag::STAT: spec = &SPEC_STAT; break;
    case Tag::RUN: spec = &SPEC_RUN; break;
    case Tag::FB: spec = &SPEC_FB; break;
    case Tag::END: spec = &SPEC_END; break;
    default: fail(E_BAD_LINE);
    }
    std::vector<Val> vals;
    for (size_t i = 0; i < fields.size(); ++i) {
        const size_t eq = fields[i].find('=');
        if (eq == std::string_view::npos)
            fail(E_BAD_LINE);
        const std::string_view key = fields[i].substr(0, eq);
        if (! key_known(tag, key))
            fail(E_UNKNOWN_KEY);
        if (i >= spec->size() || key != (*spec)[i].key)
            fail(E_KEY_ORDER);
        vals.push_back(parse_val((*spec)[i].kind, fields[i].substr(eq + 1)));
        if (tag == Tag::RUN && key == "t")
            spec = vals.back().s == "line" ? &SPEC_RUN_L : &SPEC_RUN_P;
    }
    if (fields.size() < spec->size())
        fail(E_MISSING_KEY);
    return vals;
}

std::vector<RecPoint> parse_pairs(const std::vector<std::string_view> &fields)
{
    if (fields.size() % 2 != 0 || fields.size() < 2 || fields.size() > 2 * PAIRS_PER_LINE)
        fail(E_BAD_LINE);
    std::vector<double> v;
    v.reserve(fields.size());
    for (std::string_view f : fields)
        v.push_back(parse_real(f));
    std::vector<RecPoint> out;
    for (size_t i = 0; i < v.size(); i += 2)
        out.emplace_back(v[i], v[i + 1]);
    return out;
}

struct OpenBlock
{
    enum class Expect { Stat, Body, Xy, De };
    PowderLayerRec rec;
    uint32_t       crc       = 0;
    int            nlines    = 0;   // lines after BEGIN accepted so far
    Expect         expect    = Expect::Stat;
    size_t         remaining = 0;
    bool           cur_fb    = false;   // the item being filled is an FB (else a RUN)
    bool           seen_fb   = false;
};

// One record line inside an open block (not BEGIN / END).
void body_line(OpenBlock &b, Tag tag, const std::vector<std::string_view> &fields)
{
    using St = PowderLayerRec::Status;
    using Ex = OpenBlock::Expect;
    PowderLayerRec &r = b.rec;
    if (b.expect == Ex::Stat) {
        if (tag != Tag::STAT)
            fail(E_BAD_LINE);
        const auto v = keyed(tag, fields);
        if (r.status == St::open &&
            ! (fields.size() == 5 && fields[0] == "ca=0.0" && fields[1] == "oa=0.0" && fields[2] == "ra=0.0" &&
               fields[3] == "hl=0.0" && fields[4] == "nc=0"))
            fail(E_BAD_NUMBER);
        r.ca = v[0].d;
        r.oa = v[1].d;
        r.ra = v[2].d;
        r.hl = v[3].d;
        r.nc = v[4].i;
        b.expect = Ex::Body;
    } else if (b.expect == Ex::Body) {
        if (tag == Tag::XY || tag == Tag::DE)
            fail(E_COUNT);
        if (tag == Tag::STAT || r.status != St::ok)
            fail(E_BAD_LINE);
        if (tag == Tag::RUN) {
            if (b.seen_fb)
                fail(E_BAD_LINE);
            const auto   v = keyed(tag, fields);
            PowderRunRec run;
            if (v[0].s == "line") {
                if (v[1].i < 2)
                    fail(E_COUNT);
                b.remaining = size_t(v[1].i);
            } else {
                run.point   = true;
                run.t_feed  = v[1].d;
                b.remaining = 1;
            }
            r.runs.push_back(std::move(run));
            b.cur_fb = false;
        } else { // FB
            const auto v = keyed(tag, fields);
            if (v[0].i < 2)
                fail(E_COUNT);
            FallbackPathRec fb;
            fb.width = v[1].d;
            if (! v[2].dash)
                fb.travel_feed = v[2].d;
            fb.z     = v[3].d;
            fb.ptype = v[4].s;
            r.fallbacks.push_back(std::move(fb));
            b.remaining = size_t(v[0].i);
            b.cur_fb    = true;
            b.seen_fb   = true;
        }
        b.expect = Ex::Xy;
    } else {
        const Tag want = b.expect == Ex::Xy ? Tag::XY : Tag::DE;
        if (tag != want)
            fail(E_COUNT);
        const std::vector<RecPoint> pairs = parse_pairs(fields);
        if (pairs.size() > b.remaining)
            fail(E_COUNT);
        if (want == Tag::XY) {
            std::vector<RecPoint> &pts = b.cur_fb ? r.fallbacks.back().pts : r.runs.back().pts;
            pts.insert(pts.end(), pairs.begin(), pairs.end());
        } else {
            FallbackPathRec &fb = r.fallbacks.back();
            for (const RecPoint &p : pairs) {
                fb.de.push_back(p.first);
                fb.feed.push_back(p.second);
            }
        }
        b.remaining -= pairs.size();
        if (b.remaining == 0) {
            if (want == Tag::XY && b.cur_fb) {
                b.expect    = Ex::De;
                b.remaining = r.fallbacks.back().pts.size() - 1;
            } else
                b.expect = Ex::Body;
        }
    }
}

} // namespace

ParsedRecords parse_records(std::string_view g)
{
    using St = PowderLayerRec::Status;
    enum class State { Out, In, Skip };

    ParsedRecords out;
    State         state = State::Out;
    std::set<int> seen_k;   // k of accepted blocks
    OpenBlock     cur;

    auto begin = [&](int ln, std::string_view line, const std::vector<std::string_view> &fields) {
        try {
            const auto v = keyed(Tag::BEGIN, fields);
            cur          = OpenBlock();
            PowderLayerRec &r = cur.rec;
            r.k  = v[1].i;
            r.status = v[2].s == "ok" ? St::ok : v[2].s == "open" ? St::open : St::small;
            r.z  = v[3].d;
            r.h  = v[4].d;
            r.s  = v[5].d;
            r.F  = v[6].d;
            r.fu = v[7].d;
            r.fp = v[8].h;
            r.begin_line = ln;
            if (r.k < 1)
                fail(E_BAD_NUMBER);
            cur.crc = crc32_ieee_update(crc32_ieee_update(0, line), "\n");
            state   = State::In;
        } catch (const RecError &e) {
            out.errors.emplace_back(ln, e.reason);
            state = State::Skip;
        }
    };

    auto end = [&](int ln, const std::vector<std::string_view> &fields) {
        state = State::Out;
        try {
            const auto v = keyed(Tag::END, fields);
            if (v[0].i != cur.nlines)
                fail(E_COUNT);
            if (v[1].h != cur.crc)
                fail(E_CRC);
            if (cur.expect != OpenBlock::Expect::Body || (cur.rec.status == St::ok && cur.rec.runs.empty()))
                fail(E_COUNT);
        } catch (const RecError &e) {
            out.errors.emplace_back(ln, e.reason);
            return;
        }
        if (! seen_k.insert(cur.rec.k).second) {
            out.errors.emplace_back(cur.rec.begin_line, E_DUP_K);
            return;
        }
        cur.rec.end_line = ln;
        out.blocks.push_back(std::move(cur.rec));
    };

    size_t pos    = 0;
    int    lineno = 0;
    while (pos < g.size()) {
        const size_t     e    = g.find('\n', pos);
        std::string_view line = e == std::string_view::npos ? g.substr(pos) : g.substr(pos, e - pos);
        pos = e == std::string_view::npos ? g.size() : e + 1;
        ++lineno;
        if (! line.empty() && line.back() == '\r')
            line.remove_suffix(1);

        if (line.substr(0, 5) != ";DIW:") {
            if (state == State::In) {   // a block is contiguous: drop it, skip its remaining records
                out.errors.emplace_back(cur.rec.begin_line, E_UNTERMINATED);
                state = State::Skip;
            }
            continue;
        }
        Lexed lx;
        try {
            lx = lex(line);
        } catch (const RecError &err) {
            if (state != State::Skip) {
                out.errors.emplace_back(lineno, err.reason);
                state = State::Skip;
            }
            continue;
        }
        if (state == State::Skip) {
            if (lx.tag == Tag::END)
                state = State::Out;
            else if (lx.tag == Tag::BEGIN)
                begin(lineno, line, lx.fields);
            continue;
        }
        if (state == State::Out) {
            if (lx.tag == Tag::BEGIN)
                begin(lineno, line, lx.fields);
            else if (lx.tag == Tag::END)
                out.errors.emplace_back(lineno, E_END_NO_BEGIN);
            else {
                out.errors.emplace_back(lineno, E_BAD_LINE);
                state = State::Skip;
            }
            continue;
        }
        // State::In
        if (lx.tag == Tag::BEGIN) {
            out.errors.emplace_back(lineno, E_NESTED);
            begin(lineno, line, lx.fields);
        } else if (lx.tag == Tag::END)
            end(lineno, lx.fields);
        else {
            try {
                body_line(cur, lx.tag, lx.fields);
                cur.crc = crc32_ieee_update(crc32_ieee_update(cur.crc, line), "\n");
                ++cur.nlines;
            } catch (const RecError &err) {
                out.errors.emplace_back(lineno, err.reason);
                state = State::Skip;
            }
        }
    }
    if (state == State::In)
        out.errors.emplace_back(cur.rec.begin_line, E_UNTERMINATED);
    return out;
}

// ------------------------------------------------------------------------------------------------
// Dump
// ------------------------------------------------------------------------------------------------

std::string dump_records(const ParsedRecords &recs)
{
    std::string o;
    o += "blocks " + std::to_string(recs.blocks.size()) + "\n";
    auto pt = [&o](const RecPoint &p) { o += "  p " + py_repr(p.first) + " " + py_repr(p.second) + "\n"; };
    for (const PowderLayerRec &b : recs.blocks) {
        o += "block k=" + std::to_string(b.k) + " st=" + status_name(b.status) + " z=" + py_repr(b.z) +
             " h=" + py_repr(b.h) + " s=" + py_repr(b.s) + " F=" + py_repr(b.F) + " fu=" + py_repr(b.fu) +
             " fp=" + hex8(b.fp) + "\n";
        o += "stat ca=" + py_repr(b.ca) + " oa=" + py_repr(b.oa) + " ra=" + py_repr(b.ra) + " hl=" + py_repr(b.hl) +
             " nc=" + std::to_string(b.nc) + "\n";
        for (const PowderRunRec &r : b.runs) {
            if (r.point)
                o += "run point d=" + py_repr(r.t_feed) + "\n";
            else
                o += "run line " + std::to_string(r.pts.size()) + "\n";
            for (const RecPoint &p : r.pts)
                pt(p);
        }
        for (const FallbackPathRec &f : b.fallbacks) {
            o += "fb n=" + std::to_string(f.pts.size()) + " w=" + py_repr(f.width) +
                 " tf=" + (f.travel_feed ? py_repr(*f.travel_feed) : std::string("-")) + " z=" + py_repr(f.z) +
                 " ty=" + f.ptype + "\n";
            for (const RecPoint &p : f.pts)
                pt(p);
            for (size_t i = 0; i < f.de.size(); ++i)
                o += "  s " + py_repr(f.de[i]) + " " + py_repr(f.feed[i]) + "\n";
        }
    }
    o += "errors " + std::to_string(recs.errors.size()) + "\n";
    for (const auto &e : recs.errors)
        o += "error " + std::to_string(e.first) + " " + e.second + "\n";
    return o;
}

} // namespace DIW
} // namespace Slic3r
