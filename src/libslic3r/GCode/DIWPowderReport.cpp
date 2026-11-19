#include "DIWPowderReport.hpp"
#include "DIWUMexSequencer.hpp"

#include "../Exception.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string_view>

namespace Slic3r {
namespace DIW {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::string_view kEstimateMarker = "; DIW estimate includes added time = ";

std::string fixed(double v, int prec)
{
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::fixed, prec);
    return std::string(buf, r.ptr);
}

bool parse_number(std::string_view s, double &out)
{
    if (s.empty())
        return false;
    if (s.front() == '+')
        s.remove_prefix(1);
    const auto r = std::from_chars(s.data(), s.data() + s.size(), out);
    return r.ec == std::errc() && r.ptr == s.data() + s.size();
}

std::string trim(std::string_view s)
{
    size_t b = 0, e = s.size();
    while (b < e && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r'))
        ++b;
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r'))
        --e;
    return std::string(s.substr(b, e - b));
}

// Which stage the translator's own header comments put us in ("; --- uMEX L<k> S<n> ...").
enum class Ctx { None, Paste, Relief, Wipe, Uv, Powder, Compaction };

// Parses "; --- uMEX L<k> S<n>[ end...]"; false when the line is not a stage header.
bool stage_header(const std::string &line, Ctx &ctx)
{
    static const std::string_view pre = "; --- uMEX L";
    if (line.compare(0, pre.size(), pre) != 0)
        return false;
    size_t i = pre.size();
    const size_t k0 = i;
    while (i < line.size() && std::isdigit(uint8_t(line[i])))
        ++i;
    if (i == k0 || line.compare(i, 2, " S") != 0 || i + 2 >= line.size() || ! std::isdigit(uint8_t(line[i + 2])))
        return false;
    const int n = line[i + 2] - '0';
    const bool end = line.compare(i + 3, 4, " end") == 0;
    switch (n) {
    case 1:  ctx = end ? Ctx::Relief : Ctx::Paste; break;
    case 2:  ctx = Ctx::Wipe; break;
    case 3:  ctx = Ctx::Uv; break;
    case 4:  ctx = Ctx::Powder; break;
    case 5:  ctx = Ctx::Compaction; break;
    default: ctx = Ctx::None; break;
    }
    return true;
}

// Value of the first word starting with letter `w` (upper case) among the tokens; false if absent.
bool word(const std::vector<std::string_view> &tok, char w, double &v)
{
    for (size_t i = 1; i < tok.size(); ++i)
        if ((tok[i][0] == w || tok[i][0] == char(w + 32)) && parse_number(tok[i].substr(1), v))
            return true;
    return false;
}

double parse_slicer_estimate(const std::string &line)
{
    static const std::string_view key = "; estimated printing time (normal mode) = ";
    if (line.compare(0, key.size(), key) != 0)
        return -1.;
    double total = 0., num = 0.;
    bool any = false;
    std::string cur;
    for (size_t i = key.size(); i < line.size(); ++i) {
        const char c = line[i];
        if (std::isdigit(uint8_t(c)))
            cur += c;
        else if (c == 'd' || c == 'h' || c == 'm' || c == 's') {
            if (cur.empty() || ! parse_number(cur, num))
                return -1.;
            total += num * (c == 'd' ? 86400. : c == 'h' ? 3600. : c == 'm' ? 60. : 1.);
            any = true;
            cur.clear();
        } else if (c != ' ' && c != '\r')
            return -1.;
    }
    return any && cur.empty() ? total : -1.;
}

} // namespace

PowderReportParams powder_report_params(const TranslatorParams &p)
{
    PowderReportParams r;
    if (p.umex) {
        r.powder_pin = p.U.powder_pin;
        r.comp_pin   = p.U.comp_pin;
        r.m_dot      = p.U.m_dot;
        r.lag_on     = p.U.lag_on;
        r.lag_off    = p.U.lag_off;
    }
    if (p.enable_uv) {
        const size_t at = p.uv_on.find(" P");
        if (at != std::string::npos) {
            int  pin = 0;
            bool any = false;
            for (size_t i = at + 2; i < p.uv_on.size() && std::isdigit(uint8_t(p.uv_on[i])); ++i, any = true)
                pin = pin * 10 + (p.uv_on[i] - '0');
            if (any)
                r.uv_pin = pin;
        }
    }
    return r;
}

PowderReport analyze_translated_gcode(const std::string &gcode, const PowderReportParams &P)
{
    PowderReport R;
    R.m_dot = P.m_dot;

    Ctx    ctx = Ctx::None;
    double x = kNaN, y = kNaN, z = kNaN, feed = 0.;
    bool   absolute = true;
    bool   uv_win = false, powder_win = false, comp_win = false;
    double powder_win_s = 0.;
    // paste-mode UV cure: the G0 right before the UV-on and the G0 right after the UV-off
    bool   prev_code_g0 = false, uv_positioned = false, expect_return = false;
    double prev_g0_s = 0.;

    size_t layer = 0;
    auto add = [&](PowderReport::Part part, double s, bool dwell) {
        if (R.layer_added_s.size() <= layer)
            R.layer_added_s.resize(layer + 1, 0.);
        R.layer_added_s[layer] += s;
        R.part_s[part] += s;
        (dwell ? R.dwell_s : R.move_s) += s;
    };
    auto ctx_part = [](Ctx c, PowderReport::Part &part) {
        switch (c) {
        case Ctx::Relief:     part = PowderReport::PressureRelief; return true;
        case Ctx::Wipe:       part = PowderReport::Wipe; return true;
        case Ctx::Uv:         part = PowderReport::Uv; return true;
        case Ctx::Powder:     part = PowderReport::PowderFeed; return true;
        case Ctx::Compaction: part = PowderReport::Compaction; return true;
        default:              return false;
        }
    };

    size_t line_no = 0;
    for (size_t pos = 0; pos < gcode.size();) {
        const size_t nl  = gcode.find('\n', pos);
        const size_t end = nl == std::string::npos ? gcode.size() : nl;
        std::string line(gcode, pos, end - pos);
        pos = nl == std::string::npos ? gcode.size() : nl + 1;
        ++line_no;
        if (! line.empty() && line.back() == '\r')
            line.pop_back();

        if (line.compare(0, 13, ";LAYER_CHANGE") == 0)
            ++layer;
        if (const double est = parse_slicer_estimate(line); est >= 0.)
            R.slicer_estimate_s = est;
        if (line.compare(0, kEstimateMarker.size(), kEstimateMarker) == 0) {
            double v = 0.;
            const size_t e = line.find(" s", kEstimateMarker.size());
            if (e != std::string::npos && parse_number(std::string_view(line).substr(kEstimateMarker.size(), e - kEstimateMarker.size()), v))
                R.estimate_includes_s = v;
        }
        Ctx new_ctx;
        if (stage_header(line, new_ctx)) {
            ctx = new_ctx;
            if (ctx == Ctx::Powder)
                ++R.powder_layers;
            continue;
        }
        if (line.compare(0, 24, "; --- uMEX safe shutdown") == 0) {
            ctx = Ctx::None;
            continue;
        }
        const size_t semi = line.find(';');
        const std::string code = trim(std::string_view(line).substr(0, semi));
        if (code.empty())
            continue;

        std::vector<std::string_view> tok;
        for (size_t i = 0; i < code.size();) {
            while (i < code.size() && (code[i] == ' ' || code[i] == '\t'))
                ++i;
            const size_t b = i;
            while (i < code.size() && code[i] != ' ' && code[i] != '\t')
                ++i;
            if (i > b)
                tok.emplace_back(code.data() + b, i - b);
        }
        std::string cmd(tok[0]);
        for (char &c : cmd)
            c = char(std::toupper(uint8_t(c)));
        auto bad = [&](const std::string &what) {
            throw Slic3r::RuntimeError("DIW report: line " + std::to_string(line_no) + ": " + what + ": " + code);
        };

        const bool   was_prev_g0 = prev_code_g0, was_expect_return = expect_return;
        const double was_prev_g0_s = prev_g0_s;
        prev_code_g0  = false;
        expect_return = false;

        if (cmd == "G90") {
            absolute = true;
        } else if (cmd == "G91") {
            absolute = false;
        } else if (cmd == "G0" || cmd == "G00" || cmd == "G1" || cmd == "G01" || cmd == "G2" || cmd == "G02" || cmd == "G3" || cmd == "G03") {
            double nx = x, ny = y, nz = z, f = 0., v = 0.;
            if (word(tok, 'X', v)) nx = absolute ? v : x + v;
            if (word(tok, 'Y', v)) ny = absolute ? v : y + v;
            if (word(tok, 'Z', v)) nz = absolute ? v : z + v;
            if (word(tok, 'F', f))
                feed = f;
            // distance over the axes with a known start; the first move of a file has no start
            double d2 = 0.;
            if (! std::isnan(x) && ! std::isnan(nx)) d2 += (nx - x) * (nx - x);
            if (! std::isnan(y) && ! std::isnan(ny)) d2 += (ny - y) * (ny - y);
            if (! std::isnan(z) && ! std::isnan(nz)) d2 += (nz - z) * (nz - z);
            x = nx; y = ny; z = nz;
            const double s = feed > 0. ? std::sqrt(d2) / feed * 60. : 0.;
            PowderReport::Part part;
            if (uv_win)
                add(PowderReport::Uv, s, false);
            else if (ctx_part(ctx, part))
                add(part, s, false);
            if (powder_win)
                powder_win_s += s;
            const bool g0 = cmd == "G0" || cmd == "G00";
            if (g0) {
                prev_code_g0 = true;
                prev_g0_s    = s;
            }
            if (was_expect_return && g0 && ctx != Ctx::Uv)
                add(PowderReport::Uv, s, false);
        } else if (cmd == "G04" || cmd == "G4") {
            double p = 0.;
            if (! word(tok, 'P', p) && ! word(tok, 'S', p))
                bad("G04 without a P or S word");
            if (p < 0.)
                bad("negative G04 dwell");
            PowderReport::Part part = PowderReport::Other;
            if (uv_win)
                part = PowderReport::Uv;
            else if (powder_win)
                part = PowderReport::PowderFeed;
            else if (comp_win)
                part = PowderReport::Compaction;
            else
                ctx_part(ctx, part);
            add(part, p, true);
            if (powder_win)
                powder_win_s += p;
        } else if (cmd == "M62") {
            double pin = 0., q = 0.;
            if (! word(tok, 'P', pin) || ! word(tok, 'Q', q))
                bad("M62 without P and Q words");
            const int  ipin = int(pin);
            const bool on   = q != 0.;
            if (ipin == P.uv_pin) {
                if (on && ctx != Ctx::Uv) {
                    uv_positioned = was_prev_g0;
                    if (was_prev_g0)
                        add(PowderReport::Uv, was_prev_g0_s, false);
                }
                if (! on && uv_win && ctx != Ctx::Uv && uv_positioned)
                    expect_return = true;
                uv_win = on;
            } else if (ipin == P.powder_pin) {
                if (on) {
                    powder_win   = true;
                    powder_win_s = 0.;
                } else if (powder_win) {
                    R.powder_deposition_s += std::max(0., powder_win_s - P.lag_on + P.lag_off);
                    ++R.powder_windows;
                    powder_win = false;
                }
            } else if (ipin == P.comp_pin) {
                comp_win = on;
            }
        }
    }
    R.powder_mass_g = P.m_dot / 60. * R.powder_deposition_s;
    if (R.estimate_includes_s >= 0. && R.slicer_estimate_s >= 0.)
        R.slicer_estimate_s = std::max(0., R.slicer_estimate_s - R.estimate_includes_s);
    return R;
}

std::string estimate_marker(double added_s)
{
    return std::string(kEstimateMarker) + fixed(added_s, 3) + " s\n";
}

std::string format_duration(double seconds)
{
    const long long t = std::llround(seconds < 0. ? 0. : seconds);
    const long long h = t / 3600, m = (t % 3600) / 60, s = t % 60;
    std::string out;
    if (h > 0) {
        out += std::to_string(h) + "h ";
        out += (m < 10 ? "0" : "") + std::to_string(m) + "m ";
        out += (s < 10 ? "0" : "") + std::to_string(s) + "s";
    } else if (m > 0) {
        out += std::to_string(m) + "m " + (s < 10 ? "0" : "") + std::to_string(s) + "s";
    } else
        out += std::to_string(s) + "s";
    return out;
}

std::vector<std::string> report_summary_lines(const PowderReport &r)
{
    std::vector<std::string> v;
    v.push_back("powder mass = " + fixed(r.powder_mass_g, 4) + " g");
    v.push_back("powder deposition time = " + fixed(r.powder_deposition_s, 3) + " s at " + fixed(r.m_dot, 3) + " g/min, " +
                std::to_string(r.powder_layers) + " powder layers, " + std::to_string(r.powder_windows) + " feeder passes");
    const double total = r.total_s();
    v.push_back("added time = " + fixed(total, 3) + " s, " + format_duration(total));
    v.push_back("added time by part: pressure relief " + fixed(r.part_s[PowderReport::PressureRelief], 3) + " s, wipe " +
                fixed(r.part_s[PowderReport::Wipe], 3) + " s, UV " + fixed(r.part_s[PowderReport::Uv], 3) + " s, powder feed " +
                fixed(r.part_s[PowderReport::PowderFeed], 3) + " s, compaction " + fixed(r.part_s[PowderReport::Compaction], 3) +
                " s, other dwells " + fixed(r.part_s[PowderReport::Other], 3) + " s");
    v.push_back("added time by kind: dwells " + fixed(r.dwell_s, 3) + " s, moves " + fixed(r.move_s, 3) + " s");
    if (r.slicer_estimate_s >= 0.)
        v.push_back("slicer estimate = " + format_duration(r.slicer_estimate_s) + ", with added time = " +
                    format_duration(r.slicer_estimate_s + total));
    return v;
}

std::string report_footer(const PowderReport &r)
{
    std::string out = "; --- uMEX report: added time at nominal feed, no acceleration ---\n";
    for (const std::string &l : report_summary_lines(r))
        out += "; " + l + "\n";
    out += "; --- end of uMEX report ---\n";
    return out;
}

} // namespace DIW
} // namespace Slic3r
