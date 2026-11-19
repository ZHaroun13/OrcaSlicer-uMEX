#include "DIWTranslator.hpp"
#include "DIWPyCompat.hpp"
#include "DIWUMexParams.hpp"
#include "DIWUMexSequencer.hpp"
#include "DIWPowderBlock.hpp"

#include "../Exception.hpp"
#include "../LocalesUtils.hpp"
#include "../PrintConfig.hpp"
#include "../Utils.hpp"

#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sstream>
#include <tuple>
#include <vector>

// Port of the paste path of diw_engine.py 2.0.0 (DIWPostProcessor, powder off). The structure and the
// order of the checks follow the Python code on purpose, so the two can be compared side by side and the
// v2 output can serve as the golden reference. Regular expressions of the Python code are replaced by
// small hand-written matchers with the same behaviour (noted at each one).

namespace Slic3r {

const char *DIW_TRANSLATION_HEADER = "; DIW / u-MEX translation by OrcaSlicer-uMEX (paste rules of DIW Post-Processor v2.0.0)";

namespace {

// ------------------------------------------------------------------------------------------------
// Text helpers with Python str semantics (GCode/DIWPyCompat, shared with the u-MEX sequencer). The text is
// valid UTF-8 (see sanitize_utf8()).
// ------------------------------------------------------------------------------------------------

using DIW::is_digit;
using DIW::lower;
using DIW::space_len_at;
using DIW::space_at;
using DIW::word_at;
using DIW::to_lower;
using DIW::to_upper;
using DIW::strip;
using DIW::lstrip;
using DIW::WordMatch;
using DIW::find_word;
using DIW::replace_words;

// errors="replace" decoding: every maximal invalid UTF-8 subpart becomes U+FFFD, as Python reads the file.
std::string sanitize_utf8(const std::string &s)
{
    static const char FFFD[] = "\xEF\xBF\xBD";
    std::string r;
    r.reserve(s.size());
    size_t i = 0;
    while (i < s.size()) {
        const unsigned char b = (unsigned char) s[i];
        if (b < 0x80) { r += char(b); ++i; continue; }
        size_t need; unsigned char lo = 0x80, hi = 0xBF;
        if (b >= 0xC2 && b <= 0xDF) need = 1;
        else if (b == 0xE0) { need = 2; lo = 0xA0; }
        else if ((b >= 0xE1 && b <= 0xEC) || b == 0xEE || b == 0xEF) need = 2;
        else if (b == 0xED) { need = 2; hi = 0x9F; }
        else if (b == 0xF0) { need = 3; lo = 0x90; }
        else if (b >= 0xF1 && b <= 0xF3) need = 3;
        else if (b == 0xF4) { need = 3; hi = 0x8F; }
        else { r += FFFD; ++i; continue; }
        size_t j = i + 1;
        bool ok = true;
        for (size_t k = 0; k < need; ++k, ++j) {
            const unsigned char c = j < s.size() ? (unsigned char) s[j] : 0;
            const unsigned char l = k == 0 ? lo : 0x80, h = k == 0 ? hi : 0xBF;
            if (j >= s.size() || c < l || c > h) { ok = false; break; }
        }
        if (ok) r.append(s, i, j - i);
        else    r += FFFD;
        i = j;
    }
    return r;
}

bool starts_with(const std::string &s, const char *prefix) { return s.rfind(prefix, 0) == 0; }

void append(std::vector<std::string> &dst, const std::vector<std::string> &src) { dst.insert(dst.end(), src.begin(), src.end()); }

// " ".join(s.split())
std::string collapse_ws(const std::string &s)
{
    std::string r;
    size_t i = 0;
    while (i < s.size()) {
        for (size_t n; i < s.size() && (n = space_len_at(s, i)) > 0;) i += n;
        size_t b = i;
        while (i < s.size() && ! space_at(s, i)) ++i;
        if (i > b) {
            if (! r.empty()) r += ' ';
            r.append(s, b, i - b);
        }
    }
    return r;
}

std::string first_token(const std::string &code)
{
    size_t i = 0;
    while (i < code.size() && ! space_at(code, i)) ++i;
    return to_upper(code.substr(0, i));
}

// Python f"{v:.Nf}": any length, and "nan" / "inf" / "-inf" for non-finite values.
std::string format_num(const char *f, double v)
{
    if (std::isnan(v)) return "nan";
    if (std::isinf(v)) return v < 0 ? "-inf" : "inf";
    const int n = snprintf(nullptr, 0, f, v);
    std::string r(size_t(std::max(n, 0)), '\0');
    if (n > 0)
        snprintf(&r[0], size_t(n) + 1, f, v);
    return r;
}


// ------------------------------------------------------------------------------------------------
// Line classifiers equivalent to the v2 regular expressions (all case-insensitive)
// ------------------------------------------------------------------------------------------------

// RE_LAYER_MARKER: (?:(?<!BEFORE_)LAYER_CHANGE\b|AFTER_LAYER_CHANGE\b|LAYER:\s*\d+|[\(\[]?(?:OrcaSlicer\s+)?layer[\s_:]*\d+)
// The 2nd branch is covered by the 1st, the 3rd by the 4th, and the optional prefix of the 4th does not
// change whether it matches.
bool is_layer_marker(const std::string &line)
{
    const std::string s = to_lower(line);
    static const std::string lc = "layer_change";
    for (size_t p = s.find(lc); p != std::string::npos; p = s.find(lc, p + 1)) {
        const bool after_before = p >= 7 && s.compare(p - 7, 7, "before_") == 0;
        const size_t q = p + lc.size();
        if (! after_before && ! word_at(s, q))
            return true;
    }
    for (size_t p = s.find("layer"); p != std::string::npos; p = s.find("layer", p + 1)) {
        size_t q = p + 5;
        for (size_t n; q < s.size();) {
            if ((n = space_len_at(s, q)) > 0) q += n;
            else if (s[q] == '_' || s[q] == ':') ++q;
            else break;
        }
        if (q < s.size() && is_digit(s[q]))
            return true;
    }
    return false;
}

// ";\s*<w1>(\s*|\s+)<w2>" anywhere in the line; w1/w2 lower case.
bool has_semicolon_phrase(const std::string &line, const char *w1, bool need_space, const char *w2)
{
    const std::string s = to_lower(line);
    const size_t l1 = strlen(w1), l2 = strlen(w2);
    for (size_t p = s.find(';'); p != std::string::npos; p = s.find(';', p + 1)) {
        size_t q = p + 1;
        for (size_t n; q < s.size() && (n = space_len_at(s, q)) > 0;) q += n;
        if (s.compare(q, l1, w1) != 0) continue;
        q += l1;
        const size_t q0 = q;
        for (size_t n; q < s.size() && (n = space_len_at(s, q)) > 0;) q += n;
        if (need_space && q == q0) continue;
        if (s.compare(q, l2, w2) == 0) return true;
    }
    return false;
}

bool is_custom_start(const std::string &l)  { return has_semicolon_phrase(l, "type:", false, "custom"); }   // ;\s*TYPE:\s*Custom
bool is_initial_load(const std::string &l)  { return has_semicolon_phrase(l, "initial", true, "load"); }   // ;\s*initial\s+load
bool is_intro_line(const std::string &l)    { return has_semicolon_phrase(l, "intro", true, "line"); }     // ;\s*intro\s+line
bool is_filament_gcode(const std::string &l){ return has_semicolon_phrase(l, "filament", true, "gcode"); } // ;\s*Filament\s+gcode

// Position after an optional "N<digits><spaces>" line number at the start of s (for ^(?:N\d+\s+)?).
size_t skip_line_number(const std::string &s)
{
    if (! s.empty() && (s[0] == 'N' || s[0] == 'n')) {
        size_t k = 1;
        while (k < s.size() && is_digit(s[k])) ++k;
        if (k > 1 && space_at(s, k)) {
            for (size_t n; k < s.size() && (n = space_len_at(s, k)) > 0;) k += n;
            return k;
        }
    }
    return 0;
}

// ^(?:N\d+\s+)?<CMD>\b  (the optional group is also tried empty, which only matters if s starts with N).
bool starts_with_command(const std::string &s, const char *cmd)
{
    const size_t n = strlen(cmd);
    for (size_t k : { skip_line_number(s), size_t(0) }) {
        bool ok = s.size() >= k + n;
        for (size_t i = 0; ok && i < n; ++i)
            ok = lower(s[k + i]) == lower(cmd[i]);
        if (ok && ! word_at(s, k + n))
            return true;
    }
    return false;
}

// RE_MOTION ^(?:N\d+\s+)?(G0?[0-3])\b
bool is_motion(const std::string &s)
{
    for (size_t k : { skip_line_number(s), size_t(0) }) {
        if (k >= s.size() || lower(s[k]) != 'g') continue;
        // G0?[0-3]\b: with the optional 0 taken, then without it.
        for (size_t z : { size_t(1), size_t(0) }) {
            size_t q = k + 1;
            if (z == 1) {
                if (q >= s.size() || s[q] != '0') continue;
                ++q;
            }
            if (q < s.size() && s[q] >= '0' && s[q] <= '3' && ! word_at(s, q + 1))
                return true;
        }
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
// Settings
// ------------------------------------------------------------------------------------------------

using Params = DIW::TranslatorParams;

double get_float(const DynamicPrintConfig &c, const char *key, double def)
{
    if (const auto *o = c.option<ConfigOptionFloat>(key)) return o->value;
    if (const auto *o = c.option<ConfigOptionFloats>(key)) return o->values.empty() ? def : o->values.front();
    return def;
}

bool get_bool(const DynamicPrintConfig &c, const char *key, bool def)
{
    if (const auto *o = c.option<ConfigOptionBool>(key)) return o->value;
    if (const auto *o = c.option<ConfigOptionBools>(key)) return o->values.empty() ? def : bool(o->values.front());
    return def;
}

} // namespace

// Exported for the native powder planner (PLANNER_DESIGN.md section 2.2), so that U, plan_fingerprint(U) and
// the feeds it uses are exactly the translator's.
DIW::TranslatorParams diw_read_translator_params(const DynamicPrintConfig &c)
{
    Params p;
    p.a_mode = diwaFlowScaled;
    p.df = get_float(c, "filament_diameter", p.df);
    const double d_syringe = get_float(c, "diw_syringe_diameter", 50.);
    // B from the paste volume each move needs: dV = dE * pi/4 * df^2, dB = dV / (pi/4 * D^2) = dE * (df/D)^2.
    // Kept as the ratio, computed exactly as the v2 engine does, so both give identical B words.
    p.volumetric_ratio = d_syringe > 0 ? std::pow(p.df / d_syringe, 2) : 0.;
    p.b_multiplier     = get_float(c, "diw_b_multiplier", p.b_multiplier);
    p.b_drivetrain     = get_bool(c, "diw_b_drivetrain_units", p.b_drivetrain);
    p.lead_b           = get_float(c, "diw_b_lead_screw_lead", p.lead_b);
    p.i_gear_b         = get_float(c, "diw_b_gear_ratio", p.i_gear_b);
    if (const auto *o = c.option<ConfigOptionEnumsGeneric>("diw_a_axis_mode"); o && ! o->values.empty())
        p.a_mode = o->values.front();
    p.k_screw           = get_float(c, "diw_k_screw", p.k_screw);
    p.enable_retraction = get_bool(c, "diw_retraction_enable", p.enable_retraction);
    p.max_retract_de    = get_float(c, "diw_retraction_max_length", p.max_retract_de);
    p.max_retract_f     = get_float(c, "diw_retraction_max_speed", 5.) * 60.;
    p.post_dwell        = get_float(c, "diw_post_extrusion_dwell", p.post_dwell);
    if (const auto *o = c.option<ConfigOptionString>("diw_uv_on_gcode")) p.uv_on = strip(o->value);
    if (const auto *o = c.option<ConfigOptionString>("diw_uv_off_gcode")) p.uv_off = strip(o->value);
    // An empty UV-on command means there is no UV lamp (v2: enable_uv_io = False).
    p.enable_uv      = ! p.uv_on.empty();
    p.uv_offset_y    = get_float(c, "diw_uv_offset_y", p.uv_offset_y);
    p.uv_end_dwell   = get_float(c, "diw_uv_end_dwell", p.uv_end_dwell);
    p.uv_layer_dwell = get_float(c, "diw_uv_layer_dwell", p.uv_layer_dwell);

    // v2 _umex_init: the parameters are validated up front (throws DIWConfigError), then the
    // uncalibrated-offset warning.
    p.umex = get_bool(c, "diw_powder_infill", false);
    if (p.umex) {
        std::tie(p.U, p.config_warnings) = DIW::read_umex_params(c, p.uv_on);
        if (p.U.off_x == 0.0 && p.U.off_y == 0.0 && p.U.off_z == 0.0) {
            const std::string w = "POWDER NOZZLE OFFSETS powder_offset_x/y/z ARE ALL 0.0 (UNCALIBRATED): powder would be "
                                  "dispensed at the PASTE NOZZLE position. Calibrate before any hardware run.";
            if (std::find(p.config_warnings.begin(), p.config_warnings.end(), w) == p.config_warnings.end())
                p.config_warnings.push_back(w);
        }
    }
    return p;
}

// ------------------------------------------------------------------------------------------------
// Validation before slicing (PRODUCT_DESIGN.md 6.2)
// ------------------------------------------------------------------------------------------------

const std::vector<std::pair<std::string, std::string>> &diw_validation_opt_key_table()
{
    // {prefix of the message read_umex_params / diw_read_translator_params gives -> fork key}, in v2's check order
    // (diw_engine.py UMexParams.from_config). Each prefix ends with the character after the v2 key, so no row is a
    // prefix of another row's messages.
    static const std::vector<std::pair<std::string, std::string>> table = {
        { "compaction_duration ",                    "diw_compaction_time" },
        { "powder_hatch_spacing ",                   "diw_powder_hatch_spacing" },
        { "powder_mass_flow_rate ",                  "diw_powder_mass_flow" },
        { "powder_tap_density ",                     "diw_powder_tap_density" },
        { "powder_fill_multiplier ",                 "diw_powder_fill_multiplier" },
        { "powder_f_max ",                           "diw_powder_speed_max" },
        { "powder_nozzle_dia ",                      "diw_powder_nozzle_diameter" },
        { "powder_lag_on ",                          "diw_powder_lag_on" },
        { "powder_lag_off ",                         "diw_powder_lag_off" },
        { "shell_powder_clearance ",                 "diw_shell_powder_clearance" },
        { "compaction_settle_dwell ",                "diw_compaction_settle_time" },
        { "powder_travel_hop_mm ",                   "diw_powder_travel_hop" },
        { "compaction_z_hop_mm ",                    "diw_compaction_z_hop" },
        { "compaction_interval ",                    "diw_compaction_interval" },
        { "powder_motor_voltage ",                   "diw_powder_motor_voltage" },
        { "digital output pins must be distinct: ", "diw_powder_pin" },
        { "powder_offset_z ",                        "diw_powder_offset_z" },
        { "POWDER NOZZLE OFFSETS ",                  "diw_powder_offset_x" },
    };
    return table;
}

std::string diw_validation_opt_key(const std::string &message)
{
    for (const auto &[prefix, key] : diw_validation_opt_key_table())
        if (message.compare(0, prefix.size(), prefix) == 0)
            return key;
    return std::string();
}

DIWValidation diw_validate(const DynamicPrintConfig &full)
{
    DIWValidation v;
    if (! diw_translation_enabled(full) || ! get_bool(full, "diw_powder_infill", false))
        return v;
    try {
        const Params p = diw_read_translator_params(full);
        for (const std::string &w : p.config_warnings)
            v.warnings.emplace_back(w, diw_validation_opt_key(w));
    } catch (const DIWConfigError &ex) {
        v.error   = ex.what();
        v.opt_key = diw_validation_opt_key(v.error);
    }
    return v;
}

// ------------------------------------------------------------------------------------------------
// Extruded bounding box (v2 _compute_geometric_center), shared with the powder binder
// ------------------------------------------------------------------------------------------------

namespace DIW {

void extrusion_bbox(const std::string &line, ExtrusionBBox &b)
{
    if (b.stopped)
        return;
    if (line.find("; CONFIG_BLOCK_START") != std::string::npos) {
        b.stopped = true;
        return;
    }
    const std::string s = strip(line);
    if (! b.found_first_layer && (is_layer_marker(s) || s.find(";LAYER:0") != std::string::npos))
        b.found_first_layer = true;
    if (starts_with_command(s, "M83"))
        b.rel = true;
    else if (starts_with_command(s, "M82"))
        b.rel = false;
    if (starts_with_command(s, "G92"))
        if (WordMatch em = find_word(s, "E"))
            b.prev_e = em.value;
    if (is_motion(s)) {
        const WordMatch xm = find_word(s, "X"), ym = find_word(s, "Y"), em = find_word(s, "E");
        const double lx = xm ? xm.value : b.tx, ly = ym ? ym.value : b.ty;
        if (em) {
            double de;
            if (b.rel)
                de = em.value;
            else {
                de       = em.value - b.prev_e;
                b.prev_e = em.value;
            }
            if (b.found_first_layer && de > 1e-7) {
                b.min_x = std::min({ b.min_x, b.tx, lx });
                b.max_x = std::max({ b.max_x, b.tx, lx });
                b.min_y = std::min({ b.min_y, b.ty, ly });
                b.max_y = std::max({ b.max_y, b.ty, ly });
            }
        }
        b.tx = lx;
        b.ty = ly;
    }
}

void extrusion_bbox(const std::vector<std::string> &lines, ExtrusionBBox &b)
{
    for (const std::string &line : lines) {
        if (b.stopped)
            break;
        extrusion_bbox(line, b);
    }
}

} // namespace DIW

namespace {

// ------------------------------------------------------------------------------------------------
// Translator (DIWPostProcessor)
// ------------------------------------------------------------------------------------------------

class Translator : public DIW::UMexSequencer
{
public:
    Translator(const Params &p, bool stub_powder_stage, bool v2_parity) : DIW::UMexSequencer(p, stub_powder_stage, v2_parity) {}

    // blocks: the accepted ";DIW:" blocks (powder mode only).
    std::string run(const std::vector<std::string> &raw_lines, const std::vector<DIW::PlacedBlock> &blocks);

private:
    // The state (cur_x, last_e, layer_count, warnings, ...) lives in DIW::UMexSequencer, which drives the
    // core layers through process_line() and curing_sequence(), as v2's _UMexMixin does.
    bool speed_mode() const override { return P.a_mode == diwaFlowScaled || P.a_mode == diwaProportional; }

    Lines  umex_shutdown_lines();
    Lines  umex_banner() const;
    std::string sanitize_dwell(const std::string &tok, const std::string &code, const std::string &comment_suffix);

    void   compute_geometric_center(const std::vector<std::string> &raw_lines);
    Lines  curing_sequence(double return_x, double return_y) override;
    Lines  trigger_final_curing();
    double delta_b(double delta_e) const;
    Lines  process_line(const std::string &raw_line) override;
    Lines  motion_block(const std::string &code, const std::string &comment_suffix);
    Lines  finalize();
};

void Translator::compute_geometric_center(const std::vector<std::string> &raw_lines)
{
    // The shared routine (DIW::extrusion_bbox, also the powder binder's V2 check) over the whole file.
    DIW::ExtrusionBBox bb;
    DIW::extrusion_bbox(raw_lines, bb);
    double cx = 0., cy = 0.;
    if (! bb.empty()) {
        cx = (bb.min_x + bb.max_x) / 2.0;
        cy = (bb.min_y + bb.max_y) / 2.0;
    }
    nozzle_target_x = cx;
    nozzle_target_y = cy + P.uv_offset_y;
}

Translator::Lines Translator::curing_sequence(double return_x, double return_y)
{
    Lines m;
    if (speed_mode())
        m.emplace_back("G1 A0.000\n");
    is_extruding = false;
    m.emplace_back("G90\n");
    m.emplace_back("G0 X" + format_num("%.3f", nozzle_target_x) + " Y" + format_num("%.3f", nozzle_target_y) + " F3000\n");
    m.emplace_back(P.uv_on + "\n");
    m.emplace_back("G04 P" + format_num("%.2f", P.uv_layer_dwell) + "\n");
    m.emplace_back(P.uv_off + "\n");
    m.emplace_back("G0 X" + format_num("%.3f", return_x) + " Y" + format_num("%.3f", return_y) + " F3000\n");
    uv_active = false;
    return m;
}

Translator::Lines Translator::trigger_final_curing()
{
    if (! final_curing_done && layer_count > 0) {
        final_curing_done      = true;
        extruded_on_layer      = false;
        cured_for_layer_change = true;
        if (P.enable_uv && P.uv_layer_dwell > 0)
            return curing_sequence(cur_x, cur_y);
    }
    return {};
}

double Translator::delta_b(double delta_e) const
{
    const double dl = delta_e * P.volumetric_ratio;
    if (P.b_drivetrain && P.lead_b != 0)
        return ((dl / P.lead_b) * P.i_gear_b) * P.b_multiplier;
    return dl * P.b_multiplier;
}

Translator::Lines Translator::umex_shutdown_lines()
{
    // End-of-print safe state: A0.000, UV off, powder feeder off, compaction table off (v2 _umex_shutdown_lines).
    Lines lines { "; --- uMEX safe shutdown ---\n" };
    lines.emplace_back(speed_mode() ? "G1 A0.000\n" : "; (A held: cumulative position mode, no A0.000)\n");
    lines.push_back(P.uv_off + "\n");
    lines.push_back("M62 P" + std::to_string(P.U.powder_pin) + " Q0\n");
    lines.push_back("M62 P" + std::to_string(P.U.comp_pin) + " Q0\n");
    shutdown_done = true;
    return lines;
}

Translator::Lines Translator::umex_banner() const
{
    // Comment banner listing every warning, placed right after the header line (v2 _umex_banner).
    if (warnings.empty())
        return {};
    Lines lines { "; ======================= uMEX WARNINGS =======================\n" };
    for (std::string w : warnings) {
        std::replace(w.begin(), w.end(), '[', '(');
        std::replace(w.begin(), w.end(), ']', ')');
        lines.push_back("; WARNING (uMEX): " + w + "\n");
    }
    lines.emplace_back("; =============================================================\n");
    return lines;
}

std::string Translator::sanitize_dwell(const std::string &tok, const std::string &code, const std::string &comment_suffix)
{
    // Powder mode: dwell in seconds, "G04 P0.500" (requirements 6.3). S is seconds; on a bare G4, P > 50 is
    // Marlin milliseconds; a G4 P <= 50 is taken as seconds and reported (v2 _sanitize_dwell).
    const WordMatch m   = find_word(code, "PS");
    const double    val = m.value;
    double          sec;
    if (code[m.begin] == 'S' || code[m.begin] == 's')
        sec = val;
    else if (tok == "G4" && val > 50.0)
        sec = val / 1000.0;
    else {
        sec = val;
        if (tok == "G4" && val > 0.0)
            warn("G4 P" + DIW::py_g(val) + " treated as " + DIW::py_g(val) + " SECONDS (document rule: only P > 50 is milliseconds)");
    }
    return "G04 P" + format_num("%.3f", sec) + comment_suffix + "\n";
}

Translator::Lines Translator::process_line(const std::string &raw_line)
{
    if (in_config_block) {
        if (! P.umex)
            return { raw_line };
        // Powder mode: the config footer is made bracket-free as well.
        std::string l = raw_line;
        std::replace(l.begin(), l.end(), '[', '(');
        std::replace(l.begin(), l.end(), ']', ')');
        return { l };
    }
    if (raw_line.find("; CONFIG_BLOCK_START") != std::string::npos) {
        in_config_block = true;
        Lines out = trigger_final_curing();
        out.push_back(raw_line);
        return out;
    }

    Lines out;
    std::string line = raw_line;
    while (! line.empty() && (line.back() == '\n' || line.back() == '\r'))
        line.pop_back();

    // Comment separation and RS-274 sanitation: [ ] in comments would be evaluated by PlanetCNC.
    std::string code, comment_suffix;
    if (const size_t sc = line.find(';'); sc != std::string::npos) {
        std::string content = line.substr(sc + 1);
        std::replace(content.begin(), content.end(), '[', '(');
        std::replace(content.begin(), content.end(), ']', ')');
        code           = strip(line.substr(0, sc));
        comment_suffix = " ;" + content;
    } else
        code = strip(line);

    // Final curing before the end G-code / parking.
    const std::string raw_lower = to_lower(raw_line);
    if ((final_stop_object_idx >= 0 && line_idx == final_stop_object_idx) ||
        raw_lower.find("; filament end gcode") != std::string::npos ||
        raw_lower.find("; executable_block_end") != std::string::npos)
        append(out, trigger_final_curing());

    // Inter-layer UV cure.
    const bool layer_marker = is_layer_marker(raw_line);
    if (layer_marker) {
        if (layer_count == 0) {
            layer_count            = 1;
            extruded_on_layer      = false;
            cured_for_layer_change = false;
        } else if (extruded_on_layer && ! cured_for_layer_change) {
            ++layer_count;
            extruded_on_layer      = false;
            cured_for_layer_change = true;
            if (P.enable_uv && P.uv_layer_dwell > 0)
                append(out, curing_sequence(cur_x, cur_y));
        }
    }

    if (is_custom_start(raw_line)) {
        in_custom_start    = true;
        suppress_uv_curing = true;
    }
    if (layer_marker || is_filament_gcode(raw_line)) {
        in_custom_start = suppress_uv_curing = in_initial_load = in_intro_line = false;
    }

    // The start G-code's "; initial load" prime block and its G92 are removed.
    if (is_initial_load(raw_line)) {
        in_initial_load = in_custom_start = suppress_uv_curing = true;
        return {};
    }
    if (in_initial_load) {
        if (starts_with(to_upper(code), "G92"))
            in_initial_load = false;
        return {};
    }

    // The start G-code's "; intro line" purge becomes a 20 mm line at X30..50.
    if (is_intro_line(raw_line)) {
        in_intro_line = in_custom_start = suppress_uv_curing = true;
        out.emplace_back("; intro line\n");
        append(out, motion_block("G1 X30.0 Y2.0 Z0.2 F1000", ""));
        append(out, motion_block("G1 X50.0 E3.0 F300", ""));
        return out;
    }
    if (in_intro_line) {
        if (starts_with(to_upper(code), "G92"))
            in_intro_line = false;
        else
            return {};
    }

    if (code.empty()) {
        if (! comment_suffix.empty()) {
            out.push_back(lstrip(comment_suffix) + "\n");
            return out;
        }
        if (! out.empty())
            return out;
        return { "\n" };
    }

    const std::string tok = first_token(code);

    // UV self-check (3 s flicker) right after the start G-code's G90.
    if (tok == "G90") {
        out.push_back(code + comment_suffix + "\n");
        if (in_custom_start && ! startup_uv_check_done) {
            if (P.enable_uv) {
                out.emplace_back("; --- UV Self-Check (3s Flicker) ---\n");
                out.push_back(P.uv_on + " ; UV Self-Check ON\n");
                out.emplace_back("G04 P3.00 ; Hold for 3 seconds\n");
                out.push_back(P.uv_off + " ; UV Self-Check OFF\n");
            }
            startup_uv_check_done = true;
            uv_active             = false;
            suppress_uv_curing    = true;
        }
        return out;
    }

    if (tok == "M82") {
        relative_e = false;
        return { "; (FILTERED) M82" + comment_suffix + "\n" };
    }
    if (tok == "M83") {
        relative_e = true;
        return { "; (FILTERED) M83" + comment_suffix + "\n" };
    }

    // End of print.
    if (tok == "M84" || tok == "M30" || tok == "M2") {
        append(out, trigger_final_curing());
        if (P.enable_uv) {
            if (P.uv_end_dwell > 0) {
                if (! uv_active) {
                    out.push_back(P.uv_on + "\n");
                    uv_active = true;
                }
                out.push_back("G04 P" + format_num("%.2f", P.uv_end_dwell) + " ; End-of-print UV curing dwell\n");
            }
            if (uv_active) {
                out.push_back(P.uv_off + "\n");
                uv_active = false;
            }
        }
        if (P.umex && ! shutdown_done)
            append(out, umex_shutdown_lines());
        if (tok == "M84") { // strip_m84
            out.push_back("; (FILTERED) " + code + comment_suffix + "\n");
            return out;
        }
        out.push_back(code + comment_suffix + "\n");
        return out;
    }

    static const char *prohibited[] = { "M104", "M109", "M140", "M190", "M105", "M106", "M107",
                                        "M73",  "M201", "M203", "M204", "M205", "M900", "M220", "M221" };
    if (std::find_if(std::begin(prohibited), std::end(prohibited), [&tok](const char *p) { return tok == p; }) != std::end(prohibited) ||
        (P.umex && tok == "M202"))
        return { "; (FILTERED) " + code + comment_suffix + "\n" };

    if (tok == "G28") // strip_g28
        return { "; (FILTERED) " + code + comment_suffix + "\n" };

    if (tok == "G4" || tok == "G04") {
        if (! find_word(code, "PS"))
            return { "; (FILTERED EMPTY G4) " + code + comment_suffix + "\n" };
        if (P.umex)
            return { sanitize_dwell(tok, code, comment_suffix) };
        return { code + comment_suffix + "\n" };
    }

    if (tok == "G92") {
        if (WordMatch em = find_word(code, "E")) {
            last_e       = em.value;
            cumulative_b = 0.;
            std::string replacement = "B0.0000";
            if (P.a_mode == diwaCumulative) {
                cumulative_a = 0.;
                replacement  = "B0.0000 A0.0000";
            }
            return { collapse_ws(replace_words(code, "E", replacement)) + comment_suffix + "\n" };
        }
        return { code + comment_suffix + "\n" };
    }

    if (tok == "G0" || tok == "G00" || tok == "G1" || tok == "G01" || tok == "G2" || tok == "G02" || tok == "G3" || tok == "G03")
        return motion_block(code, comment_suffix);

    return { code + comment_suffix + "\n" };
}

Translator::Lines Translator::motion_block(const std::string &code, const std::string &comment_suffix)
{
    Lines out;
    const WordMatch xm = find_word(code, "X"), ym = find_word(code, "Y"), zm = find_word(code, "Z"), fm = find_word(code, "F"),
                    em = find_word(code, "E");
    if (fm)
        cur_f = fm.value;

    const double nx = xm ? xm.value : cur_x, ny = ym ? ym.value : cur_y, nz = zm ? zm.value : cur_z;
    const double dx = nx - cur_x, dy = ny - cur_y, dz = nz - cur_z;
    const double ds = std::sqrt(dx * dx + dy * dy + dz * dz);
    cur_x = nx;
    cur_y = ny;
    cur_z = nz;

    double de = 0.;
    if (em) {
        if (relative_e)
            de = em.value;
        else {
            de     = em.value - last_e;
            last_e = em.value;
        }
    }

    if (de > 1e-7) {
        // Extrusion: syringe B by volume, auger A by the selected mode.
        extruded_on_layer      = true;
        cured_for_layer_change = false;
        cumulative_b += delta_b(de);
        const bool stationary = (dx == 0 && dy == 0) || (std::abs(dx) < 1e-7 && std::abs(dy) < 1e-7);
        double a = 0.;
        if (P.a_mode == diwaFlowScaled)
            a = stationary ? 0. : (ds > 0.0001 ? (de / ds) * cur_f * P.k_screw : 0.);
        else if (P.a_mode == diwaProportional)
            a = stationary ? 0. : cur_f * P.k_screw;
        else if (P.a_mode == diwaCumulative) {
            cumulative_a += ds > 0.0001 ? ds * P.k_screw : de * P.k_screw;
            a = cumulative_a;
        }
        is_extruding = true;
        out.push_back(collapse_ws(replace_words(code, "E", "") + " B" + format_num("%.4f", cumulative_b) + " A" + format_num("%.3f", a)) +
                      comment_suffix + "\n");
        return out;
    }

    if (de < -1e-7) {
        // Retraction: blocked when disabled or longer than the limit, otherwise clamped in speed.
        const bool blocked = ! P.enable_retraction || std::abs(de) > P.max_retract_de;
        if (blocked) {
            const bool has_xyz = ds > 0.0001 || xm || ym || zm;
            if (! has_xyz)
                return { "; (BLOCKED RETRACT) " + code + comment_suffix + "\n" };
            de = 0.; // a coordinated move: keep it as a travel without E
        } else {
            cumulative_b += delta_b(de);
            const double f_retract = std::min(cur_f, P.max_retract_f);
            const double a         = P.a_mode == diwaCumulative ? cumulative_a : 0.;
            const std::string no_e = replace_words(code, "E", "");
            const std::string fstr = "F" + format_num("%.1f", f_retract);
            const std::string with_f = find_word(no_e, "F") ? replace_words(no_e, "F", fstr) : no_e + " " + fstr;
            is_extruding = false;
            out.push_back(collapse_ws(with_f + " B" + format_num("%.4f", cumulative_b) + " A" + format_num("%.3f", a)) + comment_suffix + "\n");
            return out;
        }
    }

    // Travel: E removed; the auger stops on the first travel after an extrusion.
    std::string sanitized = replace_words(code, "E", "");
    if (is_extruding) {
        if (P.post_dwell > 0.)
            out.push_back("G04 P" + format_num("%.2f", P.post_dwell) + "\n");
        if (speed_mode())
            sanitized += " A0.000";
        is_extruding = false;
    }
    out.push_back(collapse_ws(sanitized) + comment_suffix + "\n");
    return out;
}

Translator::Lines Translator::finalize()
{
    Lines out;
    if (! in_config_block)
        append(out, trigger_final_curing());
    if (P.enable_uv && uv_active) {
        if (P.uv_end_dwell > 0)
            out.push_back("G04 P" + format_num("%.2f", P.uv_end_dwell) + " ; End-of-print UV curing dwell\n");
        out.push_back(P.uv_off + "\n");
        uv_active = false;
    }
    if (P.umex && ! shutdown_done)
        append(out, umex_shutdown_lines());
    return out;
}

std::string Translator::run(const std::vector<std::string> &raw_lines, const std::vector<DIW::PlacedBlock> &blocks)
{
    // Pass 1a: the last "; stop printing object" before the config block.
    for (size_t i = 0; i < raw_lines.size(); ++i) {
        const std::string l = to_lower(raw_lines[i]);
        if (l.find("; config_block_start") != std::string::npos)
            break;
        if (l.find("; stop printing object") != std::string::npos)
            final_stop_object_idx = long(i);
    }
    // Pass 1b: UV target = centre of the extruded bounding box + UV lamp offset.
    compute_geometric_center(raw_lines);

    std::string body;
    body.reserve(raw_lines.size() * 40);
    if (P.umex) {
        // v2 _process_lines_umex: layer-aware, core layers re-emitted in the u-MEX stage order.
        for (const std::string &l : process_lines_umex(raw_lines, blocks))
            body += l;
    } else {
        for (size_t i = 0; i < raw_lines.size(); ++i) {
            // A header line of the v2 post-processor is dropped, as v2 does.
            if (starts_with(raw_lines[i], "; Post-Processed by DIW Post-Processor"))
                continue;
            line_idx = long(i);
            for (const std::string &l : process_line(raw_lines[i]))
                body += l;
        }
    }
    for (const std::string &l : finalize())
        body += l;

    std::string out = std::string(DIW_TRANSLATION_HEADER) + "\n";
    if (P.umex)
        for (const std::string &l : umex_banner())
            out += l;
    return out + body;
}

} // namespace

bool diw_translation_enabled(const DynamicPrintConfig &config)
{
    const auto *o = config.option<ConfigOptionBool>("diw_printer");
    const auto *f = config.option<ConfigOptionEnum<GCodeFlavor>>("gcode_flavor");
    return (o != nullptr && o->value) || (f != nullptr && f->value == gcfPlanetCNC);
}

std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config)
{
    return diw_translate_gcode(gcode, config, DIWTranslateOptions());
}

std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config, const DIWTranslateOptions &opts)
{
    // Read as Python does: UTF-8 with errors="replace", universal newlines ("\n", "\r\n" and a lone "\r"
    // all end a line and become "\n"), lines keep their '\n' (readlines()).
    // ";DIW:" powder records (diw/POWDER_RECORDS.md) are removed before anything else, in paste and
    // powder modes alike, so a file translates the same with and without them. Record lines are split at
    // '\n' only (POWDER_RECORDS.md sections 1 and 4, as the reader and v2_blocks.strip_records do), so the
    // removal runs on the raw bytes before universal-newline splitting: a lone '\r' does not end a record
    // line, and a ";DIW:" after a lone '\r' is not a record and is not removed.
    // In powder mode the records are read (parse_records on the raw input) before they are removed; each
    // block's position is kept in the translator's line numbering: the number of G-code lines before it
    // once the records are gone, counting every '\n', "\r\n" and lone '\r' as one line end.
    std::string kept;
    kept.reserve(gcode.size());
    std::vector<std::pair<int, size_t>> record_pos;   // (1-based raw line of a record line, lines kept before it)
    size_t kept_lines = 0;
    int    raw_no     = 0;
    for (size_t pos = 0; pos < gcode.size();) {
        const size_t nl  = gcode.find('\n', pos);
        const size_t end = nl == std::string::npos ? gcode.size() : nl + 1;
        ++raw_no;
        if (gcode.compare(pos, 5, ";DIW:") != 0) {
            kept.append(gcode, pos, end - pos);
            for (size_t i = pos; i < end; ++i)
                if (gcode[i] == '\n' || (gcode[i] == '\r' && (i + 1 >= end || gcode[i + 1] != '\n')))
                    ++kept_lines;
        } else
            record_pos.emplace_back(raw_no, kept_lines);
        pos = end;
    }
    const std::string text = sanitize_utf8(kept);
    std::vector<std::string> lines;
    std::string cur;
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (c == '\r' || c == '\n') {
            if (c == '\r' && i + 1 < text.size() && text[i + 1] == '\n')
                ++i;
            cur += '\n';
            lines.push_back(std::move(cur));
            cur.clear();
        } else
            cur += c;
    }
    if (! cur.empty())
        lines.push_back(std::move(cur));
    if (! lines.empty() && starts_with(lines.front(), DIW_TRANSLATION_HEADER))
        return gcode; // already translated

    const Params p = diw_read_translator_params(config);
    std::vector<DIW::PlacedBlock> blocks;
    if (p.umex) {
        // A record error stops the export: the planner and the translator disagree about the file.
        DIW::ParsedRecords recs = DIW::parse_records(gcode);
        if (! recs.errors.empty()) {
            const auto &[line, reason] = recs.errors.front();
            std::string msg = "u-MEX powder records: the ;DIW: record at line " + std::to_string(line) + " of the G-code is invalid (" +
                              reason + ")";
            if (recs.errors.size() > 1)
                msg += ", and " + std::to_string(recs.errors.size() - 1) + " more record error(s)";
            throw DIWConfigError(msg + "; the powder plan cannot be used");
        }
        for (DIW::PowderLayerRec &r : recs.blocks) {
            const auto it = std::lower_bound(record_pos.begin(), record_pos.end(), std::make_pair(r.begin_line, size_t(0)));
            if (it == record_pos.end() || it->first != r.begin_line)   // the filter and the reader split lines alike
                throw Slic3r::RuntimeError("DIW translation: internal error locating the ;DIW: block at line " + std::to_string(r.begin_line));
            blocks.push_back({ std::move(r), it->second });
        }
        // Without any block no core layer is converted, so a powder job would come out as paste only.
        // Refuse it rather than print the wrong part (the plan is written by the slicer's export, PRODUCT_DESIGN.md
        // 5.3); tests set allow_unconverted_powder to run it anyway.
        if (blocks.empty() && ! opts.allow_unconverted_powder)
            throw DIWConfigError("u-MEX powder infill: this G-code has no powder plan (;DIW: records). Export it again "
                                 "from OrcaSlicer with u-MEX powder infill on; the plan is written during the export.");
    }
    Translator t(p, opts.stub_powder_stage, opts.v2_parity);
    return t.run(lines, blocks);
}

void diw_translate_gcode_file(const std::string &path, const DynamicPrintConfig &config)
{
    std::string gcode;
    {
        boost::nowide::ifstream in(path, std::ios::binary);
        if (! in)
            throw Slic3r::RuntimeError("DIW translation: cannot open " + path);
        std::ostringstream ss;
        ss << in.rdbuf();
        gcode = ss.str();
    }
    const std::string translated = diw_translate_gcode(gcode, config);
    if (translated.size() == gcode.size() && translated == gcode)
        return;
    boost::nowide::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (! out)
        throw Slic3r::RuntimeError("DIW translation: cannot write " + path);
    out.write(translated.data(), std::streamsize(translated.size()));
    if (! out)
        throw Slic3r::RuntimeError("DIW translation: write failed for " + path);
}

} // namespace Slic3r
