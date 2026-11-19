#ifndef slic3r_GCode_DIWPowderReport_hpp_
#define slic3r_GCode_DIWPowderReport_hpp_

// u-MEX time and powder mass report (diw/POWDER_PLAN.md increment 6).
//
// The report is computed from the TRANSLATED G-code, i.e. from what the controller will run, and written as comment
// lines in the footer. It never changes a motion byte. The definitions (details in diw/POWDER_PLAN.md):
//
//  added time    the time the translator's sequences add on top of the slicer's own estimate, in seconds at the
//                nominal feed (path length / F, no acceleration): every G04 dwell the translator wrote, plus the
//                moves of the wipe (S2), the UV stop (S3), the powder pass (S4: transit, hop and hatch) and the
//                compaction (S5), plus the two positioning moves of a paste-mode UV cure.
//  powder mass   m_dot [g/min] / 60 * deposition time. A feeder window (M62 powder pin Q1 .. Q0) deposits for
//                (window time - lag_on + lag_off): that is run length / F for a hatch run, and
//                max(t_feed, lag_off) for a point dwell, the same terms the sequencer's over-delivery warnings use.

#include <string>
#include <vector>

namespace Slic3r {
namespace DIW {

struct TranslatorParams;

struct PowderReportParams
{
    int    powder_pin = -1;   // M62 pin of the powder feeder; -1 = no powder mode
    int    comp_pin   = -1;   // M62 pin of the compaction (vibrator) table
    int    uv_pin     = -1;   // M62 pin of the UV array; -1 = none
    double m_dot      = 0.;   // g/min
    double lag_on     = 0.;   // s
    double lag_off    = 0.;   // s
};

// diw_powder_pin, diw_compaction_pin, the pin of diw_uv_on_gcode, diw_powder_mass_flow and the two lags; a paste job
// (no powder mode) has no powder and compaction pin.
PowderReportParams powder_report_params(const TranslatorParams &p);

struct PowderReport
{
    // The parts of the added time, in seconds. total() is their sum.
    enum Part { PressureRelief, Wipe, Uv, PowderFeed, Compaction, Other, PartCount };
    double part_s[PartCount] = {};
    // The same time by kind: G04 dwells and moves. total() is also dwell_s + move_s.
    double dwell_s = 0., move_s = 0.;

    double powder_mass_g          = 0.;
    double powder_deposition_s    = 0.;
    int    powder_layers          = 0;   // S4 stages
    int    powder_windows         = 0;   // feeder on/off pairs (hatch runs and point dwells)
    double m_dot                  = 0.;  // g/min, as used
    double slicer_estimate_s      = -1.; // "estimated printing time (normal mode)" of the file, -1 = not found
    // Increment 7: the slicer put the added time into its own estimate and says so in a comment at the end of the
    // raw file (estimate_marker()); then slicer_estimate_s is the estimate WITHOUT the added time. -1 = no marker.
    double estimate_includes_s    = -1.;
    // The added time per layer: index = number of ;LAYER_CHANGE lines seen before the time (0 = before the first
    // layer change). Sums to total_s().
    std::vector<double> layer_added_s;

    double total_s() const { double t = 0.; for (double v : part_s) t += v; return t; }
};

// "; DIW estimate includes added time = <s> s": written by the slicer at the end of the raw file when its time
// estimate already contains the added time (so the report does not count it twice).
std::string estimate_marker(double added_s);

// Scans the translated G-code. Throws Slic3r::RuntimeError with the line number for a malformed M62 or G04 word.
PowderReport analyze_translated_gcode(const std::string &gcode, const PowderReportParams &params);

// The footer: comment lines (each ends in '\n', no brackets or parentheses, so PlanetCNC reads them as comments).
std::string report_footer(const PowderReport &r);
// One-line-per-fact summary for a log or stdout (no leading ';').
std::vector<std::string> report_summary_lines(const PowderReport &r);

// "1h 02m 03s" / "2m 03s" / "5s" of a duration in seconds (rounded to whole seconds).
std::string format_duration(double seconds);

} // namespace DIW
} // namespace Slic3r

#endif // slic3r_GCode_DIWPowderReport_hpp_
