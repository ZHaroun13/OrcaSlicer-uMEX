// diw_translate: runs the built-in DIW / u-MEX G-code translation (libslic3r/GCode/DIWTranslator) on an
// existing G-code file, with default settings plus key=value overrides. Used by the regression tests that
// compare it with the v2 post-processor (diw/tests).
//
// usage: diw_translate [flags] <in.gcode> <out.gcode> [diw_key=value ...]
//   --config-from-gcode         start from the settings in the input file's config block
//   --allow-unconverted-powder  run powder mode although the input has no ";DIW:" record block (tests only)
//   --stub-powder-stage         stage 4 of a converted core layer emits only its header comment line
//                               (DIWTranslateOptions::stub_powder_stage; tests only, increment 3a;
//                               without it stage 4 emits the powder runs, increment 3b)
//   --v2-parity                 a non-carrier st=open block is left as paste with v2's warning instead of
//                               stopping the translation (DIWTranslateOptions::v2_parity; tests that translate
//                               records made by v2 only, PRODUCT_DESIGN.md 5.3)
//   --report                    append the added-time and powder-mass report as comment lines to the output and print
//                               them on stdout as "DIW report: ..." (DIWTranslateOptions::append_report; off by default
//                               so the v2 parity runs keep their bytes, POWDER_PLAN.md increment 6)
//   --product-defaults          keep the product default A mode (incremental, G91.2). Without it this tool uses v2's flow_scaled
//                               unless diw_a_axis_mode is given (override or config block), so the v2 gates stay valid
//   --m62-dwell <s>             G04 before every M62 (audit #8; DIWTranslateOptions::m62_pre_dwell)
//   --suckback <E mm> [--suckback-feed <mm/min>]   suck-back at the end of an extrusion (audit #16)
//   --retract-f-max <mm/min>    feed clamp of a B retraction (audit #17)
//   diw_translate [--config-from-gcode <in.gcode>] --validate [key=value ...]
//                               print diw_validate() of the settings (PRODUCT_DESIGN.md 6.2): one line
//                               "error<TAB><opt_key><TAB><message>" or nothing, then one line
//                               "warning<TAB><opt_key><TAB><message>" per warning; exit 3 on an error, else 0
//   diw_translate --pyrepr      read 16-hex-digit IEEE-754 bit patterns from stdin and print
//                               "<py_repr> <py_g>" for each (checks DIWPyCompat against Python)
//   diw_translate --pyhypot     per stdin line of hex doubles dx1 dy1 dx2 dy2 ...: hex of py_hypot of each
//                               pair, then hex of PySum over them (checks math.hypot / sum() parity)
//   diw_translate --dump-records <in.gcode>
//                               parse the ";DIW:" powder records and print the canonical dump
//                               (diw/POWDER_RECORDS.md section 6); exit 0 even when errors are listed
//   diw_translate --rewrite-records <in.gcode>
//                               parse the records and print write_block() of every accepted block
//   diw_translate [--config-from-gcode] --fingerprint <in.gcode> [key=value ...]
//                               print the plan fingerprint string and its 8-digit hex, one per line
//   diw_translate --plan-json <in.json> <out.txt> <debug.json>
//                               run the native powder planner (GCode/DIWPowderPlanner, PLANNER_DESIGN.md
//                               section 7) on PlanInput JSON (one case, or {"cases": [...]}); writes the record
//                               block of every case to out.txt and the cavity / Omega / narrow / revert WKT and
//                               the per-component hatch data to debug.json (diw/tests/compare_planner.py)
//   diw_translate [--config-from-gcode] --plan-records [--feed-size N] <in.gcode> <out.gcode> [key=value ...]
//                               offline shadow binder (PLANNER_DESIGN.md section 9): writes in.gcode with the
//                               native ";DIW:" record blocks inserted (text-only adapter, or native captures
//                               reloaded from ORCA_DIW_SHADOW_DUMP); summary and warnings on stderr.
//                               Shadow mode (increment 4) unless --product, whatever ORCA_DIW_POWDER_MODE says.
//     --product                 product mode (PRODUCT_DESIGN.md 2): native authority, strictness and sparse
//                               infill suppression; a failure prints the export-stopping message and exits 3
//                               (the output is still written)
//     --no-suppress             with --product: no suppression (ShadowOptions::suppress = false)
// Records output goes to stdout in binary mode (LF only).
// Exit codes: 0 ok, 1 I/O or other error, 2 usage, 3 invalid u-MEX settings (DIWConfigError).

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode/DIWPowderBlock.hpp"
#include "libslic3r/GCode/DIWPowderPlanner.hpp"
#include "libslic3r/GCode/DIWPowderShadow.hpp"
#include "libslic3r/GCode/DIWPyCompat.hpp"
#include "libslic3r/GCode/DIWTranslator.hpp"
#include "libslic3r/GCode/DIWUMexParams.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/iostream.hpp>

#include <nlohmann/json.hpp>

#include <charconv>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

using namespace Slic3r;

static int pyrepr_mode()
{
    std::string hex;
    while (boost::nowide::cin >> hex) {
        const uint64_t bits = std::stoull(hex, nullptr, 16);
        double v;
        std::memcpy(&v, &bits, sizeof(v));
        boost::nowide::cout << DIW::py_repr(v) << " " << DIW::py_g(v) << "\n";
    }
    return 0;
}

// Each stdin line: hex bit patterns of an even count of doubles (dx1 dy1 dx2 dy2 ...). Prints the hex bits of
// py_hypot of every pair, then of PySum over those hypots (Python: math.hypot / sum()).
static int pyhypot_mode()
{
    auto to_d = [](const std::string &hex) {
        const uint64_t bits = std::stoull(hex, nullptr, 16);
        double v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    };
    auto to_hex = [](double v) {
        uint64_t bits;
        std::memcpy(&bits, &v, sizeof(v));
        char buf[24];
        snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) bits);
        return std::string(buf);
    };
    std::string line;
    while (std::getline(boost::nowide::cin, line)) {
        std::istringstream ss(line);
        std::vector<double> v;
        for (std::string h; ss >> h;) v.push_back(to_d(h));
        DIW::PySum sum;
        std::string outl;
        for (size_t i = 0; i + 1 < v.size(); i += 2) {
            const double h = DIW::py_hypot(v[i], v[i + 1]);
            sum.add(h);
            outl += to_hex(h) + " ";
        }
        boost::nowide::cout << outl << to_hex(sum.result()) << "\n";
    }
    return 0;
}

static bool read_file(const std::string &path, std::string &out)
{
    boost::nowide::ifstream f(path, std::ios::binary);
    if (! f) {
        boost::nowide::cerr << "cannot open " << path << "\n";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

// Writes raw bytes to stdout (binary mode on Windows: no CRLF translation).
static int write_stdout(const std::string &s)
{
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);
#endif
    const size_t n = std::fwrite(s.data(), 1, s.size(), stdout);
    std::fflush(stdout);
    return n == s.size() ? 0 : 1;
}

static bool g_product_defaults = false;

// Default settings, optionally the input file's config block, diw_printer on, then key=value overrides.
// Returns false (after printing the reason) for a malformed override.
static bool build_config(DynamicPrintConfig &cfg, const std::string &in_path, bool config_from_gcode,
                         const std::vector<std::string> &overrides)
{
    cfg = DynamicPrintConfig::full_print_config();
    if (config_from_gcode)
        cfg.load_from_gcode_file(in_path, ForwardCompatibilitySubstitutionRule::EnableSilent);
    cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
    // The v2 engine knows only the flow_scaled / proportional / cumulative A modes, and every comparison gate runs
    // against it. The product default is now "incremental" (diw/AUDIT_FIX.md), so this tool keeps v2's flow_scaled
    // unless the A mode is chosen explicitly (override, or the config block of the input with --config-from-gcode)
    // or --product-defaults is given.
    if (! g_product_defaults) {
        bool explicit_a = false;
        for (const std::string &o : overrides)
            if (o.compare(0, 16, "diw_a_axis_mode=") == 0)
                explicit_a = true;
        if (config_from_gcode && ! explicit_a) {
            std::string text;
            if (read_file(in_path, text) && text.find("; diw_a_axis_mode") != std::string::npos)
                explicit_a = true;
        }
        if (! explicit_a)
            cfg.set_key_value("diw_a_axis_mode", new ConfigOptionEnumsGeneric{ int(diwaFlowScaled) });
    }
    ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Disable);
    for (const std::string &o : overrides) {
        const size_t eq = o.find('=');
        if (eq == std::string::npos) {
            boost::nowide::cerr << "bad override (key=value expected): " << o << "\n";
            return false;
        }
        cfg.set_deserialize(o.substr(0, eq), o.substr(eq + 1), ctx);
    }
    return true;
}

static std::string trim_ascii(const std::string &s)
{
    const char  *ws = " \t\n\v\f\r";
    const size_t b  = s.find_first_not_of(ws);
    if (b == std::string::npos)
        return std::string();
    return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

// ------------------------------------------------------------------------------------------------ --plan-json

// A JSON number, or a string holding Python repr() of a float (parsed with a correctly rounded parser, so the
// value is exactly Python's).
static double jnum(const nlohmann::json &j)
{
    if (j.is_string()) {
        const std::string s = j.get<std::string>();
        double            v = 0.;
        const auto        r = std::from_chars(s.data(), s.data() + s.size(), v, std::chars_format::general);
        if (r.ec != std::errc() || r.ptr != s.data() + s.size()) {
            if (s == "inf") return std::numeric_limits<double>::infinity();
            if (s == "-inf") return -std::numeric_limits<double>::infinity();
            throw std::runtime_error("bad number in plan JSON: " + s);
        }
        return v;
    }
    return j.get<double>();
}

static std::vector<Vec2d> jpts_mm(const nlohmann::json &j)
{
    std::vector<Vec2d> out;
    for (const auto &p : j) out.emplace_back(jnum(p.at(0)), jnum(p.at(1)));
    return out;
}

static Points jpts_nm(const nlohmann::json &j)
{
    Points out;
    for (const auto &p : j) out.emplace_back(coord_t(p.at(0).get<long long>()), coord_t(p.at(1).get<long long>()));
    return out;
}

static std::vector<DIW::TextPath> jtext(const nlohmann::json &c, const char *key)
{
    std::vector<DIW::TextPath> out;
    if (! c.contains(key))
        return out;
    for (const auto &j : c.at(key)) {
        DIW::TextPath t;
        t.ptype  = j.value("ptype", std::string());
        t.kind   = j.value("kind", std::string());
        t.pts_mm = jpts_mm(j.at("pts"));
        for (const Vec2d &v : t.pts_mm) t.pts_nm.points.push_back(DIW::to_grid(v));
        for (const auto &v : j.at("seg_de")) t.seg_de.push_back(jnum(v));
        for (const auto &v : j.at("seg_feed")) t.seg_feed.push_back(jnum(v));
        if (j.contains("seg_width"))
            for (const auto &v : j.at("seg_width")) t.seg_width.push_back(jnum(v));
        t.width = jnum(j.at("width"));
        t.z     = jnum(j.at("z"));
        if (j.contains("travel_feed") && ! j.at("travel_feed").is_null())
            t.travel_feed = jnum(j.at("travel_feed"));
        out.push_back(std::move(t));
    }
    return out;
}

static std::string wkt(const ExPolygons &ee)
{
    if (ee.empty())
        return "MULTIPOLYGON EMPTY";
    auto ring = [](const Points &pts) {
        std::string s = "(";
        for (size_t i = 0; i <= pts.size(); ++i) {
            const Point &p = pts[i % pts.size()];
            if (i) s += ", ";
            s += DIW::py_repr(double(p.x()) * 1e-6) + " " + DIW::py_repr(double(p.y()) * 1e-6);
        }
        return s + ")";
    };
    std::string s = "MULTIPOLYGON (";
    for (size_t k = 0; k < ee.size(); ++k) {
        if (k) s += ", ";
        s += "(" + ring(ee[k].contour.points);
        for (const Polygon &h : ee[k].holes) s += ", " + ring(h.points);
        s += ")";
    }
    return s + ")";
}

static int plan_json_mode(const std::string &in_path, const std::string &out_path, const std::string &dbg_path)
{
    std::string text;
    if (! read_file(in_path, text))
        return 1;
    const nlohmann::json root = nlohmann::json::parse(text);
    std::vector<nlohmann::json> cases;
    if (root.contains("cases"))
        for (const auto &c : root.at("cases")) cases.push_back(c);
    else
        cases.push_back(root);
    std::string    blocks;
    nlohmann::json dbg_cases = nlohmann::json::array();
    for (const nlohmann::json &c : cases) {
        nlohmann::json d;
        d["k"] = c.value("k", 1);
        try {
            DIW::UMexParams U;
            const auto &ju   = c.at("U");
            U.delta          = jnum(ju.at("delta"));
            U.hatch_angle    = jnum(ju.at("hatch_angle"));
            U.point_len      = jnum(ju.at("point_len"));
            U.rho_tap        = jnum(ju.at("rho_tap"));
            U.eta            = jnum(ju.at("eta"));
            U.m_dot          = jnum(ju.at("m_dot"));
            U.paste_min_area = jnum(ju.at("paste_min_area"));
            U.width_fallback = jnum(ju.at("width_fallback"));
            DIW::PlanInput in;
            in.U     = &U;
            in.h     = jnum(c.at("h"));
            in.s_eff = jnum(c.at("s_eff"));
            if (c.contains("rings"))
                for (const auto &r : c.at("rings")) in.rings.push_back({ Polygon(jpts_nm(r)) });
            if (c.contains("open_outer"))
                for (const auto &r : c.at("open_outer")) in.open_outer.push_back(jpts_mm(r));
            if (c.contains("native_walls"))
                for (const auto &w : c.at("native_walls"))
                    in.native_walls.push_back({ Polyline(jpts_nm(w.at("pts"))), jnum(w.at("width")), w.value("closed", false) });
            in.text_walls  = jtext(c, "text_walls");
            in.text_paste  = jtext(c, "text_paste");
            in.text_powder = jtext(c, "text_powder");
            DIW::PlanDebug     dbg;
            DIW::PowderLayerRec rec = DIW::plan_powder_layer_native(in, &dbg);
            rec.k  = c.value("k", 1);
            rec.z  = jnum(c.value("z", nlohmann::json(0.0)));
            rec.h  = in.h;
            rec.s  = in.s_eff;
            rec.F  = jnum(c.value("F", nlohmann::json(0.0)));
            rec.fu = jnum(c.value("fu", nlohmann::json(0.0)));
            rec.fp = uint32_t(c.value("fp", 0LL));
            blocks += DIW::write_block(rec);
            d["error"]  = nullptr;
            d["cavity"] = wkt(dbg.cavity);
            d["omega"]  = wkt(dbg.omega);
            d["narrow"] = wkt(dbg.narrow);
            d["revert"] = wkt(dbg.revert);
            nlohmann::json comps = nlohmann::json::array();
            for (const DIW::PlanComponent &pc : dbg.components)
                comps.push_back({ { "rows", pc.rows }, { "H", DIW::py_repr(pc.H) }, { "runs", pc.runs },
                                  { "point", pc.point }, { "total", DIW::py_repr(pc.total) } });
            d["components"] = comps;
            d["notes"]      = dbg.notes;
        } catch (const std::exception &ex) {
            d["error"] = std::string(ex.what());
        }
        dbg_cases.push_back(d);
    }
    {
        boost::nowide::ofstream f(out_path, std::ios::binary | std::ios::trunc);
        f.write(blocks.data(), std::streamsize(blocks.size()));
        if (! f) return 1;
    }
    const std::string dj = nlohmann::json{ { "cases", dbg_cases } }.dump();
    boost::nowide::ofstream f(dbg_path, std::ios::binary | std::ios::trunc);
    f.write(dj.data(), std::streamsize(dj.size()));
    return f ? 0 : 1;
}

// ------------------------------------------------------------------------------------------------ --plan-records

// Offline binder (PLANNER_DESIGN.md section 9): the shadow binder over a finished G-code file, fed in pieces of
// feed_size bytes. Existing ";DIW:" record lines are removed first (as the translator does). Chunks without
// placeholders use the text-only adapter; with placeholders, the native captures are reloaded from
// ORCA_DIW_SHADOW_DUMP when it is set. The summary and the warnings go to stderr.
static int plan_records_mode(const std::string &in_path, const std::string &out_path, const DynamicPrintConfig &cfg, size_t feed_size,
                             bool product, bool no_suppress)
{
    std::string raw;
    if (! read_file(in_path, raw))
        return 1;
    std::string in;
    in.reserve(raw.size());
    for (size_t pos = 0; pos < raw.size();) {
        const size_t nl  = raw.find('\n', pos);
        const size_t end = nl == std::string::npos ? raw.size() : nl + 1;
        if (raw.compare(pos, 5, ";DIW:") != 0)
            in.append(raw, pos, end - pos);
        pos = end;
    }
    const DIW::TranslatorParams p = diw_read_translator_params(cfg);
    if (! p.umex) {
        boost::nowide::cerr << "--plan-records needs a u-MEX powder job (diw_powder_infill=1)\n";
        return 2;
    }
    DIW::ShadowOptions o = DIW::ShadowOptions::from_env();
    // PRODUCT_DESIGN.md 2: shadow mode unless --product, whatever ORCA_DIW_POWDER_MODE says (the increment-4 T2(d)
    // replay and G9's reference command keep their meaning)
    o.mode               = product ? DIW::ShadowOptions::Mode::product : DIW::ShadowOptions::Mode::shadow;
    if (no_suppress)
        o.suppress = false;
    o.text_only          = true;
    if (const auto *hp = cfg.option<ConfigOptionBool>("high_precision_gcode"); hp != nullptr && hp->value)
        o.xy_decimals = 4;
    DIW::PowderShadow sh(p, o);
    if (! o.dump_dir.empty() && in.find(";DIWP ") != std::string::npos)
        boost::nowide::cerr << "reloaded " << sh.load_dump_dir(o.dump_dir) << " native capture(s) from " << o.dump_dir << "\n";
    std::string out;
    out.reserve(in.size() + in.size() / 4);
    const auto sink = [&out](std::string_view s) { out.append(s.data(), s.size()); };
    if (feed_size == 0)
        feed_size = 1;
    for (size_t pos = 0; pos < in.size(); pos += feed_size)
        sh.feed(std::string_view(in).substr(pos, feed_size), sink);
    sh.finish(sink);
    boost::nowide::cerr << sh.summary() << "\n";
    for (const std::string &w : sh.warnings()) boost::nowide::cerr << "warning: " << w << "\n";
    for (const std::string &w : sh.fallback_log()) boost::nowide::cerr << "info: " << w << "\n";
    boost::nowide::ofstream f(out_path, std::ios::binary | std::ios::trunc);
    f.write(out.data(), std::streamsize(out.size()));
    if (! f)
        return 1;
    if (sh.failed()) {
        // product mode: the export would have stopped (PRODUCT_DESIGN.md 5.1); the output is still written
        boost::nowide::cerr << "error: " << sh.failure_message() << "\n";
        return 3;
    }
    return 0;
}

int main(int argc, char **argv)
{
    boost::nowide::args nowide_args(argc, argv);
    bool config_from_gcode = false;
    enum class Mode { Translate, DumpRecords, RewriteRecords, Fingerprint, PlanRecords, Validate } mode = Mode::Translate;
    size_t feed_size = 65536;
    bool   plan_product = false, plan_no_suppress = false;
    DIWTranslateOptions opts;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--pyrepr")
            return pyrepr_mode();
        if (a == "--pyhypot")
            return pyhypot_mode();
        if (a == "--plan-json") {
            if (i + 3 >= argc) {
                boost::nowide::cerr << "usage: diw_translate --plan-json <in.json> <out.txt> <debug.json>\n";
                return 2;
            }
            try {
                return plan_json_mode(argv[i + 1], argv[i + 2], argv[i + 3]);
            } catch (const std::exception &ex) {
                boost::nowide::cerr << "error: " << ex.what() << "\n";
                return 1;
            }
        }
        if (a == "--config-from-gcode")
            config_from_gcode = true;
        else if (a == "--allow-unconverted-powder")
            opts.allow_unconverted_powder = true;
        else if (a == "--stub-powder-stage")
            opts.stub_powder_stage = true;
        else if (a == "--v2-parity")
            opts.v2_parity = true;
        else if (a == "--report")
            opts.append_report = true;   // increment 6: report comments in the footer + summary on stdout
        else if (a == "--product-defaults")
            g_product_defaults = true;
        else if (a == "--m62-dwell" && i + 1 < argc)
            opts.m62_pre_dwell = std::stod(argv[++i]);
        else if (a == "--suckback" && i + 1 < argc)
            opts.suckback_e = std::stod(argv[++i]);
        else if (a == "--suckback-feed" && i + 1 < argc)
            opts.suckback_f = std::stod(argv[++i]);
        else if (a == "--retract-f-max" && i + 1 < argc)
            opts.retract_f_max = std::stod(argv[++i]);
        else if (a == "--validate")
            mode = Mode::Validate;
        else if (a == "--dump-records")
            mode = Mode::DumpRecords;
        else if (a == "--rewrite-records")
            mode = Mode::RewriteRecords;
        else if (a == "--fingerprint")
            mode = Mode::Fingerprint;
        else if (a == "--plan-records")
            mode = Mode::PlanRecords;
        else if (a == "--product")
            plan_product = true;
        else if (a == "--no-suppress")
            plan_no_suppress = true;
        else if (a == "--feed-size" && i + 1 < argc)
            feed_size = size_t(std::stoull(argv[++i]));
        else
            args.push_back(a);
    }
    try {
        if (mode == Mode::DumpRecords || mode == Mode::RewriteRecords) {
            if (args.size() != 1) {
                boost::nowide::cerr << "usage: diw_translate --dump-records|--rewrite-records <in.gcode>\n";
                return 2;
            }
            std::string in;
            if (! read_file(args[0], in))
                return 1;
            const DIW::ParsedRecords recs = DIW::parse_records(in);
            if (mode == Mode::DumpRecords)
                return write_stdout(DIW::dump_records(recs));
            std::string out;
            for (const DIW::PowderLayerRec &b : recs.blocks)
                out += DIW::write_block(b);
            return write_stdout(out);
        }
        if (mode == Mode::Fingerprint) {
            if (args.empty()) {
                boost::nowide::cerr << "usage: diw_translate [--config-from-gcode] --fingerprint <in.gcode> [key=value ...]\n";
                return 2;
            }
            DynamicPrintConfig cfg;
            if (! build_config(cfg, args[0], config_from_gcode, std::vector<std::string>(args.begin() + 1, args.end())))
                return 2;
            std::string uv_on;
            if (const auto *o = cfg.option<ConfigOptionString>("diw_uv_on_gcode"))
                uv_on = trim_ascii(o->value);
            const DIW::UMexParams U = DIW::read_umex_params(cfg, uv_on).first;
            return write_stdout(DIW::fingerprint_string(U) + "\n" + DIW::hex8(DIW::plan_fingerprint(U)) + "\n");
        }

        if (mode == Mode::Validate) {
            // the settings only: [in.gcode with --config-from-gcode] then key=value overrides
            if (config_from_gcode && args.empty()) {
                boost::nowide::cerr << "usage: diw_translate [--config-from-gcode <in.gcode>] --validate [key=value ...]\n";
                return 2;
            }
            DynamicPrintConfig cfg;
            const std::string in_path = config_from_gcode ? args[0] : std::string();
            if (! build_config(cfg, in_path, config_from_gcode,
                               std::vector<std::string>(args.begin() + (config_from_gcode ? 1 : 0), args.end())))
                return 2;
            const DIWValidation v = diw_validate(cfg);
            std::string out;
            if (! v.error.empty())
                out += "error\t" + v.opt_key + "\t" + v.error + "\n";
            for (const auto &[msg, key] : v.warnings)
                out += "warning\t" + key + "\t" + msg + "\n";
            if (write_stdout(out) != 0)
                return 1;
            return v.error.empty() ? 0 : 3;
        }
        if (mode == Mode::PlanRecords) {
            if (args.size() < 2) {
                boost::nowide::cerr << "usage: diw_translate [--config-from-gcode] --plan-records [--product [--no-suppress]] [--feed-size N] <in.gcode> <out.gcode> [key=value ...]\n";
                return 2;
            }
            DynamicPrintConfig cfg;
            if (! build_config(cfg, args[0], config_from_gcode, std::vector<std::string>(args.begin() + 2, args.end())))
                return 2;
            return plan_records_mode(args[0], args[1], cfg, feed_size, plan_product, plan_no_suppress);
        }
        if (args.size() < 2) {
            boost::nowide::cerr << "usage: diw_translate [--config-from-gcode] [--allow-unconverted-powder] [--stub-powder-stage] [--v2-parity] <in.gcode> <out.gcode> [key=value ...]\n";
            return 2;
        }
        DynamicPrintConfig cfg;
        if (! build_config(cfg, args[0], config_from_gcode, std::vector<std::string>(args.begin() + 2, args.end())))
            return 2;
        std::string in;
        if (! read_file(args[0], in))
            return 1;
        std::vector<std::string> report;
        if (opts.append_report)
            opts.report_lines = &report;
        const std::string out = diw_translate_gcode(in, cfg, opts);
        boost::nowide::ofstream f(args[1], std::ios::binary | std::ios::trunc);
        f.write(out.data(), std::streamsize(out.size()));
        if (! f)
            return 1;
        std::string summary;
        for (const std::string &l : report)
            summary += "DIW report: " + l + "\n";
        return summary.empty() ? 0 : write_stdout(summary);
    } catch (const DIWConfigError &ex) {
        boost::nowide::cerr << "error: " << ex.what() << "\n";
        return 3;
    } catch (const std::exception &ex) {
        boost::nowide::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
