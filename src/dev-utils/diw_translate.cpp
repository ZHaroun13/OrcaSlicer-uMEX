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
// Records output goes to stdout in binary mode (LF only).
// Exit codes: 0 ok, 1 I/O or other error, 2 usage, 3 invalid u-MEX settings (DIWConfigError).

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode/DIWPowderBlock.hpp"
#include "libslic3r/GCode/DIWPyCompat.hpp"
#include "libslic3r/GCode/DIWTranslator.hpp"
#include "libslic3r/GCode/DIWUMexParams.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/iostream.hpp>

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

// Default settings, optionally the input file's config block, diw_printer on, then key=value overrides.
// Returns false (after printing the reason) for a malformed override.
static bool build_config(DynamicPrintConfig &cfg, const std::string &in_path, bool config_from_gcode,
                         const std::vector<std::string> &overrides)
{
    cfg = DynamicPrintConfig::full_print_config();
    if (config_from_gcode)
        cfg.load_from_gcode_file(in_path, ForwardCompatibilitySubstitutionRule::EnableSilent);
    cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
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

int main(int argc, char **argv)
{
    boost::nowide::args nowide_args(argc, argv);
    bool config_from_gcode = false;
    enum class Mode { Translate, DumpRecords, RewriteRecords, Fingerprint } mode = Mode::Translate;
    DIWTranslateOptions opts;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--pyrepr")
            return pyrepr_mode();
        if (a == "--pyhypot")
            return pyhypot_mode();
        if (a == "--config-from-gcode")
            config_from_gcode = true;
        else if (a == "--allow-unconverted-powder")
            opts.allow_unconverted_powder = true;
        else if (a == "--stub-powder-stage")
            opts.stub_powder_stage = true;
        else if (a == "--dump-records")
            mode = Mode::DumpRecords;
        else if (a == "--rewrite-records")
            mode = Mode::RewriteRecords;
        else if (a == "--fingerprint")
            mode = Mode::Fingerprint;
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

        if (args.size() < 2) {
            boost::nowide::cerr << "usage: diw_translate [--config-from-gcode] [--allow-unconverted-powder] [--stub-powder-stage] <in.gcode> <out.gcode> [key=value ...]\n";
            return 2;
        }
        DynamicPrintConfig cfg;
        if (! build_config(cfg, args[0], config_from_gcode, std::vector<std::string>(args.begin() + 2, args.end())))
            return 2;
        std::string in;
        if (! read_file(args[0], in))
            return 1;
        const std::string out = diw_translate_gcode(in, cfg, opts);
        boost::nowide::ofstream f(args[1], std::ios::binary | std::ios::trunc);
        f.write(out.data(), std::streamsize(out.size()));
        return f ? 0 : 1;
    } catch (const DIWConfigError &ex) {
        boost::nowide::cerr << "error: " << ex.what() << "\n";
        return 3;
    } catch (const std::exception &ex) {
        boost::nowide::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
