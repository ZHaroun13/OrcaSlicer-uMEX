// diw_translate: runs the built-in DIW / u-MEX G-code translation (libslic3r/GCode/DIWTranslator) on an
// existing G-code file, with default settings plus key=value overrides. Used by the regression tests that
// compare it with the v2 post-processor (diw/tests).
//
// usage: diw_translate [flags] <in.gcode> <out.gcode> [diw_key=value ...]
//   --config-from-gcode         start from the settings in the input file's config block
//   --allow-unconverted-powder  run powder mode although core layers are not converted yet (tests only)
//   diw_translate --pyrepr      read 16-hex-digit IEEE-754 bit patterns from stdin and print
//                               "<py_repr> <py_g>" for each (checks DIWPyCompat against Python)
// Exit codes: 0 ok, 1 I/O or other error, 2 usage, 3 invalid u-MEX settings (DIWConfigError).

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode/DIWPyCompat.hpp"
#include "libslic3r/GCode/DIWTranslator.hpp"
#include "libslic3r/GCode/DIWUMexParams.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/iostream.hpp>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

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

int main(int argc, char **argv)
{
    boost::nowide::args nowide_args(argc, argv);
    bool config_from_gcode = false;
    DIWTranslateOptions opts;
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--pyrepr")
            return pyrepr_mode();
        if (a == "--config-from-gcode")
            config_from_gcode = true;
        else if (a == "--allow-unconverted-powder")
            opts.allow_unconverted_powder = true;
        else
            args.push_back(a);
    }
    if (args.size() < 2) {
        boost::nowide::cerr << "usage: diw_translate [--config-from-gcode] [--allow-unconverted-powder] <in.gcode> <out.gcode> [key=value ...]\n";
        return 2;
    }
    try {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        if (config_from_gcode)
            cfg.load_from_gcode_file(args[0], ForwardCompatibilitySubstitutionRule::EnableSilent);
        cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
        ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Disable);
        for (size_t i = 2; i < args.size(); ++i) {
            const size_t eq = args[i].find('=');
            if (eq == std::string::npos) {
                boost::nowide::cerr << "bad override (key=value expected): " << args[i] << "\n";
                return 2;
            }
            cfg.set_deserialize(args[i].substr(0, eq), args[i].substr(eq + 1), ctx);
        }

        std::string in;
        {
            boost::nowide::ifstream f(args[0], std::ios::binary);
            if (! f) { boost::nowide::cerr << "cannot open " << args[0] << "\n"; return 1; }
            std::ostringstream ss;
            ss << f.rdbuf();
            in = ss.str();
        }
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
