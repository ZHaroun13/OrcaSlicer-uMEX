// diw_translate: runs the built-in DIW / u-MEX G-code translation (libslic3r/GCode/DIWTranslator) on an
// existing G-code file, with default settings plus key=value overrides. Used by the regression tests that
// compare it with the v2 post-processor (diw/tests).
//
// usage: diw_translate <in.gcode> <out.gcode> [diw_key=value ...]

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode/DIWTranslator.hpp"

#include <boost/nowide/args.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/iostream.hpp>

#include <sstream>
#include <string>

using namespace Slic3r;

int main(int argc, char **argv)
{
    boost::nowide::args nowide_args(argc, argv);
    if (argc < 3) {
        boost::nowide::cerr << "usage: diw_translate <in.gcode> <out.gcode> [key=value ...]\n";
        return 2;
    }
    try {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        cfg.set_key_value("diw_printer", new ConfigOptionBool(true));
        ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Disable);
        for (int i = 3; i < argc; ++i) {
            const std::string kv = argv[i];
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) {
                boost::nowide::cerr << "bad override (key=value expected): " << kv << "\n";
                return 2;
            }
            cfg.set_deserialize(kv.substr(0, eq), kv.substr(eq + 1), ctx);
        }

        std::string in;
        {
            boost::nowide::ifstream f(argv[1], std::ios::binary);
            if (! f) { boost::nowide::cerr << "cannot open " << argv[1] << "\n"; return 1; }
            std::ostringstream ss;
            ss << f.rdbuf();
            in = ss.str();
        }
        const std::string out = diw_translate_gcode(in, cfg);
        boost::nowide::ofstream f(argv[2], std::ios::binary | std::ios::trunc);
        f.write(out.data(), std::streamsize(out.size()));
        return f ? 0 : 1;
    } catch (const std::exception &ex) {
        boost::nowide::cerr << "error: " << ex.what() << "\n";
        return 1;
    }
}
