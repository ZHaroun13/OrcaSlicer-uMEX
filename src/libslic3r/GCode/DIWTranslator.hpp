#ifndef slic3r_GCode_DIWTranslator_hpp_
#define slic3r_GCode_DIWTranslator_hpp_

// DIW / u-MEX G-code translation (milestone 2 of diw/FEATURE_MAP.md).
//
// Rewrites finished FDM G-code for a DIW machine on a PlanetCNC TNG controller:
//   - virtual filament E  ->  syringe plunger B (from the paste volume each move needs)
//                            + printhead auger A (flow-scaled speed, proportional speed or cumulative position)
//   - PlanetCNC sanitation: FDM-only commands commented out, [ ] in comments -> ( ), G28 / M84 removed,
//     empty G4 removed, retraction limits
//   - fixed UV sequences: start-up self-check flicker, inter-layer cure, final cure, end-of-print cure
//
// It is a line-for-line port of the paste path of the v2 post-processor (diw_engine.py 2.0.0,
// DIWPostProcessor with powder off) and must give the same output for the same settings; the u-MEX powder
// stages (milestone 4) are not part of it yet.
//
// It runs on the exported file only (GUI export through run_post_process_scripts(), CLI after
// export_gcode()), so the preview keeps showing the untranslated FDM moves.

#include <string>

namespace Slic3r {

class DynamicPrintConfig;

// First line of a translated file. A file that already starts with it is left unchanged.
extern const char *DIW_TRANSLATION_HEADER;

// True when the config describes a DIW / u-MEX machine (printer setting "diw_printer").
bool diw_translation_enabled(const DynamicPrintConfig &config);

// Translates G-code text. Line endings of the output are '\n'.
std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config);

// Translates the file in place. Throws Slic3r::RuntimeError on I/O failure.
void diw_translate_gcode_file(const std::string &path, const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_GCode_DIWTranslator_hpp_
