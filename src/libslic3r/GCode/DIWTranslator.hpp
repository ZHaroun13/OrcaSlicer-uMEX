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
// DIWPostProcessor with powder off) and must give the same output for the same settings. In powder mode
// (diw_powder_infill) the core layers are sequenced by GCode/DIWUMexSequencer from the ";DIW:" records
// (POWDER_PLAN.md increments 3a and 3b).
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

struct DIWTranslateOptions
{
    // Powder mode (diw_powder_infill) converts the core layers that carry a ";DIW:" record block
    // (diw/POWDER_RECORDS.md; GCode/DIWUMexSequencer). A powder job without any block would come out as
    // paste only, so production refuses it with DIWConfigError; tests set this to run it anyway (only the
    // whole-file rules then apply: parameter checks, warning banner, G04 in seconds, M202, config-block
    // sanitation, safe shutdown).
    bool allow_unconverted_powder = false;
    // TEST ONLY (POWDER_PLAN increment 3a): stage 4 (powder emission) of a converted core layer emits
    // exactly its header comment line "; --- uMEX L<k> S4 powder infill F=<F:.1f> d_early=<d:.3f> ---" and
    // nothing else, with no state change. diw/tests/compare_sequencer.py --stub-s4 stubs v2 the same way.
    // Without it (production), stage 4 emits the powder runs of the block (increment 3b).
    bool stub_powder_stage = false;
};

// Translates G-code text. Line endings of the output are '\n'. Throws DIWConfigError
// (GCode/DIWUMexParams.hpp) for invalid u-MEX settings.
std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config);
std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config, const DIWTranslateOptions &opts);

// Translates the file in place. Throws Slic3r::RuntimeError on I/O failure.
void diw_translate_gcode_file(const std::string &path, const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_GCode_DIWTranslator_hpp_
