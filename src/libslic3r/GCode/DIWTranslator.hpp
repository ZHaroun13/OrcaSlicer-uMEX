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

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {

class DynamicPrintConfig;
namespace DIW { struct TranslatorParams; }

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
    // TEST ONLY (PRODUCT_DESIGN.md 5.3): v2's behaviour for a non-carrier st=open block (the layer is left as paste
    // with v2's warning). Without it (production) such a block is a DIWConfigError. Every harness that translates
    // records made by v2 sets it.
    bool v2_parity = false;
};

// Translates G-code text. Line endings of the output are '\n'. Throws DIWConfigError
// (GCode/DIWUMexParams.hpp) for invalid u-MEX settings.
std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config);
std::string diw_translate_gcode(const std::string &gcode, const DynamicPrintConfig &config, const DIWTranslateOptions &opts);

// The translator's settings read from the full print config (GCode/DIWUMexSequencer.hpp). Throws
// DIWConfigError for invalid u-MEX settings, with the message the translator gives.
DIW::TranslatorParams diw_read_translator_params(const DynamicPrintConfig &config);

// The u-MEX powder settings checked before slicing (PRODUCT_DESIGN.md 6.2): diw_read_translator_params on the full
// print config. A DIWConfigError becomes `error` with its exact text (v2's message, the one the translator gives),
// the config warnings become `warnings`; each carries the fork key of the setting (diw_validation_opt_key). Empty
// unless the config is a DIW printer with diw_powder_infill on.
struct DIWValidation
{
    std::string error, opt_key;
    std::vector<std::pair<std::string, std::string>> warnings;   // (message, opt_key)
};
DIWValidation diw_validate(const DynamicPrintConfig &full_config);
// The explicit table {v2 message prefix -> fork key} behind diw_validate's opt_keys, in v2's check order.
const std::vector<std::pair<std::string, std::string>> &diw_validation_opt_key_table();
// The fork key of a diw_read_translator_params message (the first table row whose prefix starts it), or "".
std::string diw_validation_opt_key(const std::string &message);

namespace DIW {
// The UV-centre rule of the translator (v2 _compute_geometric_center), as a streaming routine shared with the
// powder binder (PRODUCT_DESIGN.md 4.4): the XY bounding box of every extruding move (dE > 1e-7 after its own
// M82 / M83 / G92 E bookkeeping, start and end point) after the first layer marker. Feeding stops for good at
// "; CONFIG_BLOCK_START".
struct ExtrusionBBox
{
    bool   found_first_layer = false, rel = false, stopped = false;
    double prev_e = 0., tx = 0., ty = 0.;
    double min_x = std::numeric_limits<double>::infinity(), max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity(), max_y = -std::numeric_limits<double>::infinity();
    bool   empty() const { return min_x == std::numeric_limits<double>::infinity() || min_y == std::numeric_limits<double>::infinity(); }
};
void extrusion_bbox(const std::string &line, ExtrusionBBox &state);
void extrusion_bbox(const std::vector<std::string> &lines, ExtrusionBBox &state);
} // namespace DIW

// Translates the file in place. Throws Slic3r::RuntimeError on I/O failure.
void diw_translate_gcode_file(const std::string &path, const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_GCode_DIWTranslator_hpp_
