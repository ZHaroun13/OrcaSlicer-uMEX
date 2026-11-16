# v2 post-processor → OrcaSlicer source map (step 3)

Where each feature of the v2 post-processor (`diw_engine.py` v2.0.0) would live once it is
built into this fork. Line numbers refer to tag **v2.5.1** (`eef7710c`) and will drift as the
code changes; search for the function name if a line is off.

Paths below are relative to `src/`.

## 1. Feature-by-feature map

### A. Settings (config groups A–E, presets)

| v2 | Orca location | Notes |
|---|---|---|
| `DEFAULT_CONFIG` keys: B-axis syringe drivetrain, A-axis auger, syringe bore, `k_screw`, `a_axis_mode` | `libslic3r/PrintConfig.hpp` (config class macros, e.g. `PRINT_CONFIG_CLASS_DEFINE` at 1282; `use_relative_e_distances` at 1872) and `libslic3r/PrintConfig.cpp` (`PrintConfigDef`, e.g. `def = this->add("use_relative_e_distances"` at 8622) | **Printer** settings: drivetrain, syringe, A mode, I/O pins, wiper, UV offset, soft limits, powder nozzle offsets. |
| Material keys: `b_multiplier`, retraction limits, `post_extrusion_dwell_sec`, UV dwell, powder mass flow / densities | same files | **Filament** settings: they change per paste or powder. The v2 presets (Alginate, Ceramic/Clay, Silver ink) become filament presets. |
| Powder hatch: `powder_hatch_spacing`, `shell_powder_clearance`, hatch angle, compaction interval/duration | same files | **Process** settings. |
| Settings shown and saved in presets | `libslic3r/Preset.cpp`: `s_Preset_print_options` (1183), `s_Preset_filament_options` (1524), `s_Preset_printer_options` (1604) | Every new key must be added here or it won't be saved or compared. |
| GUI pages | `slic3r/GUI/Tab.cpp`: `TabPrint::build` (3033), `TabFilament::build` (4779), `TabPrinter::build_fff` (5498) | New pages: "DIW extrusion", "UV curing", "u-MEX powder", "Compaction & wiper". |
| `UMexParams.from_config` range checks | `PrintConfigDef` `min`/`max` per option + `Print::validate` (`libslic3r/Print.cpp` 2068) | Hard limits (compaction ≤ 8 s, voltage 3.65–12 V, distinct M62 pins) become validation errors shown before slicing. |

### B. Paste extrusion output (E → B / A)

| v2 | Orca location | Notes |
|---|---|---|
| New machine dialect (PlanetCNC TNG) | `enum GCodeFlavor` (`libslic3r/PrintConfig.hpp` 52) | Add `gfPlanetCNC`. Gates every behaviour below so other printers are untouched. |
| `calculate_delta_b`: ΔB = ΔE·(d_f/D_syringe)²·multiplier (optional drivetrain conversion) | `libslic3r/GCodeWriter.hpp` `emit_e` (362) and its callers `extrude_to_xy` (`GCodeWriter.cpp` 1303), the arc/xyz variants, `retract` (1464) | **Decided:** B is computed from the extrusion **volume** required (ΔB = V / A_syringe, then multiplier and optional drivetrain conversion), not from a virtual 1.75 mm filament. `filament_diameter` stops mattering. |
| A-axis auger modes (flow-scaled speed, proportional speed, cumulative position) + `A0.000` at extrusion end | `GCodeFormatter` (`GCodeWriter.hpp`, `emit_axis`), `GCodeWriter::extrude_to_xy` / `travel_to_xyz` (980) | Add an `emit_a`. "Speed" modes need A written on every extruding move and `A0` on the first travel. |
| Retraction guard (`max_retract_de_mm`, `max_retract_feedrate`) | `GCodeWriter::retract` (1464) / `unretract` + `Print::validate` | Clamp or reject in validation rather than silently dropping moves at output. |
| Strip `G28`, `M84`; dwell in seconds `G04 P<s>`; no temperature or fan codes | `GCodeWriter::set_temperature`, `set_fan` (1559), dwell helpers; start/end G-code placeholders | Flavor-specific. |
| Orca behaviours that break DIW: layer cooling slow-down, seam gap, arc fitting, wipe on loops | `GCode/CoolingBuffer.cpp` `process_layer` (340); `seam_gap` option (`PrintConfig.cpp` 6535); arc fitting in `GCode::_extrude`; wipe in `GCode::extrude_loop` (8536) | Force them off (or hide them) for `gfPlanetCNC` instead of relying on presets. |

### C. Layer-change sequence (UV, wipe, compaction, final cure)

| v2 | Orca location | Notes |
|---|---|---|
| Inter-layer UV macro: move to layer bounding-box centre + `uv_offset_y_mm`, `M62 P3 Q1`, `G04`, `M62 P3 Q0`, return | `GCode::process_layer_change_gcode` (`GCode.cpp` 6123) and the `before_layer_change_gcode` sites (6239, 6710) | **Decided: fixed sequence** generated in C++ (not a user-editable template). Only its parameters (UV pin commands, offset, dwell) come from settings. |
| Final UV cure (once, before end G-code) | `GCode::_do_export` (3419), just before `machine_end_gcode` (4860–4890) | |
| Stage 2 nozzle wipe (`wiper_*`) | fixed sequence in the per-layer code | Parameters from printer settings. |
| Stage 5 vibration compaction (`M62 P6`, duration, settle, Z hop, interval) | fixed sequence in the per-layer code | Interval and "last core layer always compacts" are decided in C++ from the layer's surface types. |
| 6-stage order (paste walls → auger stop + dwell → wipe → UV → powder → compaction → advance) | `GCode::process_layer` (6603) | Emit paste extrusions first, then the templates, then the powder paths, then compaction. This is the main change to Orca's layer loop. |

### D. u-MEX powder core

| v2 | Orca location | Notes |
|---|---|---|
| `classify_layers`: core = sparse infill and no top/bottom surface, guessed from `;TYPE:` comments | `enum SurfaceType` (`libslic3r/Surface.hpp` 16) inside `PrintObject::prepare_infill` (`PrintObject.cpp` 627) | Native: Orca already knows which regions are `stInternal` vs top/bottom. No comment parsing. |
| Cavity rebuilt from closed wall loops (shapely, `LOOP_CLOSE_TOL_MM`) | `LayerRegion` fill surfaces / fill expolygons (`libslic3r/Layer.hpp`), `offset()` (`ClipperUtils.hpp` 354) | Native: the cavity is the inner perimeter boundary itself, so the seam-gap failure **can't happen**. |
| Inward clearance `shell_powder_clearance` (+ guard) | `offset(..., -scaled(delta))` on the internal surfaces | Also gives the `umex_paste_revert_min_area_mm2` test (area after offset). |
| `hatch_polygon` (spacing `s_eff`, angle) | `Layer::make_fills` (`Fill/Fill.cpp` 1353) with the rectilinear filler; `enum InfillPattern` (`PrintConfig.hpp` 127) | Generate powder paths as their own role. |
| New path role for powder | `enum ExtrusionRole` (`libslic3r/ExtrusionEntity.hpp` 26–48) | Add `erPowder`: written with **no E/B/A**, `;TYPE:Powder`, own colour in preview. |
| `resolve_powder_feed`: F_powder from mass flow, tap density, spacing, layer height; overflow policy | `GCode::_extrude` (`GCode.cpp` 9245) for `erPowder` | Feed overrides the speed settings. Policy "increase spacing" must feed back into hatch generation, so compute it in the fill step. |
| Feeder `M62 P5 Q1/Q0` with lag-on/off compensation, point-dwell for short hatches | `GCode::_extrude` / a powder-specific `extrude_powder()` | |
| Powder nozzle offset X/Y/Z, dispensing Z, safe transit Z | `GCodeWriter` (applied to powder paths only) + Z handling in `travel_to_xyz` | **Decided:** the powder nozzle is an **offset tool on the main extruder**, not a second Orca extruder. Offset X/Y/Z are printer settings the user enters in the slicer (defaults 0, with a warning while uncalibrated). |
| Soft limits check/clamp | `Print::validate` + travel generation | Orca's bed shape (`printable_area`) already covers part of this. |

### E. Preview, time estimate, statistics

| v2 | Orca location | Notes |
|---|---|---|
| — (v2 output was only checked by reading the G-code) | `GCode/GCodeProcessor.cpp`: `process_G1` (5206), `process_M82`/`M83` (6598+), `enum class EMoveType` (`GCodeProcessor.hpp` 52) | Teach it the A/B axes, `M62`, `G04 P` in seconds and the `Powder` role, so preview and time estimate are right. |
| | `slic3r/GUI/GCodeViewer.cpp` | Role colour and legend for powder, UV and compaction markers. |

### F. Interim path (until native code lands)

| v2 | Orca location | Notes |
|---|---|---|
| Post-processor run from the process preset | `slic3r/GUI/PostProcessor.cpp` `run_post_process_scripts` (522), called from `BackgroundSlicingProcess.cpp` (296) | This already works today (step 1). Keep it until each native milestone matches it. |

## 2. What native integration fixes in v2

- **Seam gap / open loops**: no more cavity reconstruction from G-code. The cavity comes from the slicer's own polygons.
- **Arc fitting**: no layer is skipped for containing G2/G3. The writer emits B/A directly.
- **Role guessing**: core layers come from `SurfaceType`, not `;TYPE:` text.
- **Virtual filament**: B is computed from volume, so `filament_diameter` and the 1.75 mm trick disappear.
- **Preview and time**: powder, UV and compaction appear in the preview. Print time includes the dwells.

## 3. Suggested milestones

Each milestone is checked against the v2 post-processor output (step 4, golden tests) before the next one starts.

1. **Settings only** (done): 52 `diw_*` keys (27 printer, 13 filament, 12 process) in a new `DIWConfig`
   class (GCodeConfig and PrintConfig are at the Boost.PP 256-entry limit), a "DIW / u-MEX" page
   in each settings tab, and the `diw_printer` master switch that shows or greys out the rest. Slicing
   output is unchanged; the keys only add lines to the G-code config block. The PlanetCNC flavor moved to
   milestone 2, where its writer behaviour is defined.
2. **Paste translation** (done): `libslic3r/GCode/DIWTranslator.{hpp,cpp}`, a line-for-line C++ port of the
   v2 paste path. It covers E → B (by volume) + A (3 auger modes), the PlanetCNC rules (FDM commands commented
   out, `[ ]` → `( )` in comments, G28/M84/empty G4 removed, retraction limits) and the UV sequences that
   belong to the paste path (start-up self-check, inter-layer cure, final cure, end-of-print cure), so
   that part of milestone 3 is done too.
   - Runs on the **exported file** when `diw_printer` is on: GUI through `run_post_process_scripts()`
     (before user scripts, on the working copy), CLI right after `export_gcode()`. The preview keeps the
     FDM moves. A file that already starts with the translation header is not translated twice.
   - Remove `DIW_Orca_post_processor_v2.exe` from the process preset's post-processing scripts when
     `diw_printer` is on, or the start-up UV self-check is added twice.
   - Golden test (16 Nov 2026, 25 mm cube): identical to v2 line for line (only the header line differs)
     for the defaults and for the Alginate, Ceramic and Silver presets and a B-in-revolutions / no-UV case.
   - Still to do here: a native PlanetCNC flavor, and forcing DIW-hostile Orca features off (seam gap,
     arc fitting, layer cooling slow-down) instead of relying on the presets.
3. **Layer sequence**: nozzle wipe and compaction as fixed sequences (UV is done in milestone 2).
4. **Powder core**: `erPowder`, cavity offset, hatching, feed and I/O, nozzle offset. Must match v2 **powder** output on the test cube, and remove the `seam_gap 0%` requirement.
5. **Preview + validation + time**.

## 4. Decisions (16 Nov 2026)

- **B axis:** computed from the paste volume each move requires, divided by the syringe bore area. No virtual filament. Existing `b_multiplier` calibrations still apply as a correction factor.
- **UV curing, nozzle wipe, compaction:** fixed sequences in C++, configured only through their parameters.
- **Powder nozzle:** an offset tool attached to the main extruder. The X/Y/Z offset is entered later in the slicer's printer settings.
- **Where the paste translation runs (milestone 2):** on the exported file, not inside `GCodeWriter`. This keeps
  the preview, time estimate and G-code viewer working on the FDM moves, and lets the v2 output serve as
  an exact golden reference. B is still the paste volume divided by the syringe bore area. Moving the
  B/A words into `GCodeWriter` itself can come with milestone 5 (preview), when the G-code viewer learns
  the A/B axes.

Milestone 2 should check that B from volume equals v2's virtual-filament B within rounding. The two are mathematically the same when `filament_diameter` matches.
