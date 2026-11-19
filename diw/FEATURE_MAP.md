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
| `UMexParams.from_config` range checks | `PrintConfigDef` `min`/`max` per option + `Print::validate` (`libslic3r/Print.cpp` 2068) | Hard limits (compaction ≤ 8 s, voltage 3.65–12 V, distinct M62 pins) become validation errors shown before slicing. **Done (increment 5):** `diw_validate` (`GCode/DIWTranslator.cpp`) is called from `Print::validate` on DIW powder jobs. It gives v2's message verbatim, so the slicer and the translator print the same text, with the fork key as opt_key from an explicit table. v2's soft warnings become validation warnings. |
| — (v2 accepts these and prints paste or under-fills) | `Print::validate` | **Done (increment 5), product rules with no v2 counterpart:** `wall_loops = 0`, sparse infill density 0 on every region, and `infill_combination` are errors on powder jobs. So are the structural refusals (spiral vase, BBL G-code, belt / G-code axis remap, extruders with different `extruder_offset`) and a per-object scarf seam. `ORCA_DIW_TEST_SKIP_VALIDATE=1` (test only) skips them. |

### B. Paste extrusion output (E → B / A)

| v2 | Orca location | Notes |
|---|---|---|
| New machine dialect (PlanetCNC TNG) | `enum GCodeFlavor` (`libslic3r/PrintConfig.hpp` 52) | Add `gfPlanetCNC`. Gates every behaviour below so other printers are untouched. |
| `calculate_delta_b`: ΔB = ΔE·(d_f/D_syringe)²·multiplier (optional drivetrain conversion) | `libslic3r/GCodeWriter.hpp` `emit_e` (362) and its callers `extrude_to_xy` (`GCodeWriter.cpp` 1303), the arc/xyz variants, `retract` (1464) | **Decided:** B is computed from the extrusion **volume** required (ΔB = V / A_syringe, then multiplier and optional drivetrain conversion), not from a virtual 1.75 mm filament. `filament_diameter` stops mattering. |
| A-axis auger modes (flow-scaled speed, proportional speed, cumulative position) + `A0.000` at extrusion end | `GCodeFormatter` (`GCodeWriter.hpp`, `emit_axis`), `GCodeWriter::extrude_to_xy` / `travel_to_xyz` (980) | Add an `emit_a`. "Speed" modes need A written on every extruding move and `A0` on the first travel. |
| Retraction guard (`max_retract_de_mm`, `max_retract_feedrate`) | `GCodeWriter::retract` (1464) / `unretract` + `Print::validate` | Clamp or reject in validation rather than silently dropping moves at output. **Increment 5:** a validation **warning** (the translator still drops over-limit retractions per line, exactly as v2), plus warnings when syringe retraction is off with `retraction_length > 0` and when the retraction speed is clamped. Making it an error is open question Q2 in POWDER_PLAN.md. |
| Strip `G28`, `M84`; dwell in seconds `G04 P<s>`; no temperature or fan codes | `GCodeWriter::set_temperature`, `set_fan` (1559), dwell helpers; start/end G-code placeholders | Flavor-specific. |
| Orca behaviours that break DIW: layer cooling slow-down, seam gap, arc fitting, wipe on loops | `GCode/CoolingBuffer.cpp` `process_layer` (340); `seam_gap` option (`PrintConfig.cpp` 6535); arc fitting in `GCode::_extrude`; wipe in `GCode::extrude_loop` (8536) | Force them off (or hide them) for `gfPlanetCNC` instead of relying on presets. **Increment 5:** the scarf joint seam (`seam_slope_type`) is also forced to none on DIW powder jobs in `apply_diw_overrides` (`PrintApply.cpp`). The scarf ramps the outer wall below the layer, which v2 cannot plan (st=open), and doubles the paste at the seam. A per-object, per-part or layer-range scarf is a validation error. Non-DIW and paste-only DIW jobs keep their scarf. `ORCA_DIW_TEST_KEEP_SCARF=1` keeps it (test only). |

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
| `classify_layers`: core = sparse infill and no top/bottom surface, guessed from `;TYPE:` comments | `enum SurfaceType` (`libslic3r/Surface.hpp` 16) inside `PrintObject::prepare_infill` (`PrintObject.cpp` 627) | Native: Orca already knows which regions are `stInternal` vs top/bottom. No comment parsing. **Done (increments 4-5):** the native classification (roles of every object layer at the print Z, per printed layer as v2) is the authority in product mode. The text classification is only a diagnostic: `text_only` layers get no block and a warning, `native_only` layers get a block, `native_only_nosparse` layers get no block and a warning, and the final layer gets a warning (PRODUCT_DESIGN.md 3.2). |
| Sparse infill of core layers replaced by powder | `GCode/DIWPowderShadow.cpp` (binder, after planning) + `suppress_sparse` in `GCode/DIWUMexSequencer.cpp` | **Done (increment 5):** on every `st=ok` layer the sparse-infill paths are removed from the exported G-code after the layer is planned. Minimal restore lines are added, and V0-V3 prove that the translated output cannot change; a failed check releases the layer unedited (a fallback, logged). Fill generation is untouched, so `st=small` layers keep their sparse infill as paste (v2's fallback). The preview and time estimate no longer show the removed sparse infill; powder itself is shown from increment 7. |
| v2 prints paste and warns when a core layer cannot be planned | binder failure list (`PowderShadow::failed()`, checked in `GCode::do_export`) + translator (`DIWTranslateOptions::v2_parity`) | **Done (increment 5), strict product path:** a native core layer that would be printed without powder stops the export with a message naming the layers and the cause (`arc_g91`, `open`, `orphan`: an extrusion typed sparse infill outside the walls, internal errors, `no_core`). The translator refuses a powder job without `;DIW:` records and a non-carrier `st=open` block. `--v2-parity` restores v2's paste fallback, for harnesses only. `st=small` and the h <= 0 carrier keep v2's warnings. |
| — | `ORCA_DIW_POWDER_MODE=shadow` / `diw_translate --plan-records` | **Product exports are not v2 or replay inputs.** After suppression, v2 (or a text-only `--plan-records` replay) sees no sparse infill in the converted layers and plans them as paste. Only shadow-mode exports, or shadow `keep` exports + dumps, may feed v2 and the `compare_*` harnesses. `product_gate.py` keeps a manifest of every export's mode and refuses product files as oracle inputs. |
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

**Added time and powder mass report (increment 6).** Every exported u-MEX file ends with comment lines that give the powder
mass (mass flow x deposition time) and the time the u-MEX stages add (dwells, powder feed and compaction moves, UV, wipe), at
nominal feed and without acceleration, so treat it as a lower bound. The same lines go to the slicer log, and
`diw_translate --report` prints them. They are comments only, so the machine motion does not change. Since increment 7 the
slicer's own estimate includes these stages; the report footer subtracts the included part again (marker line).

### F. Interim path (until native code lands)

| v2 | Orca location | Notes |
|---|---|---|
| Post-processor run from the process preset | `slic3r/GUI/PostProcessor.cpp` `run_post_process_scripts` (522), called from `BackgroundSlicingProcess.cpp` (296) | This already works today (step 1). Keep it until each native milestone matches it. **Increment 5:** do not run v2 on a G-code file that OrcaSlicer exported in product mode (the default) for a u-MEX powder job. Its core layers no longer contain the sparse infill that v2 replaces, so v2 would print them as paste. The native translation reads the `;DIW:` records instead. |

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
   - Regression test: `diw/tests/compare_v2.py` runs `diw_translate.exe` (dev tool, built with
     ORCA_TOOLS; run it from `build/src/Release` so it finds the DLLs) and v2 on 12 edge cases (Unicode
     symbols and spaces, invalid UTF-8, lone CR, overflow to inf/NaN, huge numbers, M83, arcs, N numbers,
     start/end blocks, layer-marker variants) plus real Orca output, under 3 settings variants. 42/42
     identical on 16 Nov 2026. Known approximation: Python's Unicode `\w` is reproduced exactly for
     ASCII and Latin-1 and by block ranges elsewhere.
   - **PlanetCNC TNG flavor** (`gcfPlanetCNC`, "planetcnc"): choosing it makes the printer a DIW machine.
     `Print::apply()` replaces it with Marlin (legacy), so the slicer core writes the G-code the translator
     expects and no Marlin check elsewhere needs a PlanetCNC case, and turns `diw_printer` on. The G-code
     config block therefore reports `gcode_flavor = marlin` and `diw_printer = 1`. The enum value sits
     right after Marlin 2 because the GUI combo stores the list position as the enum value.
   - **DIW-hostile features forced off** in `Print::apply()` when `diw_printer` is on: seam gap → 0,
     wipe on loops and wipe before external loop → off, arc fitting → disabled, slow down for layer
     cooling → off. Values already off are left untouched. The GUI greys them out with the reason.
     Per-object seam gap or wipe overrides are rejected by `Print::validate()`.
   - Test: PlanetCNC flavor with all of these turned on gives the same G-code as the clean DIW preset
     (1939 lines, 0 differences).
3. **Layer sequence**: nozzle wipe and compaction as fixed sequences (UV is done in milestone 2).
   Wipe and compaction only occur inside v2's powder layers, so milestones 3 and 4 are built together;
   see [POWDER_PLAN.md](POWDER_PLAN.md) (architecture C, increment 1 done).
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
