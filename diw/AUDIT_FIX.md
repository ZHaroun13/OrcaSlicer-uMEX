# Audit fix: the G-code command layer (19 Nov 2026)

Source: `Audit_file.pdf`. Page numbers below are the audit's own section numbers as read from the page images
(s21 = 3.0 Phase 0, s22 = 3.2, s23/s24 = the change list); the page text is image-only, so these are the readings.

## Requirements

| # | Audit requirement (page) | Verdict before | Now |
|---|---|---|---|
| 1 | A is a position axis in PlanetCNC TNG: no velocity word; A advances by K_E * dE per extrusion block (s21, s23) | FAIL | incremental: each extrusion block writes dA = K_E*dE |
| 2 | no `A0` on travel; no A word on travel, curing or any other non-extrusion move (s23) | FAIL | none written; the old bare `G1 A0.000` line is gone |
| 3 | ABC mode stated in the header (G90.2 / G91.2, s21, 3.0 Phase 0 item 1) | PARTIAL (only G90) | `G91.2` after the header line (XYZ stay G90) |
| 4 | optional `G04` before `M62` (s23, #8) | PARTIAL | setting, default off |
| 5 | suck-back at the end of an extrusion sized from tau_off (s24, #16) | FAIL | setting, default off; negative dB, then a positive dB re-prime |
| 6 | B retraction feed F300-600, volume-sized (s24, #17) | PARTIAL | clamp setting (default 300) already existed; override added; retraction is a negative dB |
| 7 | G04 in seconds, `[ ]` -> `( )` in comments (s23, #6, #7) | COMPLY | unchanged (B is now a delta, see below) |

K_E: the audit gives no number (s22 item 2: it comes from the auger characteristic calibration). The translator uses
the existing `diw_k_screw` = 3.48 (`TranslatorParams::k_screw`), the same constant v2 uses as k_screw.

## Design

* A mode `incremental` = 3 (`TranslatorParams::kAIncremental`) beside flow_scaled 0, proportional 1, cumulative 2.
  It is the default of `diw_a_axis_mode` (`PrintConfig.cpp`: `s_keys_map_DIWAxisAMode`, `enum_values`, default).
  `PrintConfig.hpp` is untouched (reserved for increment 7): the option is an enum generic, so the new value
  needs no member.
* `DIWTranslator.cpp`, `Translator::motion_block`: an extrusion block writes `B<dB> A<dA>` as deltas:
  dA = K_E*dE (`diw_k_screw`) and dB = 0.001225*kB*dE (the existing `delta_b`). Retraction writes a negative dB and
  no A; travel writes neither word. `run()` writes `G91.2 ; ABC incremental: A and B words are deltas` after
  `DIW_TRANSLATION_HEADER`; XYZ stay absolute (G90).
* Quantisation with a carried residual: the translator keeps the exact running totals and the integer counts already
  emitted (B in 1e-5, A in 1e-4). Each block writes `round(total) - emitted`, so rounding never accumulates: the sum
  of the written deltas stays within half a unit of the exact position (B `%.5f`, A `%.4f`; never 2 decimals).
* `G92 E0` (and any E reset): in the incremental mode the line is filtered to a comment (`; (FILTERED) G92 ...`)
  and the totals are not reset (an incremental B has nothing to reset). The old modes still emit `G92 B0.0000`.
* A on a re-prime: stays as is. A re-prime (a positive E after a retraction) is an extrusion block and writes
  dA = K_E*dE like any other; there is no reverse A anywhere (no negative dA), as the brief says. The
  suck-back / re-prime pair (when enabled) moves B only (negative then positive dB), no A.
* Macro and template review under G91.2 (start priming, shutdown depressurisation, wiper, curing, powder passes):
  none of them writes an absolute A or B in the incremental mode. The start-G-code "initial load" prime block
  and its `G92` are removed by the translator; the `G1 A0.000` writers in `curing_sequence`, `umex_shutdown_lines`,
  the travel branch and the sequencer (`a_off_line`, the travel `A0.000`, powder-pass stage 4) are all gated on
  `speed_mode()` (flow_scaled / proportional), which is false for the incremental mode, so they emit nothing;
  the wiper and curing blocks carry XYZ and M62/G04 words only. So no `G90.2 ... G91.2` wrap was needed (it would
  only add a mode switch inside the file); the macros are correct as they are.
* Settings (`DIWTranslateOptions`, CLI flags of `diw_translate`; tolerant reads of `diw_m62_pre_dwell`,
  `diw_suckback_length`, `diw_suckback_feed` in `diw_read_translator_params` for when increment 7 adds the members):
  `m62_pre_dwell` (`--m62-dwell s`, `insert_m62_dwell`, applied after `finalize()`), suck-back
  (`suckback_e`, `suckback_f`; `Translator::suckback_lines` at the end of an extrusion, `reprime_lines` before the
  next, B only, no A), `retract_f_max` (`--retract-f-max`). All default off / unchanged.
* v2 parity: `diw_translate` forces `diw_a_axis_mode=flow_scaled` unless it is explicit (override, config block
  with `--config-from-gcode`) or `--product-defaults` is given. T0, `--v2-parity` and G5 v2open parity therefore
  stay on v2's modes. The slicer product path takes the config default (incremental).

## Deviations from v2 (deliberate, v2 is not modified)

1. A is advanced by K_E*dE per extrusion block (a delta under G91.2) instead of a feed-rate-scaled velocity word (v2 ~2155-2180).
2. B is a per-block delta (negative on retraction and suck-back) instead of a running absolute position, and
   `G92 E0` does not reset it.
3. No `A0.000` / bare `G1 A0.000` after an extrusion or on travel.
4. `G91.2` header line.

## Decisions for the user

* The audit's incremental mode (G91.2) is implemented (the user's choice). The earlier absolute (G90.2) variant was replaced.
* K_E = 3.48 is a placeholder until the auger calibration gives the real value.
* The new settings are CLI/translator options only; they become GUI/preset keys in increment 7 (needs `PrintConfig.hpp`).
* The audit's optional calibrated reverse dA after an extrusion is not implemented (the re-prime adds no reverse A).

## Results

| Gate | Result |
|---|---|
| `diw/tests/audit_gate.py` (A1-A7, B1; edge cases + cone, cube25, lshape, ring, two_objects) | 0 failures. cube25: 408 A words, 0 A/B on G0 travel. Golden: running sum of the emitted dA / dB equals v2's A_abs (K_E*cumsum dE) and absolute B within 1e-3 at every block (+5e-5 per v2 `G92 B0` reset: v2 rounds B to 4 decimals there); per-block dB equals `calculate_delta_b` within 1.5e-5; G91.2 once, no G90.2, no A/B on travel. |
| Catch2 `diw_planner_tests` | 55 test cases, 4805 assertions, 0 failures (5 `[audit]` cases rewritten for G91.2). |
| T0 (`audit\t0inc`) | `compare_sequencer` 157 ok with and without `--stub-s4`, `compare_records` 486 ok, `compare_powder` and `compare_v2` 0 failures. |
| G5 `product_gate.py g5` | 78 checks, 0 failures (v2open parity equals v2). |
| G6 `product_validate.py` | 117 checks, 0 failures. |
| Full build `build_slicer_diw.ps1 --no-configure` | SLICER EXIT CODE 0 (3 min 33 s). |

Files: `src/libslic3r/GCode/DIWUMexSequencer.hpp`, `DIWTranslator.hpp`, `DIWTranslator.cpp`,
`src/libslic3r/PrintConfig.cpp`, `src/dev-utils/diw_translate.cpp`, `tests/libslic3r/test_diw_planner.cpp`,
`diw/tests/audit_gate.py`, this file, `diw/POWDER_PLAN.md`.
