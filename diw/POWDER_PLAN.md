# u-MEX powder core: architecture and increments (milestones 3 + 4)

Chosen on 17 Nov 2026 by a design review of three architectures (two independent judges, both picked C).

## Architecture C (hybrid)

- **Slicer core plans the geometry** from its own data at export: core layers from fill roles
  (`SurfaceType` / `erInternalInfill`), the cavity from the closed `ExtrusionLoop`s (closed in memory;
  the seam is only clipped at write time), insets with `ClipperUtils` (`jtRound`, arc tolerance matched to
  GEOS `quad_segs` 32), hatch rows computed analytically in double precision (not `intersection_pl`).
  This removes v2's two structural weaknesses: open loops from a seam gap, and core layers guessed from
  `;TYPE:` text.
- **The plan travels as `;DIW:` comment records** in the exported G-code (versioned, CRC-checked,
  config-fingerprinted; no `[ ] ( )` and no word "layer" in them).
- **The translator (`GCode/DIWTranslator`) sequences** each core layer exactly as v2 does: S1 paste walls +
  auger stop dwell, S2 nozzle wipe, S3 UV cure, S4 powder (feeder I/O, lag compensation, point dwell,
  nozzle offset, Z handling), S5 compaction, S6 layer advance.

Why: the record boundary is where verification is strongest. Feeding v2's own `plan_powder_layer`
output to the C++ sequencer as records proves the whole sequencer byte-identical on any geometry; the
native geometry is then checked against v2 with explicit tolerances. No `PrintConfig.hpp` change is
needed until milestone 5.

## Increments

| # | Scope | Status |
|---|---|---|
| 1 | Translator powder mode, whole-file rules: `UMexParams` port (`GCode/DIWUMexParams`), Python-exact number formatting (`GCode/DIWPyCompat`), warning banner, G04 in seconds, M202, config-block sanitation, safe shutdown; production refuses powder jobs until core layers convert; golden harness `diw/tests/compare_powder.py` + corpus `diw/tests/corpus/cube25.gcode` | **done** |
| 2 | Freeze record format v1 ([POWDER_RECORDS.md](POWDER_RECORDS.md); `GCode/DIWPowderBlock` parser/writer, CRC-32, fingerprint); Python `v2_blocks.py` (v2 plans as records) and `v2_hooks.py` (capture / replay); `compare_records.py` | **done** |
| 3a | Sequencing from records: S1, S2 wipe, S3 UV, S5 compaction, S6 (milestone 3); S4 stubbed on both sides (`GCode/DIWUMexSequencer`, `compare_sequencer.py`) | **done** |
| 3b | S4 powder emission from records (milestone 4); byte-identical with v2's own plans as records on all fixtures | **done** |
| 4 | Slicer-core planner in shadow mode (records written by `GCode.cpp`, nothing suppressed); tolerance tests vs v2, clearance invariant, perturbation test, seam-gap 15 % test | **done** (18 Nov 2026): built and verified on the 191-slice matrix, T5 and the 10.1 stress variants; see "Increment 4 results" |
| 5 | Product path: sparse infill suppressed on core layers, translator requires valid records, `seam_slope_type` forced off, validation rules | |
| 6 | Added time and powder mass reported at export; documentation | |
| 7 | Milestone 5 (one full rebuild): preview / time estimate read the records, Powder role colour | |

## Increment 1 results (17 Nov 2026)

`compare_powder.py` against v2 (Python 3.15.0, shapely 2.2.0, GEOS 3.14.1), v2 with every layer forced
to non-core (`umex_n_bottom_layers = 10**6`):
- cube25 under 4 variants (defaults, powder offsets, warning values, cumulative A without UV): identical
  after the header, banner included;
- 5 invalid-setting variants: identical error messages;
- 3 synthetic files (G4 rules, no layer markers, M30) and all 13 paste edge cases in powder mode: identical;
- `py_repr` / `py_g`: 20020/20020 values equal to Python `repr()` / `format(v, 'g')`.
Paste mode (`compare_v2.py`): 0 failures, including the new already-post-processed-line case (v2 drops
its own header line; the milestone 2 port did not).

## Increment 2 results (17 Nov 2026)

`compare_records.py` (197 checks, 0 failures, 0 skips):
- v2 in real powder mode captured as records for cube25 under 8 variants plus a notched part. Together they cover all
  block states (`ok`, `open`, `small`), line and point runs, and paste fallback paths.
- **Replay:** v2 driven by plans read back from the records gives output byte-identical to plain v2.
  A moved point changes the output, so replay really uses the records.
- **C++ reader and writer:** `diw_translate --dump-records` / `--rewrite-records` / `--fingerprint` are
  byte-identical to the Python reference.
- **Hostile inputs:** about 20 cases (tampered CRC, nesting, nan, key order, lone CR, non-ASCII, ...) give the same
  errors in C++ and Python.
- **Record stripping:** the translator removes `;DIW:` lines from the raw bytes, so translating a file equals
  translating the same file without its records.

A three-lens adversarial review confirmed three defects, all fixed:
- the lone-CR split mismatch;
- a missing exe was counted as a skip instead of a failure;
- variant checks could not fail.

`compare_powder.py` and `compare_v2.py` still pass with 0 failures.

## Increment 3a results (17 Nov 2026)

`GCode/DIWUMexSequencer.{hpp,cpp}` ports v2's layer machinery: `scan_gcode` / `LineInfo` (the fields the
sequencer needs), `split_layers`, `parse_core_layer`, `resolve_powder_feed` and the `_UMexMixin` flow
(`_process_lines_umex`, `_convert_core_layer`, `_paste_travel`, `_feed_paste_line`, `_mv`, `_apply_limits`,
`_warn_agg` / `_flush_agg`). The paste translator derives from `UMexSequencer` as `DIWPostProcessor` derives from
`_UMexMixin`, so both share one state, one `process_line` and one curing macro. The Python str / regex helpers
moved from `DIWTranslator.cpp` to `DIWPyCompat` so both files use them.

How the translator uses the records (powder mode only; paste mode still just removes them):
- `parse_records` runs on the raw input before the records are removed. Any reader error stops the export
  (`DIWConfigError` quoting the first error's line and reason). Each block keeps its position in the translator's
  line numbering (lines before it once the records are gone; `\n`, `\r\n` and a lone `\r` each end a line).
- **A layer is converted iff it carries a block.** v2's `classify_layers` is not ported: the slicer decides the core
  layers. `last_in_stack` (S5 on the last layer of a stack) reads "the next layer has no block".
- Up front, for every block: it must sit inside a printed layer, after the start marker and before the layer's first
  motion line; `k` must equal that layer's number; `fp` must equal `plan_fingerprint` of the current settings.
- When the layer is reached: the block's `z` / `h` must equal the layer's own (v2's `model.z` and `h`). The
  translator then runs its own `resolve_powder_feed`, emits v2's overflow warnings, and requires the block's
  `F` / `s` / `fu` to be equal (bit for bit). Any mismatch is a `DIWConfigError` naming the block and both values.
- `st=open`: v2's warning, layer left as paste. `st=small`: v2's aggregated `revert_all` warning, layer left as
  paste. `st=ok`: S1 to S6.
- Guard: a powder job with at least one block proceeds. A powder job without blocks still gets the increment 1
  error unless `allow_unconverted_powder` (`--allow-unconverted-powder`).
- Stage 4 is increment 3b (below). The test-only
  `DIWTranslateOptions::stub_powder_stage` (`diw_translate --stub-powder-stage`) defines the stub: where S4 would emit,
  it emits exactly its header line `; --- uMEX L{k} S4 powder infill F={F:.1f} d_early={F / 60.0 * lag_off:.3f} ---`
  and nothing else, with no state change. `compare_sequencer.py --stub-s4` monkeypatches v2's `_emit_powder` under the same rule.

`compare_sequencer.py --stub-s4` (33 checks, 0 failures, 0 skips; Python 3.15.0, shapely 2.2.0, GEOS 3.14.1):
- cube25 under the 8 `compare_records` variants plus `interval3`, `cumulative_noUV` and `post_dwell`, and the notch
  part under `notch`, `notch_angle30` and `notch_interval3`: byte-identical with plain v2 after the header line.
  The banner is included. Blocks cover `ok`, `open` and `small`. On the notch with interval 3, the layer below an
  `open` layer is correctly not last in its stack.
- 5 negative cases exit 3 with a clear message: tampered fingerprint, F mismatch, block in the wrong layer, block
  after the first motion line, and a record parse error (crc mismatch).
- Without `--stub-s4`, every case with an `ok` block stops at S4 by design. The `small`-only case already matches.
`compare_records.py` (198 checks), `compare_powder.py` and `compare_v2.py` still pass with 0 failures.

Fixed on the way: `read_umex_params` never stored `compaction_duration` (`UMexParams::comp_t` stayed 0, so S5 would
have dwelt `G04 P0.000`).

Divergences from v2, documented:
- **v2 core layer without a block.** v2 marks a layer core but writes no block when `model.z` is None or `h <= 0`
  (and when planning raises, "powder planning failed"). `model.z` is never None for a core layer, because a core layer
  has an extruding move. `h <= 0` needs a `;HEIGHT:` of 0 or none, plus a layer Z at or below the previous layer's,
  so only hand-written G-code reaches it. The capture hook writes nothing for such a layer, so the C++ side leaves it
  as paste, as v2 does. Two differences remain:
  - v2's banner warning (`layer k: non-positive layer height; left as paste.`, or the planning failure) is missing.
  - v2 still counts the layer as core for `last_in_stack`, so the converted layer below it is not last in its stack
    in v2 but is in C++. C++ then adds an S5 compaction that v2 emits only if the interval fires.
  No parity case hits this edge: `compare_sequencer.py` checks `n_core` (v2) == number of blocks for every case
  (`blocks_are_cores`).
- **Stricter than v2** (v2's own plans never trigger these): a block whose `z` / `h` differ from the layer's, a
  block in a layer with arcs or G91, a block in the final layer, and a block in a layer without extrusion moves are
  all rejected. v2 never makes such a layer core.
- `compare_records.py`'s powder-mode lone-CR check changed with the design. In powder mode, records now drive the
  output, so `translate(F) == translate(strip_records(F))` holds in paste mode only. In powder mode, the lone-CR
  record line must stop the export with the reader's line and `bad line`. A `;DIW:` after a lone CR must stay a
  comment while all blocks are still sequenced.
- Notes for 3b: `;TYPE:` names are lower-cased in ASCII only (v2: `str.lower()`). Python's `math.hypot` and C's
  `hypot` may differ in the last ulp. 3a uses `hypot` only against thresholds (1e-9, 1e-6); S4's run lengths and
  early-stop point will need a check. (Resolved in 3b: `py_hypot` everywhere.)

## Increment 3b results (17 Nov 2026)

`UMexSequencer::emit_powder` ports v2's `_emit_powder`, `_powder_transit`, `_pmove`, `_powder_xy`, `_z_disp` and
`_z_safe`, with the point-dwell (`pt_quantum`) and short-run (`short_run`) aggregated warnings. The points come
from the block's RUN records. Production no longer refuses S4; `--stub-powder-stage` stays as a test option only.

Floating point follows v2 operation by operation:
- `DIW::py_hypot` (`DIWPyCompat`) is CPython's `vector_norm` for two coordinates (`math.hypot` since 3.10: power-of-two
  scaling, `fma`-exact squares, compensated sums, one correction step), not the C library's `hypot`. It now also
  replaces `std::hypot` in the 3a code (`has_xy`, path contiguity, `_paste_travel`).
- `DIW::PySum` is Python 3.12+ `sum()` over floats (Neumaier compensation, added at the end). v2's `PowderRun.length`
  uses `sum()`, while the early-stop loop's `cum` is a plain `+=`; the port keeps both, so `stop_s - cum` and `t`
  match bit for bit, and so does the interpolated point `a + t * (b - a)`. The 1e-9 tolerances are v2's.
- `max(t_feed - lag_off, 0.0)` and `min(0.0, off_z)` keep Python's argument order.
- `diw_translate --pyhypot` prints both for hex inputs. Against Python 3.15 on about 1 million values
  (coordinate differences, wide magnitudes, zeros, subnormals): 0 mismatches.

`compare_sequencer.py` (Python 3.15.0, shapely 2.2.0, GEOS 3.14.1), without `--stub-s4`: 153 checks, 0 failures,
0 skips. The real S4 output is byte-identical to plain v2 on all 5 corpus files (cube25, ring, lshape, two_objects,
cone) under the 8 `compare_records` variants and 6 sequencer variants. The notch part passes under its 3 variants,
and the 5 negative cases pass. New sequencer variants: `s4_lag0_nohop` (d_early = 0, no transit Z lift),
`s4_negz_longlag` (negative powder Z offset, so `z_safe` uses `min(0, off_z)`, plus a Z clearance and a long lag),
and `s4_points_lag` (point dwells shorter than `lag_off`).
The new `s4_synthetic` case replaces the first block's runs with hand-made runs. They take every early-stop branch:
a zero-length segment, a stop at a segment end (`t >= 1`), at a segment start (`t <= 0`), inside a segment, at the
run start without being short, and a run shorter than d_early. Point dwells sit at, below and above `lag_off`.
v2 replays those records and gives the same output. With `--stub-s4`, the same 153 checks also pass.
Corpus coverage: 1110 line runs (543 stop at the run start, 567 stop inside a segment) and 40 point dwells. Both
aggregated warnings appear (`short_run` on cone, `pt_quantum` on cone `point_runs`).
`compare_records.py` (486), `compare_powder.py` (62) and `compare_v2.py` (54) still pass with 0 failures.

A three-lens adversarial review (fidelity to v2, machine safety, harness soundness) confirmed one defect, now fixed with
regression cases: the sequencer accepted a lower-case `n` line-number prefix, but v2's `_RE_CMD` accepts only `N`.
Final run: `compare_sequencer.py` 157 checks with and without `--stub-s4`, all 0 failures.

Not reachable from records: v2's final `if not stopped` branch after a line run with `stop_s > 1e-9`. It would need
the plain `cum` to end more than 1e-9 short of the compensated `total - d_early`. The port keeps it as is.

## Increment 4 results (18 Nov 2026)

Design: [PLANNER_DESIGN.md](PLANNER_DESIGN.md). The slicer core now plans the powder geometry at export time, in
shadow mode:
- `GCode/DIWPowderPlanner` holds P0-P8, the analytic hatch, coverage, centroid, InteriorPointArea, the FB split and
  the LineMerger port.
- `GCode/DIWPowderShadow` captures in `process_layer` and runs the streaming binder in `GCodeOutputStream::write`.
- `GCodeScanner` / `ChunkTracker` were refactored out of `scan_gcode` / `split_layers`.
- `diw_translate` gained `--plan-json` and `--plan-records`.

The records are written into the G-code and nothing else in the file changes. A block is written only where the
native and text classifications agree.

The final build is from 18 Nov 22:07 (`OrcaSlicer.dll`; `diw_translate.exe` and `diw_planner_tests.exe` 22:09),
after `build_slicer_diw.ps1` ended with `SLICER EXIT CODE: 0`. It contains the nine review fixes ("Review fixes"
below and in PLANNER_DESIGN.md), and every gate in the table was rerun on it or on the 21:25 build, which differs
from it only in comments. The Python side is 3.15.0 with shapely 2.2.0 and GEOS 3.14.1, and v2 was unmodified.
Python runs use `py -3.15 -E -s` (see the deviations).

| Test | Scope | Result |
|---|---|---|
| T0 regression | 5 corpus files, final build | `compare_sequencer.py` 157 ok with and without `--stub-s4`; `compare_records.py` 486 ok; `compare_powder.py` 62 ok; `compare_v2.py` 54 ok; all with 0 failures and 0 skips. Rerun after every stage. |
| T9 planner units + T8 binder streams | Catch2, `diw_planner_tests` | 3489 assertions in 25 test cases, all passed (the review added the T8 case "placeholder-like custom G-code" and the cases "Review fix 2" and "Review fix 3"). |
| T1 pure planner vs v2, with T3 and T4(a-c) | corpus x variants, plus the notch and synthetics at 5 hatch angles (`compare_planner.py`) | 3443 checks, 0 failures. st/nc 492, areas/shape/hatch 488, fb 454, T3 454, T4a 79, T4b 420, T4c 79, D5 1. |
| Offline replay (stage 3 checkpoint) | `compare_replay.py`, corpus x `compare_records` variants + notch | 380 checks, 0 failures. BEGIN fields and st are string-equal to v2's. |
| T7 + T2(a)(b) + T6 (stage 4 checkpoint) | cube25, two_objects | 49 checks, 0 failures. The kill-switch export equals the pre-increment build; the statistics lines are equal; capture, plan and binder faults all degrade cleanly. |
| Step 5a, slicing matrix | 191 slices (`shadow_export.py --matrix --slice-only`) | 1915 checks, 0 failures. 3153 native blocks, one per v2 text-core layer. Every planner summary is clean. |
| T2(a-e) + T3 | the same 191 slices | 25515 checks, 0 failures: T2a 1910, T2b 191, T2c 3726, T2d 191, T2e 382, T3 3153, areas/fb/hatch/shape/st_nc 3153 each, and D5 192 (per slice and overall). The disagreement counter is 0. `single_bead` is 0 on every slice. |
| T4(d) | cube25, cone, lshape, shifted by 1 µm and 10 µm | 40 checks, 0 failures. |
| T10 | cube25, cone | 1.135x and 1.116x the kill-switch export time (limit 1.2x, best of 5); planner 33.3 ms and 32.1 ms; `max_held` = 1. |
| T5 seam gap 15 % | cube25, ring, notch, lshape x classic/Arachne (8 slices) | 1150 checks, 0 failures: T5i 8, T5ii 32, T5iv 16 (+168 per kind), T5v 184, D5 9. |
| Review regressions | `regress_review.py` R1-R9 (CLI exports, the matrix slices, the corpus) | 40 checks, 0 failures. |
| T2(a-e) + T3 on the stress variants | 16 slices (`shadow_export.py --matrix --stress`): by-object sequential (two_objects, lshape x2, multi-height), objects at 0.2 and 0.3 mm layer height (classic, Arachne), PE + fan speed-up + cooling slowdown (cube25, cone, overhang), arc fitting (cube25 + ring plate, classic and Arachne; sparse-infill-only arcs on cube25), scarf seam (cube25, ring, Arachne lshape with `all`), policy=error with an overflow (cube25, cone) | 2621 checks, 0 failures: S6 (variant reached) 47, T2a 160, T2b 18, T2c 334, T2d 16, T2e 28, T3 333, areas/fb/hatch/shape/st_nc 333 each, and D5 15. The disagreement counter is 0 on every slice. |

Measured values:

**T2 matrix** (the metrics are those of the accepted comparisons since the review fixes):
- Hausdorff: C 1.5 µm (after the 10 µm opening), Ω 0.77 µm, runs 1.24 µm. Before the fix the summary showed Ω 2.1
  and runs 3.8 µm, measured on rejected first comparisons.
- Point dwells: position 0.14 µm, d rel. 2.7e-3. `fb_pos` 0.
- D5: 117 of 3350 components have a row on a horizontal Ω edge. All are now compared without their edge rows; none
  is excluded.
- Arbitration logs: v2 discretization 342, D2 native band 42 (all `d2_seam_tail`, pieces verified against the
  text), Ω ill-conditioned (exact clearance) 17 with hatch and fb self-checked, tight excess 7, v2 at finer arcs 1,
  ra ill-conditioned 5, ra exceptions 0.

**T3:** the minimum record slack is −2.6e-7 mm, against the −10 nm tolerance. After rounding to 3 decimals it is
3.2e-4 mm. v2's own minimum is −3.4e-4 mm. Paths printed wider than their first-segment `;WIDTH:` are obstacles at
their own width.

**T5:**
- v2 gives st=open on all 168 core layers at 15 %. Native gives ok on every core layer, with the same k set as at 0 %.
- T5(iii), reported: cube25 and lshape are within 1e-6 relative and 0.5 µm.
- Ring and notch reach 1.8e-5 relative area, an Ω Hausdorff of 3.9 µm and a run Hausdorff of 2.1 µm. The cause is
  in the text inputs: rounded seam vertices in B_text and concentric solid infill clipped by the gap. The native
  inputs are identical at 0 % and 15 %.
- T5(v): the minimum slack is 4.4e-7 mm, and the translator accepts all 16 files.
- T5(iv) metrics of the accepted comparisons: Ω Hausdorff 0.72 µm, runs 1.27 µm (the summary used to show the
  3.9 µm of the rejected first comparisons).

**Stress variants (step 6b):**
- Every variant is shown to reach what it stresses (S6): the overrides are in the config block; G2/G3 on 14
  otherwise-core chunks per arc plate; both layer-height grids printed; z drops between objects in sequential
  mode; a z ramp in the outer wall with the scarf; moves at the cooling-slowdown floor speed.
- No block sits on an arc or G91 chunk. The arc plates get blocks on the 25 cube-only layers above the ring.
- Hausdorff: C 1.15 µm, Ω 0.72 µm, runs 0.47 µm. T3 minimum slack 4.4e-7 mm (3 decimals: 4.4e-4 mm).
- policy=error: every v2 text-core layer carries the `st=open` carrier with s = s_hatch and F = fu = f_calc > f_max.
  v2 and `diw_translate` abort with the same message at the first carrier's layer.
- Scarf seam: v2 cannot plan the scarf text. It gives st=open on every core layer of cube25 and lshape, and plans the
  ring without its outer ring, with Ω of 118.7 instead of 47.9 mm². The native blocks equal the native blocks of the
  same part without the scarf: they are byte-equal on cube25, and ca differs by 5e-4 mm² on the ring. The reference
  is therefore v2 on the no-scarf export (like T5(iv)). All 52 layers pass the T2(c) tolerances against it.
- Found and fixed: the capture's arc predicate treated every path with an arc-fitting result as arc-printed. A fit
  made only of linear moves prints G1, so with sparse-infill arc fitting on cube25 all 32 core layers were
  `disagree_text_only`. The predicate now also requires an arc segment that the writer prints.

**Review fixes (18 Nov 2026).** An adversarial review confirmed nine defects; all are fixed, each with a regression
test (`regress_review.py` R1-R9 and Catch2 cases). Product code:
- the `;DIWP` placeholders carry a per-export token, so user custom G-code that looks like a placeholder is no longer
  deleted from the export or counted as a capture;
- a wall printed as one bead (an Arachne odd centreline loop, a classic thin-wall loop) is no longer a cavity ring,
  so no powder is planned into the empty bore of a thin tube (`single_bead=<n>` in the summary line);
- text paths whose later segments are printed wider than the first (variable-width gap fill) are excluded at each
  segment's own `;WIDTH:`, so runs no longer land inside the printed bead.
Harness: the D2 / seam-band references verify the native pieces against the printed text; the T1 ra exception is
bounded; the ill-conditioned-Ω path checks hatch and fallbacks; the quad_segs 256 arbitration requires the same
plan; statistics come from the accepted comparisons; the D5 exclusion is bounded and D5 components are compared
without their edge rows.

Logs are in the session scratchpad `planner/`: `fx_cp2.log` (T1), `fx_5a.log` (5a), `fx_t2b.log` (T2 + T3),
`fx_t4d.log`, `fx_t10.log`, `fx_t5.log`, `fx_st.log` (stress variants), `fx_rp.log` (replay), `fx_s4.log` (stage-4
checkpoint), `fx_rr.log` (review regressions) and `t0s6b/*.log` (T0 on the final build).

### Divergences from v2 (D1-D8, design section 9) as measured

- **D1, seam gap.** Confirmed by T5: v2 gives st=open on every core layer; native closes the loops and plans them.
  The scarf seam shows the same v2 weakness: the scarf retraces the loop with a z ramp, and v2's snap + linemerge
  no longer forms the ring.
- **D2, Arachne variable widths.** The band is B_native ∪ B_text. On 42 matrix layers the T2 reference was v2
  with the verified native band added; on all of them the difference is the closed native loop covering its
  unprinted seam tail (classic and Arachne), not a width.
- **D3, inner-wall overhang pieces.** No case was logged on the matrix. The core set equals v2's on all 191 slices.
- **D4, mitre tips under 23°.** Removed. P5 ports GEOS's limited mitre, and the 15° and 22° tip synthetics revert
  exactly what v2 reverts.
- **D5, rows on a horizontal Ω edge.** 2 components in T1 and 117 of 3350 in T2 have such a row. They are compared
  without their edge rows; none is excluded (the bound is < 5 % per slice and overall).
- **D6, two native-core placeholders in one chunk.** None occurred.
- **D7, Ω containment in v2's Ω ⊕ 0.3 µm.** Holds. Where v2's own Ω is ill-conditioned, the T2(c) arbitration
  rules apply (see the deviations below).
- **D8, planner failures.** None occurred; the planner summary is clean on every slice.

### Deviations from PLANNER_DESIGN.md

The details are in that file's Deviations section.

**Harness and process**
- Python runs use `py -3.15 -E -s` instead of `python -I`, because `-I` breaks the harnesses' sibling imports.
- `diw_planner_tests` is a dev-utils target, because this build tree has no `BUILD_TESTS`.
- `--plan-records` also takes the translate-mode settings and `--feed-size`.
- `compare_replay.py` is the stage-3 checkpoint.
- T7's pre-increment baseline is the stage-3 build.
- The CLI log goes through `--logfile`.
- The matrix varies one factor at a time (191 slices) instead of the full product.
- The repo meshes are scaled with `--scale`.
- The 5a and T2 passes are separate (`--slice-only`, `--reuse`).
- `--only T10` runs T10 alone.

**Scanner and binder**
- `ChunkTracker` has `finish()` and `n_opened()`, and queues closed chunks.
- `PowderShadow` has a direct constructor, plus `load_dump_dir`, `feed(string_view)` and `stats()`.
- The config-block test is "the line contains `; CONFIG_BLOCK_START`".
- On an exception, held lines are released in order.
- The `sync` and `lost_placeholder` accounting are defined.
- In `text_only` mode, gate 0 is skipped and a missing capture falls back to the adapter.
- The dump JSON schema is fixed.
- The summary line adds `max_held`, `capture_ms` and `binder_ms`.

**Capture**
- Placeholders are `;DIWP <token> <id> m=<0|1>` with a per-export token; other `;DIWP` text passes through.
- A ring needs the layer's slice beyond the bead on one side (`bead_bounds_slice`); single-bead walls are no ring.
- Text paths carry per-segment widths; segments wider than the first are dilated at their own width.
- The arc predicate passed to `capture` requires a printed arc segment, not just an arc-fitting result (found by the
  stress variants).
- `capture` takes `CaptureLayer`s, because `LayerToPrint` is nested in `GCode`.
- `ShadowFrame` carries a `to_grid` callback.
- `create()` runs at H1 and reads spiral mode and the axis remap from the config.
- A `DIWConfigError` refuses silently.
- Warnings are also logged.
- The instance set depends on sequential mode.
- A connected all-wall, single-width loop is stored as one closed piece.
- An exception before the classification writes `m=1`.
- In passthrough, a placeholder's capture is consumed with its line.
- The gate 9 frame tolerance is `10^-d·√2 + 1 nm`.

**Geometry**
- The dilations and the erosion are unions of exact circumscribed pieces (1024 steps per 360°, radius + 1 nm), not
  chord-padded `ClipperOffset` calls. For speed (T10), they are now built as raw offset paths on a global angle grid,
  with these additions:
  - arc caps that lie outside S are replaced by chords;
  - duplicate closed curves are dropped;
  - T9 checks the raw paths against the piece construction.
- 256 and 512 steps per 360° were rejected, because T1 fails with them.
- P0-P5 run in an integer-shifted local frame.
- `PlanInput::open_outer` is a field, and P1b (snap + LineMerger) runs inside the planner.
- The P5 opening is a GEOS limited-mitre port after a 10 nm Douglas-Peucker simplification. This removes D4.
- narrow is taken of C opened by 100 nm.
- P4 computes (S eroded) − (X dilated by w/2 + D) instead of eroding C.
- The booleans call Clipper2 directly, with parallel per-polyline unions.

**Tolerances and arbitration (all logged)**
- The C Hausdorff is taken after a 10 µm opening.
- A case that passes against v2 at quad_segs 256 counts as v2's arc discretization.
- ra is accepted inside v2's own quad_segs spread.
- Point dwells use first-order area and centroid bounds.
- T4(b) bounds scale with boundary length and corner angle.
- T4(c) uses 1e-5 relative areas.
- T2(c) also accepts v2 at finer arcs, and the "tight excess" over an ill-conditioned v2 Ω.
- The T2(e) header run-count line may differ only where T2(c) accepted a different run count.
- T5(iv) falls back to the `t5_band_model` reference (20 layers).
- In T5, T3 is done against each block's own text.
- Scarf slices use v2 on the no-scarf export as the reference, with T3 done against each block's own text.
- policy=error slices replace T2(e), since there is no output to diff, by the equal-abort-message check.

**Test-only settings**
- `ORCA_DIW_TEST_KEEP_SEAM_GAP=1` (PrintApply.cpp / Print.cpp) lets a DIW export keep a seam gap for T5. Without it,
  nothing changes.
- `ORCA_DIW_TEST_KEEP_FDM=1` (PrintApply.cpp) keeps arc fitting and the cooling slowdown on a DIW export for the
  stress variants. Without it, nothing changes.
- Objects at two layer heights come from a CLI assemble list (per-object `layer_height`), and the arc variants use a
  cube25 + ring plate.
- The prime-tower variant uses `wipe_tower_type = type1` and a `;TYPE:Prime tower` tag. Upstream WipeTower2 crashes
  with one tool.

## Open questions (current defaults keep v2 parity)

- Generated XY travel feed: v2 constant 3000 mm/min (used now) or Orca's travel speed?
- Soft limits: unconfigured with a warning (as v2, used now) or derived from the bed size?
- Explicit bottom/top layer counts (requirement 4.1) vs automatic role-based classification (used now).
- Several objects on a plate: per-object classification (native planner) vs v2's per-G-code-layer.
