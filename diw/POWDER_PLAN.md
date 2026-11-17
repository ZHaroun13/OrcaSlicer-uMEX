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
| 4 | Slicer-core planner in shadow mode (records written by `GCode.cpp`, nothing suppressed); tolerance tests vs v2, clearance invariant, perturbation test, seam-gap 15 % test | |
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

## Open questions (current defaults keep v2 parity)

- Generated XY travel feed: v2 constant 3000 mm/min (used now) or Orca's travel speed?
- Soft limits: unconfigured with a warning (as v2, used now) or derived from the bed size?
- Explicit bottom/top layer counts (requirement 4.1) vs automatic role-based classification (used now).
- Several objects on a plate: per-object classification (native planner) vs v2's per-G-code-layer.
