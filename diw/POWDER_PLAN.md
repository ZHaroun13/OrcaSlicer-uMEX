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
| 2 | Freeze record format v1 (`DIWPowderBlock` parser/writer, CRC-32, fingerprint); Python `v2_blocks.py` (v2 plans as records) and `v2_hooks.py` (capture / stub plans) | |
| 3a | Sequencing from records: S1, S2 wipe, S3 UV, S5 compaction, S6 (milestone 3); S4 stubbed on both sides | |
| 3b | S4 powder emission from records (milestone 4); byte-identical with v2's own plans as records on all fixtures | |
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

## Open questions (current defaults keep v2 parity)

- Generated XY travel feed: v2 constant 3000 mm/min (used now) or Orca's travel speed?
- Soft limits: unconfigured with a warning (as v2, used now) or derived from the bed size?
- Explicit bottom/top layer counts (requirement 4.1) vs automatic role-based classification (used now).
- Several objects on a plate: per-object classification (native planner) vs v2's per-G-code-layer.
