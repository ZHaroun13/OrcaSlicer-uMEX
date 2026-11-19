# Increment 5: product path (design)

Status: design, revision 2 (19 Nov 2026), after an adversarial review (section 12 lists each point and how it was
handled). **Implemented** in stages 1-5 (19 Nov 2026). Every gate of section 8 passes; the measured numbers are in
POWDER_PLAN.md "Increment 5 results". What changed during implementation is listed at the end of this file
(Deviations, "Made during implementation").
Read with [POWDER_PLAN.md](POWDER_PLAN.md) (increments table, "Increment 4 results", "Open questions"),
[PLANNER_DESIGN.md](PLANNER_DESIGN.md) (the shadow-mode planner this increment turns into the product path; section
numbers "PD §n" below refer to it), [POWDER_RECORDS.md](POWDER_RECORDS.md) (record format v1, **frozen**, not changed
here) and [FEATURE_MAP.md](FEATURE_MAP.md).

Increment 5 in POWDER_PLAN.md: "Product path: sparse infill suppressed on core layers, translator requires valid
records, `seam_slope_type` forced off, validation rules".

**Constraints.**
- `PrintConfig.hpp` is not touched: no new config keys. Modes and test switches are environment variables, as in
  increment 4. `PrintConfig.cpp` is not touched either (its `diw_*` min/max already match v2's hard limits).
- `GCode.hpp` is not touched (the shadow is held through the existing `std::shared_ptr<DIW::PowderShadow>` and the
  forward declaration). Header edits are limited to `GCode/DIWPowderShadow.hpp`, `GCode/DIWUMexSequencer.hpp` and
  `GCode/DIWTranslator.hpp`, whose includers are few (GCode.cpp, PostProcessor.cpp, OrcaSlicer.cpp, dev-utils).
- The v2 engine (`diw_engine.py`) is the oracle and is never modified. Record v1 is frozen.
- Decided product defaults stay (v2 parity): generated XY travel feed 3000 mm/min; soft limits unconfigured with
  v2's warning; automatic role-based core classification (no explicit N_bottom / N_top). B from the volume to
  extrude; UV, wipe and compaction are fixed sequences; the powder nozzle is an offset tool of the main extruder.
- Out of scope: preview / time estimate of the powder stages / Powder role colour (increment 7), added time and
  powder mass report (increment 6).

---

## 0. Decisions in one page

| Question | Decision |
|---|---|
| Who decides that a layer is a core layer? | The **native classification** (PD §5.1: `erInternalInfill` role present, no `erTopSolidInfill` / `erBottomSurface`, no arc-fitted path, over all objects of the print-Z layer), **and** the chunk must contain at least one `powder`-kind text path (3.2, `native_only` (b)). The text classification (v2 `classify_layers`) is still computed, only for diagnostics. |
| Per layer or per object? | **Per printed layer** (v2 parity): one object with a top or bottom surface makes the whole layer non-core. Per-object classification is a user question (section 10, Q1); record v1 could carry it later (FB lines). |
| Where is sparse infill suppressed? | At G-code emission, **in the binder**, after the layer has been planned and its block written, and only on layers whose block is `st=ok`. It is not cut at fill generation or in `GCode::process_layer` (section 4.1). |
| What does the planner still get? | Everything it gets today: it plans on the full text of the layer (sparse paths included, so P8's FB clipping and the RUN data are computed exactly as in increment 4). Suppression edits the held lines afterwards. |
| What is removed? | Exactly the lines the translator already ignores in a converted layer: the extrusion moves of every `powder`-kind path (v2 `parse_core_layer`) and that path's `pre_lines`. Minimal translator-invisible **restore lines** keep position, feed, absolute E, width and (at the chunk end) the modal `;TYPE:` (section 4.3). Object-label lines in a sparse path's `pre_lines` make the chunk fall back (unedited). |
| How is it proven safe? | Per chunk, before release: the translator's emission trace, the end-of-chunk scanner state (width included) and the chunk's extruded bounding box (the UV-centre rule) of the edited chunk must equal the original's (V1, V2). Otherwise the chunk is released unedited (fallback, counted). End to end: `translate(product export) == translate(shadow export)` byte for byte apart from `M73 P<n> R<n>` lines and a frozen list of statistics comment lines outside the layers (section 8, G1), on a slice set that is required to contain FB paste and `st=small` layers. |
| Strictness | **Product mode:** a native core layer that does not get a valid block (`st=ok`, `st=small` or one of the two parity carriers) stops the export with a clear message. No silent paste fallback. Exceptions, each warned: `st=small` (v2's paste fallback), the final layer (v2 never fills it), and a native core layer whose text has no sparse path at all (`native_only_nosparse`). |
| Where is strictness enforced? | In the binder at export (it is the only place that knows the native classification; record v1 is frozen and cannot carry it). The translator adds its own checks: no block in a powder job, and a non-carrier `st=open` block, are errors in production. Structural cases are rejected by validation first; the export-time refusal is a tested safety net. |
| Test modes | `ORCA_DIW_POWDER_MODE=shadow` gives the increment-4 behaviour byte for byte (oracle exports for v2 and replay). `ORCA_DIW_TEST_NO_SUPPRESS=1` gives product authority and strictness without suppression. `diw_translate --v2-parity` gives v2's `st=open` behaviour. `diw_translate --plan-records` is shadow unless `--product`. The kill switch, fault injection, dump and keep-placeholder switches stay; new test hooks: `ORCA_DIW_TEST_SKIP_VALIDATE`, `ORCA_DIW_TEST_SUPPRESS_MUTATE`, `ORCA_DIW_TEST_SKIP_VERIFY`, fault `suppress` and `cancel`. |
| Scarf seam | `seam_slope_type` forced to `none` for DIW powder jobs in `apply_diw_overrides`; a per-object / per-part / layer-range scarf is a validation error (`ORCA_DIW_TEST_KEEP_SCARF=1` keeps it, test only). |
| Validation | `diw_validate(full config)` in `DIWTranslator.cpp`, called from `Print::validate`. It reuses `diw_read_translator_params`, so every hard limit raises **v2's exact message** before slicing. v2's soft warnings and the retraction guard become validation warnings. New product-mode errors with no v2 counterpart: `wall_loops = 0`, no sparse infill anywhere, `infill_combination` on (6.3). |

---

## 1. Data flow (product mode)

```
process_layer ── capture (unchanged): ;DIWP <token> <id> m=<native_core>
        │
        ▼ CoolingBuffer / PE / FanMover / PA (unchanged; they see the full layer, sparse infill included)
GCodeOutputStream::write ──▶ binder (PowderShadow, mode = product)
        chunk closes ─▶ finalize: gates (native authority, 3.3) ─▶ parse_core_layer ─▶ plan ─▶ write_block
                     ─▶ dump (pre-edit, if ORCA_DIW_SHADOW_DUMP)
                     ─▶ [st=ok] orphan check (3.4) ─▶ suppress_sparse (4.3) ─▶ verify V0-V3 (4.4)
                     ─▶ release: marker line, block, edited (or original) chunk lines
                     └▶ fatal reason? ─▶ recorded (5.1)
        │
        ▼ write_raw: fwrite + GCodeProcessor::process_buffer (sees the edited text: preview and time estimate
          no longer contain the suppressed sparse infill)
do_export (inside the existing try): refusal check; _do_export; file.diw_finish();
          if (shadow->failed()) throw DIWConfigError(shadow->failure_message());
        │
        ▼ diw_translate_gcode_file (strict, 5.3)
```

---

## 2. Modes and switches

| Switch | Values | Effect |
|---|---|---|
| `ORCA_DIW_POWDER_MODE` | unset or `product` (default), `shadow` | `shadow`: increment 4 exactly (agreement gate 3, no suppression, warnings only). Read by `ShadowOptions::from_env()`, which `PowderShadow::create` uses. |
| `ORCA_DIW_TEST_NO_SUPPRESS=1` | test only | product mode without the suppression step (authority and strictness only). |
| `ORCA_DIW_NO_NATIVE_PLAN=1` | test only (kill switch, unchanged) | no binder; the export succeeds and the translator refuses the powder job ("no powder plan"). |
| `ORCA_DIW_SHADOW_FAULT` | `capture`, `plan`, `binder` (existing), `suppress`, `cancel` (new) | shadow mode: degrade as in increment 4 (T6); `cancel` throws `CanceledException` from the first core chunk's finalize in both modes. Product mode: `capture` / `plan` / `binder` stop the export with the internal-error message; `suppress` makes the suppression throw, which is a counted fallback (the chunk is released unedited), never a failure. |
| `ORCA_DIW_SHADOW_DUMP`, `ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS` | unchanged | honoured in both modes. The dump is written **before** the edit (it holds the native capture, which the edit never touches) and gains `"suppressed": true/false`, `"fallback": "<reason>"` and `"chunk_sha_pre"` (hash of the unedited chunk). A product-mode `keep` export contains the edited text; replay and v2 oracles use only shadow-mode `keep` + dump (4.5, 8). |
| `ORCA_DIW_TEST_KEEP_SEAM_GAP=1`, `ORCA_DIW_TEST_KEEP_FDM=1` | unchanged | as in increment 4. |
| `ORCA_DIW_TEST_KEEP_SCARF=1` | new, test only | `apply_diw_overrides` keeps `seam_slope_type` and `Print::validate` accepts a per-object scarf (stress variants). |
| `ORCA_DIW_TEST_SKIP_VALIDATE=1` | new, test only | `Print::validate` skips the DIW powder-job rules of 6.2 and 6.3 (the existing increment 1-4 DIW rules stay), so the export-time safety nets (H1 refusal, `open`, `no_core`) can be reached (G5). |
| `ORCA_DIW_TEST_SUPPRESS_MUTATE` | new, test only: `drop_restore`, `extra_delete`, `wrong_g92` | `suppress_sparse` deliberately produces a wrong edit on every `st=ok` chunk. With verification on, every chunk must fall back (V1/V2 catch it). |
| `ORCA_DIW_TEST_SKIP_VERIFY=1` | new, test only | the binder releases the edited chunk without V0-V3. Used only with `..._MUTATE` to prove that G1/G3 fail on a wrong edit (gate-level mutation, G10). |
| `DIWTranslateOptions::v2_parity` / `diw_translate --v2-parity` | new, test only | the translator leaves a non-carrier `st=open` layer as paste with v2's warning instead of refusing. Every harness that translates records made by v2 (`v2_hooks` / `v2_blocks`) passes it, and so does every translation made by `shadow_export.py --powder-mode shadow` (8, S0). |
| `allow_unconverted_powder` / `--allow-unconverted-powder`, `stub_powder_stage` / `--stub-powder-stage` | unchanged | test only. |
| `diw_translate --plan-records [--product [--no-suppress]]` | `--product`, `--no-suppress` new | offline binder (PD §9). **Without `--product` it sets `mode = shadow` explicitly**, whatever `ORCA_DIW_POWDER_MODE` says, so the increment-4 T2(d) replay and G9's reference command keep their meaning. `--product` switches to product mode (native from `ORCA_DIW_SHADOW_DUMP` when the input kept its placeholders, else the text-only adapter); `--no-suppress` clears `suppress`. |

`ShadowOptions` gains `enum class Mode { shadow, product } mode = Mode::shadow;`, `bool suppress = true;`, and the test
fields `mutate`, `skip_verify`. The struct default stays `shadow`, so the existing Catch2 T8 streams keep their
increment-4 meaning; `from_env()` (and therefore `PowderShadow::create`) defaults to `product`. `diw_translate.cpp`
overrides `mode` after `from_env()` as described above.

---

## 3. Native classification as the authority

### 3.1 The rule

`native_core(chunk) = OR of m over the chunk's placeholders` (PD §6.3 gate 3), with `m` from PD §5.1 over every
object layer printed at that print Z. This is v2's auto rule evaluated on in-memory roles instead of `;TYPE:` text,
**per printed layer**: if any object of the layer has a top or bottom surface, no object of that layer gets powder,
exactly as v2 does with the G-code layer. Objects of a core layer that have no sparse infill (only walls, or walls and
internal solid infill) contribute rings, band and paste exclusion; their cavity is covered by their own paste, so they
get no runs (v2 parity, unchanged planner).

A native core chunk gets a block only if `parse_core_layer` finds at least one `powder`-kind path in it (3.2,
`native_only` (b)).

By-object sequential printing gives one chunk per object layer, so the rule is per object there (as in v2).

### 3.2 Disagreement classes and how product mode resolves them

The increment-4 matrix (191 slices) and the 16 stress slices had a disagreement counter of 0. The classes below are
the ones found or anticipated (PD §13, PD Deviations stage 5a / 6b):

| Class | Cause | Shadow mode (inc. 4, unchanged) | Product mode |
|---|---|---|---|
| `text_only`: text core, native not | Modal `;TYPE:` inheritance: moves without their own `;TYPE:` (custom G-code, the type1 prime tower's `; FEATURE:` tags) inherit `Sparse infill` from the previous feature on a layer that has no native sparse role (or has a native skin role). | no block, warning | **No block** (native decides: not core). NON_CRITICAL warning `"u-MEX powder: layer(s) <k list> have moves typed sparse infill but no sparse infill of the part (custom G-code or a prime tower inherits ;TYPE:); they are printed as paste."` |
| `text_only` from arc-fitted infill | Fixed in increment 4 (the arc predicate requires a printed arc segment). Arc fitting is forced off on DIW printers; only `ORCA_DIW_TEST_KEEP_FDM=1` reaches it. | no block | Native excludes arc-fitted layers itself (`has_arc`), so both agree: no block. |
| `native_only` (a): native core, text has sparse paths but a skin type too | The text carries a skin type that the slicer did not print as skin (custom G-code typed `;TYPE:Top surface`, or inheriting it), and at least one `powder`-kind path. | no block, warning | **Block** (native decides: core), subject to every other gate. Text skin-typed moves are `paste` kind, so the planner excludes them (E) and the translator re-emits them. Counted (`native_only=<n>`), no warning. |
| `native_only` (b): native core, **no** `powder`-kind text path | Every native sparse path was dropped at emission (too short), so the text has no sparse path; v2 and the slicer's own paste treat the layer as solid or empty. | no block, warning | **No block** (no oracle, and powder would go where v2 never puts it). NON_CRITICAL warning `"u-MEX powder: layer(s) <k list> have sparse infill in the slice but none was printed (all paths too short); they are printed without powder."`, counted `native_only_nosparse=<n>`. Not fatal. |
| Untyped extrusion after sparse infill | A custom `layer_change_gcode`, pause or filament G-code that extrudes without its own `;TYPE:` inherits `Sparse infill` from the end of the previous layer (Orca prints sparse after the walls by default), so it is `powder` kind on the next layer. | v2 and shadow drop it silently in core layers | In a native core layer: inside S it is removed exactly as v2 removes it (no change); **outside S it is fatal `orphan`** (3.4), on every core layer where it occurs, with the fix in the message. The test profiles' `layer_change_gcode` (`;AFTER_LAYER_CHANGE` / `;[layer_z]`) and `before_layer_change_gcode` do not extrude, and `resources/profiles` ships no DIW printer profile (checked 19 Nov at HEAD 19d98828; stage 5 re-checks). |
| text arcs or G91 in a native core layer | Arcs or `G91` in custom G-code of a core layer (`chunk.has_arc` / `has_relxyz` while native has no arc path). | `arc_g91`, no block | **Fatal** (5.1): the translator cannot convert such a layer, and leaving it as paste would print the part without powder there. |
| Two native-core placeholders in one chunk (D6) | practically unreachable | `merged_multi_core` | **Fatal** (internal error). |
| Native core on the final chunk | `top_shell_layers = 0`, or the last printed layer of a plate has sparse infill and no top surface | `final`, no block | **No block**, NON_CRITICAL warning `"u-MEX powder: the last printed layer (k=<k>) has sparse infill without a top surface; the last layer is never filled with powder, it is printed as paste."` (v2 rule; user question Q3). |
| Objects disagree on one layer | object A sparse only, object B top / bottom surface at the same print Z | not core (both rules) | Not core (per-layer rule, v2 parity). Covered by the multi-object plates of the matrix and the `tall_short` plate (G4). |

### 3.3 Binder gates in product mode

Gates of PD §6.3, in order; "fatal" means the reason is recorded in the failure list (5.1), no block is written, and
the export fails at `diw_finish`. Gates 0, 6-8 and 11 are unchanged.

| # | Shadow (unchanged) | Product |
|---|---|---|
| 0 | no placeholder: silent | same |
| 1 | `final`: count | if native core: warning `final` (3.2); else silent |
| 2 | `arc_g91`: count | if native core: **fatal** `arc_g91`; else silent |
| 3 | `text_core != native_core`: count, no block | `native_core` decides. `text_only`: warning, no block. `native_only`: counted, continue. Not native core: stop silently. |
| 3b (new) | — | native core and no `powder`-kind path in `parse_core_layer`: warning `native_only_nosparse`, no block. |
| 4 | `merged_multi_core` | **fatal** |
| 5 | `lost`, `capture` | **fatal** |
| 6 | `no_z` | **fatal** |
| 7, 8 | h <= 0 and policy=error carriers | same (carriers are valid blocks; the translator gives v2's warning / abort) |
| 9 | `frame` | **fatal** |
| 10 | `plan` | **fatal** |
| 10b (new) | — | planner `st=open` (P0 / P1c: no ring) on a native core layer: **fatal** `open` |
| 11 | `record` | **fatal** |
| 12 (new) | orphan check counted only (`orphan=<n>` in the summary) | `st=ok` only: **fatal** `orphan` (3.4) |
| 13 (new) | — | `st=ok` only: suppression (section 4) |

Also fatal in product mode: a chunk with an m=1 placeholder released by `sync()` (`sync`), `lost_placeholder` at
`finish()`, the binder's disabled mode (`binder`), and a powder export that ends with **no block at all** (`no_core`;
validation rejects the usual causes first, 6.3, so this is a safety net).

### 3.4 Orphan sparse check (gate 12)

The translator drops every `powder`-kind text path of a converted layer and fills only Ω. A `powder`-kind move that
is not part of the cavity (a prime-tower or custom-G-code extrusion inheriting `;TYPE:Sparse infill`, or sparse infill
of an island whose ring is missing) would silently disappear from the print. So, for an `st=ok` chunk: every vertex and
every segment midpoint of every `text_powder` path must lie inside S (the planner's union of rings, EvenOdd) or within
the orphan tolerance of its boundary (half the widest outer / overhang wall width of the layer, never below 10 µm; see Deviations, fix pass). S is returned by the planner (`PlanDebug::cavity_s`, or a new `PowderLayerRec`-side output that
does not enter the record). Product: fatal, with the first offending point. Shadow: counted only (no byte changes).
The orphan tolerance does not reach the UV centre: V2 (4.4) separately requires the chunk's extruded bounding box to be
unchanged by the edit.

---

## 4. Sparse infill suppression

### 4.1 Where it is cut, and why there

| Candidate | Why not / why |
|---|---|
| Fill generation (`Layer::make_fills`, `PrintObject::prepare_infill`) | The classification itself is the presence of the sparse role; `st` (`ok` / `small` / `open`) is only known after planning, and `small` layers must print their sparse infill as paste (v2's fallback); P8 needs the text sparse paths for the FB lines; removing fills changes internal-bridge detection over sparse infill, layer times (fan), travel and retraction on every layer, so the export could no longer be compared with the oracle. Rejected. |
| `GCode::process_layer` (skip `erInternalInfill` entities) | Same `st` problem: the planner needs the final text (k, z, h, text walls, paste exclusion) and runs in the binder. Rejected. |
| **Binder, after planning** | The chunk is held in full, planned on the full text (FB and RUN data computed before suppression), its block written, then edited. CoolingBuffer, PE, FanMover and PA have already run on the full layer, so fan and speed decisions do not move. The GCodeProcessor gets the edited text, so the preview and the time estimate drop the sparse infill that is never printed. **Chosen.** |

Only chunks with an `st=ok` block are edited. `st=small`, `st=open` (shadow / parity only), carriers, non-core chunks
and the final chunk are released unchanged.

### 4.2 What the translator reads from a converted chunk

From `UMexSequencer::convert_core_layer` (port of v2 `_convert_core_layer`) and `parse_core_layer`:
- the header lines (non-motion lines before the first extruding move of any kind), emitted with the S1 marker after
  the first one;
- for each `wall` / `paste` path in order: its `pre_lines` (emitted), its first point and `travel_feed`
  (`paste_travel`), and per segment the raw line, `de` and `feed` (`feed_paste_line`);
- the tail lines (emitted); `model.z` (first extruding move's z1) and `model.z_feed`;
- `chunk.height`, `chunk.z`, `has_arc`, `has_relxyz`;
- the hand-back state of the last line (`e_after`, `rel_after`, `feed`);
- whole file: `scan_gcode` state flowing into the next chunks (including the modal width, which a re-plan reads), the
  first Z-only feed of the file, the extrusion bounding box (`Translator::compute_geometric_center`, DIWTranslator.cpp
  ~327: its own scanner over every extruding move with `de > 1e-7` after the first layer marker, start and end points,
  setting the UV target of every layer) and the last `; stop printing object` line.

`powder` paths, their `pre_lines`, and every non-extruding motion line (travel, retract, wipe, z-hop) are never emitted.
Note: `pending` collects every non-motion line since the previous path was **created**, so a `powder` path's
`pre_lines` can include comment lines that sit physically inside the previous path (for example a mid-path
`;WIDTH:`). Those are dropped by the translator today, so removing them is invisible to it as well.

### 4.3 The edit (`suppress_sparse`)

Pure function in `DIWUMexSequencer.{hpp,cpp}` (next to `parse_core_layer`; no CMake change):
```cpp
struct SuppressResult {
    std::vector<std::string> lines;        // the edited chunk (line 0 = the start marker, unchanged)
    int removed_moves = 0, removed_other = 0, restore_lines = 0;
    double removed_de = 0.;                 // sum of the removed moves' de (G8)
    std::string fallback;                   // non-empty: not applied, reason
};
SuppressResult suppress_sparse(const std::vector<std::string> &lines, const std::vector<LineInfo> &infos,
                               const LayerModel &model);   // model = parse_core_layer of the same lines
```
1. **Removal set R** = for every path p of `model.paths` with `kind == "powder"`: p's extruding lines (`seg_raw`
   line indices; `PathRec` gains `std::vector<size_t> seg_lines`) and p's `pre_lines`. Nothing else.
   **Object labels:** if any line of a `powder` path's `pre_lines` is an object label (`; start printing object`,
   `; stop printing object`, `M486`, `EXCLUDE_OBJECT_START` / `EXCLUDE_OBJECT_END`), the chunk is not edited (fallback
   `label_in_sparse`), so the raw file never has unbalanced labels. Other commands in those `pre_lines` (`M106`,
   `M107`, `M204`, ...) are removed; the translator drops them today, so the machine output is unchanged, and the
   raw-file effect is documented in 4.5.
2. **Gaps.** A gap is a maximal run of consecutive line indices in R. Kept lines keep their order and bytes.
3. **Restore group** after a gap g that contains at least one removed motion line or a removed `;WIDTH:` line, and
   once more after the last line of the chunk if needed. Let `orig` be the scanner state after g's last line in the
   original text and `prod` the scanner state the edited text has at that point. Emit, each only if needed, in this
   order:
   - R1 position: `G1 X<x> Y<y>[ Z<z>] F<f>` when (x, y, z) differ, with the X/Y/Z tokens copied verbatim from the
     original lines that set them and F copied verbatim from the travel that set the current `trav_f` (the removed
     lines are extruding moves and non-motion lines, so `trav_f` is the same before and after g). If no `trav_f`
     exists yet in the chunk, or only Z differs: fallback `no_trav_f` / `z_only`.
   - R2 feed: `G1 F<feed>` (token copied) when the modal feed after R1 differs from `orig.feed`.
   - R3 absolute E: `G92 E<e>` (token copied from the E word that set `orig.e`) when E is absolute and `e` differs.
     Placed after the chunk's first kept extruding move only (before it, a non-motion line would join the header and
     be emitted): otherwise fallback `leading_abs_e` (infill printed before any wall with absolute E).
   - R5 width: `G1 F<feed> ;WIDTH:<w>` (the current feed, the original value text) when the modal width differs from
     `orig.width`. The scanner parses comments on command lines (`GCodeScanner::step`), and a tag on a motion line is
     dropped by the translator in a converted layer. It is a separate line from R4 because the scanner reads only one
     tag per line (`TYPE` before `WIDTH`).
   - R4 type, **end-of-chunk scanner `ftype` restore only**: `G1 F<feed> ;TYPE:<t>` after the last line of the chunk
     when the modal type differs. Inside a chunk it can never fire: a kept move with no `;TYPE:` of its own after a
     removed sparse path inherits `Sparse infill`, is therefore itself `powder` kind and removed. R4 keeps the
     scanner and translator `ftype` of the next chunk equal; it has **no effect on the preview** (GCodeProcessor runs
     `process_tags` only on comment-only lines, GCodeProcessor.cpp ~4216-4236), and the next chunk's first move
     without its own `;TYPE:` stays `powder` kind there exactly as in the original.
   Height is not restored (the translator does not emit it; `chunk.height` only reads `;HEIGHT:` lines themselves,
   which are never in a sparse `pre_lines` without also being in V2's `chunk.height` check). Mode lines (M82 / M83 /
   G90 / G91) and `G92 E` are absorbed by `parse_core_layer` and never in R.
4. **No other change.** Retractions, z-hops, wipes and travels around a removed path stay (they are dropped by the
   translator in a converted layer, and dropping them here would need more restore lines).

### 4.4 Verification and fallback (per chunk, before release)

Run `GCodeScanner` + `parse_core_layer` over the edited chunk (the scanner seeded with the state before the chunk)
and require:
- **V0**: no extruding move of `powder` kind remains; `n_extruding > 0`; the start marker is line 0.
- **V1 emission trace**: `trace(model_edit) == trace(model_orig)`, where `trace` is the flat sequence
  `header[0], <S1>, header[1..], for each wall/paste path: pre_lines texts, (points[0], travel_feed), per segment
  (raw, de, feed); tail texts; model.z; model.z_feed` with doubles compared bit for bit. (Flattening makes a line
  that moves from a path's `pre_lines` into the header compare equal; the output is the same in that case.)
- **V2 end state and whole-file inputs**: the scanner state after the last line of both versions is equal in x, y, z,
  e, rel, absxyz, feed, ftype and **width** (so the hand-back, every later chunk and any re-plan read the same
  LineInfos); `chunk.z`, `chunk.height`, `has_arc`, `has_relxyz` are equal; and **the chunk's extruded XY bounding
  box, computed with `compute_geometric_center`'s rule** (its own M82 / M83 / `G92 E` bookkeeping, `de > 1e-7`, start
  and end point of each extruding move), is equal bit for bit. Equal per-chunk boxes imply an equal whole-file box,
  hence an equal UV target, whatever the orphan tolerance lets through. The routine is shared with the
  translator (moved to a free function `extrusion_bbox(lines, state)` that `compute_geometric_center` calls), so the
  rule cannot drift.
- **V3**: every removed `powder` vertex lies in S (already implied by gate 12; asserted again here).
Any failure, or an exception: release the original chunk unchanged, count `suppress_fallback:<reason>`, log the
first differing trace item. Never fatal: the translated output is the same either way, only the preview and the time
estimate keep the sparse infill. Target on the matrix: 0 fallbacks (G3).

### 4.5 Effects

| Topic | Effect |
|---|---|
| Machine output (translated) | None by construction (V1, V2) and by test (G1, G3). |
| FB / RUN data | Computed before the edit from the full text, as in increment 4. The raw file has no sparse infill left inside the revert regions; the FB paste is printed by the translator from the records. The preview shows neither FB paste nor powder until increment 7. |
| **Product exports are not self-describing for v2** | After suppression, v2 (and `diw_translate --plan-records` text-only) on a product file sees no sparse infill in the converted layers: no core layers, no FB, a different plan. **v2 and replay oracles may only consume shadow-mode exports or shadow-mode `keep` + dump files.** `product_gate.py` keeps a manifest of every export it makes with its mode and refuses to run v2, `compare_*`, or a `--plan-records` replay on a product export. The dump is taken before the edit, so a T2(d)-style replay from a shadow `keep` + dump stays reproducible. A user who runs v2 on a product export gets a paste-only plan for those layers; this is documented in FEATURE_MAP.md (stage 5). |
| Travel, retraction, wipe, z-hop | Kept as generated. The raw file can show a retract / travel / unretract into an empty cavity. They are motion lines the translator drops in converted layers. |
| Layer time, cooling, fan | CoolingBuffer ran on the full layer: fan speeds unchanged (slowdown is forced off on DIW printers). The GCodeProcessor layer times and the total estimate drop the sparse time; the powder stages are not timed yet (increment 7). M73 values and positions change (masked in G1). |
| Commands in removed `pre_lines` | `M106` / `M107` / `M204` / other commands that sat in a sparse path's `pre_lines` are removed. The translator already drops them in converted layers, so the machine output is unchanged; the raw file's fan / acceleration state as read by GCodeProcessor and the preview can differ from the full-layer text until the next such command. Object-label lines there make the chunk fall back (`label_in_sparse`), never a silent removal. |
| Paste usage | The processor's filament used drops by exactly the removed `de` (G8). |
| Support | Support types are `paste` kind and never removed; support-only layers are their own non-core chunks. |
| Prime tower | Tower moves are `paste` kind and kept. A tower that inherits `;TYPE:Sparse infill` (type1 `; FEATURE:` tags) is caught by the orphan check (fatal in product) instead of being dropped. |
| Infill printed first (`is_infill_first`), absolute E | Fallback `leading_abs_e` (no suppression on those chunks; output unchanged). |
| Width tags | Restored by R5 when the modal width changes (V2 checks it). The preview can still show a path after a removed sparse path with the width of the restore line's value, which is the original modal width. |

---

## 5. Strict export and translator

### 5.1 Binder failure list (product mode)

`PowderShadow` gains `bool failed() const` and `std::string failure_message() const`. The failures are aggregated by
reason (first 10 k values each, as the warnings are). The message starts with
`"u-MEX powder: the G-code export was stopped because these core layers would be printed without powder:"` followed
by one line per reason:

| Reason | Line |
|---|---|
| `arc_g91` | `layer(s) <k>: arc moves (G2/G3) or relative positioning (G91) in the layer; arc fitting is off on DIW printers, so check the custom G-code of these layers.` |
| `open` | `layer(s) <k>: the outer walls do not enclose a cavity.` |
| `orphan` | `layer(s) <k>: extrusions typed sparse infill lie outside the part's outer walls (first at X<x> Y<y>); sparse infill must lie inside the part's outer walls (tolerance: half an outer-wall width). Possible causes: an extrusion typed sparse infill outside the part, such as custom G-code without its own ;TYPE: line (for example the layer change G-code) or a prime tower, or an island whose outer wall is missing.` |
| `merged_multi_core`, `lost`, `lost_placeholder`, `capture`, `no_z`, `frame`, `plan`, `record`, `sync`, `binder` | `layer(s) <k>: internal error (<reason>: <first what>). Please report this with the 3MF.` |
| `no_core` (no block in the whole export) | `no layer of this plate is a powder core layer (sparse infill without top or bottom surface)<; the only one is the last layer, which is never filled>. Turn u-MEX powder infill off, or give the part sparse infill.` |

`GCode::do_export` (GCode.cpp, H1). `path_tmp` is already open when `PowderShadow::create()` runs (~2925-2960), and
the cleanup `catch` belongs to the `try` that starts at ~2964. So:
- The **product refusal** is thrown **inside the existing `try`, as its first statement**: when `create()` returned
  nullptr with a non-empty refusal reason, product mode is active (`ShadowOptions::from_env().mode == product`), the
  job is a powder job and the kill switch is not set, `throw DIWConfigError("u-MEX powder infill cannot be used: " +
  m_diw_refusal);`. The existing `catch` closes and removes `path_tmp`, fires the GCodeExportFinished error event and
  rethrows. (`create()` stays outside the `try`, unchanged.)
- **After `file.diw_finish();`** inside the same `try`: `if (m_diw_shadow && m_diw_shadow->failed()) throw
  DIWConfigError(m_diw_shadow->failure_message());`.
- `create()` returns nullptr with an **empty** reason when `diw_read_translator_params` throws `DIWConfigError`
  (DIWPowderShadow.cpp ~429). H1 does not handle that case: validation (6.2) rejects it before slicing, and the
  translator raises the same message at post-processing.
- A `CanceledException` thrown inside `_do_export` goes through the same `catch`; `failed()` is never consulted on that
  path, so a cancel never shows the failure list (G5 checks it with fault `cancel`).
The refusal-reason logic in `create()` is factored into a pure function `powder_refusal_reason(full_config, spiral,
axis_remap)` so Catch2 (T12) can check every reason without a `Print`.

The GUI shows the message at slicing time (the preview export runs the same code) and the CLI exits with an error.
Validation (section 6) rejects the structural cases before slicing; H1 is the safety net and is tested with
`ORCA_DIW_TEST_SKIP_VALIDATE=1` (G5).

Warnings (`text_only`, `native_only_nosparse`, `final`; suppression fallbacks only in the log) keep using
`active_step_add_warning`.

### 5.2 Summary line

`DIW powder planner (<mode>): <n> block(s), <c> carrier(s), <reason>=<n>..., suppressed=<chunks> fallback=<n>
removed_moves=<n>; max_held=... capture_ms=... binder_ms=... single_bead=... orphan=<n> native_only=<n>
native_only_nosparse=<n>`.
In shadow mode the text before `;` keeps its increment-4 form plus `orphan=<n>` at the end (harnesses parse it by key).

### 5.3 Translator (production default = strict)

`DIWTranslateOptions` gains `bool v2_parity = false;`. Changes in `diw_translate_gcode` / `UMexSequencer`:
- **No block in a powder job**: still an error unless `allow_unconverted_powder`; the message becomes
  `"u-MEX powder infill: this G-code has no powder plan (;DIW: records). Export it again from OrcaSlicer with u-MEX
  powder infill on; the plan is written during the export."` Every harness that asserts the old text is updated.
- **Non-carrier `st=open` block** (reaches the status check, so it is neither the h <= 0 nor the policy=error
  carrier): with `v2_parity`, v2's warning and paste as today; otherwise `DIWConfigError("u-MEX powder records: layer
  <k> has no closed wall loop (st=open); this powder plan cannot be printed. Export the G-code again from OrcaSlicer.")`.
- `st=small` stays v2's aggregated `revert_all` warning (a geometric outcome, not a planner failure); the h <= 0
  carrier keeps v2's warning. Both are accepted by the strict default (T13, G5).
- Unchanged and already fatal: every record reader error, placement, k, fp, z / h, F / s / fu mismatch.
- The translator cannot check "every native core layer has a block": the native classification is not in the file
  (record v1 frozen) and the text classification is no longer the authority. The binder does it (5.1).
- `compute_geometric_center` calls the shared `extrusion_bbox` (4.4); its output is unchanged (T0 proves it).

`diw_translate.exe` defaults to production behaviour; `--v2-parity` sets the option.

---

## 6. Validation rules

### 6.1 Scarf seam forced off

`apply_diw_overrides` (PrintApply.cpp), for `diw_printer && diw_powder_infill` and unless
`ORCA_DIW_TEST_KEEP_SCARF=1`: `seam_slope_type = none` when it is not already none (values already off are left as
they are, so a correct preset is reported unchanged). It runs before the `has_scarf_joint_seam` computation (already
the case). Reason: the scarf retraces the outer wall with a z ramp below the layer plane, v2 cannot plan it (st=open,
D1-like), and the overlap double-extrudes paste at the seam.
`Print::validate`: a region with `seam_slope_type != none` on such a job (per-object, per-part or layer range) is an
error: `"u-MEX powder jobs cannot use a scarf joint seam: it ramps the outer wall below the layer and the powder cavity
is planned on flat layers. Set \"Scarf joint seam\" to None in this object's settings."`, opt_key `seam_slope_type`.
Non-DIW printers and DIW paste-only jobs keep their scarf (T7 checks both with a scarf on).

### 6.2 Hard limits and warnings (v2 `UMexParams.from_config`, diw_engine.py:483-569)

New `struct DIWValidation { std::string error, opt_key; std::vector<std::pair<std::string, std::string>> warnings; };`
and `DIWValidation diw_validate(const DynamicPrintConfig &full)` in `DIWTranslator.{hpp,cpp}`. It calls
`diw_read_translator_params(full)` inside `try`; a `DIWConfigError` becomes `error` with its **exact** text (so the
slicer and the translator give the same message, v2's), and `config_warnings` become warnings. The opt_key comes from
an **explicit table** `{v2 message prefix -> fork key}` in `DIWTranslator.cpp` (one row per line below; the pins
message maps to `diw_powder_pin`); a message with no row gets an empty opt_key and T13 fails on it. Checks in v2's
order, all only when `diw_printer` and `diw_powder_infill` are on. Tolerances are v2's and are already in
`DIWUMexParams.cpp`; the table states them so that the tests use the same boundaries:

| v2 check | Fork key (opt_key) | Kind | Message (v2, verbatim) |
|---|---|---|---|
| compaction_duration > 8.0 | `diw_compaction_time` | error | `compaction_duration = <repr> s exceeds the 8.0 s limit (requirements 4.4: Brazil-nut segregation)` |
| compaction_duration <= 0 | `diw_compaction_time` | error | `compaction_duration must be > 0` |
| compaction_duration < 1.0 | `diw_compaction_time` | warning | `compaction_duration = <repr> s is below the documented 1.0 s lower bound` |
| powder_hatch_spacing, powder_mass_flow_rate, powder_tap_density, powder_fill_multiplier, powder_f_max, powder_nozzle_dia <= 0 | `diw_powder_hatch_spacing`, `diw_powder_mass_flow`, `diw_powder_tap_density`, `diw_powder_fill_multiplier`, `diw_powder_speed_max`, `diw_powder_nozzle_diameter` | error | `<v2 key> must be > 0` |
| powder_lag_on, powder_lag_off, shell_powder_clearance, compaction_settle_dwell, powder_travel_hop_mm, compaction_z_hop_mm < 0 | `diw_powder_lag_on`, `diw_powder_lag_off`, `diw_shell_powder_clearance`, `diw_compaction_settle_time`, `diw_powder_travel_hop`, `diw_compaction_z_hop` | error | `<v2 key> must be >= 0` |
| powder_overflow_policy not in the 3 values | `diw_powder_overflow_policy` | (enum: unreachable) | — |
| compaction_interval < 1 | `diw_compaction_interval` | error | `compaction_interval must be >= 1` |
| voltage outside [3.65 - 1e-9, 12.0 + 1e-9] | `diw_powder_motor_voltage` | error | `powder_motor_voltage must be within 3.65-12.0 V` |
| \|voltage - 3.65\| > 1e-9 | `diw_powder_motor_voltage` | warning | `powder_motor_voltage = <repr> V but powder_mass_flow_rate is only valid at its calibration voltage; no discharge curve is available, re-calibrate m_dot` |
| M62 pins not distinct (powder, compaction, UV-on `P<n>`) | `diw_powder_pin` | error | `digital output pins must be distinct: {'powder_pin': <p>, 'compaction_pin': <c>[, 'uv_on_cmd pin': <u>]}` |
| shell_powder_clearance < d_powder/2 + 0.35 - 1e-9 | `diw_shell_powder_clearance` | warning | `shell_powder_clearance = <repr> mm is below d_powder/2 + c_safety,min = <.2f> mm even for a zero-dispersion jet` |
| powder_z_clearance - powder_offset_z < -1e-9 | `diw_powder_offset_z` | warning | v2's `powder_offset_z = ... BELOW the layer top ...` text |
| offsets all 0 (translator) | `diw_powder_offset_x` | warning | `POWDER NOZZLE OFFSETS powder_offset_x/y/z ARE ALL 0.0 (UNCALIBRATED): ...` |

The GUI min / max in `PrintConfig.cpp` already bound voltage (3.65-12) and compaction time (<= 8), so these errors
are reached from the CLI, 3MF files and presets; the validation makes them visible before slicing in every path.

### 6.3 Other rules

| Rule | Scope | Kind | Message |
|---|---|---|---|
| Retraction guard: `retraction_length` of a used extruder > `diw_retraction_max_length` with `diw_retraction_enable` on | DIW printer (paste and powder) | warning (Q2) | `Retractions of <L> mm are longer than the DIW limit of <M> mm ("Maximum syringe retraction"); the DIW translation may remove retractions longer than the limit, as v2 does.` The translator decides per line on \|dE\| (DIWTranslator.cpp ~699, v2 line 2192), and wipe-split retractions or `retract_restart_extra` give per-line values that differ from `retraction_length`, so the warning is advisory ("may") in both directions. |
| `diw_retraction_enable` off and `retraction_length > 0` | DIW printer | warning | `Syringe retraction is off on this DIW printer; the DIW translation removes retraction moves.` |
| `retraction_speed` > `diw_retraction_max_speed` | DIW printer | warning | `Retraction speed <v> mm/s is clamped to the DIW limit of <M> mm/s.` |
| `wall_loops = 0` on any region | powder job | error | `u-MEX powder infill needs at least one wall loop: the powder cavity is closed by the outer wall.` opt_key `wall_loops`. (Without it every core layer is P0 `st=open` and the export would stop with `open`.) |
| `sparse_infill_density = 0` on every region | powder job | error | `u-MEX powder infill needs sparse infill: the powder replaces the sparse infill of the core layers. Set a sparse infill density above 0 or turn u-MEX powder infill off.` opt_key `sparse_infill_density`. (`no_core` stays as the export-time safety net.) |
| `sparse_infill_density >= 100` on any region | powder job | error | `u-MEX powder infill needs sparse infill: at a sparse infill density of 100% the infill is solid and there is no cavity for the powder. Set a sparse infill density below 100% in this object's settings or turn u-MEX powder infill off.` opt_key `sparse_infill_density`. (Review fix F4: at 100% the sparse role disappears, so without this rule the export failed late with the `no_core` message.) |
| `infill_combination` on any region | powder job | error | `u-MEX powder jobs cannot combine infill: sparse infill printed every N layers would get the powder of one layer only. Turn "Infill combination" off.` opt_key `infill_combination`. (Both classifiers agree on the combined layers, so nothing else would flag the N-fold under-fill.) |
| spiral vase, BBL G-code, belt printer, XY axis remap, differing `extruder_offset` | powder job | error | `u-MEX powder infill cannot be used with <reason>.` (the binder's refusal reasons, PD §3, from `powder_refusal_reason`) |
| per-object scarf | powder job | error | 6.1 |
| per-object seam gap, wipe on loops (existing) | DIW printer | error | unchanged |
| soft limits | — | — | no keys: v2's banner warning stays in the translator (decided default) |

The three rules with no v2 counterpart (`wall_loops`, density, `infill_combination`) are product-mode rules: v2 does
not check them, but in product mode they would otherwise surface as export-time fatals or a silent under-fill. Stage 1
checks that no matrix / stress / T5 slice uses them (the test profiles have `wall_loops = 2`, `infill_combination =
0`); if one does, it is listed and moved to G6.

Warnings never change the G-code (T7). Errors only fire on DIW powder jobs (T7 / G6 for non-DIW configs with any
`diw_*` values).

---

## 7. Files and functions

| File | Change | Recompiles |
|---|---|---|
| `GCode/DIWUMexSequencer.hpp/.cpp` | `PathRec::seg_lines`; `std::vector<TraceItem> core_emission_trace(const LayerModel&, lines)`; `SuppressResult suppress_sparse(...)` (R1-R5, `label_in_sparse`); `UMexSequencer` takes `v2_parity`; strict `st=open` | sequencer, shadow, translator, GCode.cpp (through DIWPowderShadow.hpp), dev-utils |
| `GCode/DIWPowderShadow.hpp/.cpp` | `ShadowOptions::mode`, `suppress`, `mutate`, `skip_verify`, `from_env` (`ORCA_DIW_POWDER_MODE`, `ORCA_DIW_TEST_NO_SUPPRESS`, `..._SUPPRESS_MUTATE`, `..._SKIP_VERIFY`, faults `suppress` / `cancel`); `powder_refusal_reason`; product gates (3.3, incl. 3b), orphan check (3.4), suppression + V0-V3 + fallback, failure list, `failed()`, `failure_message()`, summary keys, pre-edit dump fields | GCode.cpp, dev-utils |
| `GCode/DIWPowderPlanner.hpp/.cpp` | expose S (union of rings) for gate 12 without changing the record | planner, shadow |
| `GCode/DIWTranslator.hpp/.cpp` | `DIWTranslateOptions::v2_parity`; new no-plan message; shared `extrusion_bbox`; `DIWValidation diw_validate(...)` with the explicit opt_key table | translator, PostProcessor.cpp, OrcaSlicer.cpp, Print.cpp |
| `PrintApply.cpp` | scarf forcing, `diw_test_keep_scarf()` (exported like `diw_test_keep_seam_gap`) | one file |
| `Print.cpp` | `Print::validate`: scarf per region, `wall_loops`, density, `infill_combination`, structural refusals, `diw_validate` (error + warnings); all skipped under `ORCA_DIW_TEST_SKIP_VALIDATE=1` | one file |
| `GCode.cpp` | H1: product refusal throw as the first statement of the existing `try`; `failed()` check after `diw_finish`; mode in the timing log line | one file |
| `src/dev-utils/diw_translate.cpp` | `--v2-parity`; `--plan-records` sets `mode = shadow` after `from_env()`; `--product [--no-suppress]`; `--validate key=value ...` (prints `diw_validate` result, for G6) | dev tool |
| `src/dev-utils` Catch2 `diw_planner_tests` (and `tests/libslic3r/test_diw_planner.cpp`) | T11-T13 cases (section 8) | test |
| `diw/tests/shadow_export.py` | `--powder-mode shadow` (env `ORCA_DIW_POWDER_MODE=shadow`) passed to every increment-4 test, **and `--v2-parity` on every translation it makes in that mode**; S0 assertion: native non-carrier `st=open` count = 0; scarf variants add `ORCA_DIW_TEST_KEEP_SCARF=1`; `--extra <list>` for the coverage set | — |
| `diw/tests/compare_sequencer.py`, `compare_records.py`, `compare_replay.py`, `regress_review.py` | `--v2-parity` on every powder-mode translation of v2-made records; updated no-plan message | — |
| `diw/tests/product_gate.py` (new) | G1-G5, G8, G10; export manifest with mode; frozen mask list; coverage counters; refuses v2 / replay on product exports | — |
| `diw/tests/product_validate.py` (new) | G6 (CLI negatives) | — |
| `diw/tests/coverage_slices.json` (new) | the coverage set (8, "Coverage set") with the parameters found in stage 1 | — |

`GCode.hpp`, `PrintConfig.hpp`, `PrintConfig.cpp`, the pipeline stages and the wipe tower are not touched.

---

## 8. Verification

All harnesses: `py -3.15 -E -s`, shapely 2.2.0 / GEOS 3.14.1, v2 unmodified, run from `build\src\Release`; a missing
exe, flag or input is a failure, never a skip; each prints `N checks, F failures` and exits non-zero on a failure.
Work and logs go under `<scratch>\product\` (`<scratch>` = the session scratchpad).

**References from increment 4** (made by the 21:25 build, which differs from HEAD 19d98828's 22:07 build only in
comments): `<scratch>\planner\fx_t2b\slices\<slice>\{on,kill,keep,dump}` (191 matrix slices),
`<scratch>\planner\fx_st\slices\...` (16 stress slices), `<scratch>\planner\fx_t5\t5` (T5). Stage 1 verifies they
still exist and that the HEAD build reproduces a sample of them; if they are gone, it regenerates them with the HEAD
build (`shadow_export.py --matrix --slice-only` and the T2 pass) **before** any source edit. Stage 1 also makes, with
the HEAD build and before any source edit, the pre5 references for the new T7 cases (scarf on) and the coverage set.

**What the increment-4 references do not cover.** The 191 `fx_t2b` `on` exports hold 3153 blocks, all `st=ok`, and
**0 `;DIW:FB` lines** (26 layers have ra > 0, but no sparse path crosses a revert region); `fx_st` has 666 `ok` and 88
`open` (policy-error carriers only); `fx_t5` has 336 `ok`. So neither FB paste replacing removed sparse infill nor the
unedited release of `st=small` layers is exercised by them.

**Coverage set** (`coverage_slices.json`, built in stage 1 with the HEAD build in shadow mode, joined to every gate
that lists "+ coverage"): at least **2 slices whose blocks contain `;DIW:FB` lines with sparse paths crossing the revert
region** (candidates, tried in this order until the counters are met: the notch / wedge meshes of
`planner_meshes.py` with a large δ; cube25 / ring with `diw_shell_powder_clearance` raised in 0.25 mm steps until
`ra > 0` cuts sparse infill) and at least **1 slice with an `st=small` block** (candidates: a narrow box, e.g. 4 x 25 x
5 mm with 2 walls; or cube25 with the clearance raised until the cavity vanishes on some layers, while other layers
stay `ok`). The parameters found are frozen in the JSON with the counts. The set runs through the increment-4 T2
checks (`shadow_export.py --extra`), logged as its own count, 0 failures (v2 agrees with the native records on them).

**Coverage counters.** G1, G3 and G9 print and assert, per run: `fb_lines > 0`, `small_blocks > 0`,
`edited_chunks_with_fb >= 2` (on >= 2 distinct slices), `small_chunks_unedited == small_blocks`. A counter at 0 is a
failure, not a pass.

| Gate | What | Pass criterion |
|---|---|---|
| **T0** | regression, after every stage | `fx_t0.ps1` copy (log folder changed): `compare_sequencer.py` 157 ok with and without `--stub-s4`, `compare_records.py` 486 ok, `compare_powder.py` and `compare_v2.py` 0 failures; all 0 failures, 0 skips, 5 corpus files. The counts must not change (harness edits only add `--v2-parity` and the new message). |
| **T7** | non-interference | With the new build vs the pre5 references: non-DIW printer exports (cube25, ring, two_objects with the non-DIW flat profile) byte-identical; **the same non-DIW slice with `seam_slope_type = all` (scarf on) byte-identical and still containing the scarf**; DIW paste-only exports (`diw_powder_infill = 0`), **with scarf off and with scarf on**, byte-identical and translated byte-identical (the scarf stays); DIW powder kill-switch exports byte-identical to `fx_t2b/.../kill` for the 13 `classic.base` slices. (`norm`: the generated-at line blanked.) |
| **S0** | shadow mode unchanged | `ORCA_DIW_POWDER_MODE=shadow` exports == the increment-4 `on` exports (all 191 + 16 stress + T5), byte for byte after `norm`; the full increment-4 T2 pass (`shadow_export.py --matrix --reuse --powder-mode shadow`, which passes `--v2-parity` to every translation) gives the same check counts as `fx_t2b.log` (25515, 0 failures), stress as `fx_st.log` (2621), T5 as `fx_t5.log` (1150); **native non-carrier `st=open` blocks in all shadow exports = 0** (asserted, so a future native P0 / P1c `st=open` shows up here and not as a translator refusal); `diw_translate --plan-records <keep>` (no `--product`) with `ORCA_DIW_POWDER_MODE` unset reproduces the increment-4 T2(d) result. The coverage set's shadow exports are made here too and kept for G1 / G3. |
| **G1** central | product ≡ shadow after translation | For every slice of the 191 matrix, the 16 stress variants, the 8 T5 slices and the coverage set: (a) the product export succeeds; (b) its `;DIW:` lines equal the shadow export's byte for byte; (c) `translate(product)` and `translate(shadow)` both exit 0, or both abort with the same message (policy=error), and are byte-identical after the **frozen mask** in `product_gate.py`: (i) delete lines matching exactly `^M73 P\d+ R\d+\s*$` anywhere (the form the profiles emit, checked in stage 1; their positions move with the time estimate, also inside layers); (ii) blank the values of these statistics comment lines, **only outside layer chunks** (before the first and after the last layer marker): `; estimated printing time`, `; total estimated time`, `; model printing time`, `; filament used [mm]`, `; filament used [cm3]`, `; filament used [g]`, `; total filament used [g]`, `; filament cost`, `; total filament cost`. A statistics line inside a layer chunk, an `M73` of another form, or any other difference is a failure. The list cannot grow without a design-doc change. (d) On the 13 `classic.base` slices, the 16 stress slices and the coverage set, the same with `disable_m73 = 1`: identical except (ii). Coverage counters asserted. |
| **G2** | authority alone changes nothing | `ORCA_DIW_TEST_NO_SUPPRESS=1` export == shadow export byte for byte (all G1 slices; the disagreement counter is 0 on all of them). |
| **G3** | the edit is exactly the specified one | Python re-derives, from the shadow export with v2's own `scan_gcode` / `split_layers` / `parse_core_layer`, the removal set R of every `st=ok` chunk and checks `strip_records(product)` against `strip_records(shadow)` with the G1 mask: non-`st=ok` chunks (`small` and carriers included) and everything outside chunks byte-identical; in each `st=ok` chunk the deleted lines are exactly R; every inserted line matches the restore grammar (`G1 X.. Y..( Z..)? F..`, `G1 F..`, `G92 E..`, `G1 F.. ;WIDTH:..`, and `G1 F.. ;TYPE:..` only as the chunk's last line) with the values the original text implies at that point; no `powder`-kind extruding move remains; the chunk's extruded bbox (translator rule) is unchanged. Fallback count = 0 (each fallback is logged with its reason; a nonzero count fails unless the reason is `leading_abs_e` or `label_in_sparse` on a slice that prints infill first, which the matrix does not contain). Runs on G1's slices, coverage counters asserted. |
| **G4** | disagreement classes | New slices (product and shadow): `text_only` via `{if layer_num == N}` custom G-code typing an extrusion outside the part as `;TYPE:Sparse infill` on an internal-solid-only layer (product: no block there, warning present, other blocks equal to the plain plate's); `native_only` (a) via custom G-code typing such an extrusion `;TYPE:Top surface` on every layer (product: blocks byte-equal to the plain plate's; shadow: `disagree_native_only` on every core layer); `native_only` (b) by a Catch2 stream (T12) and, if a slice can be found that drops every sparse path at emission (a thin core region with a large `minimum_sparse_infill_area` or tiny infill), by that slice (product: no block, `native_only_nosparse` warning); `tall_short` plate (cube25 + 10 mm cube; layers where the short cube has its top surface get no block in both modes, and v2 agrees); `final_core` (cube25, `top_shell_layers = 0`: last layer gets no block, the warning is present, v2 agrees). |
| **G5** | product refusals and safety nets | Each export fails with its exact message, **no output file and no `.tmp` file is left** in the output folder, and the log has no failure-list text for the cancel case: G91 in a core layer's custom G-code (`arc_g91`); custom extrusion inheriting `Sparse infill` on core layers (`orphan`); **an extruding `layer_change_gcode` without `;TYPE:` placed outside the part (`orphan` on every core layer)**; with `ORCA_DIW_TEST_SKIP_VALIDATE=1`: `sparse_infill_density = 0` (`no_core`), `wall_loops = 0` (`open`), and the H1 refusal for spiral vase, belt, XY axis remap and differing extruder offsets (each with its `u-MEX powder infill cannot be used: <reason>` text; a case whose non-DIW Orca validation blocks the export even with the skip is covered by Catch2 `powder_refusal_reason` only, and listed); `ORCA_DIW_SHADOW_FAULT=capture|plan|binder` (internal error lines); `ORCA_DIW_SHADOW_FAULT=cancel` (no file, no `.tmp`, cancel message, not the failure list). Successes: `ORCA_DIW_SHADOW_FAULT=suppress` in product gives a successful export whose chunks are all unedited (`fallback` = number of `st=ok` chunks) and whose translation equals the shadow export's (G1 rule). In shadow mode the faults degrade as in increment 4 T6. Translator: the kill-switch export is refused with the new no-plan message (`--allow-unconverted-powder` passes); a file with a v2-made `st=open` block (T5 seam-gap 15 % v2 records) is refused by default and matches v2 with `--v2-parity`; the coverage `st=small` export and an h <= 0 carrier file (from `fx_st`) translate with the strict default, with v2's warnings. |
| **G6** | validation | `product_validate.py`: for each error row of 6.2, a CLI slice with that value fails and the log contains exactly the message `diw_translate` gives for the same config (the two strings are compared), at the 1e-9 boundaries of the table; each warning row appears as a warning and the G-code equals the plain slice's; per-object scarf fails with 6.1's message; global scarf is forced off (the export equals the no-scarf export and the config block says `seam_slope_type = none`); with `ORCA_DIW_TEST_KEEP_SCARF=1` the scarf export is reached; the 3 retraction warnings appear; **`wall_loops = 0`, `sparse_infill_density = 0` (global and on every object), `infill_combination = 1` fail with 6.3's messages; density 0 on one object of two does not**; the 5 structural errors (spiral vase, belt, axis remap, extruder offsets; BBL by a BBL-flavoured profile if the datadir has one, else Catch2 only); the same invalid values on a non-DIW printer slice fine. `diw_translate --validate` agrees with the CLI on every row. |
| **G7** | T10 performance | product export time <= 1.2x the kill-switch time on cube25 and cone (best of 5), `max_held = 1`. |
| **G8** | processor consistency | Per product export: the processor's `filament used [mm]` equals the shadow export's minus the summed `removed_de` of the summary/dump, within one unit of the printed last digit; the estimated time is not larger. |
| **G9** | offline product replay (stage 2b, before the GCode.cpp hook) | For every `fx_t2b` / `fx_st` slice **and the coverage set**: `diw_translate --plan-records --product <keep> <out>` with the slice's dump; blocks byte-equal to the `keep` file's; `translate(out) == translate(diw_translate --plan-records <keep>)` **byte for byte** (the reference is explicitly shadow mode, 2; no processor runs offline, so there is nothing to mask); G3's structural check on `out`; fallback count 0; coverage counters asserted. Inputs are shadow-mode `keep` + dump files only. |
| **G10** | gate-level mutation (stage 4a) | On 3 matrix slices + 1 coverage FB slice, for each `ORCA_DIW_TEST_SUPPRESS_MUTATE` value: (a) with verification on, the export succeeds, every `st=ok` chunk falls back with a V1 / V2 reason, and G1 passes; (b) with `ORCA_DIW_TEST_SKIP_VERIFY=1`, G3 **fails** on every mutated slice and G1 fails on every slice where the mutation changes the translated output (`drop_restore` and `wrong_g92` must change it; the run asserts which). A mutation that no gate notices is a failure of G10. |
| **G11** | shipped in-slicer translation path (fix pass) | `product_gate.py g11`: for every set (matrix, stress, T5, coverage) at least one G1 slice, plus the G5 refusals, is exported **without** `ORCA_DIW_EXPORT_UNTRANSLATED`, so `diw_translate_gcode_file` in `OrcaSlicer.cpp` runs; the file equals `translate(product export)` after `norm` and the G1 mask, and each refusal leaves no output and no `.tmp` file. |
| **Catch2** | units | T11 `suppress_sparse`: walls then sparse last (absolute E: `G92` + end-of-chunk type restore), relative E (no `G92`), sparse between two wall paths with no travel after it (R1 with the previous `trav_f`), `M204` / `M106` in the sparse `pre_lines` (removed), **object-label lines (`; start printing object`, `M486 S1`, `EXCLUDE_OBJECT_START`) in the sparse `pre_lines` (fallback `label_in_sparse`)**, a `;WIDTH:` inside the preceding wall path and a width change carried only by a removed `pre_lines` (R5 restores it; V2 width equal), **the next chunk's first extruding move without `;TYPE:` (stays `powder` kind in chunk k+1, identical LineInfos with and without the edit)**, a sparse vertex on the chunk bbox edge (the edit keeps the bbox or falls back), infill first with absolute E (fallback `leading_abs_e`), no `trav_f` (fallback), and mutation tests (one extra deleted line, one missing restore line, a wrong `G92` value: each must fail V1 or V2). T12 product binder streams: text_only, native_only (a), native_only (b) (`native_only_nosparse`, no block, warning), final, arc in a core chunk, `st=open`, merged, lost, sync, no block at all, orphan (incl. an untyped extrusion after sparse at a layer change), binder fault, `suppress` fault (fallback, not failure), `cancel` fault (exception propagates, `failed()` false), each `powder_refusal_reason` reason; each gives the expected block / warning / failure; the T8 streams pass unchanged with the default (shadow) options. T13: `diw_validate` per row of 6.2 and 6.3 (message equality with `read_umex_params`, opt_key from the table, 1e-9 boundaries), translator `v2_parity` on and off, **strict default accepts `st=small` (warning only) and the h <= 0 carrier**. |

---

## 9. Stages

Each stage ends with T0 green and a progress-log entry; a stage never starts before the previous one's gates pass.

| Stage | Work | Gates |
|---|---|---|
| **1. Baseline, translator strictness, validation, scarf** | Before any source edit, with the HEAD build: verify the increment-4 references (sample reproduce or regenerate), make the pre5 T7 references (scarf-on non-DIW and paste-only cases), find and freeze the coverage set (`coverage_slices.json`, shadow exports + `keep` + dump), check the `M73` form and that no matrix / stress / T5 slice uses `wall_loops = 0`, density 0 or `infill_combination`. Then: `DIWTranslateOptions::v2_parity`, strict `st=open`, new no-plan message, shared `extrusion_bbox`, `diw_validate` + opt_key table, `Print::validate` rules (6.1-6.3) with `ORCA_DIW_TEST_SKIP_VALIDATE`, scarf forcing; `diw_translate --v2-parity` / `--validate`; harness edits (`--v2-parity`, message). One full build (`--no-configure`). | T0; T7; T13; G6 (all rows except the structural ones that need the stage-3 hook); coverage set counters met (>= 2 FB slices, >= 1 small slice) and its T2 checks 0 failures. |
| **2a. Suppression engine** | `PathRec::seg_lines`, `core_emission_trace`, `suppress_sparse` (R1-R5, `label_in_sparse`, `leading_abs_e`), V0-V3 helpers (incl. width and bbox), mutation hook; Catch2 T11. Dev-tools builds only (`diw_translate`, `diw_planner_tests`). | T0; T11 all cases incl. mutations. |
| **2b. Product binder, offline** | `ShadowOptions` mode / suppress / test fields and `from_env`; product gates (3.3 incl. 3b, 10b), orphan (3.4) with S from the planner, failure list, `powder_refusal_reason`, summary keys, pre-edit dump fields, faults `suppress` / `cancel`; `diw_translate --plan-records` shadow by default, `--product [--no-suppress]`; Catch2 T12. Dev-tools builds only. | T0; T8 unchanged; T12; G9 on 191 + 16 + coverage with counters; offline S0 check (`--plan-records` without `--product` reproduces increment-4 T2(d)). |
| **3. GCode.cpp hook and product exports** | H1 (refusal throw as the first statement of the existing `try`, `failed()` check after `diw_finish`, mode in the timing log); `shadow_export.py --powder-mode shadow` (+ `--v2-parity` on its translations, `--extra`, S0 assertion); `product_gate.py` skeleton with manifest and frozen mask. One full build. | T0; T7 again; G5 complete (incl. `.tmp`, cancel, `suppress` fault, skip-validate safety nets, strict translator accepting small / h <= 0); G6 structural rows. |
| **4a. Central gate** | S0 shadow re-run (all 191 + 16 + T5 + coverage, exports kept); product exports of the same slices; G1 (a)-(c) and G3 against the S0 shadow exports (reused, not re-sliced); G10 mutation runs. | S0 (counts equal to fx_t2b / fx_st / fx_t5, native non-carrier `st=open` = 0); G1 (a)-(c); G3; coverage counters; G10. |
| **4b. Variants and consistency** | `ORCA_DIW_TEST_NO_SUPPRESS=1` exports (G2), `disable_m73 = 1` shadow + product exports of the 13 `classic.base` + 16 stress + coverage slices (G1 (d)), timing runs (G7), processor statistics (G8). | G2; G1 (d); G7; G8. |
| **5. Disagreement classes, docs** | G4 slices (text_only, native_only (a) / (b), tall_short, final_core); re-check that no shipped DIW printer profile's custom G-code extrudes without `;TYPE:` (or prefixes `;TYPE:Custom`); docs: POWDER_PLAN.md "Increment 5 results" and open questions, FEATURE_MAP.md rows (suppression, strictness, scarf, validation, "product exports are not v2 inputs"), PLANNER_DESIGN.md §13 closed items, this file's Deviations. Final T0, S0 sample, G1 sample. | G4; T0; docs reviewed against measured numbers in the logs. |

---

## 10. Questions for the user

Defaults below are implemented unless the user decides otherwise; each keeps v2 parity.
- **Q1. Multi-object plates.** A layer where one object has a top or bottom surface and another has only sparse
  infill: per printed layer (default, v2: nobody gets powder on that layer) or per object (the sparse object gets
  powder, the other object's sparse infill becomes FB paste)? Per object needs no record change but has no v2 oracle.
- **Q2. Retraction guard.** Retractions longer than the DIW limit: validation warning and the translator drops them
  (default, v2), or a validation error?
- **Q3. Last layer with sparse infill and no top surface.** Printed as paste with a warning (default, v2 never fills
  the last layer), or an error?
- **Q4. `infill_combination` on a powder job.** Validation error (default here), or force it off silently like the
  scarf?

## 11. Risks

- A future Orca change that emits non-motion lines with modal meaning inside sparse infill: V1/V2 turn it into a
  fallback (no machine risk), and G3's fallback count shows it.
- Placement of M73 by `GCodeProcessor::finalize` between a start marker and a block: unchanged risk from
  increment 4 (T2(b) / G1(c) guard it).
- The orphan check could fire on a legitimate part whose ring is missing (an island bounded by an open outer wall
  that P1b cannot close). Then the export fails instead of silently missing powder; the matrix must show 0.
- Strictness in the GUI: a fatal reason surfaces at slicing time as an export error. Messages name the layers and the
  likely cause; the internal ones ask for a report.
- Users or later tools that feed a product export to v2 or a replay get a different (paste-only) plan (4.5). The
  harnesses refuse it; FEATURE_MAP.md documents it.
- The coverage set is synthetic (raised clearance, narrow boxes). It proves the FB and `small` paths through the
  product machinery; it does not prove that real parts reach them.

---

## 12. Review response (revision 2)

An adversarial review of revision 1 (checked against HEAD 19d98828, v2 and the increment-4 exports) raised 16 points.
Each was checked in the source before acting on it.

| # | Point | Handling |
|---|---|---|
| 1 | Gates never test FB or `st=small` | Accepted. Coverage set and coverage counters (8); G1 / G3 / G9 run on it and fail on a 0 counter. |
| 2 | `from_env()` defaulting to product changes `--plan-records` | Accepted (confirmed: diw_translate.cpp ~348 uses `from_env()`). `--plan-records` is shadow unless `--product` (2, 7, G9, S0). |
| 3 | H1 refusal throw outside the cleanup `try` | Accepted (confirmed: `create()` at ~2957, `try` at ~2964). Throw is the first statement inside the `try`; empty-reason `DIWConfigError` case left to validation and the translator; G5 checks no `.tmp` (5.1). |
| 4 | `native_only` (b) puts powder where v2 never would | Accepted. Gate 3b: no block, warning, `native_only_nosparse` (3.2, 3.3). |
| 5 | Missing validation rules | Accepted: `wall_loops = 0`, density 0 on every region, `infill_combination` as errors (6.3, G6, T13). For density, the rule is "every region": one object at 0 among others is a valid plate (that object is paste-only, as in v2). `infill_combination`: error rather than forcing (Q4). |
| 6 | H1 safety net untested | Accepted. `ORCA_DIW_TEST_SKIP_VALIDATE` + G5, and `powder_refusal_reason` in Catch2 T12. |
| 7 | V1/V2 miss the UV bbox and width | Accepted (confirmed `compute_geometric_center` rule and that the scanner reads `;WIDTH:` on command lines, one tag per line). V2 gains bbox (shared routine) and width; R5 added (4.3, 4.4). |
| 8 | Product exports not self-describing for v2 | Accepted. Stated in 4.5 and 8; manifest-based refusal in `product_gate.py`; pre-edit dump with `chunk_sha_pre`. No marker line is added to the export (it would change the translated bytes and break G1's equality). |
| 9 | Open-ended G1 mask | **Partly rejected.** The list is frozen and statistics lines are masked only outside layer chunks, as proposed. But "any `M73` inside a converted chunk's translated output fails" is wrong for these profiles: GCodeProcessor inserts `M73 P<n> R<n>` inside layers (confirmed in the `fx_t2b` exports), the translator emits them as header / `pre_lines`, and their positions legitimately move with the time estimate. So `M73` lines of exactly that grammar are deleted anywhere; any other `M73` form fails; and G1 (d) with `disable_m73 = 1` gives the unmasked proof on 13 + 16 + coverage slices. |
| 10 | Strict translator vs S0 / increment-4 harnesses | Accepted. `--powder-mode shadow` passes `--v2-parity` to its translations; S0 asserts native non-carrier `st=open` = 0. |
| 11 | Raw-file side effects of removing `pre_lines` | Accepted. Object labels: fallback `label_in_sparse`; other commands removed and documented (4.3, 4.5); T11 cases. |
| 12 | R4 overstated | Accepted (confirmed GCodeProcessor ~4216-4236). R4 is the end-of-chunk scanner `ftype` restore only; T11 case for the next chunk's untyped first move; G3 grammar allows R4 only as the last line. |
| 13 | Untyped extrusion after sparse missing from the table | Accepted. New row in 3.2; G5 case; profile check (test profiles do not extrude; no DIW printer profile is shipped in `resources/profiles`; stage 5 re-checks). |
| 14 | Stages 2 and 4 too big | Accepted. 2a / 2b and 4a / 4b; full list with gates in 9. |
| 15 | Missing negative tests | Accepted: (a) T7 scarf-on non-DIW and paste-only; (b) T13 + G5; (c) G5 `suppress`; (d) fault `cancel` in the binder (no GCode.cpp test code); (e) G10 with `..._SUPPRESS_MUTATE` + `..._SKIP_VERIFY`. |
| 16 | Small inaccuracies | Accepted: 1e-9 tolerances in the 6.2 table; explicit opt_key table; retraction warning worded "may", with the per-line \|dE\| reason. |

---

## Deviations

Choices that differ from the wording of POWDER_PLAN.md / FEATURE_MAP.md / the increment-5 task, made at design time:
- **Strictness lives in the binder, not only in the translator.** "Translator requires valid records" and "a native
  core layer without a valid block fails the export" cannot be checked by the translator alone: record v1 is frozen
  and carries no native classification, and the text classification is no longer the authority. The export (binder)
  fails instead; the translator adds the checks it can make (no plan, non-carrier `st=open`).
- **Suppression is a text edit in the binder** (G-code emission after the pipeline), not a fill-generation or
  `process_layer` change (4.1). Fill-level suppression is incompatible with `st=small` fallbacks, FB clipping and an
  exact oracle comparison.
- **The retraction guard is a warning, not a clamp or a rejection** (FEATURE_MAP B row says "clamp or reject in
  validation"): the translator already blocks such retractions exactly as v2 does, so a warning keeps v2 parity (Q2).
- **Validation messages are v2's verbatim** (v2 key names such as `compaction_duration`), with the fork key as
  opt_key for the GUI, so the slicer and the translator give identical texts.
- **Three validation rules have no v2 counterpart** (`wall_loops = 0`, no sparse infill, `infill_combination`). v2
  accepts such configs and silently prints paste (or under-fills); the product path would fail at export or
  under-fill, so they are rejected before slicing.
- **`native_only` (b) layers are not filled and not fatal**, although they are native core layers without a block:
  there is no v2 oracle for them and no sparse path to replace. They are warned and counted.
- **Product exports are not valid v2 / replay inputs** (4.5). Only shadow-mode exports and `keep` + dump files feed
  the oracles.

Made during implementation, stage 1 (19 Nov 2026):
- **`powder_refusal_reason` takes a `bbl_printer` flag and lands in stage 1.** `Print::is_BBL_printer()` is not in the
  config, so the pure function is `powder_refusal_reason(full_config, bbl_printer, spiral, axis_remap)`; it is needed
  by the stage-1 validation rules, so it was factored out of `PowderShadow::create` now instead of in stage 2b
  (`create` still checks the kill switch first and gives the same reasons). The extruder-offset reason now reads
  `extruders that have different extruder_offset values` (was `the extruders have ...`), so that the validation
  message `u-MEX powder infill cannot be used with <reason>.` is a sentence; only the `DIW shadow planner off:` log
  line changes. In this fork the G-code axis remap is on exactly for belt printers
  (`BeltTransformPipeline::axis_remap_enabled`), so a belt printer is refused as `G-code axis remap`.
- **The coverage set's FB slices are all `slot` variants, and four FB candidates were rejected.** No notch, wedge,
  L-shape, cube25, ring or cone slice reached `;DIW:FB` lines at any clearance tried (1.0-4.0 mm); only the slot does.
  Slot at clearance 1.5 mm (hatch angle 0 and 45) fails the increment-4 T2(c) `areas` / `fb` checks on every core
  layer (native `ra` 9.42805 mm2 vs v2 9.48028, FB points up to 0.0286 mm) and T2(e); slot at 1.3 / 1.35 / 1.4 mm
  fails T2(e) only (the B word of the layer-12 S6 section is 1e-4 apart: the cumulative B after an FB paste path whose
  length differs within the T2(c) tolerance). These are increment-4 planner / harness findings, recorded in
  `coverage_slices.json` (`rejected`) and left for a later increment; the frozen set holds the slices that pass every
  T2 check (slot 1.25 mm at 0 and 90 degrees, slot 1.45 mm; cone 3.0 mm and ring 1.35 mm for `st=small`).
- **`shadow_export.py --extra` and the stress scarf variants' `ORCA_DIW_TEST_KEEP_SCARF=1` land in stage 1** (planned
  for stage 3): the coverage gate runs through `--extra`, and the scarf forcing that changes the stress scarf variants
  is implemented here. The T2(e) translation of v2-made records (`v2rec`) passes `--v2-parity` from stage 1 on.
- **`ORCA_DIW_TEST_SKIP_VALIDATE=1` also skips the three retraction warnings** (they are 6.3 rules); the existing
  increment 1-4 DIW rules (seam gap, wipe on loops) stay.
- **"Non-DIW flat profile"** (T7) is the flat test profile pair with `diw_printer = 0` (no separate file);
  `diw_powder_infill` stays 1 there, so T7 also shows that powder settings without a DIW printer change nothing.
- **T13's `Print::validate` cases link `Print` / `Model` into the stand-alone `diw_planner_tests`**, which then needs
  nanosvg: `test_diw_shadow.cpp` defines `NANOSVG_IMPLEMENTATION` when `CATCH_CONFIG_FAST_COMPILE` is set (only that
  target sets it; `libslic3r_tests.cpp` provides it in `libslic3r_tests`), avoiding a CMake change.

Made during implementation, stage 2a (19 Nov 2026):
- **A removed group without a move defers its restore.** A powder path's `pre_lines` can hold a `;WIDTH:` that sits
  inside the previous wall path (4.2). Restoring the width right after it (4.3 step 3) would put a `G1 F.. ;WIDTH:..`
  motion line inside that wall path, which `parse_core_layer` would then split in two (V1 fails, the chunk falls
  back). So a group of removed comment / command lines only marks the restore as pending, and it is written before
  the next kept line where a motion line cannot join two paths: a non-extruding motion line, or an extruding move that
  starts a path in the original; a later removed group that holds a move writes it at its own end, and the end of
  the chunk always writes the full group (R4 included). Such a group can only change the width (and the type, which
  only R4 restores), so nothing else moves. G3's re-derivation (stage 4a) must place R5 the same way.
- **R1 is written whenever the position differs after a removed group**, also when a kept travel follows that would
  re-set it (literal 4.3; the travel's start point and `has_xy` then match the original). G3 expects it there.
- **Restore tokens**: X / Y / Z / E / F / width / type text is copied from the original chunk line that set the value;
  a value set before the chunk is written in the shortest `py_fixed` form that parses back bit for bit (fallback
  `token` if none). Extra fallbacks besides 4.3's: `relxyz` (G91 in the chunk), `marker` (line 0 in R), `no_newline`
  (a restore after a line without a newline), `token`, `state` (M82/M83 or width state that cannot be restored).
- **V0 / V2 are slightly stricter**: V0 also re-splits the edited text and requires one chunk opened at line 0; V2
  compares, besides the chunk's extruded box, the end state of the bbox scan (`prev_e`, `tx`, `ty`, `rel`), since the
  next chunks' boxes depend on it.
- **Mutation hook semantics** (`SuppressMutate`, a parameter of `suppress_sparse` / `suppress_and_verify`; the env
  wiring is stage 2b): `drop_restore` writes no restore line in the chunk; `extra_delete` also deletes the chunk's last
  kept extruding move; `wrong_g92` writes the first R3 as `G92 E<e + 1>`, or, when the chunk needs no R3 (relative E),
  adds `G92 E<e + 1>` after the last line. T11 checks each fails V1 or V2 on three chunks (absolute E, relative E,
  sparse between walls). In absolute-E chunks where a retract / unretract follows the removed path, a missing R1 / R3
  can be invisible to V1 / V2 (and to the translator: those lines are dropped in a converted layer); G10's assertion
  of which mutations change the translated output has to be measured, not assumed.
- **T11 lives in `tests/libslic3r/test_diw_shadow.cpp`** (tag `[diw_suppress]`), so `diw_planner_tests` needs no CMake
  change.

Made during implementation, stage 2b (19 Nov 2026):
- **`plan_powder_layer_native` gains an output `ExPolygons *s_out`** (default nullptr; S in the G-code frame, nm grid,
  empty without a ring) instead of a `PlanDebug` field: passing a `PlanDebug` switches the planner to its debug path
  (narrow is then always computed), so the orphan check uses a separate output that changes nothing else.
  `DIWPowderShadow.hpp` now includes `DIWTranslator.hpp` (the binder holds `ExtrusionBBox` states for V2).
- **Orphan check (gate 12) geometry**: each powder-kind vertex and segment midpoint (mm) is rounded to the nm grid
  (`to_grid`) and accepted when it lies in an S polygon (boundary included) or within 10 000 nm of an S contour or
  hole edge. The same predicate is V3's `in_s`. The first offending point is written `X<py_repr> Y<py_repr>`.
- **`native_only` is counted after gate 3b**: a native core chunk whose text has a skin type and no powder-kind path
  counts only as `native_only_nosparse`.
- **The dump is written once per planned block, after the suppression outcome is known**, so that it can carry
  `suppressed`, `fallback` and `chunk_sha_pre`; everything else in it (native capture, planner input, record) is
  the pre-edit data, as 2 requires. It also gains `mode`, `orphan` (first offending point or null) and, for an edited
  chunk, `removed` (moves, other lines, restore lines, dE). A product-mode chunk that fails the orphan check still
  gets its dump (debugging aid) but no block. Both modes write the new fields (shadow: `suppressed: false`,
  `fallback: null`); the G-code of a shadow export is unchanged.
- **`chunk_sha_pre` is FNV-1a 64** (`"fnv1a64:<16 hex>"`) over the held chunk's bytes, placeholder lines included:
  libslic3r has no SHA routine, and the boost MD5 used by AppConfig hashes into machine-endian words.
- **Summary line (5.2)**: product `DIW powder planner (product): <n> block(s), <c> carrier(s), <reason>=<n>...[,
  disabled][, no_core], suppressed=<n> fallback=<n> removed_moves=<n>; max_held=... capture_ms=... binder_ms=...
  single_bead=... orphan=<n> native_only=<n> native_only_nosparse=<n> removed_de=<py_repr>` (`orphan` and
  `native_only_nosparse` appear only after the `;`; `removed_de` is extra, for G8). Shadow: the increment-4 line with
  ` orphan=<n>` appended after `single_bead`. Fallback reasons are not in the summary: `PowderShadow::fallback_log()`
  gives one line per reason (k list, first detail); `diw_translate --plan-records` prints them as `info:` lines.
- **Failure lines without a layer number**: `lost_placeholder` (`internal error (lost_placeholder: <n> captured
  layer(s) never reached the G-code output). Please report this with the 3MF.`) and `binder` (`internal error
  (binder: <first what>). ...`) have no `layer(s) <k>:` prefix (the binder knows no k for them). An internal reason
  without a recorded message is written `internal error (<reason>)`. `no_core` is set at `finish()` only when no
  other failure was recorded (a failed export already explains why it has no block).
- **Fault `cancel`** fires once, at the first chunk that passes gates 0-5 (before planning), in both modes.
- **`ShadowOptions::from_env()` throws `std::invalid_argument`** for an unknown `ORCA_DIW_POWDER_MODE` or
  `ORCA_DIW_TEST_SUPPRESS_MUTATE` value; `PowderShadow::create` turns that into its `internal error: ...` refusal.
- **`diw_translate --plan-records --product`** writes its output even when the binder failed, then prints
  `error: <failure_message()>` and exits 3.
- **G9 (product_gate.py g9)** also runs the reference replay with `ORCA_DIW_POWDER_MODE=product` in the environment
  (it must stay shadow and equal the keep file minus its placeholders), runs S0 (placeholders kept, mode unset, ==
  keep) on every slice, checks the product dumps' new fields, and self-tests G3: one coverage FB slice replayed with
  each `ORCA_DIW_TEST_SUPPRESS_MUTATE` value and `ORCA_DIW_TEST_SKIP_VERIFY=1` must fail G3. G3 compares the scanner
  state after each inserted restore group with the original's state before the next kept line (x, y, z, E, M82/M83,
  G90/G91, feed, width; the type only for the group that ends the chunk), which accepts the deferred restores of
  stage 2a; it also requires equal end-of-chunk scanner states and the per-chunk bbox and bbox-scan state.
  `oracle_input()` of the manifest refuses product files; G3's own use of v2's `scan_gcode` / `split_layers` on a
  product file is structural parsing, not an oracle use.

Made during implementation, stage 3 (19 Nov 2026):
- **H1 reads the mode with `ShadowOptions::from_env()` and treats an invalid `ORCA_DIW_POWDER_MODE` as product**
  (`create()` already refuses it as `internal error: ...`), so a misspelled mode stops a powder export instead of
  silently giving the increment-4 behaviour. The kill switch keeps the increment-4 path (export succeeds, the
  translator refuses it with the no-plan message).
- **GCode.cpp also logs `fallback_log()`** (one info line per suppression-fallback reason, after the summary line),
  which 5.1 asks for ("suppression fallbacks only in the log") but H1's list did not name. The fallback detail
  carries the first layer: `(suppression fallback exception: k=<k>: fault injection: suppress)`.
- **G5 orphan cases**: "custom extrusion inheriting `Sparse infill`" and "an extruding `layer_change_gcode` without
  `;TYPE:`" are the same mechanism in Orca (the layer change G-code is the only custom G-code inside a core chunk on
  the test profiles), so G5 runs it untyped (orphan on the core layers that follow a layer ending in sparse infill:
  k 4-34 of cube25, not k 3, which follows an internal-solid layer) and typed `;TYPE:Sparse infill` (orphan on every
  core layer). Both use relative E (`G92 E0` in the layer change G-code) so the custom `E` value is harmless.
- **Belt and XY axis remap are one G5 case**: the fork's axis remap is on exactly for belt printers, so a belt
  printer is refused as `G-code axis remap`; the `belt printer` reason and a non-belt XY remap cannot be reached
  from the CLI (`gcode_remap_x/y` alone is not an axis remap here). Those reasons, and the BBL one (no BBL-flavoured
  DIW profile in the datadir), are covered by Catch2 `powder_refusal_reason` only.
- **The h <= 0 carrier file is made, not taken from `fx_st`**: no `fx_st` export holds one (its 44 `st=open` blocks
  are all policy=error carriers). G5 sets one core layer's `;HEIGHT:` tag of the kill-switch export to `-0.7` and
  lets the binder (`diw_translate --plan-records`, shadow) write the carrier (`st=open h=-0.7 s=0 F=0 fu=0`); the
  strict translation accepts it with v2's `non-positive layer height` warning. The `st=small` check uses a fresh
  product export of the coverage cone slice (clearance 3.0; `st=small` at k 12-14, v2's `revert_all` warning).
- **G5's translated-output comparison (fault `suppress`) applies `norm` (the generated-at line) before the frozen
  G1 mask**; the two exports are minutes apart, and `norm` is the T7 normalization. It also passes byte for byte
  after `norm` alone (the export is unedited). Fault `cancel` is checked in shadow mode too (it throws in both).
- **`shadow_export.py --powder-mode`** travels to the worker processes through `DIW_HARNESS_POWDER_MODE` (the
  harness strips `ORCA_DIW_*` from inherited environments); `--plan-records` replays are left without
  `--v2-parity` (they do not translate).

Made during implementation, stage 4a (19 Nov 2026):
- **The frozen G1 mask also matches the translator's spelling of the same lines.** The translator comments every
  `M73` out (`; (FILTERED) M73 P<n> R<n>`, prohibited commands in `Translator::process_line`) and makes every comment
  bracket-free for PlanetCNC (`[` `]` -> `(` `)`), so a translated file holds no bare `M73 P<n> R<n>` and its
  statistics lines read `; filament used (mm)`, `(cm3)`, `(g)`, `; total filament used (g)`. Mask (i) deletes
  `^(; \(FILTERED\) )?M73 P\d+ R\d+\s*$`; mask (ii) blanks the 9 keys in both spellings, still only outside layer
  chunks. No new kind of line is masked; an `M73` of another form (filtered or not) and a statistics line inside a
  layer chunk still fail (mask self-test, 6 checks). G1 (c) also applies `norm` (the generated-at line) first, as G5.
- **G3 runs on `norm` + the G1 mask of both raw texts** (the design says "with the G1 mask"): the processor places
  `M73` lines by the smaller time estimate of the product file, also inside the edited chunks.
- **G1 / G3 product exports reuse each S0 slice's own profile files, meshes (the S0 work's mesh directories),
  CLI arguments and test environment** (`product_gate.py g1`, `s0_jobs`), with `ORCA_DIW_POWDER_MODE=product`; the
  shadow side is the S0 `on` export (T5: the `gap0` and `gap15` exports, so "8 T5 slices" are 16 exports). The
  shadow translation passes `--v2-parity` (every translation of a shadow export does, 8 S0); the product translation
  is strict. S0 itself is `shadow_export.py --matrix --slice-only --powder-mode shadow` (the `on` exports) followed by
  the T2 pass with `--reuse` of it, as in increment 4 (`fx_5a` then `fx_t2b`); each S0 harness run has one more check
  than its increment-4 log (the `S0` native non-carrier `st=open` assertion). `keep` exports are compared without
  their `;DIWP` lines, whose token is random per export.
- **`wrong_g92` now mutates the end-of-chunk R3** (or adds `G92 E<e + 1>` after the last line when that group writes
  no R3), instead of the chunk's first R3. G10 showed that a wrong first R3 can never reach the output: on Orca's
  layout (walls, then sparse infill, then `; stop printing object`) the removed sparse path gets one restore group
  before the label and one at the chunk end, and the end group restores against the edited text's own state, so it
  writes a correct `G92` over the wrong one. V1 / V2 then (rightly) accepted the edit, the translation was unchanged,
  and only G3 saw it, so G10's "wrong_g92 must change the translated output" could not hold. With the new semantics
  the chunk always ends with a wrong E: V2 (`v2_state`) catches it on every chunk and the translation changes on every
  G10 slice. Test-only path; T11 / T12 unchanged and green. `DIWUMexSequencer.cpp` / `.hpp`, full build 07:48.
- **G10 `extra_delete` also changes the translated output** on all four G10 slices (G1 fails), so all three
  mutations are caught by both G1 and G3 there; only `drop_restore` and `wrong_g92` are asserted to, as specified.

### Made during implementation, stage 4b

- **G2 compares after `norm`** (the generated-at line is blanked), as S0 and G5 do: two exports never share that line.
  No other mask. The "disagreement counter" of a product export is read as the product summary's `text_only`,
  `native_only` and `native_only_nosparse` (product mode has no `disagree_*` reasons; 3.2); all three are 0 on the 228
  slices, and the summary also says `suppressed=0 removed_moves=0 removed_de=0.0`.
- **G1 (d) `disable_m73 = 1` is set by editing copies of each S0 slice's two flattened profile files** (both hold the
  key; the edit is the single `"disable_m73": "0"` -> `"1"`), and the export's config block is checked to say
  `; disable_m73 = 1` with no `M73` line at all (either spelling). The shadow side is a new shadow-mode export with the
  same profiles (not the S0 file, which has `M73`). Translation as G1 (c) (shadow `--v2-parity`, product strict), then
  `norm` + mask (ii) only. G1 (d) additionally runs G3's structural check and G8 on its pairs.
- **G8 is run on every product export of this stage and of stage 4a**: the 228 G1 exports (`product_gate.py g8
  --g1-work`), the 228 G2 exports (`removed_de = 0`, so equality) and the 34 G1 (d) exports. The removed `de` is the
  summary's `removed_de`, applied to the first extruder's value of `; filament used [mm]` (the files hold one value).
  "One unit of the printed last digit" is 0.01 mm (two decimals); the largest difference seen is 0.0081 mm. "Estimated
  time" is every `; estimated printing time (<mode>)` / `; total estimated time` / `; model printing time` line in the
  file (only the normal-mode line is present), parsed from `d h m s`.
- **G7 uses T10's procedure** (`shadow_export.py` T10: the CLI export of the default cube25 / cone meshes with the
  flat test profiles, best of 5 alternating runs of the CLI wall time), with `ORCA_DIW_POWDER_MODE=product` against the
  kill switch `ORCA_DIW_NO_NATIVE_PLAN=1`; the logged `export_ms` of the G-code export step alone is reported beside
  it (about 1.7-1.9x, a ~30 ms step), as T10 did.

### Made during implementation, stage 5 (19 Nov 2026)

- **G4 `tall_short` uses `precise_z_height = 0`.** The test profile has `precise_z_height = 1`, which gives the 10 mm
  cube its own z values near its top (7.04, 7.78, ... 10.0 mm), so its top surface never shares a printed layer
  with the 25 mm cube's sparse infill. With 0, both cubes are sliced on the 0.7 mm grid and k 14 holds the short
  cube's top surface and the tall cube's sparse infill (no block in either mode; k 13, internal solid + sparse, has
  one). The profile's own setting also runs, as `tall_short_precise` (35 blocks, none on a skin layer). The 10 mm cube is
  `cube10.stl` of `planner_meshes.py`.
- **G4 "v2 agrees" is checked at two levels; the blocks are not compared byte for byte.** The native records are on
  the nm grid and v2's come from shapely (`61.151002` vs `61.151`), as in every increment-4 comparison.
  `product_gate.py g4` requires the same v2 text-core k set, the same planned (k, st), and per block the same
  `;DIW:BEGIN` line and the same `;DIW:RUN` / `;DIW:FB` counts. The geometry is checked by the increment-4 T2 checks:
  `shadow_export.py --matrix --extra g4_slices.json --only g4_ --powder-mode shadow`, 754 checks for the three plates
  without custom G-code (tall_short, tall_short_precise, final_core). `text_only` and `native_only` (a) disagree with
  v2 on purpose, so they cannot pass T2's clean-summary check. Their v2-side check is the text classification only.
- **The custom extrusions of G4 run with relative E and move to the layer's z first.** `Print::validate` wants
  `G92 E0` in the layer change G-code with relative E, on a line of its own (a following `{if}` must start a new
  line). The record's `z` is the chunk's first extruding move's z1 (v2 rule). So an extrusion at the previous layer's
  height (Orca moves Z after the layer change G-code) changed the z of every `native_only` (a) block. The custom move
  is therefore `G1 X5 Y5 Z[layer_z] F3000`, and the blocks are then byte-equal to the plain plate's. The plain
  reference uses the same relative-E layer change G-code without the extrusion.
- **`native_only` (b) has no slice.** A search found no slice where the sparse role exists but no sparse path is
  printed. It tried 25 x w x 10 mm boxes (w 5.7-8.0 mm, 0.01 mm steps at the transition), densities 5/15/40 %,
  `minimum_sparse_infill_area` 15 and 0, and thin rings. Narrow parts end in `no_core` (no native sparse at all), and
  at the transition (w 7.43 / 7.46 mm) the sparse role and the printed sparse paths appear together. G4 runs the Catch2
  T12 case `text_only, native_only (a) and (b)` (25 assertions) instead.
- **The shadow side of G4 `native_only` (a) has no block at all**, so its translation passes
  `--allow-unconverted-powder`, which is the shadow-mode behaviour. Every other G4 export is translated: shadow
  exports with `--v2-parity`, product exports with the strict default.
- **The profile check is part of `product_gate.py g4`** (3 checks). No file in `resources/profiles` holds a `diw_*`
  key, so no DIW printer profile is shipped. None of the 30 custom G-code values of the two test profiles that run
  inside the layers extrudes. The start G-code extrudes (initial load, intro line), but Orca prints it under
  `;TYPE:Custom` before the first layer marker, outside every chunk. No `;TYPE:Custom` prefix was needed anywhere.
- **Harness changes for G4**: `planner_meshes.py` writes `cube10.stl`; `shadow_export.py` has `G4_PLATES`
  (`tall_short`), which is never part of the matrix. `--extra` now passes an entry's `env` to the job, uses the default
  meshes when the JSON has no `mesh_delta`, and asserts the coverage counters only for a set whose slices have roles.
  The coverage run is unchanged: 441 checks, 0 failures, the same counters.
- **Final samples**: S0 is `cube25.classic.*` (11 slices, 2669 checks, every export equal to fx_t2b) plus the
  coverage set (441). G1 runs on every slice whose name contains `classic.`: 127 slices (101 matrix, 13 stress,
  8 T5, 5 coverage), 1297 checks, 0 failures. A G1 run without coverage slices fails its `coverage_set` counter by
  design, so the sample includes them.

### Made during the fix pass (review findings F1-F4)

- **Orphan tolerance (F3).** Gate 12 compares the sparse region with the planned cover using
  `max(10 um, 0.5 x the widest outer / overhang wall width)` instead of the fixed 10 um (the suppression checks V0-V3 keep
  10 um). `SCover` takes the tolerance in its constructor and `near_ring` became a const member. The message no longer
  blames custom G-code. Catch2 T12 checks overshoot 9.5 / 9.45 (accepted) and 9.0 (orphan) in product and shadow mode.
- **Density rule (F4).** `Print::validate` refuses `sparse_infill_density >= 100%` on a powder job (no sparse infill
  exists to replace); rows R.density100 / R.density100_one / R.density99 in `product_validate.py`.
- **G6 row logic (F1).** A row counts as refused elsewhere only if Orca's own validate error (an ERR_RE match that is
  not u-MEX) is present; `product_validate.py --selftest` (4 checks) covers it.
- **G11 (F2).** New gate `product_gate.py g11`: the shipped in-slicer translation (no `ORCA_DIW_EXPORT_UNTRANSLATED`) equals
  `translate(product)` after `norm` for one slice per set plus the G5 refusals. `shadow_export.slice_part` and
  `product_export` gained an `untranslated` switch (default on, as before).
- **Observation.** The slice `ov60_w1` falls back to paste via `v2_bbox` under the new tolerance rules; legal and safe.
