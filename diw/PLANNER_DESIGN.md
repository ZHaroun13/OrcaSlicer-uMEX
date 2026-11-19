# Increment 4: slicer-core powder planner, shadow mode (final design)

Status: built and verified (18 Nov 2026). The design text below is the original (17 Nov 2026); what changed while
building it is in the Deviations section at the end (including the nine review fixes of 18 Nov), and the results
are in POWDER_PLAN.md, "Increment 4 results".
Read with [POWDER_PLAN.md](POWDER_PLAN.md) (architecture C) and [POWDER_RECORDS.md](POWDER_RECORDS.md) (record format v1, frozen).

**Basis.** This is design 3 ("minimal-risk") from the increment-4 design round. It adds the judges' grafts and
fixes the flaws they found:
- policy=error and h <= 0 now reproduce v2 exactly;
- the block is placed after `chunk.start`, so merged chunks are no longer skipped;
- a block needs native/text agreement;
- the band is B_native ∪ B_text, with a conservative chord pad;
- an explicit ring predicate replaces `ExtrusionLoop::role()`, and `is_inner_wall` is not trusted for `inset_idx < 0`;
- open outer walls get v2's snap + linemerge;
- the arc gate is shared;
- offline replay, `sync()`, fault injection, and the stronger clearance and perturbation tests are in;
- T5(iii) is now a reported metric, not an assertion.

**Constraint reminders.**
- Never edit `PrintConfig.hpp`. Activation uses existing keys and environment variables only.
- Shadow mode means the records are written into the G-code and nothing else in the file changes.
  Sparse infill is still printed in the G-code, and the translator sequences from the native records.

---

## 0. Decisions in one page

| Question | Decision |
|---|---|
| Where are records produced? | Two phases. **Capture** runs in `GCode::process_layer` and reads in-memory roles and closed loops. **Binder** is a streaming filter in `GCode::GCodeOutputStream::write`. It sees every byte of the final text, in order, before `fwrite` and `GCodeProcessor::process_buffer`. |
| k, z, h, final layer, arc/G91 | Computed by the binder from the final text, using the translator's own `GCodeScanner` / `ChunkTracker` (refactored from `scan_gcode` / `split_layers`). Exact by construction. |
| F, s, fu, fp | `DIW::resolve_powder_feed` and `plan_fingerprint(U)`. U comes from the translator's own parameter reader. Bit-equal by construction. |
| Core layers | Shadow mode writes a block only if **native role core** (from `LayerRegion::fills` roles plus the arc gate) **and text core** (v2 `classify_layers`) agree. Every disagreement is counted and reported. Native-only authority moves to increment 5. |
| Cavity S | Native closed `ExtrusionLoop::polygon()` loops (before `clip_end`), mapped to the exact G-code text grid. Open outer walls go through v2's snap and linemerge. |
| Band B | B_native (closed loops, per-path widths) ∪ B_text (v2's text walls and widths). Both are padded so the polygon contains the exact disk sum. |
| Paste exclusion E, sparse paths for FB | Taken from the chunk text (`parse_core_layer`), so they match v2 exactly. |
| Offsets | `Clipper2Lib::ClipperOffset` called directly with an explicit `ArcTolerance`. No ShortestEdge decimation. |
| Booleans | `ClipperUtils` `union_ex` / `diff_ex` (NonZero, or EvenOdd for S). |
| Hatch, coverage, FB clipping | Analytic, in double precision. No `intersection_pl`. |
| Grid | Fixed 1 nm integer grid (`constexpr double UNIT_MM = 1e-6`), independent of `SCALING_FACTOR`. |
| Placement | The block is inserted directly after the chunk's start-marker line. This is the line `v2_hooks.capture` uses. |
| Failures | No block, plus an aggregated warning. `st=open` is used only for v2's geometric "None" cases and for the two exact-parity carriers: h <= 0 and policy=error (section 7). |

---

## 1. Data flow

```
process_layer (generator stage, TBB serial_in_order)
  ├─ writes ;LAYER_CHANGE / ;Z: / ;HEIGHT:
  ├─ [capture] native_core? ── yes ─▶ NativeLayer{walls, rings, roles} stored in map[id] (mutex)
  │                         └─ writes ";DIWP <id> m=<0|1>\n" (always, when the shadow is active)
  └─ rest of the layer (before_layer_change_gcode, change_layer, extrusions, ...)
        │
        ▼ spiral? / PE / CoolingBuffer / FanMover / PA processor (text is rewritten; comments pass through)
        ▼
GCodeOutputStream::write(bytes)
  └─ [binder] split into lines → GCodeScanner.step → ChunkTracker.step
        ├─ before the first chunk: pass through
        ├─ hold the current chunk's lines and LineInfos
        ├─ next chunk opens  → finalize(held, final=false) → release
        ├─ "; CONFIG_BLOCK_START" or finish() → finalize(held, final=true) → release, then pass through
        └─ finalize: gates → parse_core_layer → k/z/h → resolve_powder_feed → plan_powder_layer_native
                     → write_block → parse_records round trip → insert after the start-marker line,
                     and drop every ";DIWP" line
        ▼
write_raw: fwrite + m_processor.process_buffer   (processor line ids, M73 and preview stay consistent)
        ▼
GCodeProcessor::finalize post-process (M73 only around G0-G3; never splits a block)
        ▼
diw_translate_gcode_file (PostProcessor.cpp:433 / OrcaSlicer.cpp:7203) reads the records
```

Threading:
- Capture runs on the generator's TBB thread and the binder on the output stage or the main thread.
- `write()` is never called concurrently (verified: the output stage is serial, and the main-thread writes happen before or after the pipeline).
- The capture map is a `std::unordered_map<uint64_t, NativeLayer>` behind a `std::mutex`. The binder erases each entry when it consumes it.
- At `finish()`, any entry left over is counted as `lost_placeholder`.

---

## 2. Files and functions

### 2.1 New files (libslic3r, listed in `src/libslic3r/CMakeLists.txt` after line 272)

**`src/libslic3r/GCode/DIWPowderPlanner.hpp/.cpp`** is a pure function. It depends on nothing from Print or Layer.
```cpp
namespace Slic3r::DIW {
constexpr double UNIT_MM = 1e-6;                 // planner grid: 1 integer unit = 1 nm
struct WallPiece  { Polyline pts; double width; bool closed; };   // closed: front()==back() not stored; EndType::Joined
struct NativeRing { Polygon poly; };                              // closed in-memory loop, G-code frame, nm grid
struct TextPath   { std::string ptype, kind; std::vector<Vec2d> pts_mm; Polyline pts_nm;
                    std::vector<double> seg_de, seg_feed; double width, z; std::optional<double> travel_feed; };
struct PlanInput  {
    std::vector<NativeRing> rings;        // native S input
    std::vector<WallPiece>  native_walls; // native band input
    std::vector<TextPath>   text_walls, text_paste, text_powder;   // from parse_core_layer
    double h, s_eff;                      // from the binder
    const UMexParams *U;
};
struct PlanDebug  { ExPolygons cavity, omega, narrow, revert; std::vector<std::string> notes; };
// Fills status, STAT, runs and fallbacks; the caller sets k, z, h, s, F, fu, fp.
PowderLayerRec plan_powder_layer_native(const PlanInput &in, PlanDebug *dbg = nullptr);
// exposed for unit tests:
Paths64 round_offset(const Polylines&, double r_mm, EndType);           // conservative pad, tight tol
std::vector<std::vector<Vec2d>> hatch_component(const ExPolygon&, double s, double angle_deg);
bool   covered(const ExPolygon&, Vec2d p, Vec2d q);
Vec2d  area_centroid(const ExPolygon&);  Vec2d interior_point_area(const ExPolygon&);
}
```

**`src/libslic3r/GCode/DIWPowderShadow.hpp/.cpp`** contains capture and binder.
```cpp
namespace Slic3r::DIW {
struct ShadowFrame { Vec2d writer_xy_offset; Vec2d extruder_offset; bool high_precision; };
class PowderShadow {
public:
    // nullptr + logged reason when refused (section 3)
    static std::shared_ptr<PowderShadow> create(const Print&, const DynamicPrintConfig &full, bool spiral_vase,
                                                bool axis_remap, std::string *reason);
    // generator thread: returns ";DIWP <id> m=<0|1>\n"
    std::string capture(const std::vector<GCode::LayerToPrint>&, const std::vector<const PrintInstance*>* ordering,
                        size_t single_object_instance_idx, const ShadowFrame&,
                        const std::function<bool(const ExtrusionPath&)> &path_uses_arcs);
    // output side
    using Sink = std::function<void(std::string_view)>;
    void feed(const char *bytes, const Sink&);   // splits lines, holds and releases chunks
    void sync(const Sink&);                      // releases held text; the open chunk becomes unbindable
    void finish(const Sink&);                    // the held chunk is final; flushes the partial line
    std::vector<std::string> warnings() const;   // aggregated, for active_step_add_warning
    std::string summary() const;                 // one BOOST_LOG line
};
}
```

**`diw/tests/compare_planner.py`** is the pure planner vs v2 harness (section 10, T1).
**`diw/tests/shadow_export.py`** is the end-to-end harness (T2, T5-T8).
**`diw/tests/planner_meshes.py`** writes the generated ASCII STLs of the corpus (section 10.1).
**`tests/libslic3r/test_diw_planner.cpp`** holds the Catch2 unit tests (T9). It is added to `tests/libslic3r/CMakeLists.txt`.

### 2.2 Modified files

| File | Change |
|---|---|
| `GCode/DIWUMexSequencer.hpp/.cpp` | Mechanical refactor. The loop body of `scan_gcode` becomes `class GCodeScanner { LineInfo step(const std::string&); }`. Its state is the former locals: x, y, z, e, rel, absxyz, feed, ftype, width. The loop body of `split_layers` becomes `class ChunkTracker { enum Ev{none, opened}; Ev step(size_t i, const std::string&, const LineInfo&); LayerChunk *current(); std::optional<LayerChunk> take_closed(); }` with the predicate open-iff-`is_layer_start && (!cur \|\| cur->n_extruding > 0)`. `scan_gcode` and `split_layers` become thin wrappers that keep the up-front `cfg_start` search. Also export the existing `truthy(std::optional<double>)` in the header. |
| `GCode/DIWTranslator.hpp/.cpp` | Move `read_params` (DIWTranslator.cpp:248) out of the anonymous namespace as `DIW::TranslatorParams diw_read_translator_params(const DynamicPrintConfig&)`. The body does not change. The shadow uses `.U` and `.umex` from it, so U, `plan_fingerprint(U)` and the feeds are exactly the translator's. |
| `GCode.hpp` | Forward declaration `namespace Slic3r::DIW { class PowderShadow; }`. New GCode members: `std::shared_ptr<DIW::PowderShadow> m_diw_shadow;` (shared_ptr, so the destructor is fine with an incomplete type) and `Vec2d m_diw_xy_offset{0,0};`. The latter is assigned in **both** inline setters (`set_gcode_offset`: `= {x, y}`; `set_gcode_offset_with_imex_shift`: `= {x + imex_x, y + imex_y}`, the same double expression the writer gets). `GCodeWriter::get_xy_offset()` returns a lossy `Vec2f` and must not be used. In `GCodeOutputStream`: `DIW::PowderShadow *m_diw = nullptr; void attach_diw(DIW::PowderShadow*); void diw_sync(); void diw_finish();` and a private `void write_raw(std::string_view)`. Also add `bool path_uses_arcs(const ExtrusionPath&) const;` (see the GCode.cpp row). |
| `GCode.cpp` | Hooks H1-H5 (section 4). The arc-fitting condition at about line 10077 is factored into `GCode::path_uses_arcs(path)`. This is behaviour-neutral, and the call site uses it, so capture and writer share one predicate. Add a const helper `Point GCode::text_grid_nm(const Point &p, const Vec2d &origin) const` beside `point_to_gcode_quantized` (11683) (section 5.3). |
| `src/dev-utils/diw_translate.cpp` | New modes: `--plan-json <in.json> <out.txt> <debug.json>` (pure planner from JSON) and `--plan-records <in.gcode> <out.gcode>` (offline binder with text-only capture, section 9). |
| `src/libslic3r/CMakeLists.txt` | Add the 4 new files. Set per-file `/fp:precise` and `#pragma fp_contract(off)` in both new .cpp files. On GCC/Clang set `-ffp-contract=off` with `set_source_files_properties`. |
| `diw/tests/v2_hooks.py` | Add `capture_geometry(E, cfg, src)`. It wraps `plan_powder_layer` and also returns WKT of cavity, omega, narrow and revert for each k. v2 itself is never modified. |

`PrintConfig.hpp`, the pipeline stages and the wipe tower are **not** touched.

---

## 3. Enabling and refusals

In `GCode::do_export`, right after `GCodeOutputStream file(...)` (GCode.cpp:2927):
```cpp
std::string why;
m_diw_shadow = DIW::PowderShadow::create(*print, print->full_print_config(), m_spiral_vase != nullptr,
                                         m_writer.has_axis_remap(), &why);   // remap query: whatever GCodeWriter.cpp:1324 tests
if (m_diw_shadow) file.attach_diw(m_diw_shadow.get());
else if (!why.empty()) BOOST_LOG_TRIVIAL(info) << "DIW shadow planner off: " << why;
```
`create` returns null in these cases:
- `diw_translation_enabled(full)` is false, or `diw_powder_infill` is false: no reason is logged.
- The env var `ORCA_DIW_NO_NATIVE_PLAN=1` (kill switch) is set.
- `print.is_BBL_printer()`. The tags are then in the BBL dialect, so v2 and the translator see no layers.
- Spiral vase, a GCodeWriter XY axis remap, a belt printer, or differing `extruder_offset` values across extruders.
- `diw_read_translator_params(full)` throws `DIWConfigError`. The translator raises the same message later, so it is not duplicated here.

In each refusal case except the first, one NON_CRITICAL warning is added after `finalize`. It reads "u-MEX native powder planner disabled: <reason>". A powder job on such a printer still fails at the translator's "powder job without blocks" refusal, and the warning explains why.

Verified: every GUI and CLI translator call passes `full_print_config()`. The callers are BackgroundSlicingProcess.cpp:296/871/1017, Plater.cpp:20623 → PostProcessor.cpp:433, and OrcaSlicer.cpp:7203. The fingerprint and U are therefore identical on both sides. Test T2(b) guards this.

Test-only env vars:
- `ORCA_DIW_SHADOW_FAULT=capture|plan|binder` throws at that point (T6).
- `ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS=1` keeps the `;DIWP` lines, for offline replay debugging only. They are not records. The translator ignores them, but they do count as comments in the file.
- `ORCA_DIW_SHADOW_DUMP=<dir>` writes one `k<k>.json` per block. It holds the PlanInput (rings, native walls, text paths, h, s_eff), the PlanDebug WKT, and the record text.

---

## 4. Hook points in GCode.cpp

- **H1 `do_export`**:
  - creation, as in section 3;
  - after `this->_do_export(*print, file, thumbnail_cb);` (2953) and before `file.flush()` (2954): `file.diw_finish();`;
  - on the exception path, the shadow is dropped together with the temp file;
  - after `m_processor.finalize(true)`, every `m_diw_shadow->warnings()` entry goes to `print->active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL, msg)` (the same API as GCode.cpp:3065), plus one `BOOST_LOG_TRIVIAL(info) << summary()`; then `m_diw_shadow.reset()`.
- **H2 `GCodeOutputStream::write`** (9131):
  ```cpp
  if (what == nullptr) return;
  if (m_diw) m_diw->feed(what, [this](std::string_view s){ write_raw(s); }); else write_raw(what);
  ```
  `write_raw` holds today's body: `fwrite`, then `m_processor.process_buffer(std::string(s))`. `diw_finish()` calls `m_diw->finish(sink)` and then sets `m_diw = nullptr`. `diw_sync()` calls `m_diw->sync(sink)`.
- **H3 `_do_export`**: call `file.diw_sync()` right before `m_processor.get_current_position()` (4582). That is the only processor read during the export. It normally holds nothing; it matters only if custom start G-code contains a layer marker.
- **H4 `process_layer`**: after the `;HEIGHT:` sprintf (6702-6703) and before `before_layer_change_gcode` (6708):
  ```cpp
  if (m_diw_shadow)
      gcode += m_diw_shadow->capture(layers, ordering, single_object_instance_idx,
                                     {m_diw_xy_offset, m_config.extruder_offset.get_at(0), m_writer.high_precision()},
                                     [this](const ExtrusionPath &p){ return path_uses_arcs(p); });
  ```
  It is not reached by `process_belt_brim_layer` (6640) or by the `extruders.empty()` return (6658). The belt is refused anyway. An empty layer writes no marker, so the text tracker never sees it either.
  - Placeholder format: `;DIWP <id> m=<0|1>`, where `<id>` is a decimal sequence number.
  - It does not start with `;DIW:`, so it is never a record line.
  - It does not start with `;_`, so CoolingBuffer's tag scan (CoolingBuffer.cpp:425-570) ignores it. Unknown comment lines are copied through at CoolingBuffer.cpp:1061.
  - It contains no `layer`, no brackets and no `@`.
- **H5 arc predicate**: `bool GCode::path_uses_arcs(const ExtrusionPath &p) const`. It returns the exact condition under which `_extrude_piece` emits G2/G3 for `p` (the `enable_arc_fitting` / `fitting_result` / should-disable gate at about 10072-10077). The writer site is rewritten to call it. This changes no G-code bytes; T7 checks that.

---

## 5. Phase A: capture (generator thread)

### 5.1 Native classification (v2 `classify_layers`, auto mode)

Iterate every `LayerToPrint lt` in `layers` with `lt.object_layer != nullptr`, every region of it, and `region->fills` recursively down to leaf `ExtrusionPath`s with a non-empty polyline. Support layers and `support_fills` do not count (v2 treats them as paste).
- `has_sparse |= role == erInternalInfill`
- `has_skin |= role ∈ {erTopSolidInfill, erBottomSurface}`. Orca never writes "top/bottom solid infill". Bridge, Internal Bridge, `erSolidInfill`, gap fill and ironing are **not** skin.
- `has_arc |= path_uses_arcs(path)`. This covers every extruded leaf in `fills` and `perimeters` of every object layer, and `support_fills`.
- `native_core = has_sparse && !has_skin && !has_arc`.

Use roles, not `SurfaceType`. `stInternalVoid` and density 0 give no sparse role, and `top_surface_density` 0 gives no skin role.

The final-layer, G91 and k decisions are made on the text (section 6).

For `m=0` the placeholder is still written, so the binder can count text-core layers that have no native support. No geometry is stored for it.

### 5.2 Walls (only when native_core)

**Instance set.** Mirror the instance loop at GCode.cpp:7334-7350 exactly, including any skip or filter it applies:
- non-sequential mode: every instance of `lt.original_object`;
- otherwise: only `instances()[single_object_instance_idx]`.

The implementer copies that selection code. The bidirectional frame check (section 6.3, gate 9) catches any drift.

For each instance and each region, walk `region->perimeters` recursively.
- `ExtrusionLoop L`:
  - `pts = L.polygon()` (ExtrusionEntity.cpp:175). This is the closed in-memory loop **before** `clip_end`, so there is no seam gap.
  - Band pieces: one `WallPiece` per `ExtrusionPath` of L with a wall role (`erPerimeter`, `erExternalPerimeter`, `erOverhangPerimeter`), with `width = double(path.width)`. Consecutive pieces share endpoints.
    - If all of L's paths have the same width, also store the whole loop as one closed piece (`EndType::Joined`).
    - Otherwise use only the per-path pieces with `EndType::Round` (Arachne variable width).
  - **Ring predicate**, explicit and without `L.role()`:
    ```
    is_ring(L) = L.inset_idx == 0
              || (L.inset_idx < 0 && every path of L has role ∈ {erExternalPerimeter, erOverhangPerimeter})
    ```
    - `L.role()` is `paths.front().role()`, which is `erOverhangPerimeter` on overhang-split inner walls.
    - `is_inner_wall()` falls back to `role() == erPerimeter` for `inset_idx < 0` (ExtrusionEntity.hpp:167-169), so it must not be used.
    - Inner loops are never rings. This is the known divergence D3.
  - Arachne disconnection guard: if two consecutive paths of L have ends more than 1 µm apart (in mm, before the transform), L is not a ring. It is noted as `disconnected`, and its pieces still go into the band.
- `ExtrusionMultiPath` / `ExtrusionPath` (open walls: classic thin walls, Arachne open lines, extra overhang perimeters): these become `WallPiece`s with `EndType::Round`, per path width.
  - Pieces whose role is `erExternalPerimeter` or `erOverhangPerimeter` are also added to the open-outer list. This list gets v2's ring treatment (section 6.4, step P1b).

### 5.3 Frame transform to the text grid (nm, exact)

Per instance, `origin` is computed with the same expression `set_origin` receives at GCode.cpp:7690 (`unscale(instance.shift)`). Then for every point p:
```
Vec2d g = unscale(p) + origin - extruder_offset;          // GCode::point_to_gcode order (11659-11663)
g = g - writer_xy_offset;                                  // GCodeWriter.cpp:1321 order, m_diw_xy_offset (double)
int64 q = (int64) std::round(g.x() * pow10[d]);            // GCodeFormatter::emit_axis rounding (GCodeWriter.cpp:1690-1730)
nm = q * pow10[6 - d];                                     // d = 3, or 4 with high_precision_gcode; exact integer
```
- This lives in `GCode::text_grid_nm`. The `round` and the `pow10` table must be the same expressions `emit_axis` uses. Factor them into a shared `static int64_t GCodeFormatter::axis_units(double v, size_t d)` and call it from both places (behaviour-neutral).
- Never use `GCodeFormatter::quantize`: it multiplies by the inverse power of ten and can be one ulp off.
- With this transform, every native vertex that the writer prints equals the text vertex exactly. Only vertices removed by `clip_end` (the seam tail) and the seam split point are new.

The result is stored as `map[id] = NativeLayer{ native_core, rings (Polygons per instance), pieces, open_outer, error }`.

The whole capture runs in `try { ... } catch (const std::exception&)`. An exception stores `error = what()` and still returns the placeholder. `CanceledException` is rethrown. Fault hook: `capture`.

---

## 6. Phase B: the binder (output side)

### 6.1 Stream mechanics

`feed(bytes)` appends to `m_partial` and splits at `\n`. The `\n` stays with the line, and `\r` stays part of the line, as in the translator. For each complete line, in this order:
1. If `m_passthrough` is set (config seen, or disabled mode): drop the line if it is a `;DIWP ` line, else release it.
2. `li = scanner.step(line)`. The scanner sees the placeholder lines too. They are comments, so the LineInfo state does not change.
3. If the line starts with `; CONFIG_BLOCK_START`: finalize the held chunk as **final**, release the line, and set `m_passthrough`.
4. `ev = tracker.step(i, line, li)`. On `opened`: finalize the previous held chunk as **not final**, release it, then start holding with this marker line as held line 0.
5. If a chunk is open, append `(line, li)` to it. If the line is `;DIWP <id> m=<b>`, also record `(held index, id, b)`. Otherwise release the line immediately (text before the first chunk).

Other entry points:
- `finish()` treats a non-empty `m_partial` as the last line (no `\n`), finalizes the held chunk as **final**, and releases it.
- `sync()` releases all held lines (placeholders dropped) and marks the open chunk unbindable (reason `sync`).
- `prev_z` follows the translator: after each finalized chunk, `if (chunk.z) prev_z = *chunk.z;`, starting at 0.0.
- The scanner, tracker and line indices run over the text with placeholders included. Placeholders are comments, so `split_layers` gives the same chunks with or without them, and k is unaffected. This is proven by T2(a).

### 6.2 Disabled mode

Any exception escaping `feed` or `finalize` (a bug), and the `binder` fault hook, switch the binder to disabled mode:
- held lines are released with every `;DIWP ` line removed;
- `m_passthrough` is set;
- one warning is recorded;
- the export never fails.

### 6.3 `finalize(chunk, lines, infos, placeholders, final)`: gates in order

The first gate that fails writes **no block** and counts its reason. All `;DIWP` lines of the chunk are always removed.

| # | Gate | Reason key |
|---|---|---|
| 0 | No placeholder in the chunk | (silent) |
| 1 | `final`: v2 never converts the last chunk | `final` |
| 2 | `chunk.has_arc \|\| chunk.has_relxyz` | `arc_g91` |
| 3 | `text_core = types ∩ {"sparse infill","internal infill"} ≠ ∅ && types ∩ {"top surface","bottom surface","top solid infill","bottom solid infill"} = ∅` (v2 auto rule on normalised types); `native_core = OR of m over the chunk's placeholders`. If `text_core != native_core`, count `disagree_text_only` or `disagree_native_only` and record k (no block). If both are false, stop silently. | `disagree_*` |
| 4 | Exactly one placeholder in the chunk has `m=1`; otherwise `merged_multi_core`. A chunk that merged an empty or arc-only marker with a core layer has one m=1 placeholder and **is** converted (the block is relocated, see 6.4). | `merged_multi_core` |
| 5 | Look up and erase `map[id]`. If it is missing (`lost_placeholder`) or has `error` (`capture`), stop. | `lost`, `capture` |
| 6 | `model = parse_core_layer(lines, infos, LayerChunk{chunk with start=0,end=n}, U.width_fallback)`; `z = *model.z` (never empty for a text-core chunk; if it is, `no_z`); `h = truthy(chunk.height) ? *chunk.height : z - prev_z`. | `no_z` |
| 7 | **h <= 0**: write the carrier block `st=open, z, h, s=0.0, F=0.0, fu=0.0`, with STAT zeros. The translator compares z/h, emits v2's "non-positive layer height; left as paste" and counts the layer for `last_in_stack`, exactly as v2 does. Go to 6.4. | (block) |
| 8 | `pf = resolve_powder_feed(U.m_dot, U.rho_tap, U.s_hatch, h, U.eta, U.f_max, U.policy)`. If it throws `DIWConfigError` (policy `error`), compute `pc = resolve_powder_feed(..., "clamp")` and write the carrier block `st=open, z, h, s=U.s_hatch, F=pc.f_calc, fu=pc.f_calc`, with STAT zeros. The translator calls `resolve_powder_feed` (DIWUMexSequencer.cpp:577) **before** the F/s/fu compare and the status check, so it raises v2's own abort message at the same layer. Exact v2 parity. | (block) |
| 9 | **Frame self-check** (bidirectional, tol_frame = `0.5*10^-d*sqrt(2) + 1e-6` mm). (a) Every vertex of every text path of kind `wall` must lie within tol_frame of some native `WallPiece` segment. (b) Every native ring must have at least one text wall vertex within tol_frame of it. Use an AABB over the segments (`AABBTreeLines`). This catches transform, instance, extruder-offset and printed-but-not-captured errors. | `frame` |
| 10 | Build `PlanInput` (section 6.5) and call `plan_powder_layer_native`. Catch `std::exception` (rethrow `CanceledException`). Fault hook `plan`. | `plan` |
| 11 | `rec = {k = chunk.index, st, z, h, s = pf.s_eff, F = pf.F, fu = pf.f_calc, fp = plan_fingerprint(U), STAT, runs, fallbacks}`; `txt = write_block(rec)`. If that throws `invalid_argument`, stop with `record`. Then `parse_records(txt)` must give 1 block and 0 errors, and `dump_records` must equal the dump of `rec`; otherwise `record`. | `record` |

### 6.4 Placement and k

- **k** is `chunk.index` from the tracker, the same value as the translator's `LayerChunk.index`.
- **Placement**: the block text goes **immediately after held line 0**, the chunk's start-marker line, whatever line the placeholder was on.
  - This is exactly where `v2_hooks.capture` inserts (`after[L.start]`).
  - It satisfies the translator rule `chunk.start < pos <= chunk.end` with no motion line in `[start, pos)`, because the marker line is a comment.
  - It also holds for chunks whose start marker belongs to an earlier, merged empty layer.
- The chunk's lines are then released in order: line 0, block, lines 1..n-1 minus the placeholders.

### 6.5 PlanInput assembly

- `rings`: the native rings of every instance, plus the rings that P1b forms from `open_outer` (section 7).
- `native_walls`: every native `WallPiece`.
- From `model.paths`: `text_walls` (kind wall), `text_paste` (kind paste, at least 2 points), `text_powder` (kind powder).
  - Points are converted to nm with `llround(x * 1e6)`. Text values have 3 or 4 decimals, so this is exact.
  - `ptype`, `width`, `z`, `travel_feed`, `seg_de` and `seg_feed` are copied verbatim.
- `h`, `s_eff = pf.s_eff`, `U`.

---

## 7. Geometry: `plan_powder_layer_native` (mirrors v2 `plan_powder_layer`, diw_engine.py:1025-1131)

### 7.1 Conventions and constants

- Grid: 1 unit = 1 nm (`UNIT_MM = 1e-6`). Area in mm² = `area_units * 1e-12`. `SCALING_FACTOR` is never used here.
- Tight arc tolerance: `tol(r_u) = r_u * 2 * sin²(π/384)` (= r(1 − cos(π/192)), 192 steps per 360°). `r_u` is the exact value passed to Clipper2 (`double`). Clipper2 is called directly, so no float delta is involved.
- Chord error and pad:
  - `e(r) = r(1 − cos(π/192))`.
  - Orca's patched `DoRound` can make its last step up to 1.5θ, so the worst sagitta is `2.25·e(r)`.
  - Every **dilation of walls or paste** uses the padded radius `r* = r + 2.25·e(r)`, rounded **up** to the grid: `ceil(r*·1e6)` units. The polygon then contains the exact Minkowski disk sum.
  - Pad sizes: 0.06 µm at r = 0.21 mm, 0.18 µm at r = 0.6 mm.
- Erosion (Ω) needs no pad. Its round joins sit at reflex corners of C and the chords cut inward, so the error is on the safe side.
- Offsets: `Clipper2Lib::ClipperOffset co(miter_limit, arc_tol, /*preserve_collinear*/false, /*reverse*/false); co.AddPaths(paths, jt, et); co.Execute(delta, Paths64&)`.
  - Check the constructor and setters against `deps_src/clipper2/Clipper2Lib/include/clipper2/clipper.offset.h` before coding.
  - `ClipperUtils::offset*` must not be used. It treats `miterLimit` as the arc tolerance (default 3 nm), and `append_offset_path` drops vertices within `0.005·|delta|` of the previous one (ClipperUtils.hpp:47). That is 3 µm at δ = 0.6 mm.
  - Offset output is converted to `ExPolygons` with `union_ex(Polygons, pftNonZero)`. Clipper2 emits holes with the opposite orientation, so NonZero is correct.
- Booleans: `union_ex`, `diff_ex` from ClipperUtils (NonZero, no safety offset), and `union_ex(Polygons, pftEvenOdd)` for S.
- Sums: `DIW::PySum` and `DIW::py_hypot` wherever v2 uses `sum` and `math.hypot`. Plain `+=` where v2 uses `+=`.
- v2 constants: `POWDER_GUARD_MM = 0.001`, `LOOP_CLOSE_TOL_MM = 0.05`, component area filter `> 1e-6` mm², row piece min `> 1e-4`, join gap `<= 3*s`, zero gap `< 1e-9`, coverage `1e-6`, FB min piece `>= 0.05`, contiguity `< 1e-6`, opening margin `1e-4` mm (100 units), mitre limit `5.0`.

### 7.2 Steps

- **P0.** If `rings` is empty and `open_outer` gives no ring, return `st=open` (STAT `0.0`, `nc=0`). This is v2's "no outer wall" None.
- **P1a, native rings.** Each `NativeRing` polygon (nm, closed). Drop rings with fewer than 3 distinct points or zero area.
- **P1b, open outer walls (v2 path, parity).** For each `open_outer` polyline, in mm doubles on the text grid:
  - if it has at least 3 points and `py_hypot(first − last) < 0.05`, set `last = first` (v2 snap);
  - then run a port of GEOS `LineMerger` (merge at exactly equal endpoints where the node degree is 2; direction may flip);
  - a merged line with at least 4 coordinates whose ends are within 0.05 becomes a ring.
- **P1c.** `S = union_ex(all rings, pftEvenOdd)`. If S is empty, return `st=open`.
- **P2, band.** Group by exact `double` width w, and for each width group make one offset call per EndType:
  - `B_native`: native closed whole-loop pieces with `EndType::Joined`, the rest with `EndType::Round`, `JoinType::Round`, radius `(w/2)*`, `tol(r)`.
  - `B_text`: the text wall paths with v2's width (first-segment `;WIDTH:` rule). Use `EndType::Joined` when `pts.front() == pts.back()` exactly, else `EndType::Round`.
  - `B = union(B_native ∪ B_text)`.
  - The union is never narrower than either the true printed walls or v2's band, so native Ω is never more permissive than v2's Ω.
- **P3, cavity.**
  - `E` = the text paste paths offset the same way (Joined if exactly closed, else Round, radius `(w/2)*`).
  - `C = diff_ex(S, union(B, E))`.
  - Keep ExPolygons with area `> 1e-6` mm² (holes subtracted).
  - `ca = Σ area` in Clipper output order.
- **P4, Ω.**
  - `d_eff = U.delta + POWDER_GUARD_MM` (the same double expression as v2). `D = llround(d_eff * 1e6)`.
  - `Ω = offset(C, −D, Round, EndType::Polygon, tol(D))`. Keep components with area `> 1e-6` mm².
  - `nc = |Ω|`, `oa = Σ area`.
- **P5, narrow and revert.**
  - If Ω is non-empty:
    - `opening = offset(Ω, +D, JoinType::Miter, MiterLimit 5.0)`;
    - `opening_m = offset(opening, +100, Round, tol(100))`;
    - `narrow = diff_ex(C, opening_m)`.
  - Otherwise `narrow = C`.
  - `revert = {g ∈ narrow : area(g) >= U.paste_min_area}`, `ra = Σ area`.
- **P6, order.** `std::stable_sort(Ω)` by the integer shell bbox key `(min.y, min.x)`. On the nm grid this equals v2's `(round(miny,6), round(minx,6))`. Ties keep Clipper order (documented).
- **P7, per component** (section 7.3 for the hatch):
  - `runs_c = hatch_component(ω, s_eff, U.hatch_angle)`;
  - `total = PySum_runs(PySum_segs(py_hypot))`.
  - If `total < U.point_len`, write a point dwell:
    - `c = area_centroid(ω)`. This is the GEOS area centroid in double: triangle fan from the shell's first vertex, holes subtracted. It is **not** `Polygon::centroid`, which ignores holes and rounds.
    - If `c` is not strictly inside ω, use `c = interior_point_area(ω)`, a GEOS `InteriorPointArea` port: scan y = mean of the nearest vertex y strictly below and strictly above the envelope centre y; take the widest crossing interval on that line and use its midpoint.
    - `m = (((area_mm2(ω) * h) * 1e-3) * U.rho_tap) * U.eta`; `d = (60.0 * m) / U.m_dot`; record `RUN t=point d=d`.
  - Otherwise append the runs as `RUN t=line` and do `hl += total` (plain `+=`).
- **P8, paste fallback FB** (only when the runs are non-empty and revert is non-empty).
  - `pad = min(0.5 * U.width_fallback, 0.5 * U.delta)`; `clip = union(offset(revert, +ceil(pad*1e6), Round, tol))`, converted to mm double rings.
  - For each text powder path, for each segment `a→b` with `L = sqrt(dx*dx + dy*dy)` (GEOS length formula) and `L >= 1e-9`:
    - take the crossing parameters t of a→b with every clip edge, plus 0 and 1, and sort them;
    - keep the sub-intervals whose midpoint is inside (even-odd), merging adjacent kept intervals;
    - keep the pieces with length `>= 0.05`, in segment direction;
    - `de_piece = de * len_piece / L`; the feed comes from the text segment.
  - Chain the pieces with v2's rule: extend the current FB if `py_hypot(piece.a − cur.back()) < 1e-6`, else start a new one.
  - Each FB copies `ptype`, `width`, `z` and `travel_feed` from the source path.
- **Status.**
  - `ok` if the runs are non-empty (point runs included);
  - otherwise `small` (STAT filled, no RUN/FB);
  - `open` per P0 / P1c, with STAT zeros.
  - Records list all RUNs in plan order, then all FBs.

### 7.3 Hatch (`hatch_component`, analytic, double; port of `hatch_polygon` 967-1022)

1. Convert to mm: `x = x_u * 1e-6`.
2. If `|angle| > 1e-12`, rotate exactly as shapely does:
   - `rad = (-angle) * M_PI / 180.0`; `cp = cos(rad)`, `sp = sin(rad)`, each set to `0.0` if its absolute value is `< 2.5e-16`;
   - `x' = cp*x + (-sp)*y + 0.0`, `y' = sp*x + cp*y + 0.0` (no FMA).
3. Bounds from the shell: `H = maxy - miny`; `n = int(floor(H/s + 1e-9)) + 1`; `y0 = miny + (H - (n-1)*s) / 2.0`; `y_i = y0 + i*s` (not cumulative).
4. Row i:
   - crossings `x = px + (y - py)*(qx - px)/(qy - py)` over every edge of every ring with `min(py,qy) <= y < max(py,qy)`; sort and pair even-odd into intervals;
   - add edges with `py == qy == y` as closed intervals, and merge overlapping or touching intervals (GEOS closed-set semantics);
   - a piece is `(a, b)` with `a.x <= b.x`; keep it if `py_hypot(b − a) > 1e-4`; tuple-sort by `(ax, ay, bx, by)`;
   - for odd i (empty rows count), reverse the list and swap each piece's ends.
5. Chain over the flat list:
   - `gap = py_hypot(a − cur.back())`; `joined = gap <= 3*s && (gap < 1e-9 || covered(cur.back(), a))`;
   - if joined: append a only when `gap >= 1e-9`, then append b; otherwise close the run and start `[a, b]`.
6. `covered(p, q)`:
   - split p→q at every proper crossing with the ring edges;
   - it is true iff every sub-interval has its midpoint inside (even-odd) **or** has its midpoint and both quarter points within `1e-6` of an edge.
   This stands in for `poly.buffer(1e-6).covers(connector)`.
7. Back rotation: `th = angle * (M_PI/180.0)` (one multiply by the precomputed constant, as CPython `math.radians` does); `cs = cos(th)`, `sn = sin(th)` with no snapping; `(x*cs − y*sn, x*sn + y*cs)`.

### 7.4 Clearance arithmetic (d = 3)

- B and E contain the exact disk sums, and the erosion by d_eff errs inward. So every point of Ω is at least `d_eff − 4 nm` (grid rounding of about 4 ops) from every wall and paste centreline minus w/2.
- Run points lie on or inside ∂Ω (double rotation error about 1e-15 mm). Connectors are covered to within 1 nm.
- The translator rounds run points to 3 decimals, at most 0.707 µm.
- Result: `dist ≥ δ + 1 µm − 0.707 µm − 0.005 µm > δ`. The invariant holds strictly; with d = 4 the margin is larger.

---

## 8. Failure policy

| Situation | Block | User-visible |
|---|---|---|
| Shadow refused at startup | none | one NON_CRITICAL warning (except when not a powder job) |
| Capture exception | none for that chunk | aggregated warning "u-MEX native powder planner: N core layer(s) (k list) left as paste: capture failed: <first what>" |
| Native and text classification disagree | none | aggregated warning with counts and the first 10 k values for each direction |
| More than one native-core placeholder in a chunk, or a lost placeholder | none | aggregated warning |
| Frame self-check fails | none | aggregated warning `frame` |
| Planner exception | none (v2 also writes none when planning raises) | aggregated warning `plan: <what>` |
| `write_block` / `parse_records` round trip fails | none | aggregated warning `record` |
| h <= 0 | `st=open` carrier with s/F/fu 0.0 | the translator emits v2's own warning |
| policy=error overflow | `st=open` carrier with F = fu = f_calc, s = s_hatch | the translator aborts with v2's own message at that layer (v2 parity) |
| Binder bug | disabled mode; the rest of the file passes through | one warning |
| `CanceledException` | rethrown | normal cancel |

- `st=open` keeps its record meaning ("wall loops could not be closed"). It is written for v2's None cases (P0, P1c). The two carriers never reach the status check in the translator: h <= 0 returns before it, and policy=error throws before it.
- Remaining divergence: a planner failure on a v2-core layer gives no block, so `last_in_stack` of the layer below differs from v2. This is the same as v2's own "planning failed" handling except for the banner. Record v1 is frozen.

---

## 9. Exactness against v2

**Bit-exact by construction:**
- k, placement, z, h, s, F, fu, fp;
- the final, arc and G91 exclusions;
- the h <= 0 and policy=error behaviour;
- the core set, wherever both classifications agree (and a block exists only when they do);
- FB `ty`, `w`, `tf`, `z` and the per-segment feed.

**Expected equal, not guaranteed:** `st`, `nc`, runs per component, FB count.

**Tolerance only:**
- ca, oa, ra, hl;
- C, Ω and narrow boundaries;
- RUN XY and point counts (GEOS splits rows at touching vertices);
- point-dwell XY and d;
- FB XY and de.

**Intended or known divergences:**
- D1: seam gap > 0. The native ring and band are closed; v2 snaps (gap < 0.05) or fails (`st=open`).
- D2: Arachne variable widths. Native uses per-path widths (unioned with v2's band).
- D3: inner-wall overhang pieces. v2 puts them into its ring set; native never does.
- D4: mitre tips under about 23°. GEOS truncates at 5d, Clipper2 squares at 1d, so native revert is slightly larger.
- D5: rows exactly on a horizontal Ω edge (H/s integral within 1e-9).
- D6: chunks with two native-core placeholders get no block (practically unreachable).
- D7: chord pad and band union make native Ω ⊆ v2 Ω ⊕ 0.3 µm (slightly smaller).
- D8: planner failures get no block.

**Offline replay** (`diw_translate --plan-records`): runs the binder on a finished G-code file.
- Without placeholders, capture is replaced by a text-only adapter: rings from v2's snap + linemerge over the text outer and overhang walls, and B_native = ∅.
- Uses: debugging, and T2(d) determinism on seam_gap 0 files whose placeholders were kept with `ORCA_DIW_SHADOW_KEEP_PLACEHOLDERS=1`.
- In that case the native capture data is reloaded from `ORCA_DIW_SHADOW_DUMP` JSON when present.

---

## 10. Test harness

All Python runs use `python -I` (Python 3.15, shapely 2.2.0, GEOS 3.14.1) with v2 unmodified, as in increment 3. A missing executable, flag or input is a **FAIL**, never a skip. Every script prints `N checks, F failures` and exits non-zero on any failure.

### 10.1 Corpus and mesh sources

- **Existing G-code corpus** (increment 3): cube25, ring, lshape, two_objects, cone, and the generated notch. They are the inputs of `compare_planner.py`.
- **Meshes for new slices.** `diw/tests/planner_meshes.py <out dir>` writes ASCII STLs:
  - cube25: a 25 mm cube;
  - ring: Ø30 outer, Ø14 hole, 10 mm high;
  - lshape: 30×30 L with 12 mm arms;
  - notch: 25 mm cube with a 15° V-notch;
  - cone: Ø20 to Ø4, 15 mm;
  - slot: 20×20 with a 2·d_eff + 2 µm slot;
  - overhang: a 45° overhang wedge.
  It also uses `tests/data/cube_with_hole.obj`, `cube_with_concave_hole.obj`, `overhang.obj` and `bridge.obj` from the repo.
- **Slicing**: OrcaSlicer CLI with the DIW printer, filament and process profiles, with `ORCA_DIW_EXPORT_UNTRANSLATED=1` so the raw file with records is kept. `shadow_export.py` builds these plates (3MF or CLI arrange):
  - each mesh alone, under: classic and Arachne walls; seam_gap 0 % and 15 %; hatch_angle 0, 30, 45, 90 and −30; δ 0.2, 0.4 and 0.6;
  - two objects; two instances of lshape; objects at 0.2 and 0.3 mm layer height (interleaved h);
  - by-object sequential printing; supports on (overhang); a prime tower;
  - `detect_overhang_wall` on;
  - fan_speedup_time, pressure equalizer and cooling slowdown on;
  - arc fitting on (those layers must get no block);
  - scarf seam on;
  - the 3 f_max/policy variants from `compare_records`, including policy=error with an overflow.

### 10.2 Tests

**T0 Regression gate (stage 1).**
- `compare_sequencer.py` (157, with and without `--stub-s4`), `compare_records.py` (486), `compare_powder.py` (62) and `compare_v2.py` (54).
- All must report 0 failures with the same check counts after the scanner/tracker refactor.

**T1 Pure planner vs v2** (`compare_planner.py <v2 dir> <diw_translate> <work> <corpus...>`).
- Input: for every LayerModel that `v2_hooks.capture_geometry` reaches on the corpus × the 8 compare_records variants plus hatch_angle {30, 45, 90} and δ {0.2, 0.4, 0.6}, emit `PlanInput` JSON.
  - Rings come from v2's own snapped + linemerged outer/overhang text walls.
  - `native_walls` = ∅ (pure algorithm parity).
  - The text paths are verbatim. h and s_eff are v2's.
- Run `diw_translate --plan-json` and compare with v2's plan and geometry:
  - `st` equal; `nc` equal;
  - `|Δca|, |Δoa| ≤ 1e-5·value + 1e-4` mm²;
  - `|Δra| ≤ 1e-4·ra + 1e-3` mm². A component-count mismatch is allowed only when a v2 narrow region is within 1e-3 mm² of `paste_min_area` or has a tip under 23° (logged);
  - Hausdorff(∂C), Hausdorff(∂Ω) ≤ 2 µm (budget: 0.41 µm chord + 0.3 µm pad + grid + GEOS noise);
  - containment: Ω_native ⊆ Ω_v2 ⊕ 0.3 µm;
  - hatch per component, paired in sort order and confirmed by an Ω Hausdorff ≤ 2 µm:
    - components with `dist(H/s, ℤ) < 2 µm/s` are excluded and counted; the excluded fraction must be < 5 %;
    - row count equal, run count equal;
    - Hausdorff of the polyline unions ≤ 2 µm; `|Δhl| ≤ 1e-5·hl + 4 µm·rows`;
    - RUN point counts are **not** compared;
  - point dwells: same count, position ≤ 2 µm, `|Δd|/d ≤ 1e-5`;
  - FB: same count and order (else logged with GEOS's piece order); points ≤ 2 µm; per-segment `|Δde| ≤ de/len·4 µm`; feed, w, tf, z and ty string-equal; pieces within 1 µm of 0.05 mm are excluded;
  - synthetic inputs: the slot, tips at 15/22/25°, holes, two components with equal miny, integral H/s, all 5 hatch angles.

**T2 End-to-end shadow** (`shadow_export.py`), for every slice in 10.1:
- (a) `strip_records(on)` is byte-identical to the export with `ORCA_DIW_NO_NATIVE_PLAN=1`. No `;DIWP` line remains, and every `;DIW:BEGIN` follows a start-marker line.
- (b) `diw_translate` in powder mode exits 0, or with v2's exact policy=error message on that variant. This proves placement, k, fp, z/h and F/s/fu against the translator.
- (c) Pair `strip_records(on)` → `v2_hooks.capture_geometry` with the native blocks by k:
  - the core-set difference must be explained only by logged D3/D6/D8 cases;
  - the disagreement counter must be 0 on the corpus;
  - BEGIN `k z h s F fu fp` must be string-equal; `st` equal on the seam_gap 0 set;
  - STAT, RUN and FB use the T1 tolerances, with C and Ω Hausdorff measured outside a disc of radius `w + δ + gap` around each seam point.
- (d) Determinism: on seam_gap 0 slices exported with the keep-placeholders and dump env vars, `diw_translate --plan-records` reproduces the exported blocks byte for byte.
- (e) Section-aware diff of the translated native output against translated v2 output: byte-identical outside the S4 bodies and the FB lines.

**T3 Clearance invariant**, on every block of T1, T2, T5 and T9. Exact segment-to-segment distances in double, in Python, independent of Clipper:
- record values: for every RUN segment and point dwell, `min over text walls, native closed loops and text paste of (dist − w/2) ≥ δ + POWDER_GUARD_MM − 10 nm`;
- after rounding the run points to 3 decimals: `≥ δ`;
- every RUN point lies inside S;
- report the minimum slack per part, and v2's own minimum next to it (informational).

**T4 Perturbation.**
- (a) Exact shift by (+7.000, −3.000) mm: the output equals the shifted original within 1e-9 mm, with the same nc and row counts.
- (b) 10 seeds of ±1..10 nm jitter per coordinate, plus a 1e-9 rad rotation about the bbox centre:
  - no exception, no NaN;
  - `st` and `nc` equal; areas relative difference ≤ 1e-6;
  - Hausdorff(Ω) ≤ |shift| + 20 nm; run Hausdorff ≤ |shift| + 50 nm, with D5 components allowed ±1 row;
  - T3 holds.
- (c) A 90° input rotation with hatch_angle +90 gives the same run set, rotated, within 1 µm.
- (d) End to end: move a part by 1 µm and by 10 µm on the plate. k/z/h/F are unchanged; the geometry translated back agrees within 1.5 µm.

**T5 Seam gap 15 %** (cube25, ring, notch, lshape). The gap is 0.09 mm at a 0.6 mm nozzle, above LOOP_CLOSE_TOL.
- (i) Assert that v2 gives `st=open` (or no plan) on at least one core layer, which proves the test reaches v2's weakness.
- (ii) Native gives `st ∈ {ok, small}` on every core layer, and the k set equals the seam_gap 0 slice's.
- (iii) **Reported metric, not an assertion**: native 15 % vs native 0 %:
  - ca, oa, ra relative difference;
  - Ω and run Hausdorff;
  - which text-derived inputs (paste, sparse, chunking) changed.
  Expected ≤ 1e-6 relative and ≤ 0.5 µm.
- (iv) Native at 15 % vs v2 at 0 % meets the T2(c) tolerances.
- (v) T3 holds, and the translator accepts both files.

**T6 Fault injection.** For each `ORCA_DIW_SHADOW_FAULT=capture|plan|binder`:
- the export succeeds and the warning is present;
- no `;DIWP` line remains;
- `strip_records` output is byte-identical to the kill-switch export.

**T7 Non-interference.** Shadow on vs kill switch:
- `strip_records` bytes are identical;
- the GCodeProcessor estimated time and filament statistics are identical;
- the `path_uses_arcs` and `axis_units` refactors change no byte: the kill-switch export equals the pre-increment build on the corpus.

**T8 Binder unit streams** (Catch2, on synthetic text). Each case is checked against `split_layers` on the released text and must be accepted by `diw_translate_gcode` in powder mode:
- writes split mid-line and mid-chunk; no trailing newline;
- an empty marker merged into a core layer (block relocated after chunk.start);
- an arc-only layer; G91 in custom G-code; `;LAYER:3` inside layer_change_gcode;
- a trailing empty marker (the previous chunk is not final); CONFIG_BLOCK_START;
- two m=1 placeholders in one chunk; a lost placeholder;
- h <= 0 (carrier accepted, warning emitted); policy=error (the translator throws v2's message);
- `sync()` with held text; CanceledException propagates.

**T9 Planner unit tests** (Catch2 `test_diw_planner.cpp`):
- `tol` gives steps_per_360 = 192 for r ∈ {0.2, 0.21, 0.6, 2.65} mm;
- the padded round offset contains 10 000 samples of the exact disk sum;
- hatch hand cases: a square; a square with a hole (connector across the hole rejected); 30° rotation; a row on a horizontal top or bottom edge; odd-row reversal;
- `covered`: a connector along the boundary, a tangent touch, an exit and re-entry;
- centroid with holes; InteriorPointArea on a C shape;
- the FB split against 1e5-point brute-force sampling;
- the ring predicate on an overhang-first inner loop and on an `inset_idx −1` thin wall;
- the LineMerger port against shapely on 50 random fragment sets (fixtures).

**T10 Performance.** Export time with the shadow on is ≤ 1.2× the kill-switch time on cube25 and cone. At most one chunk is held at a time (debug counter).

---

## 11. Build impact

- `GCode.hpp` changes, so every translation unit that includes it recompiles: GCode.cpp, Print.cpp, PrintObject.cpp, GCode/*.cpp and several GUI files (BackgroundSlicingProcess, Plater, GCodeViewer, ...). Estimate 10-15 min incremental on this machine, against 55 min for a `PrintConfig.hpp` rebuild.
- `GCodeWriter.hpp` (axis_units helper) recompiles its includers, which mostly overlap with GCode.hpp's.
- The DIW* .cpp files, `diw_translate` and `tests/libslic3r` are relinked.
- No new third-party dependencies. Clipper2 is already vendored (`deps_src/clipper2`).
- `PrintConfig.hpp` is untouched. No new config keys; only env vars.
- Keep the GCode.hpp edits in **one** stage (stage 4), so the expensive recompile happens once.

---

## 12. Staged implementation order

Every stage ends with a checkpoint that must pass before the next stage starts.

| Stage | Work | Checkpoint |
|---|---|---|
| 1 | `GCodeScanner` / `ChunkTracker` refactor; export `truthy`; `diw_read_translator_params`. | T0: all four harnesses at 0 failures with the same counts. |
| 2 | `DIWPowderPlanner` P0-P8 + hatch, coverage, centroid, InteriorPointArea, FB split, LineMerger; `diw_translate --plan-json`; `v2_hooks.capture_geometry`. | T9 unit tests; T1 on the corpus and synthetics; T3 and T4(a-c) on T1 outputs. |
| 3 | `DIWPowderShadow` binder (text-only adapter first) + `--plan-records`. | T8 streams; offline `--plan-records` on the corpus: translator accepts all blocks (T2(b)-style), BEGIN fields string-equal to v2's (T2(c) header part). |
| 4 | GCode.hpp/.cpp hooks H1-H5, `m_diw_xy_offset`, `axis_units`, `path_uses_arcs`; capture with native rings and walls; refusals; warnings; fault hooks. One rebuild. | T7 (kill switch equals the pre-increment build); T2(a)(b) on cube25 and two_objects; T6. |
| 5 | Full corpus slicing in `shadow_export.py` and `planner_meshes.py`. | T2(a-e) on the whole 10.1 matrix with the disagreement counter at 0; T3; T4(d); T10. |
| 6 | Seam-gap and stress variants (sequential, multi-height, PE, fan mover, scarf, arcs, policy=error). | T5 (i, ii, iv, v asserted; iii reported); T2 on these variants; update POWDER_PLAN.md with an "Increment 4 results" section and the D1-D8 list. |

---

## 13. Remaining risks to watch during implementation

- **Instance selection drift** between capture and the 7334-7350 loop: the frame self-check (gate 9) catches it, and it shows up in T2 as `frame` counts that must be 0.
- **GCodeProcessor sees text one layer late.** The only mid-export read is at 4582 (with `sync()`). Any future mid-loop processor read in Orca must call `diw_sync()` first.
- **Post-processing.** `GCodeProcessor::finalize` post-processing (M73, pre-heat) must never insert a G0-G3 between a start marker and a block. T2(b) guards this on every slice and printer profile in the corpus.
- **Lost placeholders.** A future pipeline filter that drops comment lines would lose placeholders. The binder counts `lost`, and the T2 matrix includes PE, FanMover, cooling and PA.
- **Classification disagreements** (for example sparse infill that is only arc-fitted, or roles inherited from custom G-code through a modal `;TYPE:`) are reported and not converted in shadow mode. Increment 5 must resolve them before native classification becomes the authority.
  **Closed by increment 5 (19 Nov 2026, [PRODUCT_DESIGN.md](PRODUCT_DESIGN.md) 3.2-3.4).** The native classification is the authority in product mode, and each class has a defined outcome:
  - `text_only`: no block, a warning.
  - `native_only` (a): a block.
  - `native_only` (b): no block, a warning, counted `native_only_nosparse`.
  - Final layer: a warning.
  - Arcs / G91 in a native core layer: the export fails (`arc_g91`).
  - A sparse-typed extrusion outside the walls: the export fails (`orphan`).
  - Objects that disagree on one layer: not core, as in v2.

  The arc-fitted case had already been fixed in increment 4. Measured (POWDER_PLAN.md "Increment 5 results"): the disagreement counter is 0 on all 228 matrix / stress / T5 / coverage slices (G2), and G4 reproduces each class on purpose-built plates. `text_only` and `native_only` (a) are reproduced in product and shadow mode; `tall_short` and `final_core` agree with v2. `native_only` (b) could not be reached by a slice and is covered by the Catch2 T12 stream.
- **Lost placeholders, frame, sync** (above): still counted the same way in shadow mode. In product mode, increment 5 makes `lost`, `lost_placeholder`, `frame`, `sync` and the other internal reasons stop the export with an "internal error" line (PRODUCT_DESIGN.md 5.1). The G5 fault injections test this, and none occurs on the 228 slices.
- **ra tolerances.** The ra tolerance may need tuning on sharp-tip parts (notch, cone) because of D4 and D7. Tune only with a logged justification.

---

## Deviations

- **Stage 1, Python isolation flag.** Section 10 says every Python run uses `python -I`. `-I` drops the script's
  own directory from `sys.path`, so the existing harnesses cannot import their sibling modules (`v2_blocks`,
  `v2_hooks`, `diw_cases`) and fail at import. T0 is therefore run with `py -3.15 -E -s` (environment variables
  and user site-packages ignored, script directory kept), which is the closest isolation that still works.
  New harnesses should do the same, or insert their own directory into `sys.path` explicitly.
- **Stage 1, `ChunkTracker` interface details.** Besides the members listed in section 2.2, `ChunkTracker` has
  `finish(size_t end)` (closes the open chunk; `split_layers` passes `cfg_start`) and `n_opened()`. Closed chunks
  are queued, so a missed `take_closed()` cannot lose one. While a chunk is open, `current()->end` is one past
  the last line fed (the old code pre-set it to `cfg_start`, which a streaming caller does not know); the
  closed chunks are unchanged.
- **Stage 2, dilations and erosion (sections 2.1, 7.1, P2-P4).** The band, the paste exclusion and the erosion
  are not `ClipperOffset` calls with a chord-padded radius. They are the NonZero union of exact pieces: one
  rectangle per segment (extended by 1 nm at both ends), one circumscribed circular sector per convex vertex and
  per end cap (1024 steps per 360°, edges tangent to the circle), with the radius padded by 1 grid unit for vertex
  rounding (`add_sausage`, `round_offset`, `round_erode`). The result always contains the exact Minkowski disk
  sum (T9 samples it), straight parts sit within 1 nm of it and arcs within r(1/cos(π/1024) − 1) (0.003 µm at
  0.6 mm), so D7's "Ω_native ⊆ Ω_v2 ⊕ 0.3 µm" holds with margin. Reasons: the guarantee does not depend on the
  internals of Orca's patched `DoRound` (last-step size, truncation of intersection coordinates), and Ω = C minus
  the dilated boundary of C is an erosion that is contained in the exact one by construction. Clipper2
  `ClipperOffset` is still used, as designed, for the 100-unit opening margin and the fallback clip pad.
  `round_offset` therefore has the signature `ExPolygons round_offset(const Polylines&, double r_mm, OffsetEnd)`
  instead of returning `Paths64`; `clipper_steps_per_360` reproduces the design's `tol()` check for T9.
- **Stage 2, local frame.** P0-P5 run on the inputs translated by an integer origin (bounding-box corner minus
  10 mm) and the results are translated back, so an input shifted by whole grid units gives an exactly shifted
  output (T4(a) at 1e-9 mm).
- **Stage 2, `PlanInput::open_outer`.** The open outer / overhang walls (mm, as parsed) are a field of
  `PlanInput`, and P1b (snap + LineMerger) runs inside `plan_powder_layer_native`, so `--plan-json` takes the same
  text walls v2 sees. `PlanDebug` also carries per-component hatch data (`rows`, `H`, `runs`, `point`, `total`)
  for T1.
- **Stage 2, mitre opening (P5).** `opening` is not `ClipperOffset(JoinType::Miter, 5.0)`. It is
  `mitre_dilate(Ω, D, 5.0)`, a port of GEOS's *limited* mitre (a mitre longer than 5·D is cut square to the
  bisector at 5·D from the vertex), built as the union of Ω, one rectangle per edge and one mitre wedge per convex
  vertex. Clipper2 squares such corners at 1·D, which is discontinuous in the corner angle; that is the design's
  D4, which therefore no longer exists (the 15°/22° tip synthetics now revert exactly what v2 reverts). Ω is first
  simplified by Douglas-Peucker with 10 nm: the union of grid-rounded erosion pieces leaves edges of a few nm
  whose normals point anywhere, and a mitre between two of them grows a spike of up to 5·D (ra jumped by up to
  0.2 mm² under the T4(b) nm jitter). The simplification only affects `narrow`/`revert`, by ≤ 10 nm.
- **Stage 2, narrow is taken of C opened by 100 nm (P5).** `narrow = (C ∩ open(C, 100 nm)) − opening_m`
  (Clipper2 offsets −100 / +100 units with mitre joins, limit 2; round joins at this radius with the design's
  tight arc tolerance made a single case take minutes), i.e. strips of C narrower than 0.2 µm never reach
  narrow / revert. ca and Ω use C itself, as v2 does (opening C itself removed the sub-0.2 µm crescents between
  nearly tangent circular bands on the cone and cost up to 1.7e-3 mm² of ca, above the T1 tolerance). Wall bands that are tangent in the G-code (wall spacing equal to the width, as in the lshape corpus
  file: outer band edge and inner band edge both at y = 56.4) leave a zero-width seam; nm noise opened it into a
  6 nm strip that linked the corner narrow parts with the main one, and revert jumped from 2.33 to 2.54 mm² (T4(b)).
  100 nm is v2's own noise margin (`opening.buffer(1e-4)`). Only the reverted (paste) region changes, so the
  clearance argument of 7.4 is unchanged.
- **Stage 2, `diw_planner_tests`.** Besides `tests/libslic3r/CMakeLists.txt` (with `BUILD_TESTS`), the T9 file
  is built stand-alone as the dev-utils target `diw_planner_tests` (Catch2 from `tests/catch2`), because this
  build tree is configured without `BUILD_TESTS`. Run it from `build\src\Release` (DLLs).
- **Stage 2, T1 / T4 tolerances in `compare_planner.py`.** Where the design's flat tolerances are below the
  conditioning of the quantity itself, the harness uses a derived bound and logs every use of it:
  - *Cavity Hausdorff.* C contains cusp slivers between tangent wall bands (e.g. cube corners: the outer band's
    sharp inner corner against the inner wall's round corner). A normal boundary difference ε moves a cusp tip
    tangentially by √(2rε); v2's own 0.1-0.2 µm chord error gives 10-600 µm there. ∂C is compared after a
    morphological opening by 10 µm of both sides (the raw value is reported as `haus_C_raw`). Ω, which has no
    cusps, is compared raw at 2 µm, and the D7 containment is checked raw.
  - *v2 discretization arbitration.* A failed areas/shape/hatch/fb check is re-run against v2 itself with
    `UMEX_QUAD_SEGS = 256` (module global patched for the call; v2 unmodified on disk). If it passes there, the
    difference is v2's 32-segment arcs and is logged (`v2_discretization`), not failed.
  - *ra.* The GEOS mitre opening is ill-conditioned at acute Ω corners: v2's own ra does not converge in
    `quad_segs` (e.g. the L-with-paste synthetic: 19.600, 19.666, 19.656, 19.684, 19.667, 19.664 mm² at
    32...1024). If v2's ra spread over quad_segs 32...1024 exceeds the design tolerance, native ra inside that
    span (± tolerance) is accepted and logged (`ra_illcond`); the same spread bounds ra in T4(b)/(c), and the
    fallback-path points of such a case (clipped by that revert) are logged instead of compared.
  - *Point dwells.* d is proportional to the component area and the dwell sits at the centroid. With the
    measured component Hausdorff h (≤ 2 µm, checked), perimeter P, area A and radius R, the bounds are
    |Δd|/d ≤ 1e-5 + P·h/A and |Δpos| ≤ max(2 µm, 2·P·h·R/A) (first-order area / centroid perturbation); for
    small components this is looser than the flat 1e-5 / 2 µm and is logged.
  - *T4(b).* The area bound is 1e-6·value + boundary length × (|shift| + 20 nm) (a 10 nm shift of a boundary of
    length L changes the area by up to L·10 nm, far above 1e-6 relative); the Ω Hausdorff bound is multiplied by
    1/sin(α/2) for the smallest convex Ω corner angle α (a corner moves by shift/sin(α/2); factor capped at 20);
    a run Hausdorff above |shift| + 50 nm is accepted where the offending run end lies on a row meeting ∂Ω at a
    grazing angle θ, within (|shift| + 50 nm)/sin θ (sin θ floored at 0.05).
  - *T4(c).* Areas within 1e-5 relative (the rotated input is re-gridded by the 90° map, so pieces round
    differently), runs paired as a set (component order follows the rotated bounding boxes).
  - *Synthetics.* They are placed at +40 mm on the bed (not at the origin), and one extra synthetic, an L with a
    gap-infill and a solid-infill paste path, exercises E and the revert/fallback path.
- **Stage 2, `planner_meshes.py`.** Written in stage 2 (it is only used from stage 5 on). The slot width
  2·d_eff + 2 µm depends on δ, which section 10.1 does not fix: the script takes `--delta` (default 0.4 mm, slot
  0.804 mm) and is run once per δ of the matrix. The notch tip is 12.5 mm deep (half the cube); the overhang wedge
  is 20 mm long with a 10 mm base and a 45° side up to 10 mm.
- **Stage 3, `PowderShadow` interface.** The binder is built before the GCode hooks (stage 4), so the class is
  constructed directly as `PowderShadow(const TranslatorParams&, ShadowOptions)`; `create(const Print&, ...)` and
  `capture(...)` are added in stage 4 on top of it. Besides section 2.1 it has `next_id()`, `placeholder(id, m)`,
  `put_native(id, NativeLayer)` (what `capture` stores), `load_dump_dir(dir)` (offline reload), `feed(std::string_view)`
  next to `feed(const char*)`, and `stats()`. `ShadowOptions` carries `text_only`, `keep_placeholders`, `dump_dir`,
  `fault` (the three env vars, `ShadowOptions::from_env()`), `xy_decimals` (frame tolerance) and
  `throw_if_canceled` (called once per core chunk before planning; stage 4 passes the print's cancel check).
  `NativeLayer` = `{native_core, rings (Polygons per instance), pieces, open_outer (mm), error}` as in section 5.2.
- **Stage 3, binder details not fixed by section 6.**
  - The config-block test is "the line contains `; CONFIG_BLOCK_START`" (what `split_layers` and v2 use), not
    "starts with".
  - Released text is buffered and handed to the sink once per `feed` / `sync` / `finish` call, in order.
  - On an exception the held lines are released before the line being processed (order preserved), then
    disabled mode applies (section 6.2).
  - After `sync()` the open chunk's later lines pass through at once; its placeholders' captures are still
    consumed, and the chunk is counted under `sync` when it had an m=1 placeholder. A placeholder whose capture
    is never consumed is counted once as `lost_placeholder` (no k) at `finish()`.
  - Gate 0 does not apply in `text_only` mode: a chunk without placeholders gets `native_core = text_core` and the
    text-only adapter (no gates 4, 5, 9). A chunk with placeholders whose capture is missing falls back to the
    adapter in `text_only` mode (counted as `lost` otherwise).
  - The ORCA_DIW_SHADOW_DUMP file `k<k>.json` holds `k`, `id`, `adapter`, `native` (the NativeLayer, or null),
    `input` (h, s_eff, open_outer, text paths), `debug` (WKT and notes) and `record`; mm values are Python repr
    strings, so `load_dump_dir` restores them bit for bit.
- **Stage 3, `--plan-records` command line.** `diw_translate [--config-from-gcode] --plan-records [--feed-size N]
  <in> <out> [key=value ...]`: the settings come as in the translate mode (needed for U and fp). Existing `;DIW:`
  lines are removed first (as the translator does). The binder is fed in N-byte pieces (default 65536);
  summary and warnings go to stderr. The frame tolerance uses 4 decimals when `high_precision_gcode` is set.
- **Stage 3, tests.** T8 is `tests/libslic3r/test_diw_shadow.cpp` (8 Catch2 cases, also built into
  `diw_planner_tests`; it builds its own synthetic G-code and captures). The offline checkpoint is a new harness
  `diw/tests/compare_replay.py` (corpus x compare_records variants + notch): `--plan-records` exit 0, 13-byte
  feeds byte-identical, `strip_records` gives the source back, records parse, placement at v2's
  `LayerChunk.start + 1`, core set equal to v2's planned set, BEGIN `k z h s F fu fp` and `st` string-equal, and
  the translator accepts the file (with and without `--stub-powder-stage`). `shadow_export.py` (T2) follows in
  stage 5.
- **Stage 4, capture interface.** `GCode::LayerToPrint` is a nested type of `GCode` and cannot be forward-declared
  in `DIWPowderShadow.hpp`, so `capture` takes `std::vector<DIW::CaptureLayer>` (`object_layer`, `support_layer`,
  `original_object`), which `process_layer` fills from its `layers`. `ShadowFrame` carries, besides the three
  fields of section 2.1, a `to_grid` callback that `process_layer` sets to `GCode::text_grid_nm`, so the frame
  transform is compiled next to `point_to_gcode` as section 5.3 asks (the capture has the same transform as a
  fallback for an empty callback). `PowderShadow` gained an owning constructor
  (`std::unique_ptr<TranslatorParams>`), which `create` uses.
- **Stage 4, refusal inputs.** `create` runs right after the output stream is opened (H1), before `_do_export` sets
  up `m_spiral_vase` and the writer's axis remap for this export. Spiral vase and axis remap are therefore taken
  from the config (`spiral_mode`, `BeltTransformPipeline::axis_remap_enabled`), the same conditions `_do_export`
  uses. A `DIWConfigError` from `diw_read_translator_params` refuses silently (no warning; the translator reports
  it). Every warning is also written to the log at warning level (the CLI reports print warnings only before the
  export), and the refusal reason at info level.
- **Stage 4, capture details.**
  - Instance set: in non-sequential mode every instance of `original_object` (what the island-level loop prints;
    `ordering` lists every instance), in sequential mode `instances()[single_object_instance_idx]`. Gate 9 checks it.
  - A loop whose paths all have wall roles and one width, and that is connected, is stored as the closed piece
    only (its Joined dilation contains the per-path Round dilations, so the band is the same); other loops are
    stored per wall path.
  - An exception before the classification writes `m=1` (with the error), so a core layer reports `capture` and
    not a disagreement. The `capture` fault is thrown after the classification, on every layer.
  - In passthrough (config block seen, disabled mode) a placeholder's capture is consumed with the line, so a
    disabled binder does not also report lost placeholders.
- **Stage 4, tests.** `diw/tests/shadow_export.py` is started with the stage-4 subset (cube25 and two_objects; T7,
  T2(a)(b), T6); stage 5 extends it to the 10.1 matrix. The slicing profiles are copied into `diw/tests/profiles/`
  and `planner_meshes.py` also writes `two_a.stl` / `two_b.stl` (the two_objects plate); the Orca datadir is a
  command-line argument. T7's "pre-increment build" is the stage-3 build (stages 1-3 do not touch the G-code
  export) and is passed as `--pre` (made with `--make-pre` before the stage-4 edits). The "; generated by ... at
  <time>" header line is blanked before byte comparisons.
  The CLI writes its log only to `--logfile`, so the harness slices with `--debug 3 --logfile <run>.orca.log` and
  looks for the planner warnings and the summary line there.
- **Stage 5, frame self-check tolerance (gate 9).** The tolerance is `10^-d * sqrt(2) + 1 nm` (d = 3, or 4 with
  `high_precision_gcode`), not half of that. A printed vertex is rounded to the text grid (up to
  `0.5 * 10^-d * sqrt(2)`), and so are the ends of the native segment it is compared with; a seam split point lies on
  the exact segment, so it can be off the rounded one by both roundings. With the half tolerance the matrix reported
  spurious `frame` refusals on seam splits.
- **Stage 5, planner booleans and P3/P4 evaluation order (performance, for T10).**
  - The NonZero / EvenOdd unions, differences and intersections of P1-P5 call Clipper2 (`Clipper64`, collinear
    vertices removed, PolyTree to ExPolygons) instead of the Clipper 1 based `union_ex` / `diff_ex`. Both are exact
    on the integer grid.
  - The band pieces are united per polyline (one group per wall / paste path), in parallel with TBB when the
    layer has at least 2000 piece vertices, and the group results are then united. Each group result has winding
    0 / 1, so the NonZero union of the results is the union of all pieces; the group order is fixed, so the result
    is deterministic.
  - P4 no longer erodes C. Since C = S - X with X the band and paste set, C eroded by D equals
    (S eroded by D) - (X dilated by D), and X dilated by D is the centrelines dilated by w/2 + D. Both sides are
    built from the same conservative pieces as before, so Omega stays inside the exact erosion of the exact C.
    When every ring edge is a band centreline segment (always so for native rings), S eroded by D is not computed,
    because S - (X dilated by w/2 + D) is the same set. `round_erode(C, D)` cost 0.2-0.8 s per layer on curved
    parts (one piece per vertex of the fine band arcs). P3 and P4 run in parallel on large layers.
  - On small layers the mitre opening of Omega comes first. If no component of `C - opening_m` reaches
    `paste_min_area`, revert is empty and the 100 nm opening of C is skipped (unless debug geometry is requested).
  - Test-only: `ORCA_DIW_PLAN_TIMING` adds per-phase timings to `PlanDebug::notes`. The dump's `debug` object also
    carries the per-component hatch data (`components`).
  T0, T9 (512 assertions) and the stage-5 matrix were rerun on this build.
- **Stage 5, binder statistics.** The summary line ends with `; max_held=<n> capture_ms=<ms> binder_ms=<ms>`.
  `max_held` counts the distinct chunks whose lines are held at the same time (the T10 debug counter; 1 by
  construction), and the two timers give the planner's own share of the export time.
- **Stage 5, matrix shape (`shadow_export.py --matrix`).** Section 10.1 lists the factors but not how to combine them.
  The full product (11 meshes x 2 wall generators x 5 hatch angles x 3 deltas) would be 330 slices per run. The
  matrix is one factor at a time instead:
  - every single mesh (cube25, ring, lshape, notch, cone, slot, overhang, and the repo's cube_with_hole,
    cube_with_concave_hole, overhang.obj and bridge.obj) under classic and Arachne walls, each with base,
    hatch_angle 30 / 45 / 90 / -30 and delta 0.2 / 0.4 / 0.6;
  - the two_objects plate and two lshape instances under both wall generators;
  - supports on (overhang, overhang.obj, bridge.obj), a prime tower (cube25, two_objects),
    `detect_overhang_wall` off (overhang, overhang.obj; the profile has it on), and the clamp and no-overflow
    f_max/policy variants (cube25, cone).
  That gives 191 slices. The base delta is the powder profile's own `diw_shell_powder_clearance` (0.85 mm). The slot
  mesh is regenerated for each delta, so that its width stays 2·d_eff + 2 µm. At their own size, the repo meshes
  get walls and skins only with the 1.4 mm nozzle and 0.7 mm layers, so they are scaled with the CLI `--scale`
  (cube_with_hole and cube_with_concave_hole x3, bridge x2). Seam gap 15 %, sequential printing, multi-height,
  PE / fan / cooling, arcs, scarf and policy=error are left to stage 6, as section 12 assigns them.
- **Stage 5, step 5a (`--matrix --slice-only`).** The slicing pass runs before the T2 / T3 comparisons, as a separate
  mode. Each slice is exported once with the shadow on. The checks are:
  - the export exits 0;
  - every override is in the config block;
  - no `;DIWP` line is left;
  - the records parse, and every BEGIN directly follows a start-marker line;
  - the planner summary is clean (no disagreement, lost, capture, frame, plan, record, sync or merged count;
    max_held <= 1; blocks + carriers equal the blocks in the file);
  - native blocks sit exactly on v2's own text-core layers of the same file (`scan_gcode` / `split_layers` /
    `classify_layers`, with a missing k allowed only where the planner logged D6 / D8), and every slice has at
    least one;
  - `diw_translate --config-from-gcode` exits 0.
- **Stage 5, step 5a, prime tower variant.** OrcaSlicer drops a single-filament prime tower in `normalize_fdm_2`
  unless the smooth timelapse is on, so the tower variant sets `timelapse_type = 1` and relative E (with `G92 E0` in
  the layer change G-code, which `Print::validate` requires). With the default `wipe_tower_type = type2` that print
  crashes in upstream code: `WipeTower2::prime` reads `wipe_volumes[tools[idx_tool - 1]]` at `idx_tool = 0` when
  there is one tool (access violation at WipeTower2.cpp:1292, also with `ORCA_DIW_NO_NATIVE_PLAN=1`). The wipe tower
  is not touched (section 3), so the variant uses `wipe_tower_type = type1`. That writer tags the tower
  `; FEATURE: Prime tower`, which v2's `;TYPE:` scanner does not see, so the tower moves inherited the previous
  layer's last `;TYPE:` (sparse infill) and made the top tower + walls + bridge layer a text-only core layer
  (`disagree_text_only = 1`). The tower is printed right after the layer change G-code, so the variant's
  `layer_change_gcode` ends with `;TYPE:Prime tower`; no object extrusion carries that type afterwards (checked on
  both tower exports). This is a test-profile setting only.
- **Stage 5, step 5b (T2 on the matrix, `shadow_export.py --matrix --reuse <5a work dir>`).** The "on" run of each
  slice is the stage-5a export (copied with its log); kill and keep are exported fresh and compared against it, so
  T2(a) also checks that the 5a export is reproduced. T2(c) arbitration gained two rules; both are logged and counted
  in the summary statistics:
  - *v2 at finer arcs.* When the native block fails against v2 and v2's Omega is stable over quad_segs 32-1024
    (spread <= 2 um), v2 is also run at every quad_segs above its default (64-1024; text band first, then the
    B_text U B_native band) and the block passes when it matches one of them (`v2_fine_quad`). Case:
    bridge_obj.arachne.angle45 k10, where v2 splits one hatch run at a chord vertex of its Omega boundary up to
    quad_segs 512 and joins it at 1024, where it equals the native run set.
  - *Tight excess over an ill-conditioned v2.* In the exact-clearance judgement (v2's Omega moves by more than 2 um
    between its discretizations), oa may exceed v2's whole spread by the area of native Omega outside v2's Omega
    (+0.3 um), provided the native boundary along every such region has exact slack <= 2 um, i.e. native lies
    within the shape tolerance of the exact Omega there and the excess is v2's own under-approximation
    (`omega_tight_excess`). Case: bridge_obj.arachne k11 (all variants): v2's Omega drops a 1.85 mm2 region at
    quad_segs >= 256 (>= 512 at delta 0.4); at 32-128 one edge ends on an arc chord vertex short of the exact
    corner, so it is tilted by 0..2 um over 5 mm (0.0045 mm2), while the native edge keeps a uniform 2 um slack.
  - T2(e): the header line `; WARNING (uMEX): <n> powder run(s) in layers ...` summarizes the S4 run lengths, so it
    may differ when T2(c) accepted a different run count on some layer (only then; the rest of the header must be
    identical). Logged per slice.
- **Stage 5, step 5c (T3, T4(d), T10 on the full corpus; planner speed-up for T10).** On the stage-5b build T3 and
  T4(d) passed, but T10 failed (cube25 1.23-1.32x, cone 1.24-1.29x of the kill-switch export time; almost all of
  `binder_ms` was the planner's P3 / P4 unions). The geometry construction was changed; the guarantees of the
  stage-2 entry above (the result contains the exact disk sum, arcs circumscribed at 1024 steps per 360°) are kept:
  - *Raw offset paths.* Each band / paste centreline and each eroded-set ring is dilated on its own as one Clipper2
    `Positive` union of its raw offset path(s), the way `ClipperOffset` builds a round offset: per segment the edge
    offset at the padded radius, at a convex vertex the circumscribed vertices of the stage-2 sector, at a concave
    vertex Clipper2's three-point join, circumscribed half-disc caps at open ends, and at a (near) reversal
    (cos <= -0.999) the cap arc on both sides. The polyline results are united afterwards (in parallel, fixed
    order). The set is the union of the stage-2 pieces with the same arc vertices, without the 1 nm segment-end
    extension. The P5 mitre opening is built the same way (raw mitre path per ring, GEOS limit). The stage-2
    constructions remain as `round_offset_pieces` / `mitre_dilate_pieces`, and a T9 case compares both on 1800
    random polylines (nm segments, exact reversals) and 600 random polygons.
  - *Global angle grid.* The circumscribed arc vertices lie on one global grid of tangent directions
    (`circumscribed_arc`), so neighbouring arcs share vertex directions.
  - *Arc caps outside S.* An arc cap of a band dilation (sweep <= pi) whose circumscribed polygon has no point in S
    is replaced by its chord (`OutsideTest`). Only S - band is used, which is unchanged, so this is exact for C,
    ca and Omega. It drops most arc vertices on the outer side of the outer walls.
  - *Duplicate closed curves.* A closed polyline that is the same vertex cycle (collinear vertices removed) as
    another with a radius at least as large is dropped before the union; its dilation is nested in the other one.
    This removes the G-code text copy of a native wall loop when the text reproduces it exactly.
  - P5 runs the 100 nm opening of C and the mitre opening in parallel only when debug geometry is requested;
    otherwise it is serial with the revert-empty skip of the stage-5 performance entry.
  - Tried and rejected: `SECTOR_STEPS_PER_360` = 256 or 512 (T1 then fails the T4(b) slot Omega Hausdorff bound and
    ra / oa / hatch on the notch and the L-with-paste synthetic), `ClipperOffset` round joins (straight edges move
    out by r(1/cos - 1)), one union of all raw paths, and a memo of the previous layer's plan (never hits).
  - Records differ from the stage-5b build only at the 1e-9 mm2 level in ca / oa (Clipper vertex rounding).
  - Harness: `shadow_export.py --only T10` runs T10 alone. T10 is a wall-clock ratio on a 4-core laptop that also
    runs other programs; it passes on an idle machine and was measured repeatedly (best of 5 per run). On the build before the duplicate-curve step, one run while
    the machine was loaded gave cube25 1.287x (others 1.06-1.18x); the two runs on the final build gave 1.07-1.14x.
  Final build results: T9 515 assertions / 22 cases; T1 3443 checks, 0 failures; T0 green; 5a 1915 / 0 with 3153
  native blocks; T2 + T3 25323 checks, 0 failures (T3 3153 / 0, min slack -2.6e-7 mm against the -10 nm
  tolerance); T4(d) 40 / 0; T10 cube25 1.103x, cone 1.137x, max_held 1.

### Stage 6, step 6a (T5, seam gap 15 %)
- **A DIW printer cannot export a seam gap.** `apply_diw_overrides` (PrintApply.cpp, milestone 2) forces `seam_gap` to
  0 on a DIW printer, and `Print::validate` refuses a per-object seam gap, so a 15 % slice came out at 0 % and v2
  planned every layer. Closest faithful alternative: a test-only environment variable,
  `ORCA_DIW_TEST_KEEP_SEAM_GAP=1`, which skips both (new `diw_test_keep_seam_gap()` in PrintApply.cpp, declared in
  Print.cpp; no header or config key changes). Without it nothing changes; the T5 harness sets it for both of its
  exports (a no-op at 0 %). The same overrides also force arc fitting, cooling slowdown and the loop wipes off, which
  matters for the 10.1 stress variants of step 6b.
- **Harness.** `shadow_export.py --matrix --t5` (`t5_job`): cube25, ring, notch and lshape, classic and Arachne walls
  (8 slices), each exported with the shadow on and the dump at 0 % and 15 % (gap 0.21 mm at the 1.4 mm nozzle).
  v2 runs on both texts with the records stripped.
- **T5(i)** is asserted per slice, which is stronger than "on at least one core layer". In every slice v2 gives
  st=open on every core layer (all 168).
- **T5(iii) (reported).** The native inputs (rings, pieces, h, s_eff) are identical at 0 % and 15 %, but the planner's
  text inputs are not:
  - Orca inserts the seam point into the loop's text as an extra vertex, rounded to 3 decimals. At 0 % this puts a
    kink of up to 0.5 um into B_text. At 15 % that segment instead ends gap-short at another rounded point.
  - Concentric solid infill is itself clipped by the seam gap (`Fill` loop_clipping), so the paste changes. This is
    the top layer k34 of the cube and the Arachne notch.
  - Sparse infill also changes on a few layers.
  Results: cube25 and lshape are within the expected 1e-6 / 0.5 um. Ring and notch reach 1.8e-5 relative area, an
  Omega Hausdorff of 3.9 um and a run Hausdorff of 2.1 um. The largest values are at the notch's acute tips, which
  amplify a 0.3 um band shift. Run counts and nc are equal everywhere.
- **T5(iv).** Native 15 % is first compared with v2 0 % (`compare_t2c`). The seam discs have radius w + delta + gap
  around both ends of the outer walls of both texts. When that fails, a further reference candidate,
  `t5_band_model`, is tried:
  - What it is: v2 run on its own LayerModel of the 15 % text, with the walls renamed so they form no ring, plus the
    native pieces. The 0 % outer and overhang walls are added as zero-width paths, so S is the 0 % solid and the
    band and paste are exactly the 15 % block's inputs.
  - Why: v2 cannot plan the 15 % text by itself, so this is v2's algorithm on the native planner's own inputs. It
    works like the D2 native-band rule.
  - Result: it is accepted on 20 layers (logged as `T5 seam-gap text band`, stat `t5_seam_band`), all on the ring
    and the notch.
- **T3 placement.** compare_case's T3 against the 0 % text is dropped from T5(iv). That file's rounded seam vertices
  are not obstacles of the 15 % file; v2's own 0 % runs violate them by up to 0.36 um. T3 is T5(v): every 15 % block
  is checked against its own planner input text (walls, open outer walls, paste) and the native closed loops, with
  the RUN points inside the native solid.
- Result (log scratchpad/planner/s6a_t5d.log): 1141 checks, 0 failures.
  - T5 56/0, T5i 8/0, T5ii 32/0;
  - T5iv 16/0, plus areas, fb, hatch, shape and st_nc at 168/0 each;
  - T5v 184/0 (T3 168 blocks, min slack 4.4e-7 mm; translator accepts all 16 files).
  The k sets at 15 % equal those at 0 %, and every planner summary is clean. T0 is green on the rebuilt build.

### Stage 6, step 6b (stress variants, results)
- **The stress variants were not in the stage-5 matrix.** Section 12 stage 6 runs T2 on them. They are now the
  `shadow_export.py --matrix --stress` mode (list `STRESS`, 16 slices). Each slice goes through the full stage-5
  `slice_job`: kill, on and keep exports, then T2(a-e) and T3. A new check kind, S6, shows that each variant reaches
  what it stresses:
  - the overrides are in the config block;
  - no block sits on an arc or G91 chunk;
  - arcs: G2/G3 on otherwise-core chunks;
  - multi-height: both layer-height grids are printed;
  - sequential: z drops between objects;
  - scarf: a z ramp in the outer wall;
  - cooling: moves at the `slow_down_min_speed` floor.
- **Test-only `ORCA_DIW_TEST_KEEP_FDM=1`.** `apply_diw_overrides` forces arc fitting and the cooling slowdown off on a
  DIW printer, so those variants could not be exported. This is the same situation as the seam gap in step 6a, and
  the closest faithful alternative is the same: a test-only environment variable. The new
  `diw_test_keep_fdm()` in PrintApply.cpp returns from `apply_diw_overrides` after the wipe overrides, so the wipes
  stay off. There is no header or config-key change, and nothing changes when the variable is unset. The harness sets
  it for every export of the arc and PE/fan/cooling slices.
- **Arc predicate fix (C++, GCode.cpp H4).** The capture's arc predicate was `path_uses_arcs()`, which is true for
  every path with an arc-fitting result. A fit made only of linear moves takes the writer's arc branch but prints
  G1 only. With `enable_arc_fitting = sparse_infill`, cube25's rectilinear sparse infill is fitted to lines, so all
  32 core layers were `disagree_text_only`. This is the "sparse infill that is only arc-fitted" risk of section 13.
  The capture lambda now also requires a fitting segment of arc type whose length the writer prints
  (`arc_data.length * SCALING_FACTOR >= EPSILON`). That is the exact G2/G3 condition. The writer site and
  `path_uses_arcs()` are unchanged, and the predicate is a no-op while arc fitting is off, which a DIW printer forces.
- **Objects at two layer heights.** The CLI has no per-object setting except through an assemble list
  (`--load-assemble-list`, per-object `print_params`). The `multi_height` plate puts two_a at 0.2 mm and two_b at
  0.3 mm layer height. The list arranges by itself and refuses CLI transforms, so `slice_part` omits `--arrange`
  for it.
- **Arc plate.** On the ring or the cone alone, every core layer has arcs, so there is no block at all. The
  translator then refuses the powder job ("powder job without blocks"), as it does for v2's records. The arc
  variants therefore use a cube25 + ring plate (`cube_ring`). The 14 ring-height layers have arcs and get no block;
  the 25 layers above the ring get blocks.
- **Scarf reference.** The scarf retraces the outer wall loop with a z ramp. v2's snap + linemerge of the text
  outer wall then forms no ring:
  - st=open on every core layer of cube25 and of the Arachne lshape (`seam_slope_type = all`);
  - the ring is planned without its outer ring (Ω 118.7 instead of 47.9 mm²).
  The native ring is the ExtrusionLoop itself. The native records equal those of the same part without the scarf:
  byte-equal on cube25, and on the ring ca differs by up to 5e-4 mm² through the scarf's extra rounded text
  vertices. v2 on the scarf text is therefore not a valid reference (a D1-like v2 weakness). The reference is v2 on
  the same part exported without the scarf (`seam_slope_type = none`, kill switch), compared like T5(iv):
  - `compare_t2c` uses `t5_band_model` as the fallback candidate and seam discs of radius w + δ;
  - T3 is done against each block's own planner input text and the native loops;
  - T2(e) diffs translate(on) against the scarf kill export with v2's reference blocks inserted after the same k's
    start marker;
  - the k set must equal both v2's reference planned set and v2's text core of the scarf file.
  All 52 layers pass directly against the reference; the band fallback was not needed.
- **policy=error.** v2 aborts the whole file, so T2(c) has no v2 plan to compare and T2(e) has no output to diff.
  The checks are instead:
  - every v2 text-core layer carries the `st=open` carrier, with BEGIN equal to s = s_hatch, F = fu = f_calc > f_max
    and v2's fp;
  - v2's `DIWPostProcessor` and `diw_translate` abort with the same message;
  - that message is the `resolve_powder_feed` error of the first carrier's h;
  - T2(d) replay is byte-equal.
- **Result** (log scratchpad/planner/s6b_stress3.log): 16 slices, 2606 checks, 0 failures.
  - S6 47, T2a 160, T2b 18, T2c 334, T2d 16, T2e 28, T3 333, and areas/fb/hatch/shape/st_nc 333 each.
  - The disagreement counter is 0 and every planner summary is clean (`arc_g91` 14 on each arc plate).
  - Hausdorff: C 1.15 µm, Ω 0.72 µm, runs 0.47 µm. T3 minimum slack 4.4e-7 mm.
  - On the final build (16:38): T0 is green, Catch2 gives 515 assertions in 22 cases, and the 13 `classic.base`
    matrix slices were rerun (1626 checks, 0 failures).

### Review fixes (18 Nov 2026, after increment 4)
An adversarial review of the finished increment confirmed nine defects. Each is fixed and has a regression test.
- **Placeholders are tokenised (sections 4 H4, 6.1, 6.2).** The design drops every `;DIWP ` line and parses any
  `;DIWP <n> m=<0|1>` line as a placeholder. User custom G-code with such a line (layer change, filament or start
  G-code) was then deleted from the export and counted as an m=1 capture (it changed the core decisions). The
  placeholder is now `;DIWP <token> <id> m=<0|1>`. The token is 16 hex digits drawn per export
  (`ShadowOptions::placeholder_token`, random when empty), and only lines with the shadow's own token are
  placeholders. Any other `;DIWP` text is ordinary text and passes through byte for byte. The dump (`k<k>.json`)
  stores the token, and `load_dump_dir` adopts it, so offline replay of a kept-placeholder file still binds.
  Tests: the Catch2 T8 case "placeholder-like custom G-code is ordinary text", and `regress_review.py` R1 (CLI export
  with such lines in `layer_change_gcode`).
- **Single-bead walls are no ring (section 5.2, ring predicate).** `is_ring_loop` accepts every inset-0 loop, which
  includes a wall printed as one bead: an Arachne odd centreline loop, or a classic thin-wall loop. S then held the
  whole disc inside the bead, and powder was planned into the empty bore of a thin tube. v2 does the same with the
  "Outer wall" text. The capture now also requires `bead_bounds_slice`: up to 64 segment midpoints of the loop (or
  of an open outer wall) are probed at 0.75 bead widths on both sides, against the layer's `lslices`. The wall
  bounds the cavity only when at least a quarter of the probes have the slice on exactly one side. A single bead has
  the slice on neither side. The wall stays in the band; only its ring is dropped. The count goes into the summary
  line as `single_bead=<n>`. This is a deliberate divergence from v2 (v2's plan of such a layer is wrong). Tests: the
  Catch2 case "Review fix 2", and `regress_review.py` R2 (a cube plus a thin tube, under Arachne and under classic
  with `detect_thin_wall`).
- **Per-segment widths of text paths (sections 6.5, 7.2 P2/P3).** E and B_text dilate a whole text path by half its
  first-segment `;WIDTH:` (v2's PathRec width). Variable-width gap fill grows inside one contiguous path (0.37 to
  1.93 mm in the review's slices), so runs and point dwells landed inside the printed bead. `parse_core_layer` now also
  records each segment's own `;WIDTH:` (`PathRec::seg_width`, `TextPath::seg_width`). The planner keeps v2's dilation
  and adds every segment that is printed wider than its path's first width, as its own round-capped piece at its own
  half width. So the exclusion is the union of v2's and the true one, and it is never narrower than v2's.
  The harness reference follows: `v2_hooks.capture_geometry` plans v2 on `seg_width_model`, which adds the same wider
  segments to v2's LayerModel as 2-point paste paths. v2's buffer of a 2-point path is exactly that segment's disk
  sum. `T3` uses them too (the dumped `seg_width`). Tests: the Catch2 case "Review fix 3" (it also shows that the old
  rule violates delta), and `regress_review.py` R3, an independent clearance check of every run against each printed
  segment at its own width, read from the text.
- **The D2 / seam-band reference candidates verify the native pieces (`shadow_export.compare_t2c`).** Both candidates
  added the native planner's own captured pieces to v2's band, so any capture error copied itself into the reference.
  A 1.4x width error on a classic slice passed T2 on every layer. Before either candidate is built, each native
  piece is now checked against the printed text (`verify_native_pieces`): every piece segment must lie on a text wall
  segment within gate 9's tolerance, with an equal per-segment `;WIDTH:`. Only the seam tail of a loop may be
  unprinted (2w + gap + 0.1 mm per piece). An unverified layer gets no candidate (`native_pieces_unverified`). The
  review also suggested limiting D2 to Arachne slices. That was tried and is not done: on the matrix all 42 D2 layers
  (ring Arachne k3/k4, notch classic k19/25/27/31 in every variant) differ from v2 because the native closed loop covers
  its own unprinted seam tail, not because of a width, and that happens with classic walls too. The verification is
  the guard instead, and the statistics now split the D2 acceptances into `d2_wider` (a verified piece wider than its
  text path's first-segment width) and `d2_seam_tail` (all 42 on the matrix). Test: `regress_review.py` R4 (the
  classic cube25 dump with widths x1.4, replanned, must fail; the same dump unmodified must pass).
- **T1 ra exception bounded (`compare_planner.compare_case`).** The "narrow near paste_min_area or tip < 23 deg"
  exception accepted any ra and any revert count. Most cube-like cavities have such tips. It now applies only to a
  revert COUNT mismatch, and |dra| must stay within the tolerance + |count difference| x (paste_min_area + 1e-3) +
  v2's own ra spread over quad_segs. Equal counts get the normal ra check. Test: R5 (ra + 50 mm2 on cube25.k3 fails).
- **The ill-conditioned exact-clearance path checks hatch and fb (`compare_t2c`).** The path judged only Omega,
  oa and ca, then turned every failed hatch / fb record into a pass. hatch and fb are now checked against v2's own
  algorithms run on the native geometry (`self_check_hatch_fb`): `hatch_polygon` on each native Omega component,
  with the point dwell at its centroid and v2's mass formula, and v2's fallback clip of the text sparse infill by
  the native revert. Test: R6 (with two thirds of the runs removed on the bridge_obj layer that takes this path,
  hatch fails).
- **The quad_segs 256 arbitration needs the same plan (`compare_arbitrated`).** When v2 at quad_segs 256 has another
  status or component count, the fine run stops at st_nc, so it has no areas / shape / hatch / fb record to fail.
  Every coarse failure was then logged as `v2_discretization` and passed. Such a fine run now arbitrates nothing.
  Test: R7.
- **Statistics of the accepted comparison (`compare_t2c`, `compare_arbitrated`, `compare_fb`).** The reported
  Hausdorff / position maxima are now taken from the comparison that was accepted: the first one if it passes, the
  accepted candidate's own otherwise, the better of coarse / fine per metric when arbitrated, and no Hausdorff from
  the rejected v2 on the exact-clearance path. `fb_pos` is measured on the matched pairs only. Test: R8.
- **D5 exclusion bounded in T2 (`shadow_export`), and D5 components compared (`compare_planner`).** T2(c) never
  asserted the "excluded D5 fraction < 5 %" of T1. The matrix summary now checks it per slice and overall (`D5`
  check kind). The per-slice bound is "< 5 %, or at most one excluded component on a slice with fewer than 20",
  because there a single exclusion is already 5 % or more. The first matrix run with the bound failed on 4 slices
  (cube25 and notch, Arachne, hatch angle +-30): 29 of 32 components were D5, because every core layer has the same
  Omega and its H/s is integral at those angles. So the hatch of those slices had never been compared. A D5
  component is now compared without its edge rows instead of being skipped: both run sets are clipped, in the hatch
  frame, to the band half a spacing inside the two shells' common y range, and the clipped sets must agree within
  the 2 um Hausdorff. Point dwells are compared as usual. Only a component with nothing left inside that band (or a
  point / line mismatch) counts as excluded. Results: T1 d5_excluded 0 (2 compared that way), T2 matrix d5_excluded
  0 (117 compared that way). Test: R9, and the D5 checks of the T1 and T2 runs.
- **Results on the final build (22:07, `SLICER EXIT CODE: 0`).** T0 green (157/157/486, powder and paste 0
  failures); Catch2 3489 assertions in 25 cases; T1 3443 / 0; replay 380 / 0; stage-4 checkpoint 49 / 0; 5a 1915 / 0;
  T2 + T3 25515 / 0 (D5 192 / 0); T4(d) 40 / 0; T5 1150 / 0; stress 2621 / 0; T10 1.135x / 1.116x;
  `regress_review.py` 40 / 0. Logs: scratchpad `planner/fx_*.log` and `planner/t0s6b/`.
