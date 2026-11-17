# `;DIW:` powder records, format v1

Frozen in increment 2 of [POWDER_PLAN.md](POWDER_PLAN.md). The records carry one core layer's powder plan
from the planner to the translator's sequencer. There are two planners:
- the slicer core (increment 4), which writes the records during export;
- the test hook `diw/tests/v2_hooks.py`, which turns v2's own `plan_powder_layer` output into records.

The sequencer is the same for both, so a byte-identical result against v2 on v2's own plans proves it on any geometry.

The C++ side is `src/libslic3r/GCode/DIWPowderBlock.{hpp,cpp}`; the Python reference is `diw/tests/v2_blocks.py`.
Both must produce byte-identical blocks and dumps.

## 1. Lines

- Every record line starts with `;DIW:` at column 0, then a tag (`BEGIN`, `STAT`, `RUN`, `FB`, `XY`, `DE`, `END`).
- Fields follow the tag, separated by exactly one space (0x20). There is no trailing space.
- A line ends with `\n`. A reader also accepts `\r\n`; the `\r` is not part of the line.
  A lone `\r` (not followed by `\n`) does not end a line: it is a byte inside the line, so a record line that
  holds one is `bad line`, and a `;DIW:` that follows one is not at column 0 and so is not a record.
- The characters `[ ] ( )` and the word `layer` (any case) never appear in a record line.
  This keeps records invisible to layer detection and PlanetCNC comment rules.
  The only free text is FB `ty`, whose encoding (section 3) guarantees it; a reader rejects such a line (`bad line`).
- At most 8 coordinate or value pairs go on one `XY` or `DE` line, so lines stay short.
- All bytes are ASCII.

## 2. Numbers

- **Reals** use Python `repr(float)`: shortest round-trip, `1.0` not `1`, `1e-05`, `-0.0`. In C++ this is `DIW::py_repr`.
  - A reader parses them with a correctly rounded parser (`std::from_chars` / Python `float`), so values round-trip exactly.
  - `nan`, `inf` and `-inf` are invalid in a record.
  - A reader accepts only the canonical form: the text must parse to a finite value `v` with `py_repr(v)` equal to the text.
    So `1`, `1.00`, `+1.0`, `1E-05`, `.5` and `1_0.0` are all invalid (`bad number`).
- **Integers** are plain decimal with no sign, except `-` where noted: `0` or `[1-9][0-9]*`, at most 2147483647.
  Leading zeros are invalid. `k` must be at least 1.
- **Hex** values are exactly 8 lower-case digits `[0-9a-f]`.

## 3. Block grammar

```
;DIW:BEGIN v=1 k=<int> st=<ok|open|small> z=<real> h=<real> s=<real> F=<real> fu=<real> fp=<hex>
;DIW:STAT ca=<real> oa=<real> ra=<real> hl=<real> nc=<int>
( ;DIW:RUN t=line n=<int>            followed by XY lines carrying exactly n points, n >= 2
| ;DIW:RUN t=point d=<real>          followed by exactly one XY line with 1 point
| ;DIW:FB n=<int> w=<real> tf=<real|-> z=<real> ty=<token>
                                     followed by XY lines carrying n points (n >= 2),
                                     then DE lines carrying n-1 (de, feed) pairs
)*
;DIW:END n=<int> crc=<hex>
```

Keys appear in exactly this order, and every key shown is present.

| Field | Meaning (v2 name) |
|---|---|
| `k` | 1-based printed-layer number (`LayerChunk.index`) |
| `st` | `ok` = plan with at least one run. `open` = wall loops could not be closed (`plan_powder_layer` returned None). `small` = plan without runs (whole layer reverts to the slicer's paste infill). |
| `z` | layer Z (`LayerModel.z`) |
| `h` | layer height used for planning |
| `s` | effective hatch pitch (`s_eff`) |
| `F` | powder feed (mm/min) |
| `fu` | unclamped feed `f_calc` |
| `fp` | parameter fingerprint (§5) |
| `ca` / `oa` / `ra` / `hl` / `nc` | `cavity_area` / `omega_area` / `revert_area` / `hatch_length` / `n_components` |
| RUN `d` | `PowderRun.t_feed` in seconds (point dwell) |
| FB `w` / `tf` / `z` | fallback paste path `width` / `travel_feed` (`-` = None) / `z` |
| FB `ty` | `ptype`, encoded: bytes in `[a-z0-9.-]` are kept, a space becomes `_`, every other byte becomes `%XX` (upper-case hex); then every `layer` in the result becomes `la%79er`. Example: `internal_solid_infill`. The ptype must be printable ASCII (0x20-0x7E). A reader accepts only the canonical encoding of a printable-ASCII string (decode, re-encode, compare); anything else is `bad number`. The value may be empty. |
| `XY` | `x y` pairs, so a line reads `;DIW:XY x1 y1 x2 y2 ...` |
| `DE` | `de feed` pairs for the segments |

For `st=open` and `st=small`, no RUN/FB records follow. For `st=open`, STAT is all zeros: `ca=0.0 ... nc=0`
(exactly this text; `-0.0` is not accepted there). For `st=small`, STAT holds the plan's values.
A block with `st=ok` has at least one RUN.

RUN and FB records keep v2's order: `plan.runs` in list order, then `plan.fallback_paths` in list order.
The writer emits all RUNs before all FBs, and a reader rejects a RUN after an FB.

XY and DE lines: the writer puts 8 pairs on each line and the remainder on the last one. A reader accepts any
split with 1 to 8 pairs per line (2 to 16 numbers, an even count), as long as the lines carry exactly the
announced number of pairs. `n` of a line RUN and of an FB is at least 2.

### END record and CRC

- `n` is the number of lines strictly between BEGIN and END.
- `crc` is the CRC-32 (IEEE 802.3 / zlib, reflected, init and xorout `0xFFFFFFFF`) over the bytes of every line from BEGIN through the line before END.
  Each line counts as its text plus one `\n`, never `\r`.
- A block whose `n` or `crc` does not match is rejected as a whole.

## 4. Placement and the translator

- A block sits inside the layer it describes: after that layer's start marker and before the layer's first motion line.
  It holds at most one block per layer, and `k` must equal the layer's number.
  - "Layer" is v2's `split_layers` chunk (`LayerChunk`, numbered from 1). A block that sits between a layer's last
    line and the next start marker belongs to the earlier layer, so it fails the motion-line rule.
  - In powder mode (since increment 3a), the translator checks this placement, `k` and `fp` for every block before
    it emits anything. On reaching the layer, it also checks `z` / `h` against the layer and `F` / `s` / `fu` against
    its own `resolve_powder_feed`. Stage 4 then traverses the RUN points in record order (increment 3b). A failed check, like any reader error, stops the export with `DIWConfigError`
    (POWDER_PLAN.md, "Increment 3a results"). A layer is converted iff it carries a block.
- The translator removes every line that starts with `;DIW:` before any other processing, in paste and powder modes alike.
  So a file with records and the same file without them translate identically, apart from what the records drive.
  Paste mode ignores the records. Powder mode reads them first, from the raw input, with this section's reader.
  - "Line" here is the section 1 line, the one the reader sees: the raw input is split at `\n` only, and the
    removal (the line with its `\n`; a final line without `\n` likewise) happens before UTF-8 decoding and before
    the translator's universal-newline splitting. A lone `\r` therefore never splits a record line (the whole
    line goes, text after the `\r` included), and a `;DIW:` after a lone `\r` stays in the G-code as an ordinary
    comment. The result is byte-identical to `v2_blocks.strip_records`, so `translate(F) == translate(strip_records(F))`.
- Parse errors carry a 1-based line number and a short reason. The reason strings are exact and shared by the
  C++ and Python readers:

| Reason | When |
|---|---|
| `bad line` | a byte outside 0x20-0x7E (a trailing `\r` before `\n` is not part of the line); `[`, `]`, `(`, `)` or `layer` (any case) in the line; an empty field (double, leading or trailing space, or nothing after `;DIW:`); a field without `=` on a keyed line; an XY/DE line with an odd count or not 2-16 numbers; a record of the wrong kind at this position (any record but STAT or END right after BEGIN, a second STAT, RUN/FB in an `open`/`small` block, RUN after FB, STAT/RUN/FB/XY/DE outside a block) |
| `unknown tag` | the tag is not one of `BEGIN STAT RUN FB XY DE END` |
| `unknown key` | a key the tag never takes |
| `key order` | a key the tag takes, but not the one expected at this position (also a repeated key, an extra key, `n` on a `t=point` RUN) |
| `missing key` | the line ends before every key of the tag is present |
| `bad number` | a value that does not match its type: real (section 2), int, hex, `st`, `t`, `tf`, `ty`; also `k=0` and a non-zero STAT in an `open` block |
| `unsupported version` | `v` is a valid integer other than 1 |
| `count mismatch` | XY/DE lines carry more pairs than announced, or a different record arrives before they are complete; an XY/DE line where none is expected after STAT; RUN `t=line` or FB with `n < 2`; at END: `n` differs from the number of lines, or (after the CRC check) the block is incomplete (no STAT, points or pairs still missing, `st=ok` without RUN) |
| `crc mismatch` | at END, the CRC differs |
| `nested begin` | a BEGIN inside an open block |
| `end without begin` | an END outside a block |
| `unterminated block` | the file ends, or a line that is not a record arrives, inside an open block |
| `duplicate k` | a complete, valid block whose `k` was already accepted earlier in the file |

- The reader works line by line (lines split at `\n`; a final line without `\n` counts). Lines that do not
  start with `;DIW:` are not records. States: *out*, *in* (a block is open), *skip* (dropping a rejected block).
  - Checks on one record line, first failure wins: line checks (`bad line`), tag (`unknown tag`), position
    (`bad line` / `count mismatch` as in the table), then the fields from left to right (`unknown key`,
    `key order`, value), then `missing key`, then the line's own rules (`n >= 2`, `open` STAT zeros, `k >= 1`).
  - *out*: BEGIN opens a block (an error in the BEGIN line itself goes to *skip*); END gives `end without begin`
    and stays *out*; any other record, or a line that fails the line or tag checks, gives its error and goes to *skip*.
  - *in*: an error on a line drops the block and goes to *skip*. BEGIN gives `nested begin` at its own line,
    drops the open block and opens a new one with this BEGIN. A non-record line gives `unterminated block`
    and goes to *skip*. END is checked in this order: its own fields, `n`, `crc`, completeness, then `k`;
    any failure drops the block. END always returns to *out*.
  - *skip*: lines are ignored without errors (also malformed ones) until an END (back to *out*) or a valid-tag
    BEGIN (which opens a new block as in *out*).
  - At the end of the file, a block still *in* gives `unterminated block`.
  - Line numbers: the offending line, except `unterminated block` and `duplicate k`, which carry the line of
    the block's BEGIN. Errors are listed in the order they are found.

## 5. Fingerprint

The fingerprint is FNV-1a 32-bit (offset `0x811c9dc5`, prime `0x01000193`) over the ASCII string formed by
concatenating `name=value;` for these planning inputs, in this order:

```
v=1;delta=<r>;guard=<r>;s_hatch=<r>;hatch_angle=<r>;point_len=<r>;rho_tap=<r>;eta=<r>;m_dot=<r>;f_max=<r>;policy=<str>;paste_min_area=<r>;width_fallback=<r>;quad_segs=<int>;
```

- `<r>` is `py_repr` of the value as held in `UMexParams` (`f_max` in mm/min).
- `guard` is v2's `POWDER_GUARD_MM`.
- `quad_segs` is v2's `UMEX_QUAD_SEGS`.

The translator recomputes the fingerprint from its own settings. A mismatch means the plan was made with different settings, and the block is rejected.

## 6. Dump format (`diw_translate --dump-records`, `v2_blocks.dump`)

The dump is a canonical, line-based text used to compare the C++ and Python readers.
- First line: `blocks <count>`. Then, per block in file order:
  - `block k=<k> st=<st> z=<r> h=<r> s=<r> F=<r> fu=<r> fp=<hex>`
  - `stat ca=<r> oa=<r> ra=<r> hl=<r> nc=<int>`
  - `run line <n>` + one `  p <x> <y>` line per point
  - `run point d=<r>` + `  p <x> <y>`
  - `fb n=<n> w=<r> tf=<r|-> z=<r> ty=<decoded ptype>` + `  p <x> <y>` per point + `  s <de> <feed>` per segment
- After the blocks: `errors <count>`, then one `error <line> <reason>` line per error, in the order found.
- `<r>` is `py_repr`. Lines end with `\n`. Only accepted blocks are dumped.

## 7. Test interface of `diw_translate` (dev tool)

Results go to stdout in binary mode (LF only).

```
diw_translate --dump-records <in>       the section 6 dump of every block in <in>; exit 0 even if it lists errors
diw_translate --rewrite-records <in>    every accepted block of <in> written again by the writer, concatenated in file order
diw_translate [--config-from-gcode] --fingerprint <in> [key=value ...]
                                        "<fingerprint string>\n<8 hex digits>\n" (section 5) for the settings,
                                        built as for a translation (powder settings need diw_powder_infill=1)
```

`diw/tests/compare_records.py` checks all three against `v2_blocks.py`, and checks the section 4 filter with
`translate(F) == translate(strip_records(F))` on lone-CR input in paste mode. In powder mode it checks that the
lone-CR record line stops the export with the reader's line and reason.

Translation flags used by the powder tests: `--allow-unconverted-powder` runs a powder job that has no block, and
`--stub-powder-stage` reduces stage 4 to its header comment line (increment 3a; POWDER_PLAN.md). Without it,
stage 4 emits the block's runs (increment 3b). The
sequencing itself is checked by `diw/tests/compare_sequencer.py`. A missing or unreadable `diw_translate.exe`
stops it with exit 2, and a binary without one of the flags fails those checks; nothing is skipped.
