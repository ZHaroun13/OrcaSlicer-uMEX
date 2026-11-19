# OrcaSlicer-uMEX 2.5.1-diw1

OrcaSlicer-uMEX is a fork of [OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer) 2.5.1 for direct ink writing (DIW) and
u-MEX core-shell printing: paste walls from a syringe pump (B axis) and an auger screw (A axis), with a powder core laid
between them. It replaces the separate v2 Python post-processor. The slicer translates its own G-code on export.

Licensed under the GNU AGPL v3, like OrcaSlicer. The full source of this build is on the `diw` branch of
https://github.com/ZHaroun13/OrcaSlicer-uMEX.

## Installs beside stock OrcaSlicer
- Installer `OrcaSlicer-uMEX_Windows_Installer_V2.5.1_x64.exe`. Install folder, Start-menu entry and uninstall key are
  all `OrcaSlicer-uMEX`.
- Settings live in `%APPDATA%\OrcaSlicer-uMEX`, separate from a stock OrcaSlicer's `%APPDATA%\OrcaSlicer`. To keep
  your own presets, copy the `user` folder across.

## What is new compared with OrcaSlicer
- **PlanetCNC TNG G-code flavor.** Settings that DIW can't use are forced off, with a warning.
- **Native DIW translation.** Extrusion (`E`) is converted to syringe volume (`B`) and auger motion (`A`) as
  incremental `G91.2` moves. Travel moves carry no `A` or `B`. UV-cure output is driven with `M62 P4`.
- **u-MEX powder core.**
  - A native planner replaces sparse infill on core layers with powder deposition.
  - The plan is written as `;DIW:` records (format v1).
  - A 6-stage sequencer runs it: compaction, the vibrator and UV are each separate stages.
- **Time and mass.**
  - The print-time estimate, M73 progress and layer times include the added powder time.
  - An optional footer reports the powder mass (`diw_powder_mass_flow`).
- **Preview.** A gold **Powder** line type in the legend.
- **New printer settings** in "Syringe pump (B axis)": `diw_suckback_length`, `diw_suckback_feed`,
  `diw_m62_pre_dwell` (all off by default).
- **System profiles** under vendor **DIW-uMEX**:
  - printer "DIW-uMEX Robota Precision 1.4 nozzle";
  - process "DIW-uMEX 0.7mm Core-Shell";
  - filament "DIW UV Paste 45-55".

## Verification for this build
- Golden comparison with the v2 post-processor:
  - sequencer 157/157 cases, both with and without the stub;
  - 486 records;
  - 0 failures in the powder and v2 comparisons.
- Planner unit tests: 4862 assertions in 62 test cases.
- Audit gate on the 25 mm cube: 0 failures.
- Product gates: G5 78/0 and G6 117/0.

## Known limits
- The auger scale K_E (3.48) is still a placeholder until it is calibrated.
- Re-priming does not reverse `A`.
- The legend's Powder row shows 0 s / 0 g. The time is counted in the total estimate.
- UV and compaction steps are not drawn in the preview.
- Windows x64 only. The build is not code-signed, so SmartScreen will warn.
