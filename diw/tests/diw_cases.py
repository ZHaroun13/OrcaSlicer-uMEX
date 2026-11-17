"""Edge-case G-code inputs and paste settings variants shared by compare_v2.py and compare_powder.py."""

BIG = "1" + "0" * 64            # 1e64: longer than any fixed print buffer once multiplied
HUGE = "9" * 400                # overflows to inf in both float() and fast_float

# name -> bytes of a G-code file
CASES = {
    "degree_sign_before_word": ";LAYER_CHANGE\nG1 X10 Y10 ; °E5\n;LAYER_CHANGE\n".encode(),
    "nbsp_and_separators": ";LAYER_CHANGE\nG1 X5 Y5 E1\nG1 X6\x1cY6 E2\nG1 X7 Y7 E3 ; em space\n;LAYER_CHANGE\n".encode(),
    "huge_feedrate": f";LAYER_CHANGE\nG92 E0\nG1 X1 E1 F{BIG}\nG1 X2 E2\n;LAYER_CHANGE\nG1 X5\n".encode(),
    "overflow_to_inf_and_nan": f";LAYER_CHANGE\nG92 E0\nG1 X-{HUGE} E1\nG1 X{HUGE} E2\nG1 X1 Y1\n;LAYER_CHANGE\n".encode(),
    "lone_cr_endings": b";LAYER_CHANGE\rG92 E0\rG1 X1 Y1 E1\r\nG1 X2 Y2 E2\n;LAYER_CHANGE\rG1 X3 Y3\r",
    "invalid_utf8": b";LAYER_CHANGE\nG1 X1 Y1 E1 ; bad \xff\xfe bytes \xe2\x82\nG1 X2 E2 ; trunc \xf0\x9f\n; CONFIG_BLOCK_START\n; x = \xc3\x28\n; CONFIG_BLOCK_END\n",
    "relative_e_arcs_line_numbers": (
        b"M83\nN10 G1 X0 Y0 Z0.2 F600\nN11 G1 X10 E0.5\ng1 x20 e0.5\nG2 X20 Y10 I0 J5 E0.4\n"
        b"G3 X10 Y10 I-5 J0 E0.4\nG1 E-0.8 F1800\nG1 E-6\nG1 X0 E-6\nG1 E0.8\nG4\nG4 S2\nG04 P500\n"
        b";LAYER_CHANGE\nG1 Z0.4\nG1 X5 Y5 E.5\nG1 X6 E5.\nG1 X7 E+1 ; plus\nG1 X8 E-0 ; neg zero\nM2\n"),
    "start_end_blocks": (
        b";TYPE:Custom\nG28 ; home\nG90\nM82\nG92 E0\n; initial load\nG1 E20 F100\nG92 E0\n"
        b"; intro line\nG1 X60 Y2 E10\nG1 X100 E20\nG92 E0\n; Filament gcode\nM106 S0\n"
        b";LAYER_CHANGE\nG1 X1 Y1 E1 ; [bracket] comment\n; stop printing object, id 1\n"
        b";LAYER_CHANGE\n; filament end gcode\nM104 S0\nM84\n; EXECUTABLE_BLOCK_END\n"),
    "layer_marker_variants": (
        b"; layer 1\nG92 E0\nG1 X1 E1\n;BEFORE_LAYER_CHANGE\nG1 X2 E2\n;AFTER_LAYER_CHANGE\nG1 X3 E3\n"
        b";LAYER:2\nG1 X4 E4\n; layer_count: 9\nG1 X5 E5\n(layer 3)\nG1 X6 E6\n;LAYER_CHANGEX\n"),
    "empty_and_whitespace": b"\n   \n\t\nG1 X1\n;\n  ; indented comment\nG1\tX2\tE1\n",
    "no_trailing_newline": b";LAYER_CHANGE\nG92 E0\nG1 X1 Y1 E1",
    "empty_file": b"",
    "already_postprocessed_line": b";LAYER_CHANGE\nG92 E0\nG1 X1 Y1 E1\n; Post-Processed by DIW Post-Processor v2.0.0\nG1 X2 Y2 E2\n;LAYER_CHANGE\n",
}

# name -> (v2 overrides, diw_translate overrides)
VARIANTS = {
    "default": ({}, []),
    "cumulative_drivetrain": (
        {"a_axis_mode": "Cumulative Position (A += ds * K_screw)", "b_include_drivetrain_in_gcode": True,
         "enable_retraction": True, "max_retract_de_mm": 2.0, "post_extrusion_dwell_sec": 0.1, "uv_end_dwell_sec": 4.0},
        ["diw_a_axis_mode=cumulative", "diw_b_drivetrain_units=1", "diw_retraction_enable=1",
         "diw_retraction_max_length=2", "diw_post_extrusion_dwell=0.1", "diw_uv_end_dwell=4"]),
    "proportional_no_retract_no_uv": (
        {"a_axis_mode": "Proportional Speed (A = F * K_screw)", "k_screw": 5.0, "enable_retraction": False,
         "enable_uv_io": False},
        ["diw_a_axis_mode=proportional", "diw_k_screw=5", "diw_retraction_enable=0", "diw_uv_on_gcode="]),
}
