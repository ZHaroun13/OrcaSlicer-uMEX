"""Powder-mode golden test: diw_translate.exe vs the v2 post-processor with enable_u_mex_powder_infill.

usage:
  python compare_powder.py <v2 engine dir> <diw_translate.exe> <work dir> <corpus.gcode ...> [--allow-version-drift]

Increment 1 (FEATURE_MAP milestone 4) covers the powder-mode whole-file rules only: parameter checks and
their messages, the warning banner, G04 in seconds, M202, config-block sanitation and the safe shutdown.
Core layers are not converted by the C++ side yet, so v2 runs with umex_n_bottom_layers = 10**6 (every
layer counts as a bottom layer and stays paste) and both outputs must then be identical after the header.

Corpus files are raw OrcaSlicer exports (diw_printer on, untranslated: ORCA_DIW_EXPORT_UNTRANSLATED=1);
their config block supplies the settings for both sides. Run diw_translate.exe from build/src/Release.
Exit code 0 when everything matches.
"""
import json
import os
import platform
import random
import struct
import subprocess
import sys

from diw_cases import CASES

PINNED = {"python": "3.15.0", "shapely": "2.2.0", "geos": "3.14.1"}

A_MODES = {"flow_scaled": "Flow-Scaled Speed (A = (dE/ds) * F * K_screw)",
           "proportional": "Proportional Speed (A = F * K_screw)",
           "cumulative": "Cumulative Position (A += ds * K_screw)"}
POLICIES = {"increase_spacing", "clamp", "error"}

# Increment 1 variants: fork overrides applied to both sides.
VARIANTS = {
    "noncore_default": [],
    "noncore_offsets": ["diw_powder_offset_x=12.5", "diw_powder_offset_y=-3", "diw_powder_offset_z=0.4"],
    "noncore_warn_values": ["diw_compaction_time=0.5", "diw_powder_motor_voltage=5", "diw_shell_powder_clearance=0.6"],
    "noncore_cumulative_noUV": ["diw_a_axis_mode=cumulative", "diw_uv_on_gcode="],
}
ERROR_VARIANTS = {
    "err_compaction_time": ["diw_compaction_time=9"],
    "err_pins": ["diw_compaction_pin=5"],
    "err_voltage": ["diw_powder_motor_voltage=13"],
    "err_interval": ["diw_compaction_interval=0"],
    "err_spacing": ["diw_powder_hatch_spacing=0"],
}
SYNTHETIC = {
    "g4_rules": (b";LAYER_CHANGE\nG4 P10\nG4 P500\nG4 S2\nG04 P0.5\nG4 P0\nG4\nG4 P10\nM202 X1\nG1 X1 Y1 E1\n"
                 b"; CONFIG_BLOCK_START\n; a = [1]\n; CONFIG_BLOCK_END\n"),
    "no_markers": b"G90\nG1 X1 Y1 E1\nG1 X2 Y2 E2\n",
    "m30_end": b";LAYER_CHANGE\nG92 E0\nG1 X1 Y1 E1\nM30\nG1 X5 Y5\n",
}


def parse_config_block(data):
    cfg, inside = {}, False
    for raw in data.decode("utf-8", "replace").splitlines():
        if raw.startswith("; CONFIG_BLOCK_START"):
            inside = True
            continue
        if raw.startswith("; CONFIG_BLOCK_END"):
            break
        if inside and raw.startswith("; ") and " = " in raw:
            k, v = raw[2:].split(" = ", 1)
            cfg[k.strip()] = v
    return cfg


def first(v):
    return v.split(",")[0].strip().strip('"')


def v2_config(diw, E):
    """Fork settings (strings, as in the config block) -> v2 config dict. One mapping table for every test."""
    c = dict(E.DEFAULT_CONFIG)
    f = lambda k: float(first(diw[k]))
    if "filament_diameter" in diw: c["filament_diameter_mm"] = f("filament_diameter")
    if "nozzle_diameter" in diw: c["paste_nozzle_dia"] = f("nozzle_diameter")
    simple = {  # fork key -> (v2 key, converter)
        "diw_syringe_diameter": ("syringe_diameter_mm", f), "diw_b_multiplier": ("b_multiplier", f),
        "diw_k_screw": ("k_screw", f), "diw_retraction_max_length": ("max_retract_de_mm", f),
        "diw_post_extrusion_dwell": ("post_extrusion_dwell_sec", f), "diw_uv_offset_y": ("uv_offset_y_mm", f),
        "diw_uv_end_dwell": ("uv_end_dwell_sec", f), "diw_uv_layer_dwell": ("uv_layer_dwell_sec", f),
        "diw_powder_nozzle_diameter": ("powder_nozzle_dia", f), "diw_shell_powder_clearance": ("shell_powder_clearance", f),
        "diw_powder_mass_flow": ("powder_mass_flow_rate", f), "diw_powder_tap_density": ("powder_tap_density", f),
        "diw_powder_apparent_density": ("powder_apparent_density", f), "diw_powder_fill_multiplier": ("powder_fill_multiplier", f),
        "diw_powder_hatch_spacing": ("powder_hatch_spacing", f), "diw_powder_hatch_angle": ("powder_hatch_angle_deg", f),
        "diw_powder_lag_on": ("powder_lag_on", f), "diw_powder_lag_off": ("powder_lag_off", f),
        "diw_powder_point_dwell_length": ("powder_point_dwell_max_len_mm", f), "diw_compaction_time": ("compaction_duration", f),
        "diw_compaction_settle_time": ("compaction_settle_dwell", f), "diw_compaction_z_hop": ("compaction_z_hop_mm", f),
        "diw_powder_offset_x": ("powder_offset_x", f), "diw_powder_offset_y": ("powder_offset_y", f),
        "diw_powder_offset_z": ("powder_offset_z", f), "diw_powder_z_clearance": ("powder_z_clearance_mm", f),
        "diw_powder_travel_hop": ("powder_travel_hop_mm", f), "diw_auger_stop_dwell": ("umex_stage1_dwell_sec", f),
        "diw_wiper_x_start": ("wiper_x_start", f), "diw_wiper_x_end": ("wiper_x_end", f), "diw_wiper_y": ("wiper_y", f),
        "diw_wiper_z": ("wiper_z", f), "diw_wiper_z_hop": ("wiper_z_hop_mm", f), "diw_wiper_z_min": ("wiper_z_min_abs_mm", f),
        "diw_powder_motor_voltage": ("powder_motor_voltage", f), "diw_powder_min_area": ("umex_paste_revert_min_area_mm2", f),
    }
    for k, (k2, conv) in simple.items():
        if k in diw:
            c[k2] = conv(k)
    if "diw_retraction_max_speed" in diw: c["max_retract_feedrate"] = f("diw_retraction_max_speed") * 60.0
    if "diw_powder_speed_max" in diw: c["powder_f_max"] = f("diw_powder_speed_max") * 60.0
    if "diw_wiper_speed" in diw: c["wiper_feed"] = f("diw_wiper_speed") * 60.0
    if "diw_b_lead_screw_lead" in diw: c["b_lead_screw_pitch_mm"], c["b_lead_screw_starts"] = f("diw_b_lead_screw_lead"), 1
    if "diw_b_gear_ratio" in diw: c["b_gear_ratio_driven"], c["b_gear_ratio_driver"] = f("diw_b_gear_ratio"), 1.0
    if "diw_b_drivetrain_units" in diw: c["b_include_drivetrain_in_gcode"] = first(diw["diw_b_drivetrain_units"]) == "1"
    if "diw_retraction_enable" in diw: c["enable_retraction"] = first(diw["diw_retraction_enable"]) == "1"
    if "diw_a_axis_mode" in diw: c["a_axis_mode"] = A_MODES[first(diw["diw_a_axis_mode"])]
    if "diw_powder_overflow_policy" in diw:
        pol = first(diw["diw_powder_overflow_policy"])
        assert pol in POLICIES, pol
        c["powder_overflow_policy"] = pol
    if "diw_compaction_interval" in diw: c["compaction_interval"] = int(first(diw["diw_compaction_interval"]))
    if "diw_powder_pin" in diw: c["powder_pin"] = int(first(diw["diw_powder_pin"]))
    if "diw_compaction_pin" in diw: c["compaction_pin"] = int(first(diw["diw_compaction_pin"]))
    if "diw_uv_on_gcode" in diw:
        c["uv_on_cmd"] = diw["diw_uv_on_gcode"].strip().strip('"')
        c["enable_uv_io"] = c["uv_on_cmd"] != ""
    if "diw_uv_off_gcode" in diw: c["uv_off_cmd"] = diw["diw_uv_off_gcode"].strip().strip('"')
    return c


def norm(path):
    with open(path, "rb") as fh:
        return fh.read().replace(b"\r\n", b"\n").split(b"\n")[1:]


def run_case(E, exe, work, name, data, overrides):
    """Returns (status, detail). status: ok | FAIL."""
    src = os.path.join(work, f"{name}.in.gcode")
    with open(src, "wb") as fh:
        fh.write(data)
    py_out, cpp_out = src.replace(".in.", ".v2."), src.replace(".in.", ".cpp.")
    diw = parse_config_block(data)
    for kv in overrides:
        k, v = kv.split("=", 1)
        diw[k] = v
    cfg = v2_config(diw, E)
    cfg["enable_u_mex_powder_infill"] = True
    cfg["umex_n_bottom_layers"] = 10 ** 6
    py_err = None
    try:
        E.DIWPostProcessor(cfg).process_file(src, py_out)
    except E.UMexConfigError as exc:
        py_err = str(exc)
    args = [exe, "--allow-unconverted-powder"]
    # Only real OrcaSlicer exports can be loaded by load_from_gcode_file; synthetic files with a fake
    # config block use the defaults (v2 then reads only irrelevant keys from that block).
    if b"; CONFIG_BLOCK_START" in data and b"generated by OrcaSlicer" in data:
        args.append("--config-from-gcode")
    r = subprocess.run(args + [src, cpp_out, "diw_powder_infill=1"] + overrides, capture_output=True, text=True)
    cpp_err = r.stderr.strip()[len("error: "):] if r.returncode == 3 else None
    if py_err is not None or cpp_err is not None:
        if py_err == cpp_err:
            return "ok", f"same error: {py_err}"
        return "FAIL", f"errors differ\n       v2 : {py_err!r}\n       c++: {cpp_err!r} (exit {r.returncode} {r.stderr.strip()!r})"
    if r.returncode != 0:
        return "FAIL", f"diw_translate exit {r.returncode} {r.stderr.strip()}"
    a, b = norm(py_out), norm(cpp_out)
    if a == b:
        return "ok", f"{len(a)} lines"
    i = next((k for k in range(min(len(a), len(b))) if a[k] != b[k]), min(len(a), len(b)))
    return "FAIL", (f"first difference at line {i + 1}\n       v2 : {a[i] if i < len(a) else '<end>'!r}\n"
                    f"       c++: {b[i] if i < len(b) else '<end>'!r}")


def pyrepr_selftest(exe):
    edge = [1.2, 0.15, 1e-05, 1.5e-05, 1e16, 1e15, 3.0, -0.0, 0.1 + 0.2, 1e-4, 5e-324, 1.7976931348623157e308,
            float("nan"), float("inf"), float("-inf"), 112.538, 0.4, -3.0, 12.5, 1e-4 * 0.999]
    rnd = random.Random(20261117)
    vals = edge + [struct.unpack("<d", struct.pack("<Q", rnd.getrandbits(64)))[0] for _ in range(20000)]
    hexes = "\n".join(struct.pack(">d", v).hex() for v in vals)
    r = subprocess.run([exe, "--pyrepr"], input=hexes, capture_output=True, text=True)
    got = r.stdout.split("\n")
    bad = 0
    for v, line in zip(vals, got):
        want = f"{repr(v)} {format(v, 'g')}"
        if line != want:
            if bad < 5:
                print(f"  py_repr mismatch: {struct.pack('>d', v).hex()} want {want!r} got {line!r}")
            bad += 1
    if len(got) < len(vals):
        bad += len(vals) - len(got)
    return bad, len(vals)


def main(argv):
    allow_drift = "--allow-version-drift" in argv
    argv = [a for a in argv if a != "--allow-version-drift"]
    engine_dir, exe, work = argv[1], argv[2], argv[3]
    corpus = argv[4:]
    sys.path.insert(0, engine_dir)
    import diw_engine as E  # noqa: E402
    import shapely  # noqa: E402
    versions = {"python": platform.python_version(), "shapely": shapely.__version__, "geos": shapely.geos_version_string}
    print(f"versions: {versions}")
    if versions != PINNED and not allow_drift:
        print(f"version drift from {PINNED}; pass --allow-version-drift to run anyway")
        return 2

    os.makedirs(work, exist_ok=True)
    report, failures = {"versions": versions, "cases": {}}, 0

    def record(name, status, detail):
        nonlocal failures
        report["cases"][name] = {"status": status, "detail": detail}
        print(f"{status:4} {name}: {detail}")
        if status != "ok":
            failures += 1

    inputs = {os.path.splitext(os.path.basename(p))[0]: open(p, "rb").read() for p in corpus}
    for cname, data in inputs.items():
        for vname, ov in {**VARIANTS, **ERROR_VARIANTS}.items():
            record(f"{cname}.{vname}", *run_case(E, exe, work, f"{cname}.{vname}", data, ov))
    for cname, data in {**SYNTHETIC, **CASES}.items():
        record(f"{cname}.noncore_default", *run_case(E, exe, work, f"{cname}.noncore_default", data, []))

    bad, n = pyrepr_selftest(exe)
    record("py_repr_selftest", "ok" if bad == 0 else "FAIL", f"{n - bad}/{n} values match repr() and format(v, 'g')")

    with open(os.path.join(work, "compare_powder_report.json"), "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
