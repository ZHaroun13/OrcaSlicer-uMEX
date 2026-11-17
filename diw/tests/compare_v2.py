"""Regression test: built-in DIW translation (diw_translate.exe) vs the v2 post-processor (diw_engine.py), paste mode.

usage:
  python compare_v2.py <v2 engine dir> <diw_translate.exe> <work dir> [extra.gcode ...]

Runs every edge case of diw_cases.py plus any extra G-code files (e.g. real OrcaSlicer output) through both,
with each paste settings variant, and compares the outputs after dropping the first (header) line and
normalising CRLF. Exit code 0 when all are identical. Run diw_translate.exe from build/src/Release (DLLs).
"""
import os
import subprocess
import sys

from diw_cases import CASES, VARIANTS


def norm(path):
    with open(path, "rb") as f:
        lines = f.read().replace(b"\r\n", b"\n").split(b"\n")
    return lines[1:]


def first_diff(a, b):
    i = next((k for k in range(min(len(a), len(b))) if a[k] != b[k]), min(len(a), len(b)))
    return i, (a[i] if i < len(a) else "<end>"), (b[i] if i < len(b) else "<end>")


def main(argv):
    engine_dir, translate_exe, work = argv[1], argv[2], argv[3]
    extra = argv[4:]
    sys.path.insert(0, engine_dir)
    import diw_engine as E  # noqa: E402

    os.makedirs(work, exist_ok=True)
    inputs = dict(CASES)
    for p in extra:
        with open(p, "rb") as f:
            inputs[os.path.basename(p)] = f.read()
    failures = 0
    for vname, (v2_over, cpp_over) in VARIANTS.items():
        for cname, data in inputs.items():
            src = os.path.join(work, f"{cname}.in.gcode")
            with open(src, "wb") as f:
                f.write(data)
            py_out = os.path.join(work, f"{cname}.{vname}.v2.gcode")
            cpp_out = os.path.join(work, f"{cname}.{vname}.cpp.gcode")
            cfg = dict(E.DEFAULT_CONFIG)
            cfg.update(v2_over)
            cfg["enable_u_mex_powder_infill"] = False
            E.DIWPostProcessor(cfg).process_file(src, py_out)
            r = subprocess.run([translate_exe, src, cpp_out] + cpp_over, capture_output=True, text=True)
            if r.returncode != 0:
                print(f"FAIL {vname:30} {cname}: diw_translate exit {r.returncode} {r.stderr.strip()}")
                failures += 1
                continue
            a, b = norm(py_out), norm(cpp_out)
            if a == b:
                print(f"ok   {vname:30} {cname} ({len(a)} lines)")
            else:
                failures += 1
                i, va, vb = first_diff(a, b)
                print(f"FAIL {vname:30} {cname}: first difference at line {i + 1}")
                print(f"       v2 : {va!r}")
                print(f"       c++: {vb!r}")
    print(f"\n{failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
