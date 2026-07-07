"""
RAPID interactive configuration editor.

Launched by CONFIGURE.bat. For every tunable parameter it shows the built-in
DEFAULT and the value currently in effect, then lets you keep or change it.

Controls at any prompt:
  Enter  keep the current value
  -      go back to the previous field to fix a wrong entry
The result is written to system_files/rapid_config.json and loaded automatically
on the next run.
"""

import os
import sys
import io

try:
    if hasattr(sys.stdout, "buffer") and sys.stdout.buffer:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rapid_config import (
    DEFAULTS, PARAM_META, DEFAULT_CONFIG_PATH,
    load_raw_config, load_effective_config, save_config, _deep_merge,
)

BACK = object()   # sentinel: user asked to step back


def _fmt(v):
    if v is None:
        return "(auto)"
    if isinstance(v, bool):
        return "yes" if v else "no"
    return str(v)


def _ask_float(prompt, current):
    while True:
        raw = input(prompt).strip()
        if raw == "":
            return current
        if raw == "-":
            return BACK
        try:
            return float(raw)
        except ValueError:
            print("    Enter a number, blank to keep, or - to go back.")


def _ask_optfloat(prompt, current):
    raw = input(prompt).strip()
    if raw == "":
        return current
    if raw == "-":
        return BACK
    if raw.lower() in ("none", "auto"):
        return None
    try:
        return float(raw)
    except ValueError:
        print("    Not a number; keeping current.")
        return current


def _ask_bool(prompt, current):
    raw = input(prompt).strip().lower()
    if raw == "":
        return current
    if raw == "-":
        return BACK
    if raw in ("y", "yes", "true", "1"):
        return True
    if raw in ("n", "no", "false", "0"):
        return False
    print("    Enter y/n, blank to keep, or - to go back.")
    return current


def _ask_str(prompt, current):
    raw = input(prompt).strip()
    if raw == "":
        return current
    if raw == "-":
        return BACK
    return raw


def _ask_range(label, helptext, current, default_v):
    """Two-field range with intra-range back: - on upper re-asks lower,
    - on lower steps out to the previous parameter."""
    dlo, dhi = default_v
    clo, chi = current
    print(f"\n {label}  ({helptext})")
    while True:
        lo = _ask_float(f"   lower  [default={dlo} | current={clo}]: ", clo)
        if lo is BACK:
            return BACK
        hi = _ask_float(f"   upper  [default={dhi} | current={chi}]: ", chi)
        if hi is BACK:
            clo = lo            # keep what was just entered while re-asking
            continue
        if hi < lo:
            lo, hi = hi, lo
        return [lo, hi]


def main():
    print("=" * 70)
    print(" RAPID CONFIGURATION EDITOR")
    print("=" * 70)
    print(" Enter = keep current   |   - = go back and fix the previous entry\n")

    effective = load_effective_config(DEFAULT_CONFIG_PATH)   # defaults + saved
    raw_saved = load_raw_config(DEFAULT_CONFIG_PATH)          # only prior saves
    # working answers start from the currently effective values
    answers = {(s, k): effective[s][k] for (s, k, *_r) in PARAM_META}

    i = 0
    n = len(PARAM_META)
    section_shown = None
    while i < n:
        section, key, label, typ, helptext = PARAM_META[i]
        if section != section_shown:
            section_shown = section
            print("\n" + "-" * 70)
            print(f" [{section.upper()}]      (- = back)")
            print("-" * 70)

        default_v = DEFAULTS[section][key]
        current = answers[(section, key)]

        if typ == "range":
            res = _ask_range(label, helptext, current, default_v)
        else:
            print(f"\n {label}  ({helptext})")
            prompt = f"   value  [default={_fmt(default_v)} | current={_fmt(current)}]: "
            if typ == "float":
                res = _ask_float(prompt, current)
            elif typ == "optfloat":
                res = _ask_optfloat(prompt, current)
            elif typ == "bool":
                res = _ask_bool(prompt + "[y/n] ", current)
            else:
                res = _ask_str(prompt, current)

        if res is BACK:
            if i == 0:
                print("    (already at the first field)")
                continue
            i -= 1
            section_shown = None   # reprint the header when stepping back
            continue

        answers[(section, key)] = res
        i += 1

    # assemble config, preserving any prior keys the editor does not manage
    new_cfg = {"flight": {}, "bounds": {}, "constraints": {}, "bwb": {}}
    for (section, key), val in answers.items():
        new_cfg[section][key] = val
    merged = _deep_merge(raw_saved, new_cfg)
    path = save_config(merged, DEFAULT_CONFIG_PATH)

    print("\n" + "=" * 70)
    print(" SAVED CONFIGURATION")
    print("=" * 70)
    print(f" File: {path}")
    print(f" BWB mode    : {_fmt(merged['bwb']['BWB_MODE'])}")
    print(f" Constraints : L/D>={merged['constraints']['LD_MIN']} "
          f"| CM<={merged['constraints']['CM_MAX']} "
          f"| CL>={merged['constraints']['CL_MIN']}")
    if merged["bwb"]["BWB_MODE"]:
        print(f" Static margin: {merged['bwb']['SM_MIN']} .. {merged['bwb']['SM_MAX']} (frac MAC)")
        print(f" Baseline foil: {merged['bwb'].get('BASELINE_AIRFOIL', 'mh60')}")
    print("\n Run RUN_OPTIMIZATION.bat (or RUN_BWB.bat) to apply.")


if __name__ == "__main__":
    main()
