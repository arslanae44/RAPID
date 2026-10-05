"""
RAPID installation self-check (run by SETUP.bat, safe to run any time):

    python system_files/check_install.py [--solve]

Checks the Python version, the requirements, the openvsp module, the VSPAERO
and XFOIL binaries and the airfoil catalogs. --solve additionally evaluates one
wing with VSPAERO and one airfoil with XFOIL. Exit code 0 = everything OK.
"""

import importlib
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = os.path.dirname(SCRIPT_DIR)
VSP_ROOT = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")
sys.path.insert(0, SCRIPT_DIR)

results = []


def check(label, fn):
    try:
        detail = fn()
        results.append((True, label, detail or ""))
    except Exception as e:  # noqa: BLE001 - report every failure
        results.append((False, label, f"{type(e).__name__}: {e}"))


def _python():
    mm = f"{sys.version_info.major}.{sys.version_info.minor}"
    if mm not in ("3.11", "3.13"):
        raise RuntimeError(f"Python {mm}; OpenVSP 3.47.0 needs 3.11 or 3.13")
    return f"{mm} ({sys.executable})"


def _requirements():
    return ", ".join(f"{m} {importlib.import_module(m).__version__}"
                     for m in ("numpy", "scipy", "matplotlib", "pymoo"))


def _openvsp():
    if os.path.isdir(VSP_ROOT):
        os.add_dll_directory(VSP_ROOT)
    import openvsp
    return openvsp.GetVSPVersion()


def _exists(path):
    def f():
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        return os.path.relpath(path, WORK_DIR)
    return f


def _catalogs():
    db = os.path.join(WORK_DIR, "airfoil_database")
    counts = {d: len([f for f in os.listdir(os.path.join(db, d)) if f.lower().endswith(".dat")])
              for d in sorted(os.listdir(db)) if os.path.isdir(os.path.join(db, d))}
    if not counts.get("Medium_Re"):
        raise RuntimeError(f"curated catalogs missing: {counts}")
    return ", ".join(f"{k}={v}" for k, v in counts.items())


def _solve_wing():
    import planform_opti as po
    r = po.evaluate_subprocess([9.0, 0.4, 0.8, 0.5, 4.0, 6.0] if not po.BWB_MODE else
                               [6.0, 0.4, 0.6, 0.5, 30.0, 30.0], wake_iterations=0)
    if r["F"][0] > 900:
        raise RuntimeError("VSPAERO evaluation was rejected (see the message above)")
    return f"CL={-r['F'][0]:.4f} L/D={-r['F'][1]:.2f}"


def _solve_airfoil():
    from xfoil_handler import analyze_airfoil
    af = os.path.join(WORK_DIR, "airfoil_database", "Medium_Re", "clarky.dat")
    res = analyze_airfoil(af, 1e6, 0.1, [2.0, 4.0])
    if len(res) < 2:
        raise RuntimeError(f"XFOIL converged {len(res)}/2 points")
    return ", ".join(f"a={a:g}: L/D={d['LD']}" for a, d in sorted(res.items()))


def main():
    check("Python 3.11/3.13 64-bit", _python)
    check("Python requirements", _requirements)
    check("openvsp module", _openvsp)
    check("vspaero.exe", _exists(os.path.join(VSP_ROOT, "vspaero.exe")))
    check("xfoil.exe", _exists(os.path.join(SCRIPT_DIR, "xfoil", "xfoil.exe")))
    check("airfoil catalogs", _catalogs)
    if "--solve" in sys.argv:
        os.environ.setdefault("RAPID_OUTPUT_DIR", os.path.join(WORK_DIR, "wing_models", "_selfcheck"))
        os.environ["RAPID_CONFIG_FILE"] = "<built-in defaults>"   # test the install, not the user's study
        check("VSPAERO wing solve", _solve_wing)
        check("XFOIL airfoil solve", _solve_airfoil)

    ok = all(r[0] for r in results)
    for passed, label, detail in results:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label:<26} {detail}")
    print("\n  RESULT: " + ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED (see SETUP.md troubleshooting)"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
