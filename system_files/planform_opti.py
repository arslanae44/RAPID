import os
import sys
import io

# Ensure console supports UTF-8 for Turkish logs (Safe for Spyder/IPython)
try:
    if hasattr(sys.stdout, 'buffer') and sys.stdout.buffer:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    if hasattr(sys.stderr, 'buffer') and sys.stderr.buffer:
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')
except Exception:
    pass

# Dynamically locate local OpenVSP Root in the portable distribution
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)  # Ensures seamless import of sister modules like xfoil_handler.py
VSP_ROOT = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")
if os.path.exists(VSP_ROOT):
    try:
        os.add_dll_directory(VSP_ROOT)
    except Exception:
        pass

import os, json, subprocess, time, pickle, random, shutil, sys, signal
import numpy as np
import multiprocessing
import uuid

from pymoo.core.problem import ElementwiseProblem
try:
    from pymoo.parallelization.starmap import StarmapParallelization
except ImportError:
    from pymoo.core.problem import StarmapParallelization

from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.core.callback import Callback
from pymoo.util.display.display import Display

# Aerodynamic custom modules
from xfoil_handler import analyze_airfoil

from pymoo.util.display.column import Column

os.environ["OMP_NUM_THREADS"] = "1"

# ─── PORTABLE PATH CONFIGURATION ──────────────────────────────────────────────
# Dynamic derivation ensures zero-configuration relative paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR   = os.path.dirname(SCRIPT_DIR)
VSP_ROOT   = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")

# ─── FLIGHT CONDITIONS ────────────────────────────────────────────────────────
V_MS   = 600.0 / 3.6
RHO    = 0.7364
TEMP_K = 255.7
MACH   = V_MS / np.sqrt(1.4 * 287.05 * TEMP_K)
MU     = 1.628e-5

# ─── DESIGN CONSTANTS ─────────────────────────────────────────────────────────
S_REF         = 210.0
AOA           = 1.0
INCIDENCE     = 3.0
FUSELAGE_Y    = 2.2
HALF_SPAN_MAX = 25.0
AIRFOIL       = "naca2412"

VSP_FILE   = os.path.join(SCRIPT_DIR, "altc130mod.vsp3")
OUTPUT_DIR = os.path.join(WORK_DIR, "wing_models")

C_ROOT_MIN, C_ROOT_MAX = 4.0, 9.5

BOUNDS = [
    (7.0,  10.5),
    (0.10,  0.75),
    (0.50,  1.0),
    (0.2,   0.9),
    (-2.0,  12.0),
    (-2.0,  12.0),
]

LD_MIN = 15.0
CM_MAX = 0.35
CL_MIN = 0.15

# ─── EXTERNAL CONFIG OVERRIDES (constraints / bounds / flight / BWB) ──────────
# Overridden by rapid_config.json or the RAPID_CONFIG_FILE env var (see rapid_config.py).
BWB_MODE = False
SM_MIN, SM_MAX, CM_TRIM_TOL, ALPHA_DELTA, CG_X = 0.05, 0.20, 0.02, 2.0, None
AIRFOIL_CATALOG = "Medium_Re"
BASELINE_AIRFOIL = "mh60"
try:
    from rapid_config import load_effective_config
    _cfg = load_effective_config()
    _fl, _bd, _co, _bw = _cfg["flight"], _cfg["bounds"], _cfg["constraints"], _cfg["bwb"]
    V_MS  = float(_fl["V_KMH"]) / 3.6
    RHO   = float(_fl["RHO"]); TEMP_K = float(_fl["TEMP_K"]); MU = float(_fl["MU"])
    MACH  = V_MS / np.sqrt(1.4 * 287.05 * TEMP_K)
    S_REF = float(_fl["S_REF"]); AOA = float(_fl["AOA"]); INCIDENCE = float(_fl["INCIDENCE"])
    BOUNDS = [tuple(_bd[k]) for k in ("AR", "kink_frac", "t_inner", "t_outer", "sweep_inner", "sweep_outer")]
    LD_MIN = float(_co["LD_MIN"]); CM_MAX = float(_co["CM_MAX"]); CL_MIN = float(_co["CL_MIN"])
    BWB_MODE = bool(_bw["BWB_MODE"])
    SM_MIN = float(_bw["SM_MIN"]); SM_MAX = float(_bw["SM_MAX"])
    CM_TRIM_TOL = float(_bw["CM_TRIM_TOL"]); ALPHA_DELTA = float(_bw["ALPHA_DELTA"])
    _cgx = _bw.get("CG_X", None)
    CG_X = None if _cgx in (None, "", "none", "None") else float(_cgx)
    AIRFOIL_CATALOG = str(_bw.get("AIRFOIL_CATALOG", "Medium_Re"))
    BASELINE_AIRFOIL = str(_bw.get("BASELINE_AIRFOIL", "mh60") or "")
    print(f"[cfg] BWB_MODE={BWB_MODE} | L/D>={LD_MIN} CM<={CM_MAX} CL>={CL_MIN} | S_REF={S_REF} V={V_MS*3.6:.0f}km/h")
except Exception as _e:
    print(f"[cfg] Using built-in defaults (config load skipped: {_e})")

N_CONSTR = 5 if BWB_MODE else 3

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─── AIRFOIL CO-OPTIMIZATION FEEDBACK ────────────────────────────────────────
# Dictionary holding active profile assignments for Root, Kink, and Tip stations.
# Defaults to global AIRFOIL variable (e.g., naca2412) unless a back-feed JSON exists.
ACTIVE_AIRFOILS_CONFIG = os.path.join(SCRIPT_DIR, "active_airfoils.json")
ACTIVE_AIRFOILS = {
    "root": AIRFOIL,
    "kink": AIRFOIL,
    "tip":  AIRFOIL
}
if os.path.exists(ACTIVE_AIRFOILS_CONFIG):
    try:
        with open(ACTIVE_AIRFOILS_CONFIG, "r", encoding="utf-8") as f:
            loaded_cfg = json.load(f)
            if isinstance(loaded_cfg, dict):
                ACTIVE_AIRFOILS.update(loaded_cfg)
    except Exception:
        pass

# BWB: seed stations with a reflexed section so the first pass is trimmable.
if BWB_MODE:
    _still_baseline = all(not str(v).lower().endswith(".dat") for v in ACTIVE_AIRFOILS.values())
    if _still_baseline and BASELINE_AIRFOIL:
        _bn = BASELINE_AIRFOIL if BASELINE_AIRFOIL.lower().endswith(".dat") else BASELINE_AIRFOIL + ".dat"
        _bp = os.path.join(WORK_DIR, "airfoil_database", "Reflexed", _bn)
        if os.path.exists(_bp):
            ACTIVE_AIRFOILS = {"root": _bp, "kink": _bp, "tip": _bp}
            print(f"[cfg] BWB baseline reflexed section: {_bn}")
        else:
            print(f"[cfg] BWB baseline reflexed '{_bn}' not found in airfoil_database/Reflexed; using {AIRFOIL}")

# ─── YARDIMCI FONKSİYONLAR ───────────────────────────────────────────────────
def half_span_from_ar(AR):
    return min(np.sqrt(AR * S_REF) / 2.0, HALF_SPAN_MAX)

def chord_from_geometry(AR, kink_frac, t_inner, t_outer):
    b_half = half_span_from_ar(AR) - FUSELAGE_Y
    b_kink = kink_frac * b_half
    b_tip  = b_half - b_kink
    factor = 0.5*(1+t_inner)*b_kink + 0.5*t_inner*(1+t_outer)*b_tip
    c_root = (S_REF / 2.0) / factor
    if not (C_ROOT_MIN <= c_root <= C_ROOT_MAX):
        return None
    c_kink = t_inner * c_root
    c_tip  = t_outer * c_kink
    S_i = 0.5*(c_root+c_kink)*b_kink
    S_o = 0.5*(c_kink+c_tip)*b_tip
    mi  = (2/3)*c_root*(1+t_inner+t_inner**2)/(1+t_inner)
    mo  = (2/3)*c_kink*(1+t_outer+t_outer**2)/(1+t_outer)
    mac = (mi*S_i + mo*S_o) / (S_i + S_o)
    return c_root, c_kink, c_tip, b_half, b_kink, mac

# ─── RESULTS PERSISTENCE ──────────────────────────────────────────────────────
def save_results(algorithm, res=None):
    """Checkpoint and JSON exporter. Auto-extracts opt from algorithm state."""
    ckpt_file = os.path.join(OUTPUT_DIR, "checkpoint.pkl")
    try:
        with open(ckpt_file, "wb") as f:
            pickle.dump(algorithm, f)
        print(f"\n[✓] Checkpoint saved: {ckpt_file}")
    except Exception as e:
        print(f"\n[!] Checkpoint failed to save: {e}")

    # Save history for external plotting modules
    history_file = os.path.join(OUTPUT_DIR, "history.pkl")
    try:
        hist = getattr(algorithm, "history", None)
        if hist:
            with open(history_file, "wb") as f:
                pickle.dump(hist, f)
            print(f"[✓] {len(hist)} generation history saved: {history_file}")
    except Exception as e:
        print(f"[!] History failed to save: {e}")

    try:
        if res is not None and res.F is not None:
            F, X, G = res.F, res.X, res.G
        elif algorithm.opt is not None and len(algorithm.opt) > 0:
            F = algorithm.opt.get("F")
            X = algorithm.opt.get("X")
            G = algorithm.opt.get("G")
        else:
            print("[!] No Pareto solutions found to save.")
            return

        import glob
        folder_cache = []
        for p_file in glob.glob(os.path.join(OUTPUT_DIR, "*", "params.json")):
            try:
                with open(p_file, "r") as f:
                    pd = json.load(f)
                if "x" in pd and "name" in pd:
                    folder_cache.append({
                        "x": np.array(pd["x"]),
                        "name": pd["name"]
                    })
            except Exception:
                pass

        final_results = []
        sol_counter = 1
        for i, (f, x) in enumerate(zip(F, X)):
            if f[0] == 999.0:
                continue
            
            # Store strictly feasible designs only
            if G is not None:
                cv = np.sum(np.maximum(G[i], 0.0))
                if cv > 1e-6:
                    continue

            matching_name = ""
            matching_hash = ""
            best_dist = 1e9
            for item in folder_cache:
                dist = np.linalg.norm(x - item["x"])
                if dist < best_dist:
                    best_dist = dist
                    matching_name = item["name"]
            
            if best_dist < 1e-2 and matching_name:
                parts = matching_name.split("_")
                matching_hash = parts[-1] if len(parts) > 1 else ""
            else:
                matching_name = ""

            final_results.append({
                "solution_id": sol_counter,
                "CL":          round(-f[0], 5),
                "LD":          round(-f[1], 3),
                "AR":          round(x[0], 3),
                "kink_frac":   round(x[1], 3),
                "t_inner":     round(x[2], 3),
                "t_outer":     round(x[3], 3),
                "sweep_inner": round(x[4], 2),
                "sweep_outer": round(x[5], 2),
                "folder_name": matching_name,
                "hash":        matching_hash,
            })
            sol_counter += 1

        json_path = os.path.join(OUTPUT_DIR, "pareto_results.json")
        with open(json_path, "w") as f:
            json.dump(final_results, f, indent=2)
        print(f"[✓] {len(final_results)} Pareto solutions saved: {json_path}")

        if final_results:
            print(f"\n{'#':>4} {'CL':>8} {'L/D':>8} {'AR':>7} {'kink':>7} "
                  f"{'t_in':>7} {'t_out':>7} {'sw_in':>8} {'sw_out':>8}")
            print("-" * 70)
            for r in final_results:
                print(f"{r['solution_id']:>4} {r['CL']:>8.5f} {r['LD']:>8.3f} "
                      f"{r['AR']:>7.3f} {r['kink_frac']:>7.3f} "
                      f"{r['t_inner']:>7.3f} {r['t_outer']:>7.3f} "
                      f"{r['sweep_inner']:>8.1f} {r['sweep_outer']:>8.1f}")
            print("=" * 70)

    except Exception as e:
        print(f"[!] JSON failed to save: {e}")

# ─── SUBPROCESS WORKER BETİĞİ ────────────────────────────────────────────────
WORKER_SCRIPT = r"""
import sys, os, json, time, random

VSP_ROOT = sys.argv[1]
os.add_dll_directory(VSP_ROOT)

run_dir     = sys.argv[2]
params_file = sys.argv[3]
result_file = sys.argv[4]

os.chdir(run_dir)

# Dynamic DLL loader collision protection: retry module loads under Windows scanner locks
vsp, np = None, None
for _ in range(20):
    try:
        if vsp is None:
            import openvsp as vsp
        if np is None:
            import numpy as np
        break
    except Exception:
        time.sleep(0.1 + random.random() * 0.2)

if vsp is None or np is None:
    sys.exit("ERROR: Failed to load openvsp/numpy DLLs due to Windows concurrent scan lock")

# Safe parameter load with Windows OS lock backoff
p = None
for _ in range(20):
    try:
        with open(params_file, "r") as f:
            p = json.load(f)
        break
    except Exception:
        time.sleep(0.05 + random.random() * 0.1)

if p is None:
    sys.exit("ERROR: Failed to read parameters due to Windows OS lock")

# Tell OpenVSP where the central solver binaries are located
vsp.VSPCheckSetup()
vsp.SetVSPAEROPath(VSP_ROOT)

x         = p["x"]
name      = p["name"]
geo       = p["geo"]
b_ref     = p["b_ref"]
mac       = p["mac"]
base_vsp  = p["base_vsp"]
S_REF     = p["S_REF"]
AOA       = p["AOA"]
INCIDENCE = p["INCIDENCE"]
AIRFOIL   = p["AIRFOIL"]
MACH      = p["MACH"]
V_MS      = p["V_MS"]
RHO       = p["RHO"]
MU        = p["MU"]
WAKE_ITERATIONS = p.get("WAKE_ITERATIONS", 0)
ACTIVE_AIRFOILS = p.get("ACTIVE_AIRFOILS", {"root": AIRFOIL, "kink": AIRFOIL, "tip": AIRFOIL})
BWB_MODE    = p.get("BWB_MODE", False)
ALPHA_DELTA = p.get("ALPHA_DELTA", 2.0)
CG_X        = p.get("CG_X", None)

AR, kink_frac, t_inner, t_outer, sw_in, sw_out = x
c_root, c_kink, c_tip, b_half, b_kink, _mac = geo
b_tip_sec = b_half - b_kink

def sp(wid, xsec_idx, parm, value):
    xsec_surf = vsp.GetXSecSurf(wid, 0)
    xsec = vsp.GetXSec(xsec_surf, xsec_idx)
    pid = vsp.GetXSecParm(xsec, parm)
    if pid != "":
        vsp.SetParmVal(pid, value)

def set_airfoil(wid, xsec_idx, airfoil_def):
    xsec_surf = vsp.GetXSecSurf(wid, 0)
    
    # Dynamic airfoil selector: Checks if reference points to existing file or fallback to NACA 4-digit
    if isinstance(airfoil_def, str) and airfoil_def.lower().endswith(".dat") and os.path.exists(airfoil_def):
        vsp.ChangeXSecShape(xsec_surf, xsec_idx, vsp.XS_FILE_AIRFOIL)
        vsp.Update()
        xsec = vsp.GetXSec(xsec_surf, xsec_idx)
        vsp.ReadFileAirfoil(xsec, airfoil_def)
    else:
        vsp.ChangeXSecShape(xsec_surf, xsec_idx, vsp.XS_FOUR_SERIES)
        vsp.Update()
        xsec = vsp.GetXSec(xsec_surf, xsec_idx)
        code = str(airfoil_def).lower().replace("naca", "")
        # Fall back to the baseline NACA if the reference is not a 4-digit code.
        if not (code.isdigit() and len(code) >= 4):
            code = "2412"
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "Camber"),     int(code[0]) / 100.0)
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "CamberLoc"),  int(code[1]) / 10.0)
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "ThickChord"), int(code[2:]) / 100.0)
    vsp.Update()

vsp.VSPCheckSetup()
vsp.ClearVSPModel()
if os.path.exists(base_vsp):
    # Safe model load with Windows OS lock backoff
    for _ in range(20):
        try:
            vsp.ReadVSPFile(base_vsp)
            break
        except Exception:
            time.sleep(0.05 + random.random() * 0.1)

for g in vsp.FindGeomsWithName("Wing"):
    vsp.DeleteGeom(g)

wid = vsp.AddGeom("WING")
vsp.SetGeomName(wid, "Wing")

pid = vsp.FindParm(wid, "Sym_Planar_Flag", "Sym")
if pid != "": vsp.SetParmVal(pid, 2.0)
pid = vsp.FindParm(wid, "X_Rel_Rotation", "XForm")
if pid != "": vsp.SetParmVal(pid, INCIDENCE)

vsp.InsertXSec(wid, 1, vsp.XS_FOUR_SERIES)
vsp.Update()

sp(wid, 1, "Span",           b_kink)
sp(wid, 1, "Root_Chord",     c_root)
sp(wid, 1, "Tip_Chord",      c_kink)
sp(wid, 1, "Sweep",          sw_in)
sp(wid, 1, "Sweep_Location", 0.0)
sp(wid, 1, "Dihedral",       0.0)
sp(wid, 1, "Twist",          0.0)

sp(wid, 2, "Span",           b_tip_sec)
sp(wid, 2, "Root_Chord",     c_kink)
sp(wid, 2, "Tip_Chord",      c_tip)
sp(wid, 2, "Sweep",          sw_out)
sp(wid, 2, "Sweep_Location", 0.0)
sp(wid, 2, "Dihedral",       0.0)
sp(wid, 2, "Twist",          -INCIDENCE)
sp(wid, 2, "Twist_Location", 1.0)

pid_w = vsp.FindParm(wid, "Tess_W", "")
if pid_w != "": vsp.SetParmVal(pid_w, 40.0)
pid_u = vsp.FindParm(wid, "Tess_U", "")
if pid_u != "": vsp.SetParmVal(pid_u, 20.0)

set_airfoil(wid, 0, ACTIVE_AIRFOILS["root"])
set_airfoil(wid, 1, ACTIVE_AIRFOILS["kink"])
set_airfoil(wid, 2, ACTIVE_AIRFOILS["tip"])

vsp.SetSetFlag(wid, 3, True)
vsp.SetSetFlag(wid, 4, True)
vsp.Update()

vsp3_path = os.path.abspath(f"{name}.vsp3")
vsp.WriteVSPFile(vsp3_path)

vsp.VSPCheckSetup()
vsp.ClearVSPModel()
vsp.ReadVSPFile(vsp3_path)
vsp.Update()

vsp.SetAnalysisInputDefaults("VSPAEROComputeGeometry")
vsp.SetIntAnalysisInput("VSPAEROComputeGeometry", "AnalysisMethod", [0])
vsp.ExecAnalysis("VSPAEROComputeGeometry")

# Read the generated vspaero file, and update/rewrite its lines with our custom parameters
import subprocess
vspaero_file = f"{name}.vspaero"
Re_mac = RHO * V_MS * mac / MU

for _ in range(20):
    try:
        with open(vspaero_file, "r") as f:
            lines = f.readlines()
        break
    except Exception:
        time.sleep(0.05 + random.random() * 0.1)

new_lines = []
for line in lines:
    if "=" in line:
        parts = line.split("=")
        key = parts[0].strip()
        if key == "Sref":
            new_lines.append(f"Sref = {S_REF} \n")
        elif key == "Cref":
            new_lines.append(f"Cref = {mac} \n")
        elif key == "Bref":
            new_lines.append(f"Bref = {b_ref} \n")
        elif key == "Mach":
            new_lines.append(f"Mach = {MACH} \n")
        elif key == "AoA":
            if BWB_MODE:
                new_lines.append(f"AoA = {AOA}, {AOA + ALPHA_DELTA} \n")
            else:
                new_lines.append(f"AoA = {AOA} \n")
        elif key == "X_cg" and BWB_MODE:
            _cg = (0.25 * mac) if CG_X is None else CG_X
            new_lines.append(f"X_cg = {_cg} \n")
        elif key == "ReCref":
            new_lines.append(f"ReCref = {Re_mac} \n")
        elif key == "Vinf":
            new_lines.append(f"Vinf = {V_MS} \n")
        elif key == "Rho":
            new_lines.append(f"Rho = {RHO} \n")
        elif key == "WakeIters":
            new_lines.append(f"WakeIters = {WAKE_ITERATIONS} \n")
        else:
            new_lines.append(line)
    else:
        new_lines.append(line)

for _ in range(20):
    try:
        with open(vspaero_file, "w") as f:
            f.writelines(new_lines)
        break
    except Exception:
        time.sleep(0.05 + random.random() * 0.1)

# Run vspaero subprocess directly
vspaero_path = os.path.join(VSP_ROOT, "vspaero.exe" if sys.platform.startswith("win") else "vspaero")
subprocess.run(
    [vspaero_path, "-omp", "1", name],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

result = {"CL": None, "CD": None, "LD": None, "CM": None, "error": None}
try:
    polar_file = f"{name}.polar"
    if not os.path.exists(polar_file):
        result["error"] = f"Polar file {polar_file} not generated by VSPAERO"
    else:
        for _ in range(20):
            try:
                with open(polar_file, "r") as f:
                    p_lines = f.readlines()
                break
            except Exception:
                time.sleep(0.05 + random.random() * 0.1)
        
        headers = p_lines[2].strip().split()
        data_rows = []
        for _ln in p_lines[3:]:
            parts = _ln.strip().split()
            if len(parts) == len(headers):
                try:
                    data_rows.append([float(v) for v in parts])
                except ValueError:
                    pass
        if not data_rows:
            raise ValueError("No numeric polar rows parsed")

        def _row(vals):
            d = dict(zip(headers, vals))
            cl = d.get("CLwtot", d.get("CLtot", d.get("CL", 0.0)))
            cd = d.get("CDwtot", d.get("CDtot", d.get("CD", 0.0)))
            cm = d.get("CMytot", d.get("CMiy", d.get("CMoy", 0.0)))
            return cl, cd, cm

        cl, cd, cm = _row(data_rows[0])
        ld = cl / cd if cd != 0 else 0
        result.update({"CL": cl, "CD": cd, "LD": ld, "CM": cm})

        if BWB_MODE and len(data_rows) >= 2:
            cl2, cd2, cm2 = _row(data_rows[1])
            dCL = cl2 - cl
            dCmdCL = (cm2 - cm) / dCL if abs(dCL) > 1e-6 else 0.0
            result["dCmdCL"] = dCmdCL
            result["SM"] = -dCmdCL          # static margin (fraction of MAC, about X_cg)
            result["CM_trim"] = cm          # pitching moment at design alpha
except Exception as e:
    result["error"] = str(e)

# Safe result save with Windows OS lock backoff
for _ in range(20):
    try:
        with open(result_file, "w") as f:
            json.dump(result, f)
        break
    except Exception:
        time.sleep(0.05 + random.random() * 0.1)
"""

WORKER_SCRIPT_PATH = os.path.join(WORK_DIR, "_vsp_worker.py")

def write_worker_script():
    with open(WORKER_SCRIPT_PATH, "w", encoding="utf-8") as f:
        f.write(WORKER_SCRIPT)

# ─── MULTIPROCESSING INIT & LAUNCH LOCK ───────────────────────────────────────
launch_lock = None

def init_worker(l):
    global launch_lock
    launch_lock = l
    # Prevent pool workers from cluttering console tracebacks on Ctrl+C
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def evaluate_subprocess(x, wake_iterations=4):
    # Apply randomized fallback penalties for failed geometries
    def get_penalty():
        p = 999.0 + random.uniform(0.01, 5.0)
        return {"F": [p, p], "G": [p] * N_CONSTR}

    AR, kink_frac, t_inner, t_outer, sw_in, sw_out = x
    geo = chord_from_geometry(AR, kink_frac, t_inner, t_outer)
    if geo is None:
        return get_penalty()

    c_root, c_kink, c_tip, b_half, b_kink, mac = geo
    b_ref = 2.0 * b_half

    # ─── GEOMETRIC LABEL GENERATION ───────────────────────────────────────────
    b_theoretical = 2.0 * half_span_from_ar(AR)
    ar_val     = int(round(AR * 10))
    span_val   = int(round(b_theoretical))
    kink_val   = int(round(kink_frac * 10))
    sw_in_val  = int(round(sw_in))
    sw_out_val = int(round(sw_out))
    t_in_val   = int(round(t_inner * 10))
    t_out_val  = int(round(t_outer * 10))

    # Descriptive console tag (e.g. W92/40/4/3-3/5-2)
    pretty_name = f"W{ar_val}/{span_val}/{kink_val}/{sw_in_val}-{sw_out_val}/{t_in_val}-{t_out_val}"
    
    # Unique directory name appended with short UUID hash
    safe_label = f"W{ar_val}_{span_val}_{kink_val}_{sw_in_val}-{sw_out_val}_{t_in_val}-{t_out_val}"
    unique_id  = uuid.uuid4().hex[:6]
    name       = f"{safe_label}_{unique_id}"
    
    run_dir = os.path.join(OUTPUT_DIR, name)
    os.makedirs(run_dir, exist_ok=True)

    local_base_vsp = os.path.join(run_dir, "altc130mod_local.vsp3")
    
    # Define base model copy parameters
    copy_tasks = [(VSP_FILE, local_base_vsp, "Base VSP")]

    # File operations safeguarded against simultaneous read locks
    for target_src, target_dst, label in copy_tasks:
        if os.path.exists(target_src):
            success = False
            for attempt in range(20):
                try:
                    shutil.copyfile(target_src, target_dst)
                    success = True
                    break
                except (PermissionError, OSError):
                    time.sleep(0.03 + random.random() * 0.07)
            
            if not success:
                print(f"\n[!] Critical file copy failed ({label})")
                return get_penalty()

    params = {
        "x": list(x), "name": name, "geo": list(geo),
        "b_ref": b_ref, "mac": mac, "base_vsp": local_base_vsp,
        "S_REF": S_REF, "AOA": AOA, "INCIDENCE": INCIDENCE,
        "AIRFOIL": AIRFOIL, "MACH": MACH, "V_MS": V_MS,
        "RHO": RHO, "MU": MU, "WAKE_ITERATIONS": wake_iterations,
        "ACTIVE_AIRFOILS": ACTIVE_AIRFOILS,
        "BWB_MODE": BWB_MODE, "SM_MIN": SM_MIN, "SM_MAX": SM_MAX,
        "CM_TRIM_TOL": CM_TRIM_TOL, "ALPHA_DELTA": ALPHA_DELTA, "CG_X": CG_X,
    }
    params_file = os.path.join(run_dir, "params.json")
    result_file = os.path.join(run_dir, "result.json")
    with open(params_file, "w") as f:
        json.dump(params, f)

    # Pass stand-alone packages into subprocess syspath
    import sysconfig
    venv_site = sysconfig.get_path("purelib")
        
    final_worker_script = f"import sys; sys.path.insert(0, r'{venv_site}')\n" + WORKER_SCRIPT

    # Spawn primary binary directly
    python_bin = getattr(sys, "_base_executable", sys.executable)

    cmd = [python_bin, "-",
           VSP_ROOT, run_dir, params_file, result_file]
    env = os.environ.copy()
    env["OMP_NUM_THREADS"] = "1"
    env["PYTHONDONTWRITEBYTECODE"] = "1"

    # Stagger worker load timers
    time.sleep(random.random() * 2.0)

    exec_success = False
    for attempt in range(5):
        proc = None
        try:
            global launch_lock
            if launch_lock is not None:
                launch_lock.acquire()
            try:
                # Protect Windows dynamic loader bindings
                proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
                proc.stdin.write(final_worker_script)
                proc.stdin.flush()
                proc.stdin.close()
                time.sleep(0.25)
            finally:
                if launch_lock is not None:
                    launch_lock.release()
            
            try:
                stdout, stderr = proc.communicate(timeout=300)
            except subprocess.TimeoutExpired:
                proc.kill()
                stdout, stderr = proc.communicate()
                print("\n[!] Subprocess timed out.")
                break

            if proc.returncode == 0 and os.path.exists(result_file):
                exec_success = True
                break
            else:
                err_msg = f"{stderr or ''} {stdout or ''}"
                if "being used by another process" in err_msg.lower() or "access is denied" in err_msg.lower() or not os.path.exists(result_file):
                    time.sleep(0.5 + random.random() * 1.5)
                    continue
                else:
                    print(f"\n[!] Subprocess crash output:\n{stderr}\n{stdout}")
                    break
                    
        except Exception as e:
            print(f"\n[!] Subprocess execution error: {e}")
            if proc is not None:
                try: proc.kill()
                except: pass
            time.sleep(0.5)

    if not exec_success:
        return get_penalty()

    res = None
    for attempt in range(20):
        try:
            with open(result_file, "r") as f:
                res = json.load(f)
            break
        except (PermissionError, OSError, json.JSONDecodeError):
            time.sleep(0.05 + random.random() * 0.1)
            
    if res is None:
        print(f"[{pretty_name}] REJECTED: Result file empty (VSPAERO crashed)")
        sys.stdout.flush()
        return get_penalty()

    if res.get("error") or res["CL"] is None:
        err_msg = res.get("error", "Unknown VSPAERO Error")
        print(f"[{pretty_name}] REJECTED: {err_msg}")
        sys.stdout.flush()
        return get_penalty()

    CL = res["CL"]
    CD = res["CD"]
    LD = res["LD"]
    CM = abs(res["CM"])

    # Eliminate non-physical diverging configurations
    if LD > 60.0 or LD <= 0.0 or CL <= 0.0 or CL > 1.5:
        print(f"[{pretty_name}] REJECTED: Unphysical (CL: {CL:.2f}, L/D: {LD:.2f})")
        sys.stdout.flush()
        return get_penalty()

    print(f"[{pretty_name}] CL: {CL:.4f} | L/D: {LD:.2f}")
    sys.stdout.flush()

    f1 = -CL
    f2 = -LD
    g1 = LD_MIN - LD   # L/D >= LD_MIN
    g3 = CL_MIN - CL   # CL >= CL_MIN
    if BWB_MODE:
        # Tailless longitudinal stability + trim from a 2-point alpha slope.
        SM      = res.get("SM")
        CM_trim = res.get("CM_trim")
        if SM is None or CM_trim is None:
            return get_penalty()
        g_sm_lo = SM_MIN - SM                  # SM >= SM_MIN (stable enough)
        g_sm_hi = SM - SM_MAX                  # SM <= SM_MAX (not over-stable)
        g_trim  = abs(CM_trim) - CM_TRIM_TOL   # trimmable near the design point
        print(f"[{pretty_name}] SM: {SM*100:.1f}% | Cm_trim: {CM_trim:+.3f}")
        sys.stdout.flush()
        return {"F": [f1, f2], "G": [g1, g3, g_sm_lo, g_sm_hi, g_trim]}
    g2 = CM - CM_MAX   # |CM| <= CM_MAX
    return {"F": [f1, f2], "G": [g1, g2, g3]}

# ─── OPTİMİZASYON PROBLEMİ ───────────────────────────────────────────────────
class WingOptimization(ElementwiseProblem):
    def __init__(self, runner=None, wake_iterations=4):
        self.wake_iterations = wake_iterations
        xl = [b[0] for b in BOUNDS]
        xu = [b[1] for b in BOUNDS]
        kwargs = {}
        if runner is not None:
            kwargs["elementwise_runner"] = runner
        super().__init__(n_var=6, n_obj=2, n_ieq_constr=N_CONSTR, xl=xl, xu=xu, **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):
        result = evaluate_subprocess(x, wake_iterations=self.wake_iterations)
        out["F"] = result["F"]
        out["G"] = result["G"]

# ─── CUSTOM DISPLAY ──────────────────────────────────────────────────────────
class WingDisplay(Display):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.verbose = True  # Force print always!
        self.n_gen = Column("n_gen", width=6)
        self.n_eval = Column("n_eval", width=7)
        self.n_nds = Column("n_nds", width=6)
        self.ideal_ld = Column("ideal_LD", width=8)
        self.ideal_cl = Column("ideal_CL", width=8)
        self.eps_col = Column("eps", width=9)
        self.hv_col = Column("hv_area", width=9)
        self.cv_min = Column("cv_min", width=8)
        self.cv_avg = Column("cv_avg", width=8)
        
        self.columns = [
            self.n_gen, self.n_eval, self.n_nds, 
            self.ideal_ld, self.ideal_cl,
            self.cv_min, self.cv_avg,
            self.eps_col, self.hv_col
        ]
        self.prev_ideal = None
        
        try:
            from pymoo.indicators.hv import Hypervolume
            self.hv_calc = Hypervolume(ref_point=np.array([-0.15, -15.0]))
        except Exception:
            self.hv_calc = None

    def _do(self, problem, evaluator, algorithm, **kwargs):
        self.n_gen.set(algorithm.n_iter)
        self.n_eval.set(evaluator.n_eval)
        
        opt = algorithm.opt
        self.n_nds.set(len(opt))
        
        # Calculate min/avg constraint violations from population
        pop = algorithm.pop
        if pop is not None and len(pop) > 0:
            CV = pop.get("CV")
            if CV is not None and len(CV) > 0:
                self.cv_min.set(f"{np.min(CV):.4E}")
                self.cv_avg.set(f"{np.mean(CV):.4E}")
            else:
                self.cv_min.set("-")
                self.cv_avg.set("-")
        else:
            self.cv_min.set("-")
            self.cv_avg.set("-")
        
        if len(opt) > 0:
            F = opt.get("F")
            valid = F[F[:, 0] < 900]
            if len(valid) > 0:
                best_cl = -np.min(valid[:, 0])
                best_ld = -np.min(valid[:, 1])
                self.ideal_cl.set(f"{best_cl:.4f}")
                self.ideal_ld.set(f"{best_ld:.2f}")
                
                # 1. Epsilon (Relative improvement magnitude of vector)
                current_ideal = np.array([best_cl, best_ld])
                if self.prev_ideal is not None:
                    diff = current_ideal - self.prev_ideal
                    rel_change = diff / np.maximum(self.prev_ideal, 1e-6)
                    growth = np.sum(np.maximum(rel_change, 0.0))
                    
                    if growth > 0:
                        self.eps_col.set(f"{growth:.2E}")
                    else:
                        self.eps_col.set("0.00E+00")
                else:
                    self.eps_col.set("-")
                self.prev_ideal = current_ideal
                
                # 2. Hypervolume (Area inside boundary CL >= 0.2, LD >= 15)
                if self.hv_calc is not None:
                    try:
                        feasible = valid[(valid[:, 0] <= -0.15) & (valid[:, 1] <= -15.0)]
                        if len(feasible) > 0:
                            val = self.hv_calc.do(feasible)
                            self.hv_col.set(f"{val:.3f}")
                        else:
                            self.hv_col.set("0.000")
                    except Exception:
                        self.hv_col.set("0.000")
                else:
                    self.hv_col.set("-")
            else:
                self.ideal_cl.set("-")
                self.ideal_ld.set("-")
                self.eps_col.set("-")
                self.hv_col.set("-")
        else:
            self.ideal_cl.set("-")
            self.ideal_ld.set("-")
            self.eps_col.set("-")
            self.hv_col.set("-")

# ─── FLUSH CALLBACK ──────────────────────────────────────────────────────────
class FlushCallback(Callback):
    def notify(self, algorithm):
        # Validate and restore modern display table structures on checkpoint load
        if not isinstance(algorithm.display, WingDisplay) or not hasattr(algorithm.display, "cv_min"):
            old_prev = getattr(algorithm.display, "prev_ideal", None)
            algorithm.display = WingDisplay()
            if old_prev is not None:
                algorithm.display.prev_ideal = old_prev
            
        sys.stdout.flush()
        
        # Hook into Pymoo's stdout mechanism to append dynamic table headers sequentially
        if not hasattr(self, "_patched"):
            try:
                disp_cls  = type(algorithm.display)
                orig_call = disp_cls.__call__
                
                def custom_call(self_disp, *args, **kwargs):
                    try:
                        # Print boundary-aligned column labels prior to metric outputs
                        if hasattr(self_disp, 'output') and self_disp.output:
                            header_str = self_disp.output.header(border=True)
                            if header_str:
                                print('\n' + header_str)
                    except Exception:
                        pass
                    return orig_call(self_disp, *args, **kwargs)
                
                disp_cls.__call__ = custom_call
                self._patched = True
            except Exception:
                pass
        
        # Live save checkpoint and Pareto front at the end of every generation
        save_results(algorithm)
        sys.stdout.flush()

# ─── PARALLEL XFOIL CANDIDATE SWEEPER ────────────────────────────────────────
def _parallel_xfoil_worker(args):
    """Top-level picklable worker process function for dynamic pool mapping."""
    from xfoil_handler import analyze_airfoil
    airfoil_path, reynolds, mach, alphas = args
    try:
        res = analyze_airfoil(airfoil_path, reynolds, mach, alphas)
        return (os.path.basename(airfoil_path), res)
    except Exception:
        return (os.path.basename(airfoil_path), {})

def _get_airfoil_thickness(filepath):
    """Calculates approximate max thickness (t/c) of an airfoil from coordinate file."""
    try:
        y_vals = []
        with open(filepath, 'r') as f:
            for line in f.readlines():
                pts = line.split()
                if len(pts) >= 2:
                    try:
                        y_vals.append(float(pts[1]))
                    except ValueError:
                        pass
        if not y_vals: return 0.0
        return max(y_vals) - min(y_vals)
    except Exception:
        return 0.0

def sweep_candidates_for_sections(sections, catalog_dir, num_cores):
    """Loops through extraction stations, runs multi-core sweeps of all catalog profiles,
    filters pitching constraints, and yields top candidates by Aerodynamic Efficiency."""
    print(f"\n[*] Accessing catalog: {os.path.basename(catalog_dir)}")
    all_files = [os.path.join(catalog_dir, f) for f in os.listdir(catalog_dir) if f.lower().endswith(".dat")]
    
    if not all_files:
        print(f"[!] No airfoil coordinate .dat files located in {catalog_dir}!")
        return None
        
    print(f"[i] Preparing {len(all_files)} candidates across {num_cores} parallel cores...")
    
    final_selections = {}
    
    # Launch dynamic local pool
    pool = multiprocessing.Pool(processes=num_cores)
    
    try:
        for s in sections:
            sec_id = s["id"]
            zone   = s["zone"]
            Re     = s["Re"]
            alpha_center = s["alpha"]
            
            # Evaluate 3 neighboring angles centered at expected local incidence for robust matching
            alphas_to_test = [float(round(alpha_center - 2.0, 2)), 
                              float(round(alpha_center, 2)), 
                              float(round(alpha_center + 2.0, 2))]
            
            print(f"\n[+] Sweeping Station #{sec_id} ({zone}) | Target Re: {Re:,} | Sweep Alphas: {alphas_to_test}")
            
            # Form payload with Thickness PRE-SCREENING (C-130 Hercules Baseline)
            # C-130 Root: ~18% (NACA 64A318), Tip: ~12% (NACA 64A412)
            # We enforce minimum structural thickness bounds based on the zone
            if "ROOT" in zone.upper():
                min_thick = 0.18  # At least 18% thick (Exact C-130 18%)
            elif "TIP" in zone.upper():
                min_thick = 0.12  # At least 12% thick (C-130 tip is 12%)
            else:
                min_thick = 0.16  # Kink/Outer at least 16% thick
                
            mach_val = 0.1
            payload = []
            screened_out = 0
            
            for f in all_files:
                t = _get_airfoil_thickness(f)
                if t >= min_thick:
                    payload.append((f, Re, mach_val, alphas_to_test))
                else:
                    screened_out += 1
            
            print(f"    [i] Structural Screen: {screened_out} thin airfoils rejected. {len(payload)} robust candidates proceed.")
            
            t0 = time.time()
            # Dispatch batch jobs across cores
            results = pool.map(_parallel_xfoil_worker, payload)
            t1 = time.time()
            
            print(f"    Analysis completed in {t1-t0:.2f} seconds.")
            
            # Collate performance metrics
            candidates = []
            for name, polar in results:
                if not polar:
                    continue
                    
                cls = [d["CL"] for d in polar.values()]
                cms = [d["CM"] for d in polar.values()]
                lds = [d["LD"] for d in polar.values()]
                
                # Fewer than two converged alphas means an unreliable / blown-up
                # solve, so skip the section entirely.
                if len(cls) < 2:
                    continue
                    
                avg_ld = float(np.mean(lds))
                avg_cm = float(np.mean(cms))
                
                # Check for physically unrealistic data (XFOIL anomalies)
                # and DELETE the corrupted profile from the active database permanently!
                is_corrupted = False
                if Re > 10000000 and avg_ld > 200.0:
                    is_corrupted = True
                elif Re <= 10000000 and avg_ld > 100.0:
                    is_corrupted = True
                    
                if is_corrupted:
                    corrupted_path = os.path.join(catalog_dir, name)
                    try:
                        if os.path.exists(corrupted_path):
                            os.remove(corrupted_path)
                            print(f"    [!] Purged unphysical airfoil from database: {name} (Avg L/D: {avg_ld:.1f} at Re: {Re:,})")
                    except Exception:
                        pass
                    continue
                
                # Pitching-moment envelope: BWB sections need near-zero/positive Cm to self-trim.
                if BWB_MODE:
                    if avg_cm < -0.03:
                        continue
                else:
                    if avg_cm < -0.08:
                        continue
                    
                candidates.append({
                    "name": name,
                    "avg_ld": avg_ld,
                    "avg_cm": avg_cm,
                    "polar": polar
                })
                
            # Rank candidates. Conventional: pure L/D. BWB/tailless: reward high L/D
            # but strongly prefer near-zero / slightly positive Cm (reflexed) for trim.
            if BWB_MODE:
                candidates.sort(key=lambda x: x["avg_ld"] - 200.0 * abs(x["avg_cm"] - 0.01), reverse=True)
            else:
                candidates.sort(key=lambda x: x["avg_ld"], reverse=True)
            
            if not candidates:
                print(f"    [!] Zero feasible profiles converged for Station #{sec_id}.")
                final_selections[sec_id] = None
            else:
                best = candidates[0]
                print(f"    [✓] Station Winner: {best['name']} (Average L/D: {best['avg_ld']:.2f}, CM: {best['avg_cm']:.4f})")
                final_selections[sec_id] = {
                    "id": sec_id,
                    "zone": zone,
                    "winner": best["name"],
                    "winner_ld": best["avg_ld"],
                    "winner_cm": best["avg_cm"],
                    "ranking": candidates[:5] # Store top 5 for UI representation
                }
                
    finally:
        pool.close()
        pool.join()
        
    return final_selections

# ─── GEOMETRIC REFINEMENT SUBSYSTEM ──────────────────────────────────────────
def _parallel_geom_refine_worker(args):
    """Top-level picklable worker for parallel geometry tweaks (TFAC scaling)."""
    from xfoil_handler import analyze_airfoil
    airfoil_path, reynolds, mach, alphas, geom_factors = args
    try:
        res = analyze_airfoil(airfoil_path, reynolds, mach, alphas, geom_factors=geom_factors)
        return (geom_factors, res)
    except Exception:
        return (geom_factors, {})

def optimize_airfoil_geometry(airfoil_path, Re, target_alpha, original_cm, num_cores):
    """Sweeps variations of thickness and camber in parallel, keeping CM constant to maximize L/D."""
    print(f"\n[*] Launching AUTONOMOUS SHAPE REFINEMENT for {os.path.basename(airfoil_path)}")
    print(f"    Optimizing at Local Flow: Re={Re:,} | Alpha={target_alpha:.2f} | Target CM={original_cm:.4f}")
    
    # Construct 2D scaling search space for Thickness and Camber
    t_factors = [0.90, 0.95, 1.00, 1.05, 1.10]
    c_factors = [0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20]
    
    grid = []
    for t in t_factors:
        for c in c_factors:
            grid.append((t, c))
            
    print(f"[i] Generating {len(grid)} variant geometries. Dispatched across {num_cores} cores...")
    
    pool = multiprocessing.Pool(processes=num_cores)
    
    try:
        mach_val = 0.1
        alphas_to_test = [round(target_alpha, 2)]
        
        # Formulate payloads
        payload = [(airfoil_path, Re, mach_val, alphas_to_test, factors) for factors in grid]
        
        t0 = time.time()
        sweep_results = pool.map(_parallel_geom_refine_worker, payload)
        t1 = time.time()
        print(f"    Refinement variations completed in {t1-t0:.2f} seconds.")
        
        candidates = []
        target_k = round(target_alpha, 2)
        
        for (tf, cf), polar in sweep_results:
            if not polar:
                continue
            
            # Pull result for target angle
            data = None
            for alpha_key in polar.keys():
                if abs(alpha_key - target_k) < 0.05:
                    data = polar[alpha_key]
                    break
            
            if not data:
                continue
                
            ld = float(data["LD"])
            cm = float(data["CM"])
            
            # STABILITY ENVELOPE CONSTRAINT: Keep CM constant!
            # We ensure pitching variation does not drift beyond highly tight bounds (abs delta <= 0.0075)
            if abs(cm - original_cm) > 0.0075:
                continue
                
            candidates.append({
                "factors": (tf, cf),
                "ld": ld,
                "cm": cm
            })
            
        # Rank by Aerodynamic efficiency descending
        candidates.sort(key=lambda x: x["ld"], reverse=True)
        
        if not candidates:
            print("    [!] Optimization sweep returned zero feasible shapes matching stability constraint bounds.")
            return None
            
        best = candidates[0]
        baseline = None
        for c in candidates:
            if abs(c["factors"][0] - 1.0) < 1e-3 and abs(c["factors"][1] - 1.0) < 1e-3:
                baseline = c
                break
                
        base_ld = baseline["ld"] if baseline else best["ld"]
        improvement = ((best["ld"] - base_ld) / base_ld * 100) if base_ld > 0 else 0.0
        
        print("\n" + "=" * 60)
        print(" AIRFOIL GEOMETRIC REFINEMENT SUMMARY")
        print("=" * 60)
        print(f" Baseline Profile (x1.0):   L/D = {base_ld:.2f}")
        print(f" Optimized Profile Variant:  L/D = {best['ld']:.2f}  (Thick x{best['factors'][0]:.2f}, Camber x{best['factors'][1]:.2f})")
        print(f" Net Performance Shift:      +{improvement:.2f}% Efficiency Gain")
        print(f" Final Pitching Moment CM:   {best['cm']:.4f} (Baseline: {original_cm:.4f})")
        print("=" * 60)
        print("=" * 60)
        
        return {"best": best, "baseline": baseline}
        
    finally:
        pool.close()
        pool.join()

# ─── SECTIONAL EXTRACTION UTILITY ───────────────────────────────────────────
def extract_sectional_properties(selected_wing, num_sections):
    """Linearly slices the wing half-span and calculates local chord, twist,
    local Reynolds number, and local geometric angle of attack for each station."""
    AR          = selected_wing["AR"]
    kink_frac   = selected_wing["kink_frac"]
    t_inner     = selected_wing["t_inner"]
    t_outer     = selected_wing["t_outer"]
    
    res = chord_from_geometry(AR, kink_frac, t_inner, t_outer)
    if not res:
        return None
        
    c_root, c_kink, c_tip, b_half, b_kink, mac = res
    
    # Distribute linearly from Root (y=0) to Tip (y=b_half)
    y_stations = np.linspace(0.0, b_half, num_sections)
    
    sections = []
    for i, y in enumerate(y_stations):
        # Structural boundary assignment
        if y <= b_kink:
            frac = y / b_kink if b_kink > 0.0 else 0.0
            c_loc = c_root + (c_kink - c_root) * frac
            twist_loc = 0.0
            zone = "ROOT" if i == 0 else "INNER"
        else:
            frac = (y - b_kink) / (b_half - b_kink) if (b_half - b_kink) > 0.0 else 0.0
            c_loc = c_kink + (c_tip - c_kink) * frac
            twist_loc = -INCIDENCE * frac
            zone = "TIP" if i == num_sections - 1 else "OUTER"
            
        # Flow math: Re_local = (rho * V * c) / mu
        Re_loc = RHO * V_MS * c_loc / MU
        alpha_loc = AOA + INCIDENCE + twist_loc
        
        sections.append({
            "id": i + 1,
            "y_loc": y,
            "chord": c_loc,
            "twist": twist_loc,
            "alpha": alpha_loc,
            "Re": int(round(Re_loc)),
            "zone": zone
        })
        
    return {
        "sections": sections,
        "c_root": c_root, "c_kink": c_kink, "c_tip": c_tip,
        "b_half": b_half, "b_kink": b_kink, "mac": mac
    }

# ─── MAIN LAUNCHER ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import openvsp as vsp
    print("VSP Version: " + str(vsp.GetVSPVersion()))
    
    print("=" * 70)
    print(f"Constraints: L/D >= {LD_MIN} | CM <= {CM_MAX} | CL >= {CL_MIN}")
    print("=" * 70)

    # ─── CO-OPTIMIZATION STATUS MONITOR ─────────────────────────────────────────
    # Instantly report to the user if customized persistence shapes are being enforced!
    print("\n" + "-" * 70)
    is_custom = any([isinstance(v, str) and v.lower().endswith(".dat") for v in ACTIVE_AIRFOILS.values()])
    if is_custom:
        print("[✓] CO-OPTIMIZATION ACTIVE: Enforcing Custom Multi-Sectional Profiles!")
        print("    - Root Cross-Section: " + os.path.basename(ACTIVE_AIRFOILS["root"]))
        print("    - Kink Cross-Section: " + os.path.basename(ACTIVE_AIRFOILS["kink"]))
        print("    - Tip Cross-Section:  " + os.path.basename(ACTIVE_AIRFOILS["tip"]))
        print("    (Every wing generated in this optimization session will utilize these customized shapes)")
    else:
        print("[i] CO-OPTIMIZATION INACTIVE: Executing Standard Uniform Sweep.")
        print(f"    - All wing sections default to baseline uniform profile: {AIRFOIL.upper()}")
    print("-" * 70)

    # ─── CORE SELECTION ───────────────────────────────────────────────────────
    default_cores = 32
    while True:
        core_input = input(f"\nNumber of CPU cores to use? (Enter = default {default_cores}): ").strip()
        if not core_input:
            num_cores = default_cores
            break
        try:
            num_cores = int(core_input)
            if num_cores > 0:
                break
            print("    Please enter a positive integer.")
        except ValueError:
            print("    Invalid input, please enter an integer.")

    # ─── WAKE ITERATION SELECTION ─────────────────────────────────────────────
    default_wake = 0
    while True:
        wake_input = input(f"VSPAERO Wake Iteration count? (Enter = default {default_wake}): ").strip()
        if not wake_input:
            wake_iterations = default_wake
            break
        try:
            wake_iterations = int(wake_input)
            if wake_iterations >= 0:
                break
            print("    Please enter a positive integer or 0.")
        except ValueError:
            print("    Invalid input, please enter an integer.")

    ckpt_file = os.path.join(OUTPUT_DIR, "checkpoint.pkl")
    
    # ─── CORE EXECUTION CONTROL LOOP ──────────────────────────────────────────
    # This allows looping back to extend generations OR entering airfoil modules
    while True:
        ckpt_exists = os.path.exists(ckpt_file)
        extra = 0
        pop_size = 32

        if ckpt_exists:
            try:
                with open(ckpt_file, "rb") as f:
                    temp_alg = pickle.load(f)
                current_gen_preview = getattr(temp_alg, "n_iter", 0)
                active_pop = getattr(temp_alg, "pop_size", "Unknown")
                print(f"\n[i] Existing checkpoint found — generation {current_gen_preview} completed.")
                print(f"[i] Population size restored from checkpoint: {active_pop} wings")
            except Exception:
                print("\n[!] Failed to preview checkpoint. Proceeding as active.")
            
            skip_opt = False
            while True:
                ans_skip = input("\n[?] Bypass planform generations and jump straight to Airfoil Co-Optimization? (Y/N) [Enter = N]: ").strip().lower()
                if not ans_skip:
                    ans_skip = 'n'
                if ans_skip in ['y', 'n']:
                    skip_opt = (ans_skip == 'y')
                    break
                print("    Please answer 'Y' or 'N'.")
                
            if skip_opt:
                extra = 0
            else:
                while True:
                    try:
                        extra_input = input("How many MORE generations do you wish to run? (Enter = default 10): ").strip()
                        if not extra_input:
                            extra = 10
                            break
                        extra = int(extra_input)
                        if extra > 0:
                            break
                        print("    Please enter a positive integer.")
                    except ValueError:
                        print("    Invalid input, please enter an integer.")
        else:
            print("\n[i] No checkpoint found — starting fresh optimization.")
            default_pop = 32
            while True:
                pop_input = input(f"Wings per generation (pop_size)? (Enter = default {default_pop}): ").strip()
                if not pop_input:
                    pop_size = default_pop
                    break
                try:
                    pop_size = int(pop_input)
                    if pop_size > 0:
                        break
                    print("    Please enter a positive integer.")
                except ValueError:
                    print("    Invalid input, please enter an integer.")

            while True:
                try:
                    extra_input = input("How many generations do you wish to run? (Enter = default 20): ").strip()
                    if not extra_input:
                        extra = 20
                        break
                    extra = int(extra_input)
                    if extra > 0:
                        break
                    print("    Please enter a positive integer.")
                except ValueError:
                    print("    Invalid input, please enter an integer.")

        print(f"\n[i] Launching: Activating {num_cores} cores...")
        sys.stdout.flush()
        write_worker_script()

        lock    = multiprocessing.Lock()
        pool    = multiprocessing.Pool(num_cores, initializer=init_worker, initargs=(lock,))
        runner  = StarmapParallelization(pool.starmap)
        problem = WingOptimization(runner=runner, wake_iterations=wake_iterations)

        if ckpt_exists:
            print("\n[i] Restored previous optimization state!")
            try:
                with open(ckpt_file, "rb") as f:
                    algorithm = pickle.load(f)
                algorithm.display = WingDisplay()
                algorithm.display.verbose = True
                current_gen = getattr(algorithm, "n_iter", 0)
                target_gen  = current_gen + extra
                from pymoo.termination import get_termination
                algorithm.termination = get_termination("n_gen", target_gen)
                problem.elementwise_runner = runner
                if hasattr(algorithm, "problem"):
                    algorithm.problem = problem
                print(f"[i] Resuming from gen {current_gen} → Target: gen {target_gen} ({extra} more)")
            except Exception as e:
                print(f"[!] Failed to load checkpoint: {e}. Aborting run.")
                pool.terminate()
                pool.join()
                break
        else:
            print("\n[i] Starting new optimization...")
            algorithm = NSGA2(
                pop_size=pop_size,
                n_offsprings=pop_size,
                sampling=FloatRandomSampling(),
                crossover=SBX(prob=0.9, eta=15),
                mutation=PM(eta=20),
                eliminate_duplicates=True,
            )
            target_gen = extra
            print(f"[i] Pop size: {pop_size} | Target: {target_gen} generations")

        rnd_seed = random.randint(1, 10000)
        res = None
        algorithm.verbose = True
        algorithm.display = WingDisplay()

        sys.stdout.flush()
        try:
            res = minimize(
                problem, algorithm,
                ("n_gen", target_gen),
                seed=rnd_seed,
                copy_algorithm=False,
                verbose=True,
                display=WingDisplay(),
                callback=FlushCallback(),
                save_history=True,
            )
            print("\n" + "=" * 70)
            print("GENERATION BATCH COMPLETED SUCCESSFULLY!")
            print("=" * 70)
        except KeyboardInterrupt:
            print("\n\n[!] Interrupted by user.")
        finally:
            pool.terminate()
            pool.join()
            save_results(algorithm, res)

        # ─── POST-OPTIMIZATION CONTROL LOGIC ──────────────────────────────────
        sys.stdout.flush()
        while True:
            ans_airfoil = input("\nDo you wish to proceed to Airfoil Co-Optimization? (Y/N): ").strip().lower()
            if ans_airfoil in ['y', 'n']:
                break
            print("    Please answer 'Y' or 'N'.")

        if ans_airfoil == 'y':
            print("\n" + "═" * 70)
            print(" ENTERING AIRFOIL CO-OPTIMIZATION MODULE")
            print("═" * 70)
            
            # Auto-run performance plotter to show the Pareto front to the user
            try:
                plotter_bat = os.path.join(WORK_DIR, "RUN_PERFORMANCE_PLOTTER.bat")
                if os.path.exists(plotter_bat):
                    print("[*] Launching Performance Plotter in a new window...")
                    # 'start' ensures it opens in a separate, non-blocking CMD window
                    subprocess.Popen(f'start "" "{plotter_bat}"', shell=True)
                    time.sleep(1.0)
            except Exception as e:
                print(f"[i] Could not auto-launch plotter: {e}")
            
            # Prompt for specific candidate index from Pareto front
            pareto_json = os.path.join(OUTPUT_DIR, "pareto_results.json")
            if not os.path.exists(pareto_json):
                print("[!] No Pareto results found! Please complete an optimization run first.")
                continue
                
            try:
                with open(pareto_json, "r") as f:
                    pareto_data = json.load(f)
            except Exception as e:
                print(f"[!] Failed to load Pareto data: {e}")
                continue
                
            if not pareto_data:
                print("[!] Pareto list is empty! No feasible wings to select.")
                continue
                
            while True:
                idx_input = input(f"\nEnter index of wing to optimize from Pareto front (1-{len(pareto_data)}): ").strip()
                try:
                    wing_idx = int(idx_input)
                    if 1 <= wing_idx <= len(pareto_data):
                        selected_wing = next((w for w in pareto_data if w["solution_id"] == wing_idx), None)
                        if selected_wing:
                            break
                    print(f"    Please enter an index between 1 and {len(pareto_data)}.")
                except ValueError:
                    print("    Invalid input, please enter an integer.")
            
            print(f"\n[✓] Selected Wing Solution ID #{selected_wing['solution_id']} (CL={selected_wing['CL']}, L/D={selected_wing['LD']})")
            
            # Extract target geometric boundaries
            AR_sel          = selected_wing["AR"]
            kink_frac_sel   = selected_wing["kink_frac"]
            t_inner_sel     = selected_wing["t_inner"]
            t_outer_sel     = selected_wing["t_outer"]
            sweep_inner_sel = selected_wing["sweep_inner"]
            sweep_outer_sel = selected_wing["sweep_outer"]
            
            print(f"[i] Base Geometry parameters pulled: AR={AR_sel} | Kink={kink_frac_sel} | t={t_inner_sel}/{t_outer_sel}")
            
            # Prompt for section extraction count (N)
            while True:
                sec_input = input("\nEnter number of extraction sections (N) [e.g. 3 = Root/Kink/Tip]: ").strip()
                try:
                    num_sec = int(sec_input)
                    if num_sec >= 2:
                        break
                    print("    Please enter at least 2 sections.")
                except ValueError:
                    print("    Invalid input, please enter an integer.")
            
            # Run scientific extraction
            print(f"\n[*] Computing local flow conditions for {num_sec} stations...")
            results = extract_sectional_properties(selected_wing, num_sec)
            
            if not results:
                print("[!] Geometry calculations failed.")
                continue
                
            # Display Gorgeous Mathematical Properties Table (ASCII-safe)
            print("\n" + "=" * 80)
            print(f" STATIONARY EXTRACTION REPORT — SOLUTION #{selected_wing['solution_id']}")
            print("=" * 80)
            print(f"{'Sec ID':^8} | {'Zone':^8} | {'y [m]':^10} | {'Chord [m]':^10} | {'Twist [deg]':^12} | {'Alpha [deg]':^12} | {'Re':^10}")
            print("-" * 80)
            for s in results["sections"]:
                print(f"{s['id']:^8} | {s['zone']:^8} | {s['y_loc']:^10.3f} | {s['chord']:^10.3f} | {s['twist']:^12.2f} | {s['alpha']:^12.2f} | {s['Re']:^10,}")
            print("=" * 80)
            
            print("\n[✓] Section properties successfully computed!")
            
            # ─── STEP 2: RUN PARALLEL AIRFOIL SWEEPER ────────────────────────
            print("\n" + "=" * 80)
            print(" AUTONOMOUS CANDIDATE SWEEP SELECTION")
            print("=" * 80)
            print("Available catalogs for aerodynamic profile matching:")
            print("  [1] Low_Re     (Curated profiles for low speeds)")
            print("  [2] Medium_Re  (Curated profiles for general speeds)")
            print("  [3] High_Re    (Curated high-efficiency profiles)")
            print("  [4] UIUC_All   (Full UIUC database, ~1650 airfoils - recommended, slow)")
            print("  [5] Reflexed   (Tailless / BWB self-trimming sections)")

            catalog_map = {"1": "Low_Re", "2": "Medium_Re", "3": "High_Re",
                           "4": "UIUC_All", "5": "Reflexed"}
            _rev_map = {v: k for k, v in catalog_map.items()}
            _default_choice = _rev_map.get(AIRFOIL_CATALOG, "5" if BWB_MODE else "2")

            while True:
                cat_input = input(f"\nSelect catalog number (1-5) [Enter = default {_default_choice} ({catalog_map[_default_choice]})]: ").strip()
                if not cat_input:
                    cat_choice = _default_choice
                    break
                if cat_input in catalog_map:
                    cat_choice = cat_input
                    break
                print("    Please choose between 1 and 5.")

            catalog_folder = catalog_map[cat_choice]
            # Note: airfoil_database lives in the parent dir or root, SCRIPT_DIR points to system_files
            catalog_path = os.path.join(os.path.dirname(SCRIPT_DIR), "airfoil_database", catalog_folder)
            
            if not os.path.exists(catalog_path):
                print(f"[!] Path invalid: {catalog_path}. Defaulting to Medium_Re.")
                catalog_path = os.path.join(os.path.dirname(SCRIPT_DIR), "airfoil_database", "Medium_Re")

            # Perform parallel sweeps!
            print(f"\n[*] Commencing batch aerodynamic matching using {num_cores} cores...")
            selections = sweep_candidates_for_sections(results["sections"], catalog_path, num_cores)
            
            if not selections:
                print("\n[!] Sweeping failed or aborted.")
                continue
                
            # Display Golden Summary!
            print("\n" + "=" * 80)
            print(" OPTIMAL SECTIONAL MATCHING RECOMMENDATIONS")
            print("=" * 80)
            print(f"{'Sec':^4} | {'Zone':^6} | {'Winning Airfoil':^25} | {'Avg L/D':^10} | {'Avg CM':^10}")
            print("-" * 80)
            for sec_id, data in selections.items():
                if not data:
                    print(f" {sec_id:<2} | --- | {'[NO CONVERGENCE]':^25} | {'---':^10} | {'---':^10}")
                else:
                    print(f" {data['id']:^2} | {data['zone']:^6} | {data['winner']:^25} | {data['winner_ld']:^10.2f} | {data['winner_cm']:^10.4f}")
            print("=" * 80)
            
            # --- AUTO-PLOT WINNING AIRFOILS ---
            import matplotlib.pyplot as plt
            try:
                plt.figure(figsize=(10, 5))
                plt.title("Optimal Sectional Airfoils", fontweight='bold')
                plt.xlabel("x/c")
                plt.ylabel("y/c")
                plt.grid(True, linestyle='--', alpha=0.6)
                plt.axis("equal")
                for sec_id, data in selections.items():
                    if data and data["winner"]:
                        af_path = os.path.join(catalog_path, data["winner"])
                        if os.path.exists(af_path):
                            coords = []
                            with open(af_path, "r") as f:
                                lines = f.readlines()
                                # handle headers by skipping alphanumeric lines
                                for line in lines:
                                    pts = line.split()
                                    if len(pts) >= 2:
                                        try:
                                            coords.append([float(pts[0]), float(pts[1])])
                                        except ValueError:
                                            pass
                            if coords:
                                coords = np.array(coords)
                                plt.plot(coords[:,0], coords[:,1], label=f"Sec {sec_id} ({data['zone']}): {data['winner']}")
                plt.legend()
                plt.tight_layout()
                print("\n[*] Opening Interactive Airfoil Viewer... (Close the window to proceed)")
                plt.show()
            except Exception as e:
                print(f"[!] Could not display airfoil plotter: {e}")
                
            # ─── AUTONOMOUS XFOIL GEOMETRIC REFINEMENT INTEGRATION ───────────
            print("\n" + "=" * 80)
            while True:
                ans_opt_geom = input("Perform autonomous geometry optimization (tweak thickness/camber) to boost L/D while keeping CM constant? (Y/N): ").strip().lower()
                if ans_opt_geom in ['y', 'n']:
                    break
                print("    Please answer 'Y' or 'N'.")
                
            if ans_opt_geom == 'y':
                print("\n[*] Commencing parallel geometric tweaks on winning airfoils...")
                
                # Prepare safe folder for custom refined coordinates
                active_af_dir = os.path.join(SCRIPT_DIR, "active_airfoils")
                os.makedirs(active_af_dir, exist_ok=True)
                
                for sec_id, data in selections.items():
                    if not data or not data["winner"]:
                        continue
                        
                    sec_data = results["sections"][sec_id - 1]
                    
                    # Target conditions for optimizer
                    af_path = os.path.join(catalog_path, data["winner"])
                    Re = sec_data["Re"]
                    alpha = sec_data["alpha"]
                    target_cm = data["winner_cm"]
                    
                    print(f"\n>>> Optimizing Section #{sec_id} ({data['zone']}) based on {data['winner']}...")
                    
                    opt_res = optimize_airfoil_geometry(af_path, Re, alpha, target_cm, num_cores)
                    if opt_res and opt_res["best"]["factors"] != (1.0, 1.0):
                        best_data = opt_res["best"]
                        base_data = opt_res["baseline"]
                        
                        # Correctly compare the baseline AT THIS EXACT ALPHA to the tweaked shape
                        old_ld = base_data["ld"] if base_data else data["winner_ld"]
                        old_cm = base_data["cm"] if base_data else data["winner_cm"]
                        new_ld = best_data["ld"]
                        new_cm = best_data["cm"]
                        
                        print(f"\n    [?] GEOMETRY OPTIMIZATION PROPOSAL (Section #{sec_id}):")
                        print(f"        Original L/D: {old_ld:^8.1f}  ->  Tweaked L/D: {new_ld:^8.1f}")
                        print(f"        Original CM:  {old_cm:^8.4f}  ->  Tweaked CM:  {new_cm:^8.4f}")
                        
                        while True:
                            ans_acc = input("        Accept this tweaked shape? (Y/N) [Enter = Y]: ").strip().lower()
                            if not ans_acc: ans_acc = 'y'
                            if ans_acc in ['y', 'n']: break
                            print("        Please answer Y or N.")
                            
                        if ans_acc == 'y':
                            # Discovery! Export tweaked profile
                            t_fac, c_fac = best_data["factors"]
                            
                            safe_af_name = data["winner"].lower().replace(".dat", "")
                            custom_filename = f"opt_{data['zone'].lower()}_{safe_af_name}_t{t_fac:.2f}_c{c_fac:.2f}.dat"
                            export_path = os.path.abspath(os.path.join(active_af_dir, custom_filename))
                            
                            # Single XFOIL run to generate and save coordinate geometry
                            analyze_airfoil(
                                af_path, 
                                Re, 
                                mach=0.1, 
                                alphas=[round(alpha, 2)], 
                                geom_factors=(t_fac, c_fac), 
                                save_coords_path=export_path
                            )
                            
                            if os.path.exists(export_path):
                                print(f"    [✓] Successfully applied optimized coordinate file: {custom_filename}")
                                # Re-route selection pointers
                                data["winner"] = custom_filename
                                data["winner_path"] = export_path # direct absolute path
                                data["winner_ld"] = best_data["ld"]
                                data["winner_cm"] = best_data["cm"]
                            else:
                                print("    [!] Coordinate extraction failure. Reverting to baseline shape.")
                        else:
                            print(f"    [i] Tweak rejected. Retaining baseline shape for Section #{sec_id}.")
                    else:
                        print(f"    [i] Baseline profile is the optimal geometric variant for Section #{sec_id}.")
                        

                        
                # Print Revised recommendations table
                print("\n" + "=" * 80)
                print(" REVISED SECTIONAL RECOMMENDATIONS (AFTER GEOMETRIC REFINEMENT)")
                print("=" * 80)
                print(f"{'Sec':^4} | {'Zone':^6} | {'Winning Airfoil':^25} | {'Avg L/D':^10} | {'Avg CM':^10}")
                print("-" * 80)
                for sec_id, data in selections.items():
                    if not data:
                        print(f" {sec_id:<2} | --- | {'[NO CONVERGENCE]':^25} | {'---':^10} | {'---':^10}")
                    else:
                        disp_name = data["winner"]
                        print(f" {data['id']:^2} | {data['zone']:^6} | {disp_name:^25} | {data['winner_ld']:^10.2f} | {data['winner_cm']:^10.4f}")
                print("=" * 80)

            # ─── BACK-FEED TO OPENVSP TRIGGER ────────────────────────────────
            print("\n" + "=" * 80)
            while True:
                ans_feed = input("Back-feed these winning profiles into OpenVSP for Phase-2 Optimization? (Y/N): ").strip().lower()
                if ans_feed in ['y', 'n']:
                    break
                print("    Please enter Y or N.")
                
            if ans_feed == 'y':
                sec_ids = sorted(list(selections.keys()))
                active_af_dir = os.path.join(SCRIPT_DIR, "active_airfoils")
                os.makedirs(active_af_dir, exist_ok=True)
                
                root_data = selections.get(sec_ids[0])
                tip_data  = selections.get(sec_ids[-1])
                mid_idx   = len(sec_ids) // 2
                kink_data = selections.get(sec_ids[mid_idx])
                
                new_config = {}
                
                def deploy_airfoil(zone, data_dict):
                    if not data_dict or not data_dict.get("winner"):
                        print(f"[i] Section {zone.upper()} has no valid convergence data. Defaulting.")
                        return AIRFOIL
                    
                    filename = data_dict["winner"]
                    
                    # Detect if file already resides in local active directory from optimization
                    if "winner_path" in data_dict and os.path.exists(data_dict["winner_path"]):
                        src_path = data_dict["winner_path"]
                    else:
                        src_path = os.path.join(catalog_path, filename)
                        
                    dst_path = os.path.abspath(os.path.join(active_af_dir, f"phase2_{zone}_{filename}"))
                    
                    if os.path.abspath(src_path) == os.path.abspath(dst_path):
                        return dst_path
                        
                    try:
                        shutil.copyfile(src_path, dst_path)
                        return dst_path
                    except Exception as e:
                        print(f"[!] Deployment copy error for {zone} ({filename}): {e}")
                        return src_path
                        
                print("\n[*] Finalizing deployment of optimal profiles for co-optimization feedback loop...")
                new_config["root"] = deploy_airfoil("root", root_data)
                new_config["kink"] = deploy_airfoil("kink", kink_data)
                new_config["tip"]  = deploy_airfoil("tip", tip_data)
                
                # Persist mapping config JSON
                try:
                    with open(ACTIVE_AIRFOILS_CONFIG, "w", encoding="utf-8") as f:
                        json.dump(new_config, f, indent=2)
                    
                    # Live update global registry in active runtime memory
                    ACTIVE_AIRFOILS.update(new_config)
                    
                    print(f"\n[✓] AIRFOILS SUCCESSFULLY BACK-FED INTO SYSTEM!")
                    print(f"[i] Active Config: {ACTIVE_AIRFOILS_CONFIG}")
                    print("    - Root Station: " + os.path.basename(ACTIVE_AIRFOILS["root"]))
                    print("    - Kink Station: " + os.path.basename(ACTIVE_AIRFOILS["kink"]))
                    print("    - Tip Station:  " + os.path.basename(ACTIVE_AIRFOILS["tip"]))
                    print("\nAll future OpenVSP wing models generated in this optimization cycle")
                    print("will automatically instantiate with these specialized profiles!")
                except Exception as e:
                    print(f"[!] Failed to save back-feed configuration: {e}")
            
            print("\n[✓] Iterative planform-airfoil co-optimization cycle completed!")
            print("Exiting solver UI loop safely.")
            break

        # If user declined airfoil module, ask if they want to extend planform runs
        while True:
            ans_cont = input("Do you wish to continue running the Planform Optimization? (Y/N): ").strip().lower()
            if ans_cont in ['y', 'n']:
                break
            print("    Please answer 'Y' or 'N'.")

        if ans_cont == 'y':
            # Continues execution loop, will automatically see ckpt_exists and prompt extra_gen
            continue
        else:
            print("\n[✓] Optimization workflow complete. Exiting RAPID. Goodbye!")
            break

