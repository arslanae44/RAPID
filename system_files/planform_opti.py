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

os.makedirs(OUTPUT_DIR, exist_ok=True)

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

AR, kink_frac, t_inner, t_outer, sw_in, sw_out = x
c_root, c_kink, c_tip, b_half, b_kink, _mac = geo
b_tip_sec = b_half - b_kink

def sp(wid, xsec_idx, parm, value):
    xsec_surf = vsp.GetXSecSurf(wid, 0)
    xsec = vsp.GetXSec(xsec_surf, xsec_idx)
    pid = vsp.GetXSecParm(xsec, parm)
    if pid != "":
        vsp.SetParmVal(pid, value)

def set_airfoil(wid, xsec_idx, naca_code):
    xsec_surf = vsp.GetXSecSurf(wid, 0)
    vsp.ChangeXSecShape(xsec_surf, xsec_idx, vsp.XS_FOUR_SERIES)
    vsp.Update()
    xsec = vsp.GetXSec(xsec_surf, xsec_idx)
    code = naca_code.lower().replace("naca", "")
    vsp.SetParmVal(vsp.GetXSecParm(xsec, "Camber"),     int(code[0]) / 100.0)
    vsp.SetParmVal(vsp.GetXSecParm(xsec, "CamberLoc"),  int(code[1]) / 10.0)
    vsp.SetParmVal(vsp.GetXSecParm(xsec, "ThickChord"), int(code[2:]) / 100.0)

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

set_airfoil(wid, 0, AIRFOIL)
set_airfoil(wid, 1, AIRFOIL)
set_airfoil(wid, 2, AIRFOIL)

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

vsp.SetAnalysisInputDefaults("VSPAEROSweep")
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "RefArea",  [S_REF])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "RefSpan",  [b_ref])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "RefChord", [mac])
vsp.SetIntAnalysisInput("VSPAEROSweep", "AnalysisMethod",   [0])
vsp.SetIntAnalysisInput("VSPAEROSweep", "WakeNumIter",      [WAKE_ITERATIONS])
vsp.SetIntAnalysisInput("VSPAEROSweep", "NumCPU",           [1])
vsp.SetIntAnalysisInput("VSPAEROSweep", "ParasiteDragFlag", [1])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "MachStart",  [MACH])
vsp.SetIntAnalysisInput("VSPAEROSweep",   "MachNpts",    [1])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "AlphaStart", [AOA])
vsp.SetIntAnalysisInput("VSPAEROSweep",   "AlphaNpts",   [1])

Re_mac = RHO * V_MS * mac / MU
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "ReCref", [Re_mac])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "Rho",    [RHO])
vsp.SetDoubleAnalysisInput("VSPAEROSweep", "Vinf",   [V_MS])

vsp.ExecAnalysis("VSPAEROSweep")

result = {"CL": None, "CD": None, "LD": None, "CM": None, "error": None}
try:
    all_res_names = vsp.GetAllResultsNames()
    target = ""
    for possible_name in ["VSPAERO_Polar", "VSPAERO_History"]:
        if possible_name in all_res_names:
            target = possible_name
            break

    if target == "":
        result["error"] = "No result found"
    else:
        res_id     = vsp.FindLatestResultsID(target)
        data_names = vsp.GetAllDataNames(res_id)

        # Trefftz Plane Wake integration (wtot) for stable drag values
        cl = vsp.GetDoubleResults(res_id, "CLwtot")[-1] if "CLwtot" in data_names else \
             (vsp.GetDoubleResults(res_id, "CLtot")[-1] if "CLtot"  in data_names else
              (vsp.GetDoubleResults(res_id, "CL")[-1]    if "CL"     in data_names else 0.0))
        cd = vsp.GetDoubleResults(res_id, "CDwtot")[-1] if "CDwtot" in data_names else \
             (vsp.GetDoubleResults(res_id, "CDtot")[-1] if "CDtot"  in data_names else
              (vsp.GetDoubleResults(res_id, "CD")[-1]    if "CD"     in data_names else 0.0))

        if   "CMytot" in data_names: cm = vsp.GetDoubleResults(res_id, "CMytot")[-1]
        elif "CMy"    in data_names: cm = vsp.GetDoubleResults(res_id, "CMy")[-1]
        elif "CMm"    in data_names: cm = vsp.GetDoubleResults(res_id, "CMm")[-1]
        else: cm = 0.0

        ld = cl / cd if cd != 0 else 0
        result.update({"CL": cl, "CD": cd, "LD": ld, "CM": cm})

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
        return {"F": [p, p], "G": [p, p, p]}

    AR, kink_frac, t_inner, t_outer, sw_in, sw_out = x
    geo = chord_from_geometry(AR, kink_frac, t_inner, t_outer)
    if geo is None:
        return get_penalty()

    c_root, c_kink, c_tip, b_half, b_kink, mac = geo
    b_ref = 2.0 * b_half

    # ─── GEOMETRIC LABEL GENERATION ───────────────────────────────────────────
    ar_val     = int(round(AR * 10))
    span_val   = int(round(b_ref))
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
        return get_penalty()

    if res.get("error") or res["CL"] is None:
        return get_penalty()

    CL = res["CL"]
    CD = res["CD"]
    LD = res["LD"]
    CM = abs(res["CM"])

    # Eliminate non-physical diverging configurations
    if LD > 60.0 or LD <= 0.0 or CL <= 0.0 or CL > 0.7:
        return get_penalty()

    print(f"[{pretty_name}] CL: {CL:.4f} | L/D: {LD:.2f}")
    sys.stdout.flush()

    f1 = -CL
    f2 = -LD
    g1 = LD_MIN - LD   # L/D >= 15
    g2 = CM - CM_MAX   # CM <= 0.35
    g3 = CL_MIN - CL   # CL >= 0.15
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
        super().__init__(n_var=6, n_obj=2, n_ieq_constr=3, xl=xl, xu=xu, **kwargs)

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

# ─── MAIN LAUNCHER ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import openvsp as vsp
    print("VSP Version: " + str(vsp.GetVSPVersion()))
    
    print("=" * 70)
    print(f"Constraints: L/D >= {LD_MIN} | CM <= {CM_MAX} | CL >= {CL_MIN}")
    print("=" * 70)

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

    ckpt_file   = os.path.join(OUTPUT_DIR, "checkpoint.pkl")
    ckpt_exists = os.path.exists(ckpt_file)

    if ckpt_exists:
        current_gen_preview = getattr(pickle.load(open(ckpt_file, "rb")), "n_iter", 0)
        print(f"\n[i] Existing checkpoint found — generation {current_gen_preview} completed.")
        
        # Extract and inform population metrics from restored state
        with open(ckpt_file, "rb") as f:
            temp_alg = pickle.load(f)
            active_pop = getattr(temp_alg, "pop_size", "Unknown")
        print(f"[i] Population size restored from checkpoint: {active_pop} wings")

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
        sys.stdout.flush()
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
        sys.stdout.flush()

    rnd_seed = random.randint(1, 10000)
    res = None

    # Enforce updated display layout structures
    algorithm.verbose = True
    algorithm.display = WingDisplay()

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
        print("OPTIMIZATION COMPLETED SUCCESSFULLY!")

    except KeyboardInterrupt:
        print("\n\n[!] Stopped by user.")

    finally:
        # Terminate remaining workers safely
        pool.terminate()
        pool.join()
        save_results(algorithm, res)
