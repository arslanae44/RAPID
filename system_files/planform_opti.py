"""
RAPID wing planform optimizer (NSGA-II + OpenVSP/VSPAERO VLM).

Interactive (RUN_OPTIMIZATION.bat / RUN_BWB.bat):
    python planform_opti.py
Headless (smoke tests, CI, batch studies):
    python planform_opti.py --yes --pop 8 --gens 2 --cores 4 [--fresh] [--airfoil]
                            [--output-dir DIR]

Every candidate wing is evaluated in its own process by vsp_worker.py.
Phase 2 (airfoil co-optimization) lives in airfoil_opti.py.
"""

import os
import sys
import io

# UTF-8 console (safe for Spyder/IPython)
try:
    if hasattr(sys.stdout, "buffer") and sys.stdout.buffer:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "buffer") and sys.stderr.buffer:
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")
except Exception:
    pass

import argparse
import glob
import json
import multiprocessing
import pickle
import random
import shutil
import signal
import subprocess
import sysconfig
import time
import uuid

import numpy as np

# ─── PORTABLE PATHS ───────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR   = os.path.dirname(SCRIPT_DIR)
VSP_ROOT   = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")
sys.path.insert(0, SCRIPT_DIR)
if os.path.exists(VSP_ROOT):
    try:
        os.add_dll_directory(VSP_ROOT)
    except Exception:
        pass

os.environ["OMP_NUM_THREADS"] = "1"

from pymoo.core.problem import ElementwiseProblem  # noqa: E402
try:
    from pymoo.parallelization.starmap import StarmapParallelization
except ImportError:
    from pymoo.core.problem import StarmapParallelization
from pymoo.algorithms.moo.nsga2 import NSGA2  # noqa: E402
from pymoo.optimize import minimize  # noqa: E402
from pymoo.operators.sampling.rnd import FloatRandomSampling  # noqa: E402
from pymoo.operators.crossover.sbx import SBX  # noqa: E402
from pymoo.operators.mutation.pm import PM  # noqa: E402
from pymoo.core.callback import Callback  # noqa: E402
from pymoo.util.display.display import Display  # noqa: E402
from pymoo.util.display.column import Column  # noqa: E402

from rapid_config import load_effective_config  # noqa: E402
import geometry  # noqa: E402
import airfoil_opti  # noqa: E402
import vsp_runner  # noqa: E402

VSP_FILE      = os.path.join(SCRIPT_DIR, "altc130mod.vsp3")
WORKER_SCRIPT = os.path.join(SCRIPT_DIR, "vsp_worker.py")
OUTPUT_DIR    = os.environ.get("RAPID_OUTPUT_DIR") or os.path.join(WORK_DIR, "wing_models")
AIRFOIL       = airfoil_opti.BASELINE_AIRFOIL
DESIGN_KEYS   = ("AR", "kink_frac", "t_inner", "t_outer", "sweep_inner", "sweep_outer")

# ─── CONFIGURATION (rapid_config.json / RAPID_CONFIG_FILE / defaults) ────────
CFG = load_effective_config()
_fl, _co, _bw = CFG["flight"], CFG["constraints"], CFG["bwb"]
V_MS      = float(_fl["V_KMH"]) / 3.6
RHO       = float(_fl["RHO"])
TEMP_K    = float(_fl["TEMP_K"])
MU        = float(_fl["MU"])
MACH      = V_MS / np.sqrt(1.4 * 287.05 * TEMP_K)
S_REF     = float(_fl["S_REF"])
AOA       = float(_fl["AOA"])
INCIDENCE = float(_fl["INCIDENCE"])
BOUNDS    = [tuple(CFG["bounds"][k]) for k in DESIGN_KEYS]
LD_MIN, CM_MAX, CL_MIN = float(_co["LD_MIN"]), float(_co["CM_MAX"]), float(_co["CL_MIN"])

BWB_MODE    = bool(_bw["BWB_MODE"])
SM_MIN      = float(_bw["SM_MIN"])
SM_MAX      = float(_bw["SM_MAX"])
CM_TRIM_TOL = float(_bw["CM_TRIM_TOL"])
ALPHA_DELTA = float(_bw["ALPHA_DELTA"])
_cgx        = _bw.get("CG_X", None)
CG_X        = None if _cgx in (None, "", "none", "None") else float(_cgx)
AIRFOIL_CATALOG  = str(_bw.get("AIRFOIL_CATALOG", "Medium_Re"))
BASELINE_AIRFOIL = str(_bw.get("BASELINE_AIRFOIL", "mh60") or "")

GEO_CFG    = CFG["geometry"]
N_SECTIONS = max(3, int(GEO_CFG["N_SECTIONS"]))
MESH       = CFG["mesh"]

N_CONSTR = 5 if BWB_MODE else 3
IS_MAIN_PROCESS = multiprocessing.current_process().name == "MainProcess"
if IS_MAIN_PROCESS:
    print(f"[cfg] BWB_MODE={BWB_MODE} | L/D>={LD_MIN} CL>={CL_MIN}"
          f"{'' if BWB_MODE else f' |Cm|<={CM_MAX}'} | S_REF={S_REF} V={V_MS*3.6:.0f}km/h "
          f"M={MACH:.3f} | sections={N_SECTIONS} mesh={MESH['TESS_W']}x{MESH['TESS_U']}")

# ─── AIRFOIL CO-OPTIMIZATION FEEDBACK ────────────────────────────────────────
ACTIVE_AIRFOILS_CONFIG = airfoil_opti.ACTIVE_AIRFOILS_CONFIG
ACTIVE_AIRFOILS = airfoil_opti.load_active_airfoils()
if BWB_MODE:
    _seeded = not airfoil_opti.is_custom(ACTIVE_AIRFOILS)
    ACTIVE_AIRFOILS, _warn = airfoil_opti.bwb_baseline(ACTIVE_AIRFOILS, BASELINE_AIRFOIL)
    if IS_MAIN_PROCESS and _warn:
        print(f"[cfg] {_warn}; using {AIRFOIL}")
    elif IS_MAIN_PROCESS and _seeded:
        print(f"[cfg] BWB baseline reflexed section: {BASELINE_AIRFOIL}")


# ─── GEOMETRY HELPERS ────────────────────────────────────────────────────────
def half_span_from_ar(AR):
    return geometry.half_span_from_ar(AR, S_REF, GEO_CFG)


def chord_from_geometry(AR, kink_frac, t_inner, t_outer):
    return geometry.chord_from_geometry(AR, kink_frac, t_inner, t_outer, S_REF, GEO_CFG)


def design_label(x):
    AR, kink_frac, t_inner, t_outer, sw_in, sw_out = x
    v = [int(round(AR * 10)), int(round(2.0 * half_span_from_ar(AR))), int(round(kink_frac * 10)),
         int(round(sw_in)), int(round(sw_out)), int(round(t_inner * 10)), int(round(t_outer * 10))]
    pretty = f"W{v[0]}/{v[1]}/{v[2]}/{v[3]}-{v[4]}/{v[5]}-{v[6]}"
    safe = f"W{v[0]}_{v[1]}_{v[2]}_{v[3]}-{v[4]}_{v[5]}-{v[6]}"
    return pretty, safe


def build_worker_params(x, name, base_vsp, wake_iterations):
    """Everything vsp_worker.py needs for one wing, or None if infeasible."""
    stations = geometry.planform_stations(x, S_REF, N_SECTIONS, INCIDENCE, GEO_CFG)
    if stations is None:
        return None
    mac, x_le_mac = geometry.mac_reference(stations)
    panels = geometry.panels_from_stations(stations, (x[4], x[5]))
    return {
        "name": name, "x": [float(v) for v in x], "base_vsp": base_vsp,
        "incidence": INCIDENCE, "panels": panels,
        "airfoils": airfoil_opti.assign_airfoils(stations, ACTIVE_AIRFOILS, float(x[1])),
        "tess_w": int(MESH["TESS_W"]),
        "tess_u": geometry.spanwise_tess(panels, int(MESH["TESS_U"])),
        "S_REF": S_REF, "b_ref": 2.0 * stations[-1]["y"], "c_ref": mac,
        "x_ref": x_le_mac + 0.25 * mac,          # moment reference = quarter-MAC point
        "x_cg": x_le_mac + 0.25 * mac,
        "mach": MACH, "Re": RHO * V_MS * mac / MU, "V_MS": V_MS, "RHO": RHO,
        "alphas": [AOA, AOA + ALPHA_DELTA] if BWB_MODE else [AOA],
        "wake_iters": int(wake_iterations), "timeout": float(MESH["TIMEOUT_S"]),
    }


def longitudinal_stability(rows, x_ref, c_ref):
    """Neutral point, CG, static margin and trim moment from a 2-alpha polar.

    Moments in `rows` are about x_ref. dCm/dCL = (x_ref - x_np)/c. If CG_X is
    not configured the CG is placed at the target static margin (mid of
    SM_MIN..SM_MAX) and the constraint that matters is trim at that CG.
    """
    r1, r2 = rows[0], rows[1]
    dCL = r2["CL"] - r1["CL"]
    if abs(dCL) < 1e-6:
        return None
    dCm_dCL = (r2["CM"] - r1["CM"]) / dCL
    x_np = x_ref - c_ref * dCm_dCL
    x_cg = CG_X if CG_X is not None else x_np - 0.5 * (SM_MIN + SM_MAX) * c_ref
    sm = (x_np - x_cg) / c_ref
    cm_trim = r1["CM"] + r1["CL"] * (x_cg - x_ref) / c_ref   # transfer Cm to the CG
    return {"x_np": x_np, "x_cg": x_cg, "SM": sm, "CM_trim": cm_trim}


# ─── RESULTS PERSISTENCE ──────────────────────────────────────────────────────
def _eval_cache():
    cache = []
    for p_file in glob.glob(os.path.join(OUTPUT_DIR, "*", "eval.json")):
        try:
            with open(p_file, "r") as f:
                cache.append(json.load(f))
        except Exception:
            pass
    return cache


def save_results(algorithm, res=None):
    """Checkpoint + history + feasible Pareto set (pareto_results.json)."""
    ckpt_file = os.path.join(OUTPUT_DIR, "checkpoint.pkl")
    try:
        with open(ckpt_file, "wb") as f:
            pickle.dump(algorithm, f)
        print(f"\n[✓] Checkpoint saved: {ckpt_file}")
    except Exception as e:
        print(f"\n[!] Checkpoint failed to save: {e}")

    try:
        hist = getattr(algorithm, "history", None)
        if hist:
            with open(os.path.join(OUTPUT_DIR, "history.pkl"), "wb") as f:
                pickle.dump(hist, f)
            print(f"[✓] {len(hist)} generation history saved")
    except Exception as e:
        print(f"[!] History failed to save: {e}")

    try:
        if res is not None and res.F is not None:
            F, X, G = np.atleast_2d(res.F), np.atleast_2d(res.X), res.G
        elif algorithm.opt is not None and len(algorithm.opt) > 0:
            F, X, G = algorithm.opt.get("F"), algorithm.opt.get("X"), algorithm.opt.get("G")
        else:
            print("[!] No Pareto solutions found to save.")
            return
        if G is not None:
            G = np.atleast_2d(G)

        cache = _eval_cache()
        final_results = []
        for i, (f, x) in enumerate(zip(F, X)):
            if f[0] > 900.0:                                     # penalty
                continue
            if G is not None and np.sum(np.maximum(G[i], 0.0)) > 1e-6:
                continue                                         # strictly feasible only
            match = min(cache, key=lambda c: np.linalg.norm(np.array(c["x"]) - x), default=None)
            if match is not None and np.linalg.norm(np.array(match["x"]) - x) > 1e-6:
                match = None
            entry = {
                "solution_id": len(final_results) + 1,
                "CL": round(-f[0], 5), "LD": round(-f[1], 3),
                **{k: round(float(v), 3 if k[0] in "Akt" else 2) for k, v in zip(DESIGN_KEYS, x)},
                "folder_name": match["name"] if match else "",
                "hash": match["name"].split("_")[-1] if match else "",
            }
            if match:
                for k in ("CM", "SM", "CM_trim", "x_np", "x_cg", "CD"):
                    if match.get(k) is not None:
                        entry[k] = round(float(match[k]), 5)
            final_results.append(entry)

        json_path = os.path.join(OUTPUT_DIR, "pareto_results.json")
        with open(json_path, "w") as fh:
            json.dump(final_results, fh, indent=2)
        print(f"[✓] {len(final_results)} Pareto solutions saved: {json_path}")

        if final_results:
            print(f"\n{'#':>4} {'CL':>8} {'L/D':>8} {'AR':>7} {'kink':>7} "
                  f"{'t_in':>7} {'t_out':>7} {'sw_in':>8} {'sw_out':>8}" + ("    SM%  Cm_trim" if BWB_MODE else ""))
            print("-" * (70 + (17 if BWB_MODE else 0)))
            for r in final_results:
                line = (f"{r['solution_id']:>4} {r['CL']:>8.5f} {r['LD']:>8.3f} "
                        f"{r['AR']:>7.3f} {r['kink_frac']:>7.3f} {r['t_inner']:>7.3f} {r['t_outer']:>7.3f} "
                        f"{r['sweep_inner']:>8.1f} {r['sweep_outer']:>8.1f}")
                if BWB_MODE and "SM" in r:
                    line += f" {r['SM']*100:>6.1f} {r['CM_trim']:>+8.4f}"
                print(line)
            print("=" * 70)
    except Exception as e:
        print(f"[!] JSON failed to save: {e}")


# ─── WORKER LAUNCH ────────────────────────────────────────────────────────────
launch_lock = None


def init_worker(lock):
    global launch_lock
    launch_lock = lock
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C is handled by the parent only


def _penalty():
    p = 999.0 + random.uniform(0.01, 5.0)   # randomized so NSGA-II can still rank failures
    return {"F": [p, p], "G": [p] * N_CONSTR}


def _run_worker(run_dir, params_file, result_file, timeout):
    """Launch vsp_worker.py; returns (ok, diagnostic)."""
    return vsp_runner.run_worker(run_dir, params_file, result_file, timeout, lock=launch_lock)


def evaluate_subprocess(x, wake_iterations=None):
    if wake_iterations is None:
        wake_iterations = int(MESH["WAKE_ITERS"])
    x = [float(v) for v in x]
    pretty, safe = design_label(x)
    name = f"{safe}_{uuid.uuid4().hex[:6]}"
    run_dir = os.path.join(OUTPUT_DIR, name)

    params = build_worker_params(x, name, None, wake_iterations)
    if params is None:
        return _penalty()   # root chord / span outside admissible geometry

    os.makedirs(run_dir, exist_ok=True)
    if os.path.exists(VSP_FILE):
        local_base = os.path.join(run_dir, "base_local.vsp3")
        for _ in range(20):
            try:
                shutil.copyfile(VSP_FILE, local_base)
                params["base_vsp"] = local_base
                break
            except OSError:
                time.sleep(0.03 + random.random() * 0.07)

    params_file = os.path.join(run_dir, "params.json")
    result_file = os.path.join(run_dir, "result.json")
    with open(params_file, "w") as f:
        json.dump(params, f)

    time.sleep(random.random() * 0.5)   # stagger simultaneous launches
    ok, diag = _run_worker(run_dir, params_file, result_file, params["timeout"])
    if not ok:
        print(f"[{pretty}] REJECTED: worker failed: {diag}")
        sys.stdout.flush()
        return _penalty()

    res = None
    for _ in range(20):
        try:
            with open(result_file, "r") as f:
                res = json.load(f)
            break
        except (OSError, json.JSONDecodeError):
            time.sleep(0.05 + random.random() * 0.1)
    if not res or res.get("error") or not res.get("rows"):
        print(f"[{pretty}] REJECTED: {res.get('error') if res else 'empty result file'}")
        sys.stdout.flush()
        return _penalty()

    r0 = res["rows"][0]
    CL, LD, CM = r0["CL"], r0["LD"], r0["CM"]
    if LD > 60.0 or LD <= 0.0 or CL <= 0.0 or CL > 1.5:
        print(f"[{pretty}] REJECTED: unphysical (CL {CL:.2f}, L/D {LD:.2f})")
        sys.stdout.flush()
        return _penalty()

    summary = {"name": name, "x": x, "CL": CL, "CD": r0["CD"], "LD": LD, "CM": CM}
    f1, f2 = -CL, -LD
    g_ld = LD_MIN - LD
    g_cl = CL_MIN - CL
    if BWB_MODE:
        stab = longitudinal_stability(res["rows"], params["x_ref"], params["c_ref"]) if len(res["rows"]) >= 2 else None
        if stab is None:
            print(f"[{pretty}] REJECTED: stability derivative unavailable")
            return _penalty()
        summary.update(stab)
        G = [g_ld, g_cl, SM_MIN - stab["SM"], stab["SM"] - SM_MAX, abs(stab["CM_trim"]) - CM_TRIM_TOL]
        print(f"[{pretty}] CL: {CL:.4f} | L/D: {LD:.2f} | SM: {stab['SM']*100:.1f}% | Cm_trim: {stab['CM_trim']:+.4f}")
    else:
        G = [g_ld, abs(CM) - CM_MAX, g_cl]
        print(f"[{pretty}] CL: {CL:.4f} | L/D: {LD:.2f} | Cm(c/4): {CM:+.4f}")
    sys.stdout.flush()
    with open(os.path.join(run_dir, "eval.json"), "w") as f:
        json.dump(summary, f)
    return {"F": [f1, f2], "G": G}


# ─── OPTIMIZATION PROBLEM ─────────────────────────────────────────────────────
class WingOptimization(ElementwiseProblem):
    def __init__(self, runner=None, wake_iterations=None):
        self.wake_iterations = wake_iterations
        kwargs = {"elementwise_runner": runner} if runner is not None else {}
        super().__init__(n_var=6, n_obj=2, n_ieq_constr=N_CONSTR,
                         xl=[b[0] for b in BOUNDS], xu=[b[1] for b in BOUNDS], **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):
        result = evaluate_subprocess(x, wake_iterations=self.wake_iterations)
        out["F"] = result["F"]
        out["G"] = result["G"]


# ─── CUSTOM DISPLAY ──────────────────────────────────────────────────────────
class WingDisplay(Display):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.verbose = True
        self.n_gen = Column("n_gen", width=6)
        self.n_eval = Column("n_eval", width=7)
        self.n_nds = Column("n_nds", width=6)
        self.ideal_ld = Column("ideal_LD", width=8)
        self.ideal_cl = Column("ideal_CL", width=8)
        self.eps_col = Column("eps", width=9)
        self.hv_col = Column("hv_area", width=9)
        self.cv_min = Column("cv_min", width=8)
        self.cv_avg = Column("cv_avg", width=8)
        self.columns = [self.n_gen, self.n_eval, self.n_nds, self.ideal_ld, self.ideal_cl,
                        self.cv_min, self.cv_avg, self.eps_col, self.hv_col]
        self.prev_ideal = None
        self.ref_point = np.array([-CL_MIN, -LD_MIN])
        try:
            from pymoo.indicators.hv import Hypervolume
            self.hv_calc = Hypervolume(ref_point=self.ref_point)
        except Exception:
            self.hv_calc = None

    def _blank(self, *cols):
        for c in cols:
            c.set("-")

    def _do(self, problem, evaluator, algorithm, **kwargs):
        self.n_gen.set(algorithm.n_iter)
        self.n_eval.set(evaluator.n_eval)
        opt = algorithm.opt
        self.n_nds.set(len(opt))

        pop = algorithm.pop
        CV = pop.get("CV") if pop is not None and len(pop) > 0 else None
        if CV is not None and len(CV) > 0:
            self.cv_min.set(f"{np.min(CV):.4E}")
            self.cv_avg.set(f"{np.mean(CV):.4E}")
        else:
            self._blank(self.cv_min, self.cv_avg)

        F = opt.get("F") if len(opt) > 0 else None
        valid = F[F[:, 0] < 900] if F is not None else []
        if len(valid) == 0:
            self._blank(self.ideal_cl, self.ideal_ld, self.eps_col, self.hv_col)
            return
        best_cl, best_ld = -np.min(valid[:, 0]), -np.min(valid[:, 1])
        self.ideal_cl.set(f"{best_cl:.4f}")
        self.ideal_ld.set(f"{best_ld:.2f}")

        current = np.array([best_cl, best_ld])
        if self.prev_ideal is not None:
            growth = np.sum(np.maximum((current - self.prev_ideal) / np.maximum(self.prev_ideal, 1e-6), 0.0))
            self.eps_col.set(f"{growth:.2E}")
        else:
            self.eps_col.set("-")
        self.prev_ideal = current

        if self.hv_calc is None:
            self.hv_col.set("-")
            return
        try:
            feas = valid[(valid[:, 0] <= self.ref_point[0]) & (valid[:, 1] <= self.ref_point[1])]
            self.hv_col.set(f"{self.hv_calc.do(feas):.3f}" if len(feas) else "0.000")
        except Exception:
            self.hv_col.set("0.000")


class FlushCallback(Callback):
    def notify(self, algorithm):
        if not isinstance(algorithm.display, WingDisplay) or not hasattr(algorithm.display, "cv_min"):
            old_prev = getattr(algorithm.display, "prev_ideal", None)
            algorithm.display = WingDisplay()
            if old_prev is not None:
                algorithm.display.prev_ideal = old_prev
        sys.stdout.flush()

        # Re-print the column header before every generation row.
        if not hasattr(self, "_patched"):
            try:
                disp_cls = type(algorithm.display)
                orig_call = disp_cls.__call__

                def custom_call(self_disp, *args, **kwargs):
                    try:
                        if hasattr(self_disp, "output") and self_disp.output:
                            header = self_disp.output.header(border=True)
                            if header:
                                print("\n" + header)
                    except Exception:
                        pass
                    return orig_call(self_disp, *args, **kwargs)

                disp_cls.__call__ = custom_call
                self._patched = True
            except Exception:
                pass

        save_results(algorithm)   # live checkpoint + Pareto front every generation
        sys.stdout.flush()


def geometry_feasibility(n=2000, seed=0):
    """Fraction of random designs inside BOUNDS that the geometry limits admit,
    plus the sampled root-chord range (to diagnose an impossible setup)."""
    rng = np.random.default_rng(seed)
    lo, hi = np.array([b[0] for b in BOUNDS]), np.array([b[1] for b in BOUNDS])
    ok, chords = 0, []
    for x in lo + rng.random((n, 6)) * (hi - lo):
        b_half = half_span_from_ar(x[0]) - float(GEO_CFG["FUSELAGE_Y"])
        if b_half > 0:
            b_k = x[1] * b_half
            chords.append((S_REF / 2.0) / (0.5 * (1 + x[2]) * b_k + 0.5 * x[2] * (1 + x[3]) * (b_half - b_k)))
        ok += chord_from_geometry(*x[:4]) is not None
    return ok / n, (min(chords), max(chords)) if chords else None


def warn_if_geometry_infeasible():
    frac, c_range = geometry_feasibility()
    mid_half_span = half_span_from_ar(0.5 * (BOUNDS[0][0] + BOUNDS[0][1]))
    if float(GEO_CFG["FUSELAGE_Y"]) > 0.25 * mid_half_span:
        print(f"\n[!] FUSELAGE_Y={GEO_CFG['FUSELAGE_Y']} m removes more than 25% of the ~{mid_half_span:.2f} m "
              f"half-span (S_REF={S_REF} m^2). Set [GEOMETRY] in CONFIGURE.bat to your aircraft's scale.")
    if frac >= 0.02:
        return
    print("\n" + "!" * 70)
    print(f"[!] Only {frac:.1%} of the design space passes the geometry limits — the run cannot find wings.")
    if c_range is None:
        print(f"    FUSELAGE_Y={GEO_CFG['FUSELAGE_Y']} m is larger than every half-span for S_REF={S_REF} m^2.")
    else:
        print(f"    Root chords in these bounds span {c_range[0]:.2f}..{c_range[1]:.2f} m, but "
              f"C_ROOT_MIN..MAX = {GEO_CFG['C_ROOT_MIN']}..{GEO_CFG['C_ROOT_MAX']} m.")
    print("    Fix: CONFIGURE.bat -> [GEOMETRY] (FUSELAGE_Y, C_ROOT_MIN/MAX) to match your aircraft scale.")
    print("!" * 70)


def completed_generations(algorithm):
    # pymoo's n_iter is the index of the *next* generation once a run has finished.
    return max(int(getattr(algorithm, "n_iter", 1) or 1) - 1, 0)


# ─── PROMPTS ──────────────────────────────────────────────────────────────────
def ask_int(prompt, default, minimum, interactive):
    if not interactive:
        return default
    while True:
        raw = airfoil_opti.read_input(f"{prompt} (Enter = default {default}): ")
        raw = (raw or "").strip()
        if not raw:
            return default
        try:
            v = int(raw)
            if v >= minimum:
                return v
        except ValueError:
            pass
        print(f"    Please enter an integer >= {minimum}.")


def ask_yn(prompt, default, interactive):
    if not interactive:
        return default
    d = "Y" if default else "N"
    while True:
        a = airfoil_opti.read_input(f"{prompt} (Y/N) [Enter = {d}]: ")
        if a is None:
            return default
        a = (a.strip() or d).lower()
        if a in ("y", "n"):
            return a == "y"
        print("    Please answer 'Y' or 'N'.")


def print_coopt_status():
    print("\n" + "-" * 70)
    if airfoil_opti.is_custom(ACTIVE_AIRFOILS):
        print("[✓] CO-OPTIMIZATION ACTIVE: custom sectional profiles")
        for s in ACTIVE_AIRFOILS:
            eta = s["eta"] if s["eta"] == "kink" else f"{float(s['eta']):.3f}"
            print(f"    - eta {eta}: {os.path.basename(str(s['airfoil']))}")
    else:
        print(f"[i] CO-OPTIMIZATION INACTIVE: uniform baseline profile {AIRFOIL.upper()}")
    print("-" * 70)


# ─── PHASE 2 ──────────────────────────────────────────────────────────────────
def run_phase2(num_cores, interactive, wing_id=None):
    global ACTIVE_AIRFOILS
    print("\n" + "═" * 70)
    print(" AIRFOIL CO-OPTIMIZATION")
    print("═" * 70)
    if interactive:
        plotter_bat = os.path.join(WORK_DIR, "RUN_PERFORMANCE_PLOTTER.bat")
        if os.path.exists(plotter_bat):
            print("[*] Launching Performance Plotter in a new window...")
            subprocess.Popen(f'start "" "{plotter_bat}"', shell=True)
            time.sleep(1.0)

    pareto_json = os.path.join(OUTPUT_DIR, "pareto_results.json")
    try:
        with open(pareto_json, "r") as f:
            pareto = json.load(f)
    except Exception:
        pareto = []
    if not pareto:
        print("[!] No feasible Pareto wings to co-optimize. Run (or extend) the planform optimization first.")
        return False

    if wing_id is None and not interactive:
        wing_id = max(pareto, key=lambda w: w["LD"])["solution_id"]   # headless: best-L/D wing
    while wing_id is None:
        raw = airfoil_opti.read_input(f"\nIndex of the Pareto wing to co-optimize (1-{len(pareto)}): ")
        if raw is None:
            print("[i] No input available; skipping airfoil co-optimization.")
            return False
        raw = raw.strip()
        if raw.isdigit() and any(w["solution_id"] == int(raw) for w in pareto):
            wing_id = int(raw)
        else:
            print(f"    Enter an index between 1 and {len(pareto)}.")
    wing = next(w for w in pareto if w["solution_id"] == wing_id)
    print(f"\n[✓] Wing #{wing_id}: CL={wing['CL']} L/D={wing['LD']} | AR={wing['AR']} kink={wing['kink_frac']} "
          f"t={wing['t_inner']}/{wing['t_outer']} sweep={wing['sweep_inner']}/{wing['sweep_outer']}")

    n_sec = ask_int("\nNumber of spanwise stations (3 = root/kink/tip)", N_SECTIONS, 3, interactive)
    sections = airfoil_opti.sections_from_design(wing, n_sec, CFG)
    if not sections:
        print("[!] Geometry calculation failed.")
        return False
    airfoil_opti.print_sections(sections, f"SECTIONAL CONDITIONS — WING #{wing_id}")

    stations = airfoil_opti.run_pipeline(sections, num_cores, CFG, catalog=None if interactive else AIRFOIL_CATALOG,
                                         interactive=interactive,
                                         refine=None if interactive else False,
                                         write_active=None if interactive else True)
    if stations:
        ACTIVE_AIRFOILS = stations
        print("\nEvery later planform evaluation will instantiate these profiles.")
    return True


# ─── MAIN ─────────────────────────────────────────────────────────────────────
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="RAPID wing planform optimizer")
    ap.add_argument("--yes", action="store_true", help="non-interactive: take defaults / flags, no prompts")
    ap.add_argument("--cores", type=int, help="parallel worker processes (default: all CPUs)")
    ap.add_argument("--wake", type=int, help="VSPAERO wake iterations (default: mesh.WAKE_ITERS)")
    ap.add_argument("--pop", type=int, help="population size for a fresh run (default 32)")
    ap.add_argument("--gens", type=int, help="generations to run (fresh: default 20, resume: default 10)")
    ap.add_argument("--fresh", action="store_true", help="ignore an existing checkpoint")
    ap.add_argument("--airfoil", action="store_true", help="headless: run Phase 2 on the best-L/D wing afterwards")
    ap.add_argument("--wing", type=int, help="Pareto solution id for Phase 2")
    ap.add_argument("--seed", type=int, help="random seed")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    interactive = not args.yes
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    # Record the config this study runs with, so the plotters draw the same aircraft.
    with open(os.path.join(OUTPUT_DIR, "run_config.json"), "w", encoding="utf-8") as f:
        json.dump(CFG, f, indent=2)

    import openvsp as vsp
    print("VSP Version: " + str(vsp.GetVSPVersion()))
    print("=" * 70)
    if BWB_MODE:
        print(f"Constraints: L/D >= {LD_MIN} | CL >= {CL_MIN} | SM {SM_MIN:.0%}..{SM_MAX:.0%} | "
              f"|Cm_trim| <= {CM_TRIM_TOL} | CG {'auto (target SM)' if CG_X is None else f'x={CG_X} m'}")
    else:
        print(f"Constraints: L/D >= {LD_MIN} | |Cm(c/4)| <= {CM_MAX} | CL >= {CL_MIN}")
    print("=" * 70)
    warn_if_geometry_infeasible()
    print_coopt_status()

    cpu = os.cpu_count() or 4
    num_cores = args.cores or ask_int(f"\nNumber of CPU cores to use?", cpu, 1, interactive)
    wake_iterations = args.wake if args.wake is not None else ask_int(
        "VSPAERO wake iteration count?", int(MESH["WAKE_ITERS"]), 0, interactive)

    ckpt_file = os.path.join(OUTPUT_DIR, "checkpoint.pkl")
    fresh = args.fresh
    while True:
        ckpt_exists = os.path.exists(ckpt_file) and not fresh
        pop_size = args.pop or 32
        if ckpt_exists:
            try:
                with open(ckpt_file, "rb") as f:
                    temp_alg = pickle.load(f)
                print(f"\n[i] Checkpoint found — {completed_generations(temp_alg)} generation(s) completed, "
                      f"population {getattr(temp_alg, 'pop_size', '?')}.")
            except Exception:
                print("\n[!] Failed to preview checkpoint.")
            if interactive and ask_yn("\n[?] Skip planform generations and go straight to Airfoil Co-Optimization?",
                                      False, interactive):
                run_phase2(num_cores, interactive, args.wing)
                return
            extra = args.gens or ask_int("How many MORE generations?", 10, 1, interactive)
        else:
            print("\n[i] Starting a fresh optimization.")
            pop_size = args.pop or ask_int("Wings per generation (pop_size)?", 32, 2, interactive)
            extra = args.gens or ask_int("How many generations?", 20, 1, interactive)

        print(f"\n[i] Launching {num_cores} worker processes...")
        sys.stdout.flush()
        lock = multiprocessing.Lock()
        pool = multiprocessing.Pool(num_cores, initializer=init_worker, initargs=(lock,))
        runner = StarmapParallelization(pool.starmap)
        problem = WingOptimization(runner=runner, wake_iterations=wake_iterations)

        if ckpt_exists:
            try:
                with open(ckpt_file, "rb") as f:
                    algorithm = pickle.load(f)
                current_gen = completed_generations(algorithm)
                target_gen = current_gen + extra
                from pymoo.termination import get_termination
                algorithm.termination = get_termination("n_gen", target_gen)
                if hasattr(algorithm, "problem"):
                    algorithm.problem = problem
                print(f"[i] Resuming from gen {current_gen} → gen {target_gen}")
            except Exception as e:
                print(f"[!] Failed to load checkpoint: {e}. Aborting.")
                pool.terminate()
                pool.join()
                return
        else:
            algorithm = NSGA2(pop_size=pop_size, n_offsprings=pop_size, sampling=FloatRandomSampling(),
                              crossover=SBX(prob=0.9, eta=15), mutation=PM(eta=20), eliminate_duplicates=True)
            target_gen = extra
            print(f"[i] Pop size: {pop_size} | Target: {target_gen} generations")

        algorithm.verbose = True
        algorithm.display = WingDisplay()
        res = None
        t0 = time.time()
        try:
            res = minimize(problem, algorithm, ("n_gen", target_gen),
                           seed=args.seed if args.seed is not None else random.randint(1, 10000),
                           copy_algorithm=False, verbose=True, display=WingDisplay(),
                           callback=FlushCallback(), save_history=True)
            print("\n" + "=" * 70)
            print(f"GENERATION BATCH COMPLETED in {time.time() - t0:.0f}s")
            print("=" * 70)
        except KeyboardInterrupt:
            print("\n\n[!] Interrupted by user.")
        finally:
            pool.terminate()
            pool.join()
            save_results(algorithm, res)
        fresh = False

        sys.stdout.flush()
        if ask_yn("\nProceed to Airfoil Co-Optimization?", args.airfoil, interactive):
            run_phase2(num_cores, interactive, args.wing)
            print("\n[✓] Planform-airfoil co-optimization cycle completed.")
            return
        if not ask_yn("Continue running the planform optimization?", False, interactive):
            print("\n[✓] Optimization workflow complete. Goodbye!")
            return


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
