"""
RAPID transonic planform optimizer (Navy jet trainer cruise wing).

    python transonic_opti.py [--config system_files/navy_trainer_config.json]
                             [--pop 48] [--gens 30] [--cores N] [--seed S] [--fresh]
                             [--no-tc] [--select range|drag|weight|weighted] [--weight W]
                             [--evaluate AR,SWEEP_LE,TAPER,WASHOUT[,TC_ROOT,TC_TIP]]
                             [--report-only] [--out DIR]

Each wing is solved once by VSPAERO (VLM) at M 0.15 and M 0.75 for two angles of
attack; transonic.py trims it to the cruise CL and adds wave drag (Korn + Lock),
XFOIL-polar profile drag, Raymer wing weight and the constraints. NSGA-II minimizes
(C_D,total, wing weight). The benchmark "chart wing" is evaluated through the exact
same chain and seeded into the first generation.
"""

import io
import os
import sys

try:
    if hasattr(sys.stdout, "buffer") and sys.stdout.buffer:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
except Exception:
    pass

# --config must be known before rapid_config is read (spawned workers inherit the env var).
for _i, _a in enumerate(sys.argv):
    if _a == "--config" and _i + 1 < len(sys.argv):
        os.environ["RAPID_CONFIG_FILE"] = os.path.abspath(sys.argv[_i + 1])
    elif _a.startswith("--config="):
        os.environ["RAPID_CONFIG_FILE"] = os.path.abspath(_a.split("=", 1)[1])

import argparse
import copy
import glob
import json
import math
import multiprocessing
import pickle
import random
import shutil
import signal
import time
import uuid

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)
os.environ["OMP_NUM_THREADS"] = "1"

from rapid_config import load_effective_config  # noqa: E402
import transonic as T  # noqa: E402
import vsp_runner  # noqa: E402

CFG = load_effective_config()
TC = CFG["transonic"]
MESH = CFG["mesh"]
OUT_DIR = os.environ.get("RAPID_OUTPUT_DIR") or os.path.join(WORK_DIR, "wing_models", "transonic")
CHART_X = [5.25, 30.0, 0.30, 3.0, 0.12, 0.10]       # Study-3 chart wing: AR, LE sweep, taper, washout, t/c r, t/c t
NAMES = ("AR", "SWEEP_LE", "TAPER", "WASHOUT", "TC_ROOT", "TC_TIP")
N_CONSTR = len(T.CONSTRAINT_NAMES)
NL = chr(10)

_POLARS = None
launch_lock = None


def polars():
    global _POLARS
    if _POLARS is None:
        _POLARS = T.load_polars(TC, WORK_DIR)
    return _POLARS


def init_worker(lock):
    global launch_lock
    launch_lock = lock
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def abspath(p):
    return p if os.path.isabs(p) else os.path.join(WORK_DIR, p)


def design_vector(d):
    keys = NAMES if TC["TC_VARIABLE"] else NAMES[:4]
    return [d[k] for k in keys]


# ─── ONE WING ────────────────────────────────────────────────────────────────
def worker_params(d, geo, name, wake_iters):
    st = T.cruise_state(TC)
    x_ref = geo["x_le_mac"] + 0.25 * geo["mac"]
    return {
        "name": name, "base_vsp": "", "incidence": 0.0,
        "panels": [{"span": geo["b"] / 2.0, "root_chord": geo["cr"], "tip_chord": geo["ct"],
                    "sweep": d["SWEEP_LE"], "twist": -d["WASHOUT"]}],
        "airfoils": [abspath(TC["AIRFOIL_ROOT"]), abspath(TC["AIRFOIL_TIP"])],
        "tess_w": int(MESH["TESS_W"]), "tess_u": [int(MESH["TESS_U"])],
        "S_REF": float(TC["S_REF"]), "b_ref": geo["b"], "c_ref": geo["mac"], "x_cg": x_ref,
        "mach": float(TC["MACH"]), "machs": [float(TC["STALL_MACH"]), float(TC["MACH"])],
        "Re": TC["RHO"] * st["V"] * geo["mac"] / TC["MU"], "V_MS": st["V"], "RHO": float(TC["RHO"]),
        "alphas": [1.0, 3.5], "wake_iters": int(wake_iters), "timeout": float(MESH["TIMEOUT_S"]),
        "strips": True,
    }


def _penalty(reason=""):
    p = 999.0 + random.uniform(0.01, 5.0)
    return {"F": [p, p], "G": [p] * N_CONSTR, "reason": reason, "ok": False}


def evaluate_design(x, label=None, keep_files=False, quiet=False, run_root=None):
    """Solve + analyse one design. Returns {"F","G","ok", "name", "design", "metrics"}."""
    d = T.unpack_design(x, TC)
    tag = (f"T{d['AR']*10:.0f}_{d['SWEEP_LE']:.0f}_{d['TAPER']*100:.0f}_{d['WASHOUT']*10:.0f}")
    name = label or f"{tag}_{uuid.uuid4().hex[:6]}"
    if d["TC_TIP"] > d["TC_ROOT"]:
        return _penalty("t/c tip > t/c root")
    geo = T.planform(d, float(TC["S_REF"]))
    run_dir = os.path.join(run_root or OUT_DIR, name)
    os.makedirs(run_dir, exist_ok=True)
    params = worker_params(d, geo, name, int(MESH["WAKE_ITERS"]))
    pf, rf = os.path.join(run_dir, "params.json"), os.path.join(run_dir, "result.json")
    if os.path.exists(rf):
        os.remove(rf)
    with open(pf, "w") as f:
        json.dump(params, f)
    time.sleep(random.random() * 0.4)
    ok, diag = vsp_runner.run_worker(run_dir, pf, rf, params["timeout"], lock=launch_lock)
    if not ok:
        print(f"[{tag}] REJECTED: worker failed: {diag}")
        return _penalty(diag)
    with open(rf, "r") as f:
        res = json.load(f)
    if res.get("error") or not res.get("rows"):
        print(f"[{tag}] REJECTED: {res.get('error')}")
        return _penalty(res.get("error", ""))
    try:
        m = T.analyse(res, d, TC, polars())
    except Exception as e:  # noqa: BLE001
        print(f"[{tag}] REJECTED: analysis failed: {type(e).__name__}: {e}")
        return _penalty(str(e))
    if not (-6.0 < m["alpha_trim"] < 16.0) or m["CD"] <= 0:
        print(f"[{tag}] REJECTED: unphysical trim (alpha {m['alpha_trim']:.1f}, CD {m['CD']:.4f})")
        return _penalty("unphysical")

    g = m["constraints"]
    feasible = all(v <= 1e-9 for v in g)
    rec = {"name": name, "design": d, "metrics": m, "F": [m["CD"], m["W_wing_kg"]], "G": g, "feasible": feasible}
    with open(os.path.join(run_dir, "eval.json"), "w") as f:
        json.dump(rec, f)
    if not keep_files:
        vsp_runner.slim_run_dir(run_dir)
    if not quiet:
        flag = "ok " if feasible else "viol " + ",".join(n.split()[0] for n, v in zip(T.CONSTRAINT_NAMES, g) if v > 1e-9)
        print(f"[{tag}] CD {m['CD']*1e4:6.1f} cts (0:{m['CD0']*1e4:.0f} i:{m['CDi']*1e4:.0f} w:{m['CDw']*1e4:.1f}) "
              f"L/D {m['LD']:.2f} | Mdd {m['Mdd_min']:.3f} | W {m['W_wing_lb']:.0f} lb | {flag}")
    return {"F": rec["F"], "G": g, "ok": True, "name": name, "design": d, "metrics": m, "feasible": feasible}


# ─── NSGA-II ─────────────────────────────────────────────────────────────────
from pymoo.core.problem import ElementwiseProblem  # noqa: E402
from pymoo.core.callback import Callback  # noqa: E402


class SaveCallback(Callback):
    """Checkpoint after every generation (module level so the algorithm stays picklable)."""

    def notify(self, algorithm):
        try:
            with open(os.path.join(OUT_DIR, "checkpoint.pkl"), "wb") as f:
                pickle.dump(algorithm, f)
        except Exception as e:  # noqa: BLE001
            print(f"[!] checkpoint failed: {e}")
        n = len(glob.glob(os.path.join(OUT_DIR, "*", "eval.json")))
        print(f"--- generation {algorithm.n_iter - 1} done | {n} wings solved ---", flush=True)


class TransonicProblem(ElementwiseProblem):
    """Minimize (C_D,total, wing weight) subject to T.CONSTRAINT_NAMES (module level so workers can unpickle it)."""

    def __init__(self, runner=None):
        bounds = T.design_bounds(TC)
        kw = {"elementwise_runner": runner} if runner is not None else {}
        super().__init__(n_var=len(bounds), n_obj=2, n_ieq_constr=N_CONSTR,
                         xl=[b[0] for b in bounds], xu=[b[1] for b in bounds], **kw)

    def _evaluate(self, x, out, *args, **kwargs):
        r = evaluate_design(x)
        out["F"], out["G"] = r["F"], r["G"]


def initial_population(pop, seed):
    bounds = T.design_bounds(TC)
    lo, hi = np.array([b[0] for b in bounds]), np.array([b[1] for b in bounds])
    rng = np.random.default_rng(seed)
    X = lo + rng.random((pop, len(bounds))) * (hi - lo)
    X[0] = np.clip(CHART_X[:len(bounds)], lo, hi)      # seed the benchmark wing
    return X


def run_optimization(args):
    from pymoo.algorithms.moo.nsga2 import NSGA2
    from pymoo.operators.crossover.sbx import SBX
    from pymoo.operators.mutation.pm import PM
    from pymoo.optimize import minimize
    from pymoo.termination import get_termination
    try:
        from pymoo.parallelization.starmap import StarmapParallelization
    except ImportError:
        from pymoo.core.problem import StarmapParallelization

    ckpt = os.path.join(OUT_DIR, "checkpoint.pkl")
    polars()                                           # build the XFOIL polars once, before forking workers
    cores = args.cores or os.cpu_count() or 4
    lock = multiprocessing.Lock()
    pool = multiprocessing.Pool(cores, initializer=init_worker, initargs=(lock,))
    problem = TransonicProblem(StarmapParallelization(pool.starmap))

    resume = os.path.exists(ckpt) and not args.fresh
    if resume:
        with open(ckpt, "rb") as f:
            algorithm = pickle.load(f)
        done = max(int(algorithm.n_iter) - 1, 0)
        target = done + args.gens
        algorithm.termination = get_termination("n_gen", target)
        algorithm.problem = problem
        print(f"[i] Resuming from generation {done} -> {target}")
    else:
        algorithm = NSGA2(pop_size=args.pop, n_offsprings=args.pop, sampling=initial_population(args.pop, args.seed),
                          crossover=SBX(prob=0.9, eta=15), mutation=PM(eta=20), eliminate_duplicates=True)
        target = args.gens
        print(f"[i] Fresh run: pop {args.pop}, {target} generations, {cores} workers")
    t0 = time.time()
    try:
        minimize(problem, algorithm, ("n_gen", target), seed=args.seed, copy_algorithm=False,
                 verbose=False, callback=SaveCallback())
        print(f"\n[✓] Optimization finished in {(time.time() - t0) / 60:.1f} min")
    except KeyboardInterrupt:
        print("\n[!] Interrupted; reporting what has been solved so far.")
    finally:
        pool.terminate()
        pool.join()


# ─── RESULTS ─────────────────────────────────────────────────────────────────
def rejudge(rec):
    """Re-evaluate the constraints of a stored wing with the CURRENT config, so thresholds
    (M_dd min, stall margin, fuel share, pitch-up table ...) can change without re-solving."""
    m = rec["metrics"]
    T.apply_stall(m, TC)
    m["fuel_share"] = m["fuel_m3"] / float(TC["FUEL_TOTAL_M3"])
    m["AR_pitchup_limit"] = T.pitchup_ar_limit(m["sweep_c4"], TC)
    m["constraints"] = rec["G"] = T.constraint_values(m, rec["design"], TC)
    rec["feasible"] = all(v <= 1e-9 for v in rec["G"])
    return rec


def load_evals(out_dir):
    recs = []
    for p in glob.glob(os.path.join(out_dir, "*", "eval.json")):
        try:
            with open(p, "r") as f:
                recs.append(rejudge(json.load(f)))
        except Exception:  # noqa: BLE001
            pass
    return recs


def nondominated(recs):
    pts = sorted(recs, key=lambda r: (r["F"][0], r["F"][1]))
    front, best_w = [], float("inf")
    for r in pts:                                       # sweep in CD order, keep strictly lighter wings
        if r["F"][1] < best_w - 1e-12:
            front.append(r)
            best_w = r["F"][1]
    return front


def violation(r):
    return float(sum(max(v, 0.0) for v in r["G"]))


def select_wing(front, chart, mode, weight):
    cd0, w0 = chart["F"]
    if mode == "drag":
        return min(front, key=lambda r: r["F"][0])
    if mode == "weight":
        return min(front, key=lambda r: r["F"][1])
    if mode == "weighted":
        return min(front, key=lambda r: r["F"][0] / cd0 + weight * r["F"][1] / w0)
    ref_lb = chart["metrics"]["W_wing_lb"]
    return max(front, key=lambda r: T.breguet_range_factor(r["metrics"]["LD"], r["metrics"]["W_wing_lb"], ref_lb, TC))


def row(r):
    d, m = r["design"], r["metrics"]
    return {"AR": d["AR"], "SWEEP_LE": d["SWEEP_LE"], "TAPER": d["TAPER"], "WASHOUT": d["WASHOUT"],
            "TC_ROOT": d["TC_ROOT"], "TC_TIP": d["TC_TIP"], "name": r["name"], "feasible": r["feasible"],
            **{k: v for k, v in m.items() if k not in ("span", "constraints")},
            "constraints": dict(zip(T.CONSTRAINT_NAMES, m["constraints"]))}


def evaluate_chart(out_dir, keep=True):
    """Chart wing through the same solver + analysis; cached in chart_wing/."""
    p = os.path.join(out_dir, "chart_wing", "eval.json")
    if os.path.exists(p):
        with open(p, "r") as f:
            return rejudge(json.load(f))
    x = CHART_X[:len(T.design_bounds(TC))]
    r = evaluate_design(x, label="chart_wing", keep_files=keep, run_root=out_dir)
    if not r["ok"]:
        raise RuntimeError("chart wing evaluation failed: " + str(r.get("reason")))
    with open(p, "r") as f:
        return rejudge(json.load(f))


def korn_min(r, chord_frac):
    """Min-over-span Korn M_dd of a stored wing for a given sweep reference line."""
    d, sp = r["design"], r["metrics"]["span"]
    sw = T.sweep_at(chord_frac, d["SWEEP_LE"], d["TAPER"], d["AR"])
    return float(np.min(T.korn_mdd(sp["cl"], sp["tc"], sw, TC["KORN_KAPPA"])))


def make_report(out_dir, select="range", weight=0.5, show=False, tag=""):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    chart = evaluate_chart(out_dir)
    recs = [r for r in load_evals(out_dir) if r["name"] != "chart_wing"]
    feas = [r for r in recs if r["feasible"]]
    print(f"\n[i] {len(recs)} wings solved, {len(feas)} feasible")
    if not feas:
        rep_dir = os.path.join(out_dir, "report" + (f"_{tag}" if tag else ""))
        os.makedirs(rep_dir, exist_ok=True)
        best = sorted(recs, key=violation)[:10]
        lines = ["# No feasible wing", "", f"{len(recs)} wings solved; none satisfies every constraint.", "",
                 "Best attainable value of each constraint quantity among all solved wings:", ""]
        for key, lab, fn in (("Mdd_min", "highest M_dd (min over span)", max), ("stall_out_ratio", "lowest outboard cl/clmax at stall onset", min),
                             ("fuel_share", "highest fuel share", max)):
            v = fn(r["metrics"][key] for r in recs)
            lines.append(f"* {lab}: {v:.3f}")
        lines += ["", "Least-violating designs:", "", "| AR | LE sweep | taper | washout | t/c r | t/c t | CD cts | W lb | M_dd | stall out | violates |", "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in best:
            d, m = r["design"], r["metrics"]
            bad = [n for n, v in zip(T.CONSTRAINT_NAMES, r["G"]) if v > 1e-9]
            lines.append(f"| {d['AR']:.2f} | {d['SWEEP_LE']:.1f} | {d['TAPER']:.2f} | {d['WASHOUT']:.1f} | {d['TC_ROOT']:.3f} | {d['TC_TIP']:.3f} | "
                         f"{m['CD']*1e4:.1f} | {m['W_wing_lb']:.0f} | {m['Mdd_min']:.3f} | {m['stall_out_ratio']:.2f} | {', '.join(bad)} |")
        with open(os.path.join(rep_dir, "infeasible.md"), "w", encoding="utf-8") as f:
            f.write(NL.join(lines) + NL)
        print(NL.join(lines[:12]))
        return None
    front = nondominated(feas)
    chosen = select_wing(front, chart, select, weight)

    rep = os.path.join(out_dir, "report" + (f"_{tag}" if tag else ""))
    os.makedirs(rep, exist_ok=True)
    with open(os.path.join(rep, "pareto.json"), "w") as f:
        json.dump([row(r) for r in front], f, indent=1)
    with open(os.path.join(rep, "chosen_wing.json"), "w") as f:
        json.dump({"design": chosen["design"], "metrics": chosen["metrics"], "select": select}, f, indent=1)
    with open(os.path.join(rep, "chart_wing.json"), "w") as f:
        json.dump({"design": chart["design"], "metrics": chart["metrics"], "feasible": chart["feasible"]}, f, indent=1)

    # spanwise CSV (chosen + chart)
    for tag, r in (("chosen", chosen), ("chart", chart)):
        sp = r["metrics"]["span"]
        with open(os.path.join(rep, f"spanwise_{tag}.csv"), "w") as f:
            f.write("eta,y_m,chord_m,t_c,cl,Mdd,Mcrit,CDw_local,cd_profile,Re,stall_eta,cl_over_clmax\n")
            for i in range(len(sp["eta"])):
                f.write(",".join(f"{sp[k][i]:.6g}" for k in ("eta", "y", "chord", "tc", "cl", "Mdd", "Mcrit", "CDw", "cd", "Re",
                                                              "stall_eta", "stall_ratio")) + "\n")

    # --- plots ---
    cm, ch = chosen["metrics"], chart["metrics"]
    M = float(TC["MACH"])
    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    ax.scatter([r["F"][0] * 1e4 for r in feas], [r["F"][1] for r in feas], s=10, c="#cfcfcf", label=f"feasible ({len(feas)})")
    sc = ax.scatter([r["F"][0] * 1e4 for r in front], [r["F"][1] for r in front], c=[r["metrics"]["Mdd_min"] for r in front],
                    cmap="viridis", s=55, edgecolor="k", zorder=3, label="Pareto front")
    fig.colorbar(sc, label="wing M_dd (min over span)")
    ax.plot([r["F"][0] * 1e4 for r in front], [r["F"][1] for r in front], "k--", lw=0.8, zorder=2)
    ax.scatter([chosen["F"][0] * 1e4], [chosen["F"][1]], marker="*", s=300, c="gold", edgecolor="k", zorder=5, label=f"chosen ({select})")
    ax.scatter([chart["F"][0] * 1e4], [chart["F"][1]], marker="X" if not chart["feasible"] else "s", s=120,
               c="#d62728", edgecolor="k", zorder=5,
               label="chart wing" + ("" if chart["feasible"] else " (violates constraints)"))
    ax.set_xlabel("cruise C_D,total [counts]")
    ax.set_ylabel("wing weight [kg] (Raymer)")
    ax.set_title(f"Pareto front: drag vs wing weight (M {M}, CL {TC['CL_TARGET']})")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "pareto.png"), dpi=150)
    plt.close(fig)

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    for r, c, ls, lab in ((chosen, "#1f77b4", "-", "chosen"), (chart, "#d62728", "--", "chart wing")):
        sp = r["metrics"]["span"]
        axs[0, 0].plot(sp["eta"], sp["cl"], ls, c=c, label=lab)
        axs[0, 1].plot(sp["eta"], sp["Mdd"], ls, c=c, label=f"M_dd {lab}")
        axs[0, 1].plot(sp["eta"], sp["Mcrit"], ls, c=c, alpha=0.5, label=f"M_crit {lab}")
        axs[1, 0].plot(sp["eta"], np.array(sp["CDw"]) * 1e4, ls, c=c, label=f"{lab}: C_Dw = {r['metrics']['CDw']*1e4:.1f} cts")
        axs[1, 1].plot(sp["stall_eta"], sp["stall_ratio"], ls, c=c, label=f"{lab}: first stall at eta {r['metrics']['stall_eta_first']:.2f}")
    axs[0, 1].axhline(M, c="k", lw=1, ls=":", label=f"cruise M {M}")
    axs[0, 1].axhline(TC["MDD_MIN"], c="g", lw=0.8, ls=":", label=f"M_dd min {TC['MDD_MIN']}")
    axs[1, 1].axvline(TC["STALL_ETA_OUT"], c="gray", lw=0.8, ls=":")
    axs[1, 1].axhline(TC["STALL_MARGIN"], c="gray", lw=0.8, ls=":", label=f"outboard limit {TC['STALL_MARGIN']}")
    for a, t, yl in ((axs[0, 0], "Section lift coefficient at cruise (CL = %.3f)" % TC["CL_TARGET"], "c_l"),
                     (axs[0, 1], "Drag-divergence and critical Mach (Korn)", "Mach"),
                     (axs[1, 0], "Local wave drag (Lock)", "C_Dw [counts]"),
                     (axs[1, 1], "Stall margin at CL_max (M %.2f, sea level)" % TC["STALL_MACH"], "c_l / c_l,max")):
        a.set_title(t)
        a.set_ylabel(yl)
        a.set_xlabel("semi-span fraction eta")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "spanwise.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.5))
    for r, c, lab in ((chart, "#d62728", "chart wing"), (chosen, "#1f77b4", "chosen")):
        d, m = r["design"], r["metrics"]
        tan_le = math.tan(math.radians(d["SWEEP_LE"]))
        yb = m["b"] / 2.0
        xs = [0, yb * tan_le, yb * tan_le + m["ct"], m["cr"], 0]
        ys = [0, yb, yb, 0, 0]
        ax.fill(ys + [-v for v in ys[::-1]], [-v for v in xs] + [-v for v in xs[::-1]], alpha=0.25, c=c, label=lab)
        ax.plot(ys + [-v for v in ys[::-1]], [-v for v in xs] + [-v for v in xs[::-1]], c=c)
    ax.set_aspect("equal")
    ax.set_xlabel("y [m]")
    ax.set_ylabel("-x [m]")
    ax.set_title("Planform (S = %.2f m^2)" % TC["S_REF"])
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(rep, "planform.png"), dpi=150)
    plt.close(fig)

    # --- markdown summary ---
    def fmt(r, title):
        d, m = r["design"], r["metrics"]
        return (f"| {title} | {d['AR']:.2f} | {d['SWEEP_LE']:.1f} / {m['sweep_c4']:.1f} | {d['TAPER']:.3f} | {d['WASHOUT']:.2f} | "
                f"{d['TC_ROOT']:.3f} / {d['TC_TIP']:.3f} | {m['b']:.2f} | {m['CD']*1e4:.1f} ({m['CD0']*1e4:.0f}/{m['CDi']*1e4:.0f}/{m['CDw']*1e4:.1f}) | "
                f"{m['LD']:.2f} | {m['Mdd_min']:.3f} | {m['W_wing_lb']:.0f} | {100*m['fuel_share']:.0f} % | "
                f"{m['stall_eta_first']:.2f} | {m['CM']:+.3f} | {'yes' if r['feasible'] else 'NO'} |")
    lines = [
        "# Transonic planform study: Navy jet trainer",
        "",
        f"Cruise M {TC['MACH']}, CL {TC['CL_TARGET']}, S {TC['S_REF']} m^2. {len(recs)} wings solved, {len(feas)} feasible, "
        f"{len(front)} on the Pareto front. Selection rule: **{select}**.",
        "",
        "| wing | AR | LE / c4 sweep [deg] | taper | washout [deg] | t/c root / tip | b [m] | C_D counts (0/i/w) | L/D | M_dd min | W_wing [lb] | fuel share | first stall eta | Cm | feasible |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
        fmt(chosen, "**chosen**"), fmt(chart, "chart wing"),
        "",
        f"* Drag change vs chart wing: {100*(cm['CD']/ch['CD']-1):+.1f} % ({(cm['CD']-ch['CD'])*1e4:+.1f} counts); "
        f"wing weight {100*(cm['W_wing_lb']/ch['W_wing_lb']-1):+.1f} % ({cm['W_wing_lb']-ch['W_wing_lb']:+.0f} lb).",
        f"* Highest wing M_dd on the Pareto front: {max(r['metrics']['Mdd_min'] for r in front):.3f}; "
        f"highest among all feasible wings: {max(r['metrics']['Mdd_min'] for r in feas):.3f} (goal {TC['MDD_GOAL']}).",
        f"* Chart wing violates: {[n for n, v in zip(T.CONSTRAINT_NAMES, chart['G']) if v > 1e-9] or 'nothing'}.",
        f"* Korn M_dd (min over span) with half-chord sweep / quarter-chord sweep: chosen {korn_min(chosen, 0.5):.3f} / {korn_min(chosen, 0.25):.3f}, "
        f"chart wing {korn_min(chart, 0.5):.3f} / {korn_min(chart, 0.25):.3f} (the Study-3 report quotes 0.775-0.800, which is the quarter-chord form).",
        "",
        "## Pareto front",
        "",
        "| CD counts | W [lb] | L/D | AR | LE sweep | taper | washout | t/c r | t/c t | M_dd |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in front:
        d, m = r["design"], r["metrics"]
        lines.append(f"| {m['CD']*1e4:.1f} | {m['W_wing_lb']:.0f} | {m['LD']:.2f} | {d['AR']:.2f} | {d['SWEEP_LE']:.1f} | "
                     f"{d['TAPER']:.3f} | {d['WASHOUT']:.2f} | {d['TC_ROOT']:.3f} | {d['TC_TIP']:.3f} | {m['Mdd_min']:.3f} |")
    with open(os.path.join(rep, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines[:9]))
    print(f"\n[✓] Report written to {rep}")
    return {"chosen": chosen, "chart": chart, "front": front}


# ─── AIRFOIL STAGE ───────────────────────────────────────────────────────────
def run_airfoil_stage(chosen, cores, catalog):
    """XFOIL airfoil search for the chosen wing at its cruise Reynolds number and trimmed angle of attack."""
    import airfoil_opti as A
    name = chosen["name"]
    path = os.path.join(OUT_DIR, name, name + ".vsp3")
    if not os.path.exists(path):
        print(f"[!] Cannot find the chosen wing's geometry: {path}")
        return None
    print("\n" + "=" * 70 + "\n AIRFOIL STAGE (chosen wing)\n" + "=" * 70)
    # Two stations (root and tip): the transonic wing is one straight-tapered panel with linear t/c
    stations, incidence = A.stations_from_vsp3(path, None, 2)
    st = T.cruise_state(TC)
    fs = {"V_MS": st["V"], "RHO": float(TC["RHO"]), "MU": float(TC["MU"]), "AOA": chosen["metrics"]["alpha_trim"], "INCIDENCE": incidence}
    sections = A.sectional_conditions(stations, fs)
    A.print_sections(sections, f"SECTIONAL CONDITIONS (trim alpha {fs['AOA']:.2f} deg)")
    return A.run_pipeline(sections, cores or os.cpu_count() or 4, CFG, catalog=catalog, interactive=False,
                          refine=False, write_active=False, show_plot=False)


# ─── CLI ─────────────────────────────────────────────────────────────────────
def main(argv=None):
    ap = argparse.ArgumentParser(description="RAPID transonic planform optimizer")
    ap.add_argument("--config")
    ap.add_argument("--pop", type=int, default=48)
    ap.add_argument("--gens", type=int, default=30)
    ap.add_argument("--cores", type=int)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--fresh", action="store_true")
    ap.add_argument("--no-tc", action="store_true", help="fix t/c at TC_ROOT / TC_TIP")
    ap.add_argument("--select", default="range", choices=["range", "drag", "weight", "weighted"])
    ap.add_argument("--weight", type=float, default=0.5)
    ap.add_argument("--evaluate", help="AR,SWEEP_LE,TAPER,WASHOUT[,TC_ROOT,TC_TIP]: evaluate one wing and exit")
    ap.add_argument("--report-only", action="store_true")
    ap.add_argument("--airfoil", action="store_true", help="afterwards, search airfoils for the chosen wing (uses the 'airfoil' config section)")
    ap.add_argument("--catalog", default="UIUC_All", help="airfoil catalog for --airfoil")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override a transonic config value (JSON), e.g. --set PITCHUP_ENABLED=false --set CLMAX_TIP=1.72")
    ap.add_argument("--tag", default="", help="write the report to report_<tag>/ (for sensitivity runs)")
    ap.add_argument("--out")
    args = ap.parse_args(argv)

    global OUT_DIR
    if args.out:
        OUT_DIR = os.path.abspath(args.out)
        os.environ["RAPID_OUTPUT_DIR"] = OUT_DIR
    if args.no_tc:
        TC["TC_VARIABLE"] = False
    for kv in args.set:
        k, v = kv.split("=", 1)
        TC[k.strip()] = json.loads(v)
        print(f"[cfg] override {k.strip()} = {TC[k.strip()]}")
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"[cfg] transonic: M {TC['MACH']} CL {TC['CL_TARGET']} S {TC['S_REF']} | vars "
          f"{'6 (incl. t/c)' if TC['TC_VARIABLE'] else '4'} | constraints: " + ", ".join(T.CONSTRAINT_NAMES))
    if TC["PITCHUP_ENABLED"]:
        print("[cfg] NOTE: pitch-up boundary is the PLACEHOLDER table in the config (Raymer Fig. 4.21 not digitized).")

    if args.evaluate:
        v = [float(t) for t in args.evaluate.split(",")]
        if len(v) == 4:
            v += [TC["TC_ROOT"], TC["TC_TIP"]]
        was = TC["TC_VARIABLE"]
        TC["TC_VARIABLE"] = True
        r = evaluate_design(v, label="single_eval", keep_files=True)
        TC["TC_VARIABLE"] = was
        if r["ok"]:
            m = {k: v for k, v in r["metrics"].items() if k not in ("span", "constraints")}
            print(json.dumps(m, indent=1))
            print("constraints:", dict(zip(T.CONSTRAINT_NAMES, [round(g, 3) for g in r["G"]])))
        return 0

    if not args.report_only:
        evaluate_chart(OUT_DIR)
        run_optimization(args)
    res = make_report(OUT_DIR, args.select, args.weight, tag=args.tag)
    if args.airfoil and res:
        run_airfoil_stage(res["chosen"], args.cores, args.catalog)
    return 0


if __name__ == "__main__":
    multiprocessing.freeze_support()
    sys.exit(main())
