"""
RAPID airfoil co-optimization (Phase 2), usable two ways:

  1. From the planform optimizer's post-run menu (planform_opti.py), for a wing
     picked from the Pareto front.
  2. Standalone (RUN_AIRFOIL.bat), for any saved OpenVSP geometry:
         python airfoil_opti.py [--config system_files/airfoil_config.json] [--yes]
     airfoil_config.json points to a .vsp3 preset; no NSGA-II run is needed.

Pipeline: spanwise stations -> local Re / alpha -> parallel XFOIL sweep of a
catalog -> best section per station (L/D; BWB favours near-zero Cm) ->
optional thickness/camber refinement -> active_airfoils.json, which every
later planform evaluation instantiates.
"""

import argparse
import io
import json
import math
import multiprocessing
import os
import shutil
import sys
import time

import numpy as np

try:
    if hasattr(sys.stdout, "buffer") and sys.stdout.buffer and getattr(sys.stdout, "encoding", "").lower() != "utf-8":
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
except Exception:
    pass

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

from rapid_config import load_effective_config  # noqa: E402
import geometry  # noqa: E402

AIRFOIL_DB = os.path.join(WORK_DIR, "airfoil_database")
# RAPID_ACTIVE_AIRFOILS relocates the back-feed file (and its profile folder), e.g. for test runs.
ACTIVE_AIRFOILS_CONFIG = os.environ.get("RAPID_ACTIVE_AIRFOILS") or os.path.join(SCRIPT_DIR, "active_airfoils.json")
ACTIVE_AIRFOILS_DIR = os.path.join(os.path.dirname(os.path.abspath(ACTIVE_AIRFOILS_CONFIG)), "active_airfoils")
DEFAULT_AIRFOIL_CONFIG = os.path.join(SCRIPT_DIR, "airfoil_config.json")
BASELINE_AIRFOIL = "naca2412"
CATALOGS = ["Low_Re", "Medium_Re", "High_Re", "UIUC_All", "Reflexed"]


# ─── FLIGHT STATE ────────────────────────────────────────────────────────────
def flight_state(cfg):
    fl = cfg["flight"]
    v = float(fl["V_KMH"]) / 3.6
    return {
        "V_MS": v, "RHO": float(fl["RHO"]), "MU": float(fl["MU"]),
        "AOA": float(fl["AOA"]), "INCIDENCE": float(fl["INCIDENCE"]),
        "WASHOUT": float(fl["INCIDENCE"]) - float(fl.get("TIP_INCIDENCE", 0.0)),
        "MACH": v / math.sqrt(1.4 * 287.05 * float(fl["TEMP_K"])),
    }


# ─── ACTIVE AIRFOIL ASSIGNMENT (active_airfoils.json) ────────────────────────
# Format: {"stations": [{"eta": 0.0, "airfoil": "<.dat path | nacaXXXX>"}, ...]}
# eta = y / exposed half-span. The legacy {"root","kink","tip"} format is still
# read: kink maps to the design's own kink position.
def load_active_airfoils(path=ACTIVE_AIRFOILS_CONFIG):
    data = {}
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f) or {}
        except Exception:
            data = {}
    if isinstance(data.get("stations"), list) and data["stations"]:
        return [{"eta": s["eta"], "airfoil": _resolve(s["airfoil"])} for s in data["stations"]]
    if any(k in data for k in ("root", "kink", "tip")):
        return [{"eta": 0.0, "airfoil": _resolve(data.get("root", BASELINE_AIRFOIL))},
                {"eta": "kink", "airfoil": _resolve(data.get("kink", BASELINE_AIRFOIL))},
                {"eta": 1.0, "airfoil": _resolve(data.get("tip", BASELINE_AIRFOIL))}]
    return [{"eta": 0.0, "airfoil": BASELINE_AIRFOIL}, {"eta": 1.0, "airfoil": BASELINE_AIRFOIL}]


def _resolve(airfoil):
    """Project-relative .dat paths (as saved) -> absolute; NACA codes pass through."""
    a = str(airfoil)
    if a.lower().endswith(".dat") and not os.path.isabs(a):
        return os.path.normpath(os.path.join(WORK_DIR, a))
    return a


def _portable(airfoil):
    a = str(airfoil)
    if a.lower().endswith(".dat") and os.path.isabs(a):
        try:
            rel = os.path.relpath(a, WORK_DIR)
            if not rel.startswith(".."):
                return rel.replace("\\", "/")
        except ValueError:   # different drive
            pass
    return a


def save_active_airfoils(stations, path=ACTIVE_AIRFOILS_CONFIG):
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"stations": [{"eta": s["eta"], "airfoil": _portable(s["airfoil"])} for s in stations]},
                  f, indent=2)
    return path


def is_custom(active):
    return any(str(s["airfoil"]).lower().endswith(".dat") for s in active)


def assign_airfoils(stations, active, kink_eta):
    """Pick, for every geometry station, the active airfoil nearest in eta."""
    resolved = [(kink_eta if s["eta"] == "kink" else float(s["eta"]), s["airfoil"]) for s in active]
    return [min(resolved, key=lambda r: abs(r[0] - st["eta"]))[1] for st in stations]


def bwb_baseline(active, baseline_name):
    """BWB bootstrap: seed every station with a reflexed section until co-opt has run."""
    if is_custom(active) or not baseline_name:
        return active, None
    fn = baseline_name if baseline_name.lower().endswith(".dat") else baseline_name + ".dat"
    path = os.path.join(AIRFOIL_DB, "Reflexed", fn)
    if not os.path.exists(path):
        return active, f"BWB baseline reflexed '{fn}' not found in airfoil_database/Reflexed"
    return [{"eta": 0.0, "airfoil": path}, {"eta": 1.0, "airfoil": path}], None


# ─── SECTIONAL CONDITIONS ────────────────────────────────────────────────────
def sectional_conditions(stations, fs):
    """Attach id / local Re / local alpha to geometry stations."""
    out = []
    for i, s in enumerate(stations):
        out.append({
            "id": i + 1, "zone": s["zone"], "eta": s["eta"], "y_loc": s["y"],
            "chord": s["chord"], "twist": s["twist"],
            "alpha": fs["AOA"] + fs["INCIDENCE"] + s["twist"],
            "Re": int(round(fs["RHO"] * fs["V_MS"] * s["chord"] / fs["MU"])),
        })
    return out


def sections_from_design(design, n_sections, cfg):
    """Stations for a planform design dict (pareto_results.json entry)."""
    fs = flight_state(cfg)
    x = [design[k] for k in ("AR", "kink_frac", "t_inner", "t_outer", "sweep_inner", "sweep_outer")]
    st = geometry.planform_stations(x, float(cfg["flight"]["S_REF"]), n_sections,
                                    fs["WASHOUT"], cfg.get("geometry"))
    return sectional_conditions(st, fs) if st else None


def _interp_stations(stations, n_sections):
    ys = [s["y"] for s in stations]
    out = []
    for i, y in enumerate(np.linspace(ys[0], ys[-1], n_sections)):
        zone = "ROOT" if i == 0 else ("TIP" if i == n_sections - 1 else "MID")
        out.append({"y": float(y), "eta": float(y / ys[-1]), "zone": zone,
                    "chord": float(np.interp(y, ys, [s["chord"] for s in stations])),
                    "x_le": float(np.interp(y, ys, [s["x_le"] for s in stations])),
                    "twist": float(np.interp(y, ys, [s["twist"] for s in stations]))})
    return out


def stations_from_vsp3(vsp3_path, wing_name=None, n_sections=None):
    """Read a wing's sections from a saved OpenVSP model.

    Returns (stations, incidence_deg). Stations sit at the model's own
    cross-sections unless n_sections asks for uniform interpolated stations.
    """
    vsp_root = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")
    if os.path.isdir(vsp_root):
        os.add_dll_directory(vsp_root)
    import openvsp as vsp

    vsp.ClearVSPModel()
    vsp.ReadVSPFile(vsp3_path)
    vsp.Update()
    wings = [g for g in vsp.FindGeoms() if vsp.GetGeomTypeName(g).upper() == "WING"]
    if wing_name:
        wings = [g for g in wings if vsp.GetGeomName(g) == wing_name]
    if not wings:
        raise ValueError(f"No wing{' named ' + wing_name if wing_name else ''} in {vsp3_path}")
    wid = wings[0]
    incidence = vsp.GetParmVal(wid, "Y_Rel_Rotation", "XForm")

    surf = vsp.GetXSecSurf(wid, 0)
    n_x = vsp.GetNumXSec(surf)
    val = lambda xs, n: vsp.GetParmVal(vsp.GetXSecParm(xs, n))  # noqa: E731
    first = vsp.GetXSec(surf, 1)
    y, x_le, twist = 0.0, 0.0, 0.0
    stations = [{"y": 0.0, "chord": val(first, "Root_Chord"), "x_le": 0.0, "twist": 0.0}]
    for i in range(1, n_x):
        xs = vsp.GetXSec(surf, i)
        span, cr, ct = val(xs, "Span"), val(xs, "Root_Chord"), val(xs, "Tip_Chord")
        sw, loc = val(xs, "Sweep"), val(xs, "Sweep_Location")
        tan_le = math.tan(math.radians(sw)) - loc * (ct - cr) / span if span > 0 else 0.0
        y += span
        x_le += span * tan_le
        twist = val(xs, "Twist")
        stations.append({"y": y, "chord": ct, "x_le": x_le, "twist": twist})
    b_half = stations[-1]["y"]
    for i, s in enumerate(stations):
        s["eta"] = s["y"] / b_half
        s["zone"] = "ROOT" if i == 0 else ("TIP" if i == len(stations) - 1 else "MID")
    if n_sections and n_sections != len(stations):
        stations = _interp_stations(stations, n_sections)
    return stations, incidence


def print_sections(sections, title):
    print("\n" + "=" * 86)
    print(f" {title}")
    print("=" * 86)
    print(f"{'Sec':^5} | {'Zone':^6} | {'eta':^6} | {'y [m]':^8} | {'Chord [m]':^9} | {'Twist':^7} | {'Alpha':^7} | {'Re':^12}")
    print("-" * 86)
    for s in sections:
        print(f"{s['id']:^5} | {s['zone']:^6} | {s['eta']:^6.3f} | {s['y_loc']:^8.3f} | {s['chord']:^9.3f} | "
              f"{s['twist']:^7.2f} | {s['alpha']:^7.2f} | {s['Re']:^12,}")
    print("=" * 86)


# ─── PARALLEL XFOIL SWEEP ────────────────────────────────────────────────────
def _parallel_xfoil_worker(args):
    """Top-level (picklable) pool worker."""
    from xfoil_handler import analyze_airfoil
    airfoil_path, reynolds, mach, alphas, geom_factors = args
    try:
        return (airfoil_path, geom_factors,
                analyze_airfoil(airfoil_path, reynolds, mach, alphas, geom_factors=geom_factors))
    except Exception:
        return (airfoil_path, geom_factors, {})


def airfoil_thickness(filepath):
    """Approximate max thickness (t/c) from a coordinate file."""
    ys = []
    try:
        with open(filepath, "r", errors="ignore") as f:
            for line in f:
                pts = line.split()
                if len(pts) >= 2:
                    try:
                        float(pts[0])
                        ys.append(float(pts[1]))
                    except ValueError:
                        pass
    except OSError:
        return 0.0
    return (max(ys) - min(ys)) if ys else 0.0


def _min_tc(section, af_cfg):
    if section["zone"] == "ROOT":
        return float(af_cfg["MIN_TC_ROOT"])
    if section["zone"] == "TIP":
        return float(af_cfg["MIN_TC_TIP"])
    return float(af_cfg["MIN_TC_MID"])


def _max_tc(section, af_cfg):
    key = "MAX_TC_ROOT" if section["zone"] == "ROOT" else "MAX_TC_TIP" if section["zone"] == "TIP" else "MAX_TC_MID"
    return float(af_cfg.get(key, 1.0))


def _pmap(pool, fn, items, label):
    """pool.map that prints a '[progress] done/total label' heartbeat about every 5 s (the GUI shows it as a bar)."""
    total, out = len(items), []
    t0 = last = time.time()
    for i, r in enumerate(pool.imap(fn, items, chunksize=2), 1):
        out.append(r)
        now = time.time()
        if now - last >= 5 or i == total:
            print(f"[progress] {i}/{total} {label} ({now - t0:.0f}s)", flush=True)
            last = now
    return out


def _score(c, bwb):
    # Conventional: pure L/D. BWB / tailless: strongly prefer near-zero /
    # slightly positive Cm (reflexed, self-trimming).
    return c["avg_ld"] - 200.0 * abs(c["avg_cm"] - 0.01) if bwb else c["avg_ld"]


# ─── STALL CHARACTER ─────────────────────────────────────────────────────────
STALL_ALPHAS = [float(a) for a in range(6, 25)]


def stall_drop(polar, delta):
    """Fraction of CLmax lost `delta` degrees past the CLmax angle.

    Returns (drop, alpha_clmax, CLmax). drop is None when XFOIL has no converged
    point that far past CLmax (an abrupt, separated stall usually looks like that)."""
    pts = sorted((a, v["CL"]) for a, v in polar.items())
    if len(pts) < 3:
        return None, None, None
    i = max(range(len(pts)), key=lambda k: pts[k][1])
    a0, cl0 = pts[i]
    target = a0 + delta
    lo, hi = [q for q in pts if q[0] <= target], [q for q in pts if q[0] >= target]
    if not hi or not lo or cl0 <= 0:
        return None, a0, cl0
    (a1, c1), (a2, c2) = lo[-1], hi[0]
    cl_t = c1 if a2 == a1 else c1 + (c2 - c1) * (target - a1) / (a2 - a1)
    return max(0.0, (cl0 - cl_t) / cl0), a0, cl0


def _stall_worker(args):
    from xfoil_handler import analyze_airfoil
    path, reynolds, mach = args
    try:
        return analyze_airfoil(path, reynolds, mach, STALL_ALPHAS, max_iter=200, timeout=180)
    except Exception:
        return {}


def apply_stall_filter(pool, candidates, section, mach, af_cfg):
    """Run a post-stall XFOIL sweep on the best `STALL_TOPK` candidates and drop the ones that lose
    more than STALL_DROP_MAX of CLmax within STALL_DELTA_ALPHA degrees. Returns (kept, evaluated)."""
    top = candidates[:int(af_cfg["STALL_TOPK"])]
    delta, dmax = float(af_cfg["STALL_DELTA_ALPHA"]), float(af_cfg["STALL_DROP_MAX"])
    t0 = time.time()
    # Stall happens at low speed, so the check can use its own Mach and a scaled Reynolds number
    # (STALL_MACH_LOW, STALL_RE_FACTOR = Re at the stall / approach speed divided by the cruise Re). Defaults: same as cruise.
    mach_s = float(af_cfg.get("STALL_MACH_LOW", mach))
    re_s = section["Re"] * float(af_cfg.get("STALL_RE_FACTOR", 1.0))
    polars = _pmap(pool, _stall_worker, [(c["path"], re_s, mach_s) for c in top], f"stall check, station {section['id']}")
    kept, n_unknown, n_abrupt = [], 0, 0
    for c, pol in zip(top, polars):
        drop, a_cl, clmax = stall_drop(pol, delta)
        c["stall_drop"], c["clmax"] = drop, clmax
        c["stall_ok"] = drop is not None and drop <= dmax
        if drop is None:
            n_unknown += 1
        elif drop > dmax:
            n_abrupt += 1
        if c["stall_ok"]:
            kept.append(c)
    print(f"    Stall check ({len(top)} best, drop <= {dmax:.0%} within {delta:g} deg of CLmax): {len(kept)} pass, "
          f"{n_abrupt} too abrupt, {n_unknown} no converged post-stall point ({time.time() - t0:.0f}s)")
    return kept, top


def _evaluate_files(pool, files, section, mach, alphas, cm_min):
    """XFOIL every file at the station; return the feasible candidates."""
    if not files:
        return []
    t0 = time.time()
    results = _pmap(pool, _parallel_xfoil_worker, [(f, section["Re"], mach, alphas, None) for f in files], f"XFOIL sweep, station {section['id']}")
    candidates, n_unconverged, n_unphysical, n_cm = [], 0, 0, 0
    for path, _gf, polar in results:
        if not polar or len(polar) < 2:   # <2 converged alphas = unreliable solve
            n_unconverged += 1
            continue
        avg_ld = float(np.mean([d["LD"] for d in polar.values()]))
        avg_cm = float(np.mean([d["CM"] for d in polar.values()]))
        if avg_ld > (200.0 if section["Re"] > 1e7 else 100.0):   # XFOIL blow-up; skip, never delete
            n_unphysical += 1
            continue
        if avg_cm < cm_min:
            n_cm += 1
            continue
        candidates.append({"name": os.path.basename(path), "path": path,
                           "avg_ld": avg_ld, "avg_cm": avg_cm, "polar": polar})
    print(f"    XFOIL: {len(files)} run in {time.time() - t0:.1f}s -> {len(candidates)} feasible "
          f"({n_unconverged} unconverged, {n_unphysical} unphysical, {n_cm} Cm < {cm_min})")
    return candidates


def sweep_candidates_for_sections(sections, catalog_dir, num_cores, bwb=False, af_cfg=None, pool=None):
    """XFOIL-sweep every catalog profile at every station; return the winners."""
    af_cfg = af_cfg or load_effective_config()["airfoil"]
    print(f"\n[*] Accessing catalog: {os.path.basename(catalog_dir)}")
    all_files = sorted(os.path.join(catalog_dir, f) for f in os.listdir(catalog_dir) if f.lower().endswith(".dat"))
    if not all_files:
        print(f"[!] No airfoil .dat files in {catalog_dir}")
        return None
    thickness = {f: airfoil_thickness(f) for f in all_files}
    print(f"[i] {len(all_files)} candidates across {num_cores} cores")

    spread = float(af_cfg["ALPHA_SPREAD"])
    mach = float(af_cfg["XFOIL_MACH"])
    own_pool = pool is None
    pool = pool or multiprocessing.Pool(processes=num_cores)
    final = {}
    try:
        for s in sections:
            alphas = [round(s["alpha"] - spread, 2), round(s["alpha"], 2), round(s["alpha"] + spread, 2)]
            print(f"\n[+] Station #{s['id']} ({s['zone']}) | Re {s['Re']:,} | alphas {alphas}")

            min_tc = _min_tc(s, af_cfg)
            max_tc = _max_tc(s, af_cfg)
            files = [f for f in all_files if min_tc <= thickness[f] <= max_tc]
            print(f"    [i] Thickness screen {min_tc:.2f} <= t/c <= {max_tc:.2f}: {len(all_files) - len(files)} rejected, "
                  f"{len(files)} proceed.")

            cm_min = float(af_cfg["CM_MIN_BWB"] if bwb else af_cfg["CM_MIN"])
            candidates = _evaluate_files(pool, files, s, mach, alphas, cm_min)
            rest = [f for f in all_files if f not in files and thickness[f] <= max_tc]   # relax the minimum only, never the maximum
            if not candidates and rest:
                # Never let the structural screen empty a station: fall back to the thinner sections.
                print(f"    [!] No screened profile is feasible; relaxing the thickness screen "
                      f"({len(rest)} more candidates).")
                candidates = _evaluate_files(pool, rest, s, mach, alphas, cm_min)
            candidates.sort(key=lambda c: _score(c, bwb), reverse=True)
            evaluated = candidates
            if candidates and af_cfg.get("STALL_CHECK", True):
                candidates, evaluated = apply_stall_filter(pool, candidates, s, mach, af_cfg)
                if not candidates:
                    print("    [!] No airfoil meets the stall limit: raise STALL_DROP_MAX / STALL_DELTA_ALPHA or switch STALL_CHECK off.")

            if not candidates:
                print(f"    [!] No feasible profile converged for station #{s['id']}.")
                final[s["id"]] = None
                continue
            best = candidates[0]
            print(f"    [✓] Winner: {best['name']} (L/D {best['avg_ld']:.2f}, Cm {best['avg_cm']:.4f})")
            final[s["id"]] = {"id": s["id"], "zone": s["zone"], "eta": s["eta"],
                              "winner": best["name"], "winner_path": best["path"],
                              "winner_ld": best["avg_ld"], "winner_cm": best["avg_cm"],
                              "ranking": candidates[:5],
                              "all": [{"name": c["name"], "path": c["path"], "avg_ld": c["avg_ld"], "avg_cm": c["avg_cm"],
                                       "stall_drop": c.get("stall_drop"), "clmax": c.get("clmax"), "stall_ok": c.get("stall_ok", True)}
                                      for c in evaluated]}
    finally:
        if own_pool:
            pool.close()
            pool.join()
    return final


# ─── GEOMETRIC REFINEMENT (XFOIL GDES TFAC) ──────────────────────────────────
def optimize_airfoil_geometry(airfoil_path, Re, target_alpha, original_cm, num_cores, mach=0.1, pool=None):
    """Grid-search thickness x camber scaling that maximizes L/D at ~constant Cm."""
    print(f"\n[*] Shape refinement for {os.path.basename(airfoil_path)} | Re {Re:,} | "
          f"alpha {target_alpha:.2f} | target Cm {original_cm:.4f}")
    grid = [(t, c) for t in (0.90, 0.95, 1.00, 1.05, 1.10)
            for c in (0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20)]
    alpha = round(target_alpha, 2)
    own_pool = pool is None
    pool = pool or multiprocessing.Pool(processes=num_cores)
    try:
        t0 = time.time()
        res = _pmap(pool, _parallel_xfoil_worker, [(airfoil_path, Re, mach, [alpha], g) for g in grid], "shape refinement")
        print(f"    {len(grid)} variants in {time.time() - t0:.1f}s")
    finally:
        if own_pool:
            pool.close()
            pool.join()

    candidates = []
    for _p, factors, polar in res:
        data = next((v for k, v in (polar or {}).items() if abs(k - alpha) < 0.05), None)
        if data and abs(float(data["CM"]) - original_cm) <= 0.0075:   # keep Cm (trim) constant
            candidates.append({"factors": factors, "ld": float(data["LD"]), "cm": float(data["CM"])})
    if not candidates:
        print("    [!] No variant satisfies the constant-Cm bound.")
        return None
    candidates.sort(key=lambda c: c["ld"], reverse=True)
    best = candidates[0]
    baseline = next((c for c in candidates if c["factors"] == (1.0, 1.0)), None)
    base_ld = baseline["ld"] if baseline else best["ld"]
    gain = (best["ld"] - base_ld) / base_ld * 100 if base_ld > 0 else 0.0
    print(f"    Baseline L/D {base_ld:.2f} -> best {best['ld']:.2f} "
          f"(thick x{best['factors'][0]:.2f}, camber x{best['factors'][1]:.2f}, {gain:+.2f}%)")
    return {"best": best, "baseline": baseline}


# ─── PIPELINE ────────────────────────────────────────────────────────────────
def read_input(prompt):
    """input() that returns None when stdin is closed (e.g. piped answers ran out)."""
    try:
        return input(prompt)
    except EOFError:
        print()
        return None


def _ask_yn(prompt, default, interactive):
    if not interactive:
        return default
    d = "Y" if default else "N"
    while True:
        a = read_input(f"{prompt} (Y/N) [Enter = {d}]: ")
        if a is None:
            return default
        a = a.strip().lower() or d.lower()
        if a in ("y", "n"):
            return a == "y"
        print("    Please answer 'Y' or 'N'.")


def choose_catalog(default, interactive):
    if not interactive:
        return default
    labels = {"Low_Re": "curated, low speed", "Medium_Re": "curated, general",
              "High_Re": "curated, high efficiency", "UIUC_All": "full UIUC ~1650 - recommended, slow",
              "Reflexed": "tailless / BWB self-trimming"}
    print("\nAirfoil catalogs:")
    for i, c in enumerate(CATALOGS, 1):
        print(f"  [{i}] {c:<10} ({labels[c]})")
    d = str(CATALOGS.index(default) + 1) if default in CATALOGS else "2"
    while True:
        a = read_input(f"Select catalog (1-{len(CATALOGS)}) [Enter = {d} ({CATALOGS[int(d) - 1]})]: ")
        a = (a or "").strip() or d
        if a.isdigit() and 1 <= int(a) <= len(CATALOGS):
            return CATALOGS[int(a) - 1]
        print("    Invalid choice.")


# ─── RESULTS FILE (shapes, type, Pareto) ─────────────────────────────────────
FAMILIES = (("naca64a", "NACA 6-series (64A)"), ("naca", "NACA"), ("opt_", "Refined (TFAC) variant"), ("sc2", "NASA supercritical SC(2)"),
            ("sc", "NASA supercritical"), ("whitcomb", "Whitcomb supercritical"), ("mh", "Hepperle MH"), ("eh", "Althaus EH"),
            ("fx", "Wortmann FX"), ("goe", "Goettingen"), ("clark", "Clark"), ("rae", "RAE"), ("ag", "Drela AG"), ("dae", "Drela DAE"),
            ("sd", "Selig/Donovan SD"), ("sg", "Selig/Giguere SG"), ("ua", "UA"), ("e", "Eppler"), ("s", "Selig S"), ("n", "NACA/NASA N"))


def family_of(name):
    stem = os.path.splitext(name)[0].lower().replace("phase2_", "")
    for prefix, fam in FAMILIES:
        if stem.startswith(prefix):
            return fam
    return "UIUC database"


def read_coords(path):
    pts = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            t = line.split()
            if len(t) >= 2:
                try:
                    pts.append((float(t[0]), float(t[1])))
                except ValueError:
                    pass
    return pts


def shape_info(coords):
    """Thickness, camber and reflex of a Selig-ordered coordinate list."""
    xs = np.array([c[0] for c in coords])
    le = int(np.argmin(xs))
    up, lo = coords[:le + 1][::-1], coords[le:]
    grid = np.linspace(0.0, 1.0, 101)
    yu = np.interp(grid, [c[0] for c in up], [c[1] for c in up])
    yl = np.interp(grid, [c[0] for c in lo], [c[1] for c in lo])
    camber = (yu + yl) / 2.0
    return {"tc": float((yu - yl).max()), "camber": float(camber[np.argmax(np.abs(camber))]),
            "x_camber": float(grid[np.argmax(np.abs(camber))]), "reflexed": bool(camber[85:].mean() < -0.001)}


def thin_coords(coords, n=70):
    k = max(1, len(coords) // n)
    out = coords[::k]
    if out[-1] != coords[-1]:
        out.append(coords[-1])
    return [[round(x, 4), round(y, 4)] for x, y in out]


def airfoil_entry(name, path, ld, cm, with_shape, stall_drop=None, clmax=None):
    e = {"name": os.path.splitext(name)[0], "family": family_of(name), "ld": round(ld, 2), "cm": round(cm, 4),
         "stall_drop": None if stall_drop is None else round(stall_drop, 4), "clmax": None if clmax is None else round(clmax, 3)}
    if with_shape:
        coords = read_coords(path)
        e.update(shape_info(coords))
        e["coords"] = thin_coords(coords)
    return e


def pareto_front(points, bwb):
    """Non-dominated set maximizing L/D and a Cm score (BWB: closeness to +0.01; otherwise the higher, less nose-down Cm)."""
    score = (lambda p: -abs(p["avg_cm"] - 0.01)) if bwb else (lambda p: p["avg_cm"])
    front, best = [], -1e9
    for p in sorted(points, key=lambda p: (-p["avg_ld"], -score(p))):
        if score(p) > best + 1e-12:
            front.append(p)
            best = score(p)
    return front


def results_dir():
    return os.environ.get("RAPID_OUTPUT_DIR") or os.path.join(WORK_DIR, "wing_models", "airfoil")


def write_results(selections, sections, catalog, bwb):
    by_id = {s["id"]: s for s in sections}
    out = {"catalog": catalog, "bwb": bool(bwb), "stall_delta": float(load_effective_config()["airfoil"]["STALL_DELTA_ALPHA"]),
           "sections": []}
    for sid, d in selections.items():
        if not d:
            continue
        sec = by_id[sid]
        front = pareto_front([p for p in d.get("all", []) if p.get("stall_ok", True)], bwb)
        if len(front) > 10:                                   # thin to <= 10 cards, keeping both ends
            idx = np.linspace(0, len(front) - 1, 10).round().astype(int)
            front = [front[i] for i in sorted(set(idx))]
        names = {p["name"] for p in front}
        out["sections"].append({
            "id": sid, "zone": d["zone"], "eta": round(float(d["eta"]), 4), "Re": sec["Re"], "alpha": round(sec["alpha"], 2),
            "winner": airfoil_entry(d["winner"], d.get("winner_path"), d["winner_ld"], d["winner_cm"], True,
                                    next((p.get("stall_drop") for p in d.get("all", []) if p["name"] == d["winner"]), None),
                                    next((p.get("clmax") for p in d.get("all", []) if p["name"] == d["winner"]), None)),
            "points": [{**airfoil_entry(p["name"], p["path"], p["avg_ld"], p["avg_cm"], p["name"] in names, p.get("stall_drop"), p.get("clmax")),
                        "pareto": p["name"] in names, "stall_ok": p.get("stall_ok", True)}
                       for p in d.get("all", [])]})
    os.makedirs(results_dir(), exist_ok=True)
    path = os.path.join(results_dir(), "airfoil_results.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f)
    print(f"[✓] Airfoil results (shapes + Pareto) saved: {path}")


def plot_winners(selections, out_png=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        plt.figure(figsize=(10, 5))
        plt.title("Optimal Sectional Airfoils", fontweight="bold")
        plt.xlabel("x/c")
        plt.ylabel("y/c")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.axis("equal")
        for sid, d in selections.items():
            if not d:
                continue
            pts = []
            with open(d["winner_path"], "r", errors="ignore") as f:
                for line in f:
                    p = line.split()
                    if len(p) >= 2:
                        try:
                            pts.append([float(p[0]), float(p[1])])
                        except ValueError:
                            pass
            if pts:
                pts = np.array(pts)
                plt.plot(pts[:, 0], pts[:, 1], label=f"Sec {sid} ({d['zone']}): {d['winner']}")
        plt.legend()
        plt.tight_layout()
        if out_png:
            os.makedirs(os.path.dirname(out_png), exist_ok=True)
            plt.savefig(out_png, dpi=120)
            print(f"[✓] Airfoil plot saved: {out_png}")
        if show:
            print("\n[*] Airfoil viewer open (close the window to continue)")
            plt.show()
    except Exception as e:
        print(f"[!] Could not plot airfoils: {e}")
    finally:
        plt.close("all")


def print_selections(selections, title):
    print("\n" + "=" * 80)
    print(f" {title}")
    print("=" * 80)
    print(f"{'Sec':^4} | {'Zone':^6} | {'Winning Airfoil':^32} | {'Avg L/D':^9} | {'Avg CM':^9}")
    print("-" * 80)
    for sid, d in selections.items():
        if not d:
            print(f"{sid:^4} | {'---':^6} | {'[NO CONVERGENCE]':^32} | {'---':^9} | {'---':^9}")
        else:
            print(f"{sid:^4} | {d['zone']:^6} | {d['winner']:^32} | {d['winner_ld']:^9.2f} | {d['winner_cm']:^9.4f}")
    print("=" * 80)


def run_pipeline(sections, num_cores, cfg, catalog=None, interactive=True,
                 refine=None, write_active=None, show_plot=None):
    """Sweep -> (refine) -> (back-feed). Returns the station list written, or None."""
    bwb = bool(cfg["bwb"]["BWB_MODE"])
    af_cfg = cfg["airfoil"]
    catalog = choose_catalog(catalog or cfg["bwb"].get("AIRFOIL_CATALOG") or ("Reflexed" if bwb else "Medium_Re"),
                             interactive)
    catalog_path = os.path.join(AIRFOIL_DB, catalog)
    if not os.path.isdir(catalog_path):
        print(f"[!] Catalog folder missing: {catalog_path} (run SETUP.bat for UIUC_All). Using Medium_Re.")
        catalog_path = os.path.join(AIRFOIL_DB, "Medium_Re")

    pool = multiprocessing.Pool(processes=num_cores)
    try:
        selections = sweep_candidates_for_sections(sections, catalog_path, num_cores, bwb, af_cfg, pool=pool)
        if not selections:
            print("\n[!] Sweep failed.")
            return None
        print_selections(selections, "OPTIMAL SECTIONAL MATCHING")
        plot_winners(selections, os.path.join(WORK_DIR, "PLOTS", "airfoil_winners.png"),
                     show=interactive if show_plot is None else show_plot)

        do_refine = refine if refine is not None else _ask_yn(
            "Refine thickness/camber to boost L/D at constant Cm?", False, interactive)
        if do_refine:
            os.makedirs(ACTIVE_AIRFOILS_DIR, exist_ok=True)
            by_id = {s["id"]: s for s in sections}
            for sid, d in selections.items():
                if not d:
                    continue
                sec = by_id[sid]
                opt = optimize_airfoil_geometry(d["winner_path"], sec["Re"], sec["alpha"], d["winner_cm"],
                                                num_cores, mach=float(af_cfg["XFOIL_MACH"]), pool=pool)
                if not opt or opt["best"]["factors"] == (1.0, 1.0):
                    print(f"    [i] Baseline shape already optimal for station #{sid}.")
                    continue
                best = opt["best"]
                if not _ask_yn(f"    Accept tweaked shape for station #{sid} (L/D {best['ld']:.1f})?",
                               True, interactive):
                    continue
                tf, cf = best["factors"]
                stem = os.path.splitext(d["winner"])[0].lower()
                fname = f"opt_s{sid}_{stem}_t{tf:.2f}_c{cf:.2f}.dat"
                export = os.path.join(ACTIVE_AIRFOILS_DIR, fname)
                from xfoil_handler import analyze_airfoil
                analyze_airfoil(d["winner_path"], sec["Re"], mach=float(af_cfg["XFOIL_MACH"]),
                                alphas=[round(sec["alpha"], 2)], geom_factors=(tf, cf), save_coords_path=export)
                if os.path.exists(export):
                    d.update({"winner": fname, "winner_path": export, "winner_ld": best["ld"], "winner_cm": best["cm"]})
                    print(f"    [✓] Refined profile saved: {fname}")
                else:
                    print("    [!] Coordinate export failed; keeping the catalog shape.")
            print_selections(selections, "REVISED RECOMMENDATIONS (AFTER REFINEMENT)")
    finally:
        pool.close()
        pool.join()

    write_results(selections, sections, catalog, bwb)

    do_write = write_active if write_active is not None else _ask_yn(
        "Back-feed these profiles into the planform optimizer (active_airfoils.json)?", True, interactive)
    if not do_write:
        return None
    os.makedirs(ACTIVE_AIRFOILS_DIR, exist_ok=True)
    stations = []
    for s in sections:
        d = selections.get(s["id"])
        if not d:
            print(f"[i] Station #{s['id']} has no converged winner; it will use its neighbours' sections.")
            continue
        dst = os.path.join(ACTIVE_AIRFOILS_DIR, f"phase2_s{s['id']}_{os.path.basename(d['winner_path'])}")
        if os.path.abspath(d["winner_path"]) != os.path.abspath(dst):
            shutil.copyfile(d["winner_path"], dst)
        stations.append({"eta": round(s["eta"], 5), "airfoil": os.path.abspath(dst)})
    if not stations:
        print("[!] Nothing converged; active_airfoils.json left unchanged.")
        return None
    path = save_active_airfoils(stations)
    print(f"\n[✓] {len(stations)} station airfoils written to {path}")
    for st in stations:
        print(f"    eta {st['eta']:.3f}: {os.path.basename(st['airfoil'])}")
    return stations


# ─── STANDALONE ENTRY POINT ──────────────────────────────────────────────────
AIRFOIL_CONFIG_TEMPLATE = {
    "vsp3": "system_files/altc130mod_preset.vsp3",
    "wing_name": None,
    "n_sections": None,
    "catalog": "Medium_Re",
    "cores": None,
    "refine": False,
    "write_active": True,
}


def main(argv=None):
    ap = argparse.ArgumentParser(description="RAPID standalone airfoil optimization for a saved .vsp3 wing")
    ap.add_argument("--config", default=DEFAULT_AIRFOIL_CONFIG, help="airfoil_config.json path")
    ap.add_argument("--vsp3", help="override the .vsp3 geometry path")
    ap.add_argument("--yes", action="store_true", help="non-interactive: use config values, no prompts")
    args = ap.parse_args(argv)

    if not os.path.exists(args.config):
        with open(args.config, "w", encoding="utf-8") as f:
            json.dump(AIRFOIL_CONFIG_TEMPLATE, f, indent=2)
        print(f"[i] Wrote a template config to {args.config}; edit 'vsp3' and run again.")
        return 1
    with open(args.config, "r", encoding="utf-8") as f:
        acfg = {**AIRFOIL_CONFIG_TEMPLATE, **json.load(f)}

    vsp3 = args.vsp3 or acfg["vsp3"]
    if not os.path.isabs(vsp3):
        vsp3 = os.path.join(WORK_DIR, vsp3)
    if not os.path.exists(vsp3):
        print(f"[!] Geometry not found: {vsp3}")
        return 1

    cfg = load_effective_config()
    fs = flight_state(cfg)
    stations, incidence = stations_from_vsp3(vsp3, acfg.get("wing_name"), acfg.get("n_sections"))
    fs["INCIDENCE"] = incidence   # the saved model carries its own incidence
    sections = sectional_conditions(stations, fs)
    print_sections(sections, f"SECTIONAL CONDITIONS — {os.path.basename(vsp3)} (incidence {incidence:.2f} deg)")

    cores = int(acfg.get("cores") or os.cpu_count() or 4)
    result = run_pipeline(sections, cores, cfg, catalog=acfg.get("catalog"), interactive=not args.yes,
                          refine=acfg.get("refine") if args.yes else None,
                          write_active=acfg.get("write_active") if args.yes else None)
    return 0 if result else 2


if __name__ == "__main__":
    multiprocessing.freeze_support()
    sys.exit(main())
