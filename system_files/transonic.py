"""
RAPID transonic layer: closed-form physics that sits on top of the VSPAERO
vortex-lattice strip loads. No solver is imported here (except in build_polars),
so everything below is unit-testable.

    wave drag      Korn drag-divergence Mach per strip + Lock 4th-power rise
    profile drag   XFOIL section polars (transition fixed at 5 %) per strip
    induced drag   VSPAERO Trefftz-plane CDi, rescaled to the trimmed CL
    wing weight    Raymer fighter/attack wing weight (eq. 15.25)
    fuel volume    Raymer integral-tank estimate
    pitch-up       Raymer Fig. 4.21 boundary (table in the config)
    tip stall      local cl / cl_max at the low-speed CL_max condition

Design vector x = [AR, sweep_LE, taper, washout, tc_root, tc_tip] (the last two
only when transonic.TC_VARIABLE is true). Geometry is a single straight-tapered
semi-wing (no kink, no fuselage cut-out) of fixed area S.
"""

import json
import math
import os

import numpy as np

LB_TO_KG = 0.45359237
M2_TO_FT2 = 10.7639104
DESIGN_KEYS = ("AR", "SWEEP_LE", "TAPER", "WASHOUT", "TC_ROOT", "TC_TIP")


# ─── GEOMETRY ────────────────────────────────────────────────────────────────
def unpack_design(x, tcfg):
    """Design vector -> dict (fixed t/c from the config when TC_VARIABLE is off)."""
    x = [float(v) for v in x]
    d = dict(zip(DESIGN_KEYS[:4], x[:4]))
    if tcfg["TC_VARIABLE"]:
        d["TC_ROOT"], d["TC_TIP"] = x[4], x[5]
    else:
        d["TC_ROOT"], d["TC_TIP"] = float(tcfg["TC_ROOT"]), float(tcfg["TC_TIP"])
    return d


def design_bounds(tcfg):
    keys = DESIGN_KEYS if tcfg["TC_VARIABLE"] else DESIGN_KEYS[:4]
    return [tuple(tcfg["BOUNDS"][k]) for k in keys]


def sweep_at(frac, sweep_le_deg, taper, AR):
    """Sweep [deg] of the line at chord fraction `frac` (0 = LE, 0.25, 0.5 ...)."""
    t = math.tan(math.radians(sweep_le_deg)) - 4.0 * frac * (1.0 - taper) / (AR * (1.0 + taper))
    return math.degrees(math.atan(t))


def planform(d, S):
    """Trapezoid geometry from the design dict."""
    AR, lam, sw = d["AR"], d["TAPER"], d["SWEEP_LE"]
    b = math.sqrt(AR * S)
    cr = 2.0 * S / (b * (1.0 + lam))
    mac = (2.0 / 3.0) * cr * (1.0 + lam + lam * lam) / (1.0 + lam)
    y_mac = b / 6.0 * (1.0 + 2.0 * lam) / (1.0 + lam)
    tan_le = math.tan(math.radians(sw))
    return {"b": b, "cr": cr, "ct": lam * cr, "mac": mac, "y_mac": y_mac,
            "x_le_mac": y_mac * tan_le,
            "sweep_c4": sweep_at(0.25, sw, lam, AR), "sweep_c2": sweep_at(0.5, sw, lam, AR)}


def tc_at(eta, d):
    return d["TC_ROOT"] + (d["TC_TIP"] - d["TC_ROOT"]) * np.asarray(eta)


# ─── FLIGHT STATE ────────────────────────────────────────────────────────────
def cruise_state(tcfg):
    v = tcfg["MACH"] * tcfg["A_SOUND"]
    return {"V": v, "q": 0.5 * tcfg["RHO"] * v * v}


# ─── WAVE DRAG (Korn + Lock) ─────────────────────────────────────────────────
MCRIT_OFFSET = (0.1 / 80.0) ** (1.0 / 3.0)       # M_dd - M_crit for dCd/dM = 0.1 (Korn/Mason)


def korn_mdd(cl, tc, sweep_deg, kappa):
    c = math.cos(math.radians(sweep_deg))
    return kappa / c - np.asarray(tc) / c ** 2 - np.asarray(cl) / (10.0 * c ** 3)


def lock_cdw(mach, mcrit):
    """C_Dw = 20 (M - M_crit)^4 above M_crit, else 0."""
    return 20.0 * np.maximum(mach - np.asarray(mcrit), 0.0) ** 4


def wave_drag_strips(cl, tc, sweep_deg, mach, kappa):
    mdd = korn_mdd(cl, tc, sweep_deg, kappa)
    mcrit = mdd - MCRIT_OFFSET
    return mdd, mcrit, lock_cdw(mach, mcrit)


# ─── PROFILE DRAG FROM XFOIL POLARS ──────────────────────────────────────────
def hoerner_ff(tc):
    """Form factor of a section: 1 + 2 t/c + 60 (t/c)^4 (Hoerner)."""
    tc = np.asarray(tc)
    return 1.0 + 2.0 * tc + 60.0 * tc ** 4


class PolarTable:
    """cd(cl) of the root and tip sections: quadratic fits of the XFOIL polars."""

    def __init__(self, data):
        self.data = data
        self.tc = {k: float(data[k]["tc"]) for k in ("root", "tip")}
        self.coef = {k: np.array(data[k]["fit"]) for k in ("root", "tip")}   # cd = c0 + c1 cl + c2 cl^2
        self.Re = float(data["Re"])

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            return cls(json.load(f))

    def cd(self, cl, eta):
        cl = np.asarray(cl, dtype=float)
        eta = np.asarray(eta, dtype=float)
        cd = {k: np.polyval(self.coef[k][::-1], cl) for k in ("root", "tip")}
        return (1.0 - eta) * cd["root"] + eta * cd["tip"]

    def nominal_tc(self, eta):
        return (1.0 - np.asarray(eta)) * self.tc["root"] + np.asarray(eta) * self.tc["tip"]


def build_polars(tcfg, work_dir, out_path=None, verbose=True):
    """Run XFOIL (transition forced at 5 % by xfoil_handler for Re > 1e7) on the two
    section files and store quadratic cd(cl) fits."""
    from xfoil_handler import analyze_airfoil
    out_path = out_path or os.path.join(work_dir, tcfg["POLAR_FILE"])
    Re, mach = float(tcfg["POLAR_RE"]), float(tcfg["POLAR_MACH"])
    alphas = [round(a, 2) for a in np.arange(-1.5, 5.01, 0.5)]
    data = {"Re": Re, "mach": mach, "alphas": alphas, "note": "XFOIL 6.99, xtr 0.05/0.05, Ncrit 11"}
    for key, path_key, tc in (("root", "AIRFOIL_ROOT", 0.12), ("tip", "AIRFOIL_TIP", 0.10)):
        path = os.path.join(work_dir, tcfg[path_key])
        polar = analyze_airfoil(path, Re, mach, alphas, max_iter=200)
        pts = sorted((v["CL"], v["CD"]) for v in polar.values() if 0.0 <= v["CL"] <= 0.95)
        if len(pts) < 5:
            raise RuntimeError(f"XFOIL converged only {len(pts)} usable points for {path}")
        cl, cd = np.array(pts).T
        fit = np.polyfit(cl, cd, 2)[::-1]                       # c0, c1, c2
        data[key] = {"file": tcfg[path_key], "tc": tc, "cl": cl.tolist(), "cd": cd.tolist(),
                     "fit": fit.tolist(), "cl_range": [float(cl.min()), float(cl.max())]}
        if verbose:
            print(f"[polar] {key}: {len(pts)} points, cd(0.27) = {np.polyval(fit[::-1], 0.272):.5f}")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)
    return out_path


def load_polars(tcfg, work_dir):
    path = os.path.join(work_dir, tcfg["POLAR_FILE"])
    if not os.path.exists(path):
        print(f"[transonic] {tcfg['POLAR_FILE']} missing: running XFOIL to build it...")
        build_polars(tcfg, work_dir, path)
    return PolarTable.load(path)


# ─── WEIGHT, FUEL, PITCH-UP ──────────────────────────────────────────────────
def raymer_wing_weight_lb(S_m2, AR, sweep_c4_deg, taper, tc_root, tcfg, k_dw=1.0, k_vs=1.0):
    """Raymer (4th ed.) eq. 15.25, fighter/attack wing weight [lb]:
         0.0103 Kdw Kvs (Wdg Nz)^0.5 Sw^0.622 A^0.785 (t/c)r^-0.4 (1+L)^0.05 cos(L_c/4)^-1 Scsw^0.04
    Sw and Scsw in ft^2, Wdg in lb, Nz = ultimate load factor."""
    sw = S_m2 * M2_TO_FT2
    scsw = tcfg["CSW_FRAC"] * sw
    nz = tcfg["NZ_ULT_FACTOR"] * tcfg["NZ_LIMIT"]
    return (0.0103 * k_dw * k_vs * (tcfg["W0_LB"] * nz) ** 0.5 * sw ** 0.622 * AR ** 0.785
            * tc_root ** -0.4 * (1.0 + taper) ** 0.05 / math.cos(math.radians(sweep_c4_deg)) * scsw ** 0.04)


def wing_fuel_volume_m3(S_m2, b, taper, tc_root, tc_tip):
    """Raymer integral-tank estimate: 0.54 S^2/b (t/c)r (1 + L sqrt(tau) + L^2 tau)/(1 + L)^2."""
    tau = tc_tip / tc_root
    return 0.54 * S_m2 ** 2 / b * tc_root * (1.0 + taper * math.sqrt(tau) + taper ** 2 * tau) / (1.0 + taper) ** 2


def pitchup_ar_limit(sweep_c4_deg, tcfg):
    return float(np.interp(sweep_c4_deg, tcfg["PITCHUP_SWEEP_DEG"], tcfg["PITCHUP_AR_MAX"]))


def breguet_range_factor(ld, wing_weight_lb, ref_wing_weight_lb, tcfg):
    """(L/D) ln(W0 / (W0 - fuel)) at fixed gross weight: every extra pound of wing
    displaces a pound of fuel. Used to pick one wing from the Pareto front."""
    w0 = tcfg["W0_LB"]
    fuel = tcfg["FUEL_TOTAL_LB"] - (wing_weight_lb - ref_wing_weight_lb)
    return ld * math.log(w0 / (w0 - fuel))


# ─── TIP STALL ───────────────────────────────────────────────────────────────
def stall_onset(eta, cl_a, cl_b, alpha_a, alpha_b, clmax_root, clmax_tip):
    """First-stall analysis from two linear (VLM) strip distributions.

    cl_i(alpha) = A_i + B_i alpha. Returns (alpha_stall, r, eta_first) where
    r_i = cl_i/clmax_i at the angle where the first strip reaches 1.
    """
    eta = np.asarray(eta, dtype=float)
    cl_a, cl_b = np.asarray(cl_a, dtype=float), np.asarray(cl_b, dtype=float)
    B = (cl_b - cl_a) / (alpha_b - alpha_a)
    A = cl_a - B * alpha_a
    clmax = clmax_root + (clmax_tip - clmax_root) * eta
    with np.errstate(divide="ignore", invalid="ignore"):
        a_i = np.where(B > 1e-9, (clmax - A) / B, np.inf)
    alpha_s = float(np.min(a_i))
    r = (A + B * alpha_s) / clmax
    return alpha_s, r, float(eta[int(np.argmax(r))])


def induced_drag(r1, r2, cl_target):
    """CDi at cl_target from two solved points, using CDi = s^2 (CL - CL0)^2.

    sqrt(CDi) is linear in CL on one side of the minimum-drag CL0, so two points fix
    s and CL0 (a twisted wing has CL0 != 0, which a plain k CL^2 scaling gets wrong)."""
    c1, c2 = r1["CDi"], r2["CDi"]
    if c1 > 0 and c2 > 0 and r2["CL"] != r1["CL"]:
        s = (math.sqrt(c2) - math.sqrt(c1)) / (r2["CL"] - r1["CL"])
        if s > 0:
            cl0 = r1["CL"] - math.sqrt(c1) / s
            return float(s * s * (cl_target - cl0) ** 2)
    return float(c2 * (cl_target / r2["CL"]) ** 2)


# ─── FULL ANALYSIS OF ONE WING ───────────────────────────────────────────────
def _case(result, mach):
    """The two (alpha) rows and the two strip sets of the run closest to `mach`."""
    rows = [r for r in result["rows"] if abs(r["mach"] - mach) < 1e-3]
    strips = [s for s in result["strips"] if abs(s["mach"] - mach) < 1e-3]
    rows.sort(key=lambda r: r["alpha"])
    strips.sort(key=lambda s: s["alpha"])
    if len(rows) < 2 or len(strips) < 2:
        raise ValueError(f"need two alpha cases at M={mach}")
    return rows, strips


def analyse(result, d, tcfg, polars, work_dir=None):
    """Aero + wave drag + weight + constraints for one solved wing.

    `result` is the worker's result.json (machs [stall, cruise], two alphas each).
    Returns a flat dict of scalars plus the spanwise arrays under "span".
    """
    S = float(tcfg["S_REF"])
    geo = planform(d, S)
    st = cruise_state(tcfg)
    M = float(tcfg["MACH"])
    CLt = float(tcfg["CL_TARGET"])

    # --- trim to CL_TARGET at cruise (VLM is linear in alpha, so exact) ------
    rows, strips = _case(result, M)
    r1, r2 = rows
    t = (CLt - r1["CL"]) / (r2["CL"] - r1["CL"])
    alpha_trim = r1["alpha"] + t * (r2["alpha"] - r1["alpha"])
    lerp = lambda a, b: np.asarray(a) + t * (np.asarray(b) - np.asarray(a))        # noqa: E731
    y = np.array(strips[0]["y"])
    dA = np.array(strips[0]["dA"])
    chord = np.array(strips[0]["chord"])
    cl = lerp(strips[0]["cl"], strips[1]["cl"])
    cm = float(lerp(r1["CM"], r2["CM"]))
    cdi = induced_drag(r1, r2, CLt)

    eta = y / (geo["b"] / 2.0)
    w = dA / dA.sum()
    tc = tc_at(eta, d)

    # --- wave drag -------------------------------------------------------------
    sweep_korn = sweep_at(float(tcfg["KORN_CHORD_FRAC"]), d["SWEEP_LE"], d["TAPER"], d["AR"])
    mdd, mcrit, cdw_i = wave_drag_strips(cl, tc, sweep_korn, M, tcfg["KORN_KAPPA"])
    cdw = float(np.sum(cdw_i * w))

    # --- profile drag ------------------------------------------------------------
    re_i = tcfg["RHO"] * st["V"] * chord / tcfg["MU"]
    cd_i = (polars.cd(cl, eta) * (re_i / polars.Re) ** tcfg["RE_EXPONENT"]
            * hoerner_ff(tc) / hoerner_ff(polars.nominal_tc(eta)))
    cd0 = float(np.sum(cd_i * w))

    cd_total = cd0 + cdi + cdw

    # --- tip stall source data (evaluated below by apply_stall; stored so it can be re-judged) ---
    s_rows, s_strips = _case(result, float(tcfg["STALL_MACH"]))
    stall_src = {"eta": (np.array(s_strips[0]["y"]) / (geo["b"] / 2.0)).tolist(),
                 "cl_a": s_strips[0]["cl"], "cl_b": s_strips[1]["cl"],
                 "alpha_a": s_strips[0]["alpha"], "alpha_b": s_strips[1]["alpha"],
                 "CL_a": s_rows[0]["CL"], "CL_b": s_rows[1]["CL"]}

    # --- weight, fuel, pitch-up --------------------------------------------------
    w_lb = raymer_wing_weight_lb(S, d["AR"], geo["sweep_c4"], d["TAPER"], d["TC_ROOT"], tcfg)
    fuel = wing_fuel_volume_m3(S, geo["b"], d["TAPER"], d["TC_ROOT"], d["TC_TIP"])
    ar_lim = pitchup_ar_limit(geo["sweep_c4"], tcfg)

    mdd_min = float(mdd.min())
    out = {
        "alpha_trim": alpha_trim, "CL": CLt, "CM": cm, "CD0": cd0, "CDi": cdi, "CDw": cdw,
        "CD": cd_total, "LD": CLt / cd_total, "oswald": CLt ** 2 / (math.pi * d["AR"] * cdi),
        "Mdd_min": mdd_min, "Mdd_mean": float(np.sum(mdd * w)), "Mcrit_min": float(mcrit.min()),
        "Mdd_goal_gap": float(tcfg["MDD_GOAL"]) - mdd_min,
        "W_wing_lb": w_lb, "W_wing_kg": w_lb * LB_TO_KG, "fuel_m3": fuel,
        "fuel_share": fuel / float(tcfg["FUEL_TOTAL_M3"]),
        "sweep_c4": geo["sweep_c4"], "sweep_c2": geo["sweep_c2"], "sweep_korn": sweep_korn, "b": geo["b"], "cr": geo["cr"],
        "ct": geo["ct"], "mac": geo["mac"], "AR_pitchup_limit": ar_lim,
        "span": {"eta": eta.tolist(), "y": y.tolist(), "chord": chord.tolist(), "dA": dA.tolist(),
                 "cl": cl.tolist(), "tc": tc.tolist(), "Mdd": mdd.tolist(), "Mcrit": mcrit.tolist(),
                 "CDw": cdw_i.tolist(), "cd": cd_i.tolist(), "Re": re_i.tolist(),
                 "stall_src": stall_src},
    }
    apply_stall(out, tcfg)
    out["constraints"] = constraint_values(out, d, tcfg)
    return out


def apply_stall(out, tcfg):
    """Tip-stall quantities from the stored low-speed strip loads and the CURRENT clmax / limits."""
    src = out["span"]["stall_src"]
    eta = np.array(src["eta"])
    alpha_s, r, eta_first = stall_onset(eta, src["cl_a"], src["cl_b"], src["alpha_a"], src["alpha_b"],
                                        tcfg["CLMAX_ROOT"], tcfg["CLMAX_TIP"])
    outboard = eta >= tcfg["STALL_ETA_OUT"]
    b_cl = (src["CL_b"] - src["CL_a"]) / (src["alpha_b"] - src["alpha_a"])
    out.update({"stall_eta_first": eta_first, "stall_out_ratio": float(r[outboard].max()) if outboard.any() else 0.0,
                "alpha_stall": alpha_s, "CL_max_est": src["CL_a"] + b_cl * (alpha_s - src["alpha_a"])})
    out["span"]["stall_eta"], out["span"]["stall_ratio"] = eta.tolist(), r.tolist()
    return out


CONSTRAINT_NAMES = ("Mdd >= MDD_MIN", "pitch-up", "tip stall", "fuel volume", "|Cm|", "t/c tip <= root")


def constraint_values(out, d, tcfg):
    """g <= 0 is feasible. Each is scaled to be O(1) so no constraint dominates NSGA-II's
    violation sum."""
    g = [
        (float(tcfg["MDD_MIN"]) - out["Mdd_min"]) / 0.05,
        (d["AR"] - out["AR_pitchup_limit"]) / 1.0 if tcfg["PITCHUP_ENABLED"] else -1.0,
        max((out["stall_eta_first"] - float(tcfg["STALL_ETA_OUT"])) / 0.1,      # first stall must be inboard
            (out["stall_out_ratio"] - float(tcfg["STALL_MARGIN"])) / 0.05),     # optional margin to outboard strips
        (float(tcfg["FUEL_SHARE_MIN"]) - out["fuel_share"]) / 0.1,
        (abs(out["CM"]) - float(tcfg["CM_MAX"])) / 0.1,
        (d["TC_TIP"] - d["TC_ROOT"]) / 0.01,
    ]
    return g
