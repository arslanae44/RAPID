"""
RAPID OpenVSP/VSPAERO worker (single source of truth).

Runs in its own Python process for each candidate wing:
    python vsp_worker.py <VSP_ROOT> <run_dir> <params.json> <result.json>

params.json (written by planform_opti.evaluate_subprocess / build_worker_params):
    name, base_vsp, incidence, panels[{span, root_chord, tip_chord, sweep, twist}],
    airfoils[len(panels)+1], tess_w, tess_u[len(panels)],
    S_REF, b_ref, c_ref, x_cg, mach, Re, V_MS, RHO, alphas[], wake_iters, timeout

result.json:
    {"rows": [{"alpha", "CL", "CD", "CM", "LD"}, ...], "error": str|None}
"""

import sys, os, json, time, random, subprocess

VSP_ROOT, run_dir, params_file, result_file = sys.argv[1:5]
# The parent may run from a venv while spawning the base interpreter: inherit its site-packages.
for _p in os.environ.get("RAPID_EXTRA_PATHS", "").split(os.pathsep):
    if _p and _p not in sys.path:
        sys.path.insert(0, _p)
os.add_dll_directory(VSP_ROOT)
os.chdir(run_dir)


def _retry(fn, tries=20, base=0.05):
    """Retry a callable through transient Windows file / DLL locks."""
    last = None
    for _ in range(tries):
        try:
            return fn()
        except Exception as e:  # noqa: BLE001 - lock errors come in many types
            last = e
            time.sleep(base + random.random() * 2 * base)
    raise last


def _write_result(payload):
    def _w():
        with open(result_file, "w") as f:
            json.dump(payload, f)
    _retry(_w)


try:
    import openvsp as vsp  # noqa: E402
except Exception:
    import openvsp as vsp  # second attempt after the scanner releases the DLL


def _load_params():
    with open(params_file, "r") as f:
        return json.load(f)


p = _retry(_load_params)
name = p["name"]


def sp(wid, xsec_idx, parm, value):
    xsec = vsp.GetXSec(vsp.GetXSecSurf(wid, 0), xsec_idx)
    pid = vsp.GetXSecParm(xsec, parm)
    if pid != "":
        vsp.SetParmVal(pid, value)


def set_airfoil(wid, xsec_idx, airfoil_def):
    xsec_surf = vsp.GetXSecSurf(wid, 0)
    # A .dat path that exists -> file airfoil; anything else -> NACA 4-digit.
    if isinstance(airfoil_def, str) and airfoil_def.lower().endswith(".dat") and os.path.exists(airfoil_def):
        vsp.ChangeXSecShape(xsec_surf, xsec_idx, vsp.XS_FILE_AIRFOIL)
        vsp.Update()
        vsp.ReadFileAirfoil(vsp.GetXSec(xsec_surf, xsec_idx), airfoil_def)
    else:
        vsp.ChangeXSecShape(xsec_surf, xsec_idx, vsp.XS_FOUR_SERIES)
        vsp.Update()
        xsec = vsp.GetXSec(xsec_surf, xsec_idx)
        code = str(airfoil_def).lower().replace("naca", "")
        # Fall back to the baseline NACA if the reference is not a 4-digit code
        # (stale .dat paths must never crash a run).
        if not (code.isdigit() and len(code) >= 4):
            code = "2412"
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "Camber"),     int(code[0]) / 100.0)
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "CamberLoc"),  int(code[1]) / 10.0)
        vsp.SetParmVal(vsp.GetXSecParm(xsec, "ThickChord"), int(code[2:]) / 100.0)
    vsp.Update()


def build_wing():
    vsp.VSPCheckSetup()
    vsp.ClearVSPModel()
    base_vsp = p.get("base_vsp")
    if base_vsp and os.path.exists(base_vsp):
        _retry(lambda: vsp.ReadVSPFile(base_vsp))
    for g in vsp.FindGeomsWithName("Wing"):
        vsp.DeleteGeom(g)

    wid = vsp.AddGeom("WING")
    vsp.SetGeomName(wid, "Wing")
    vsp.SetParmVal(wid, "Sym_Planar_Flag", "Sym", 2.0)          # XZ mirror
    vsp.SetParmVal(wid, "Y_Rel_Rotation", "XForm", float(p["incidence"]))  # root incidence (pitch)
    vsp.SetParmVal(wid, "Tess_W", "Shape", float(p["tess_w"]))   # chordwise resolution

    panels = p["panels"]
    for i in range(1, len(panels)):
        vsp.InsertXSec(wid, i, vsp.XS_FOUR_SERIES)
    vsp.Update()

    for i, pan in enumerate(panels, start=1):
        sp(wid, i, "Span",           pan["span"])
        sp(wid, i, "Root_Chord",     pan["root_chord"])
        sp(wid, i, "Tip_Chord",      pan["tip_chord"])
        sp(wid, i, "Sweep",          pan["sweep"])
        sp(wid, i, "Sweep_Location", 0.0)
        sp(wid, i, "Dihedral",       0.0)
        sp(wid, i, "Twist",          pan["twist"])
        sp(wid, i, "Twist_Location", 0.25)
        sp(wid, i, "SectTess_U",     float(p["tess_u"][i - 1]))
        vsp.Update()

    for i, af in enumerate(p["airfoils"]):
        set_airfoil(wid, i, af)

    vsp.SetSetFlag(wid, vsp.SET_FIRST_USER, True)
    vsp.Update()
    return wid


def write_vspaero_inputs(vsp3_path):
    """Generate <name>.vspgeom (VLM, thin surfaces) and <name>.vspaero."""
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(vsp3_path)
    vsp.Update()

    geo = "VSPAEROComputeGeometry"
    vsp.SetAnalysisInputDefaults(geo)
    vsp.SetIntAnalysisInput(geo, "GeomSet", [vsp.SET_NONE])
    vsp.SetIntAnalysisInput(geo, "ThinGeomSet", [vsp.SET_FIRST_USER])   # VLM
    vsp.ExecAnalysis(geo)

    alphas = [float(a) for a in p["alphas"]]
    sw = "VSPAEROSweep"
    vsp.SetAnalysisInputDefaults(sw)
    vsp.SetIntAnalysisInput(sw, "GeomSet", [vsp.SET_NONE])
    vsp.SetIntAnalysisInput(sw, "ThinGeomSet", [vsp.SET_FIRST_USER])
    vsp.SetIntAnalysisInput(sw, "RefFlag", [0])                          # manual reference values
    vsp.SetDoubleAnalysisInput(sw, "Sref", [float(p["S_REF"])])
    vsp.SetDoubleAnalysisInput(sw, "bref", [float(p["b_ref"])])
    vsp.SetDoubleAnalysisInput(sw, "cref", [float(p["c_ref"])])
    vsp.SetDoubleAnalysisInput(sw, "Xcg",  [float(p["x_cg"])])
    machs = [float(m) for m in (p.get("machs") or [p["mach"]])]   # e.g. [0.15, 0.75]: one run, both flight states
    vsp.SetDoubleAnalysisInput(sw, "MachStart", [machs[0]])
    vsp.SetDoubleAnalysisInput(sw, "MachEnd", [machs[-1]])
    vsp.SetIntAnalysisInput(sw, "MachNpts", [len(machs)])
    vsp.SetDoubleAnalysisInput(sw, "AlphaStart", [alphas[0]])
    vsp.SetDoubleAnalysisInput(sw, "AlphaEnd",   [alphas[-1]])
    vsp.SetIntAnalysisInput(sw, "AlphaNpts", [len(alphas)])
    vsp.SetDoubleAnalysisInput(sw, "ReCref", [float(p["Re"])])
    vsp.SetDoubleAnalysisInput(sw, "Vinf",   [float(p["V_MS"])])
    vsp.SetDoubleAnalysisInput(sw, "Rho",    [float(p["RHO"])])
    vsp.SetIntAnalysisInput(sw, "WakeNumIter", [int(p["wake_iters"])])
    vsp.SetIntAnalysisInput(sw, "NCPU", [1])
    vsp.SetIntAnalysisInput(sw, "StopBeforeRun", [1])                    # only write <name>.vspaero
    vsp.ExecAnalysis(sw)

    if not os.path.exists(f"{name}.vspaero"):
        raise RuntimeError(f"{name}.vspaero was not written by OpenVSP")


def parse_polar(polar_file):
    with open(polar_file, "r") as f:
        lines = [ln for ln in f.readlines() if ln.strip()]
    headers = None
    rows = []
    for ln in lines:
        parts = ln.split()
        if headers is None:
            if "AoA" in parts and any(h.startswith("CL") for h in parts):
                headers = parts
            continue
        if len(parts) != len(headers):
            continue
        try:
            vals = [float(v) for v in parts]
        except ValueError:
            continue
        d = dict(zip(headers, vals))
        # Wake (Trefftz-plane) totals are preferred: more reliable induced drag for VLM.
        cl = d.get("CLwtot", d.get("CLtot", d.get("CL", 0.0)))
        cd = d.get("CDwtot", d.get("CDtot", d.get("CD", 0.0)))
        cm = d.get("CMytot", d.get("CMiy", d.get("CMy", 0.0)))
        rows.append({"alpha": d.get("AoA"), "mach": d.get("Mach"), "CL": cl, "CD": cd, "CM": cm,
                     "LD": (cl / cd) if cd else 0.0,
                     "CDi": d.get("CDiw", d.get("CDi")),     # Trefftz-plane induced drag
                     "CDo": d.get("CDo")})                   # VSPAERO's own friction estimate
    if not rows:
        raise ValueError("No numeric polar rows parsed")
    return rows


def parse_lod(path):
    """Spanwise strip loads from <name>.lod, one entry per (Mach, alpha) case.

    Right half-wing only (Yavg > 0), sorted root -> tip. cl is the strip lift
    coefficient on the local chord and freestream dynamic pressure.
    """
    cases, cur, headers = [], None, None
    with open(path, "r") as f:
        for ln in f:
            s = ln.strip()
            if s.startswith("Mach_"):
                cur = {"mach": float(s.split()[1]), "alpha": None, "rows": []}
                cases.append(cur)
            elif s.startswith("AoA_") and cur is not None:
                cur["alpha"] = float(s.split()[1])
            elif s.startswith("Iter"):
                headers = s.split()
            elif headers and cur is not None and s and (s[0].isdigit() or s[0] == "-"):
                parts = s.split()
                if len(parts) == len(headers):
                    try:
                        cur["rows"].append(dict(zip(headers, (float(v) for v in parts))))
                    except ValueError:
                        pass
    out = []
    for c in cases:
        rows = sorted((r for r in c["rows"] if r["Yavg"] > 0), key=lambda r: r["Yavg"])
        if rows:
            out.append({"mach": c["mach"], "alpha": c["alpha"],
                        "y": [r["Yavg"] for r in rows], "chord": [r["Chord"] for r in rows],
                        "dA": [r["dArea"] for r in rows], "cl": [r["Cl"] for r in rows]})
    return out


def main():
    build_wing()
    vsp3_path = os.path.abspath(f"{name}.vsp3")
    vsp.WriteVSPFile(vsp3_path)
    write_vspaero_inputs(vsp3_path)

    exe = os.path.join(VSP_ROOT, "vspaero.exe" if os.name == "nt" else "vspaero")
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    with open(f"{name}.vspaero.log", "w") as log:
        subprocess.run([exe, "-omp", "1", name], stdout=log, stderr=subprocess.STDOUT,
                       timeout=float(p.get("timeout", 240)), creationflags=flags)

    polar = f"{name}.polar"
    if not os.path.exists(polar):
        raise RuntimeError(f"VSPAERO did not write {polar} (see {name}.vspaero.log)")
    return parse_polar(polar)


if __name__ == "__main__":
    try:
        rows = main()
        extra = {}
        if p.get("strips"):
            lod = f"{name}.lod"
            if not os.path.exists(lod):
                raise RuntimeError(f"VSPAERO did not write {lod} (see {name}.vspaero.log)")
            extra["strips"] = parse_lod(lod)
        # Top-level CL/CD/LD/CM mirror the design-alpha row (read by the plotters).
        _write_result({"rows": rows, "error": None, **extra,
                       **{k: rows[0][k] for k in ("CL", "CD", "LD", "CM")}})
    except Exception as e:  # report every failure to the parent instead of dying silently
        _write_result({"rows": [], "error": f"{type(e).__name__}: {e}", "CL": None})
