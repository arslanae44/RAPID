
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
