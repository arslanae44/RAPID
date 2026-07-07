
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
