"""
RAPID centralized configuration schema + loader.

Holds the built-in DEFAULTS for every user-tunable parameter (flight conditions,
design-variable bounds, optimization constraints, and BWB / tailless settings)
and merges an optional override JSON on top.

Resolution order for the active config file:
  1. Path in the RAPID_CONFIG_FILE environment variable (used by RUN_BWB.bat)
  2. system_files/rapid_config.json  (written by configure_constraints.py)
  3. built-in DEFAULTS only

Both planform_opti.py and configure_constraints.py import from this module so
there is a single source of truth and no value can silently drift.
"""

import os
import json
import copy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "rapid_config.json")

# ─── BUILT-IN DEFAULTS ────────────────────────────────────────────────────────
DEFAULTS = {
    "flight": {
        "V_KMH":   600.0,      # true airspeed [km/h]
        "RHO":     0.7364,     # air density [kg/m^3]
        "TEMP_K":  255.7,      # static temperature [K]
        "MU":      1.628e-5,   # dynamic viscosity [Pa.s]
        "S_REF":   210.0,      # reference wing area [m^2]
        "AOA":     1.0,        # analysis angle of attack [deg]
        "INCIDENCE": 3.0,      # root incidence / built-in twist reference [deg]
    },
    "bounds": {
        # (lower, upper) for each of the 6 design variables
        "AR":          [7.0, 10.5],
        "kink_frac":   [0.10, 0.75],
        "t_inner":     [0.50, 1.0],
        "t_outer":     [0.2, 0.9],
        "sweep_inner": [-2.0, 12.0],
        "sweep_outer": [-2.0, 12.0],
    },
    "constraints": {
        "LD_MIN": 15.0,   # minimum lift-to-drag ratio
        "CM_MAX": 0.35,   # max |pitching moment| (conventional mode only)
        "CL_MIN": 0.15,   # minimum lift coefficient
    },
    "bwb": {
        "BWB_MODE":        False,      # enable tailless / blended-wing-body stability handling
        "SM_MIN":          0.05,       # minimum static margin (fraction of MAC)
        "SM_MAX":          0.20,       # maximum static margin (fraction of MAC)
        "CM_TRIM_TOL":     0.02,       # allowed |Cm| at the design point (trim)
        "ALPHA_DELTA":     2.0,        # 2nd alpha offset [deg] used to measure dCm/dCL
        "CG_X":            None,       # CG x-location [m] for the moment reference; None -> 0.25*MAC
        "AIRFOIL_CATALOG": "Medium_Re",# airfoil_database sub-folder used by the co-optimization sweep
        "BASELINE_AIRFOIL": "mh60"     # reflexed section seeded on all stations before co-opt (BWB only)
    },
}

# ─── EDITOR METADATA ──────────────────────────────────────────────────────────
# (section, key, label, type, help) — drives the interactive editor ordering/prompts.
PARAM_META = [
    ("constraints", "LD_MIN",      "Minimum L/D",                 "float", "Reject wings below this lift-to-drag ratio"),
    ("constraints", "CM_MAX",      "Max |Cm| (conventional)",     "float", "Longitudinal moment cap; ignored in BWB mode"),
    ("constraints", "CL_MIN",      "Minimum CL",                  "float", "Reject wings below this lift coefficient"),

    ("flight",      "V_KMH",       "True airspeed [km/h]",        "float", "Cruise/analysis speed"),
    ("flight",      "RHO",         "Air density [kg/m^3]",        "float", "Freestream density at altitude"),
    ("flight",      "TEMP_K",      "Static temperature [K]",      "float", "Drives Mach number"),
    ("flight",      "MU",          "Dynamic viscosity [Pa.s]",    "float", "Drives Reynolds number"),
    ("flight",      "S_REF",       "Reference area [m^2]",        "float", "Wing planform reference area"),
    ("flight",      "AOA",         "Angle of attack [deg]",       "float", "Analysis alpha"),
    ("flight",      "INCIDENCE",   "Root incidence/twist [deg]",  "float", "Built-in incidence reference"),

    ("bounds",      "AR",          "Aspect ratio bounds",         "range", "Lower/upper aspect ratio"),
    ("bounds",      "kink_frac",   "Kink fraction bounds",        "range", "Spanwise kink location (0-1)"),
    ("bounds",      "t_inner",     "Inner taper bounds",          "range", "c_kink / c_root"),
    ("bounds",      "t_outer",     "Outer taper bounds",          "range", "c_tip / c_kink"),
    ("bounds",      "sweep_inner", "Inner sweep bounds [deg]",    "range", "Inboard panel LE sweep"),
    ("bounds",      "sweep_outer", "Outer sweep bounds [deg]",    "range", "Outboard panel LE sweep"),

    ("bwb",         "BWB_MODE",        "BWB / tailless mode",     "bool",  "Enable static-margin + trim stability constraints"),
    ("bwb",         "SM_MIN",          "Min static margin",       "float", "Fraction of MAC (e.g. 0.05 = 5%)"),
    ("bwb",         "SM_MAX",          "Max static margin",       "float", "Fraction of MAC"),
    ("bwb",         "CM_TRIM_TOL",     "Trim tolerance |Cm|",     "float", "Allowed pitching moment at design point"),
    ("bwb",         "ALPHA_DELTA",     "Alpha delta [deg]",       "float", "2nd alpha used to estimate dCm/dCL"),
    ("bwb",         "CG_X",            "CG x-location [m]",       "optfloat", "Moment reference; blank = 0.25*MAC"),
    ("bwb",         "AIRFOIL_CATALOG", "Airfoil catalog",         "str",   "airfoil_database sub-folder for co-opt sweep"),
    ("bwb",         "BASELINE_AIRFOIL","Baseline reflexed section","str",  "Reflexed .dat seeded before co-opt, e.g. mh60 (BWB only)"),
]


def _deep_merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def resolve_config_path():
    """Return the active config path per resolution order (may not exist)."""
    env_path = os.environ.get("RAPID_CONFIG_FILE")
    if env_path:
        return env_path
    return DEFAULT_CONFIG_PATH


def load_raw_config(path=None):
    """Load the override JSON as-is (returns {} if missing/invalid)."""
    if path is None:
        path = resolve_config_path()
    if path and os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                return data
        except Exception:
            pass
    return {}


def load_effective_config(path=None):
    """Return DEFAULTS deep-merged with the override file."""
    return _deep_merge(DEFAULTS, load_raw_config(path))


def save_config(cfg, path=None):
    if path is None:
        path = DEFAULT_CONFIG_PATH
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    return path
