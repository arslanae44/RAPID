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
    "geometry": {
        "FUSELAGE_Y":    2.2,    # half fuselage width removed from the gross half-span [m]
        "HALF_SPAN_MAX": 25.0,   # gross half-span cap [m]
        "C_ROOT_MIN":    4.0,    # admissible root chord [m]; designs outside are rejected
        "C_ROOT_MAX":    9.5,
        "N_SECTIONS":    3,      # spanwise airfoil stations (>=3: root, kink, tip + extras)
    },
    "mesh": {
        "TESS_W":     33,        # chordwise tessellation (VLM chordwise resolution)
        "TESS_U":     24,        # total spanwise tessellation, split over panels by span
        "WAKE_ITERS": 3,         # VSPAERO wake iterations (0 = fastest)
        "TIMEOUT_S":  240.0,     # per-wing VSPAERO timeout [s]
    },
    "airfoil": {
        "MIN_TC_ROOT":  0.18,    # thickness screen for the co-opt sweep (fraction of chord)
        "MIN_TC_MID":   0.16,
        "MIN_TC_TIP":   0.12,
        "XFOIL_MACH":   0.1,     # Mach passed to XFOIL (Karman-Tsien correction)
        "ALPHA_SPREAD": 2.0,     # +/- alpha [deg] around the local design alpha
        "CM_MIN":      -0.08,    # reject sections with avg Cm below this (conventional)
        "CM_MIN_BWB":  -0.03,    # ... and in BWB mode (self-trimming sections)
    },
    # Transonic layer (transonic_opti.py / RUN_TRANSONIC.bat). Defaults = Navy jet trainer study.
    "transonic": {
        "MACH": 0.75, "RHO": 0.3803, "TEMP_K": 218.9, "A_SOUND": 296.6, "MU": 1.432e-05,
        "S_REF": 16.73,              # m^2, held fixed
        "W0_LB": 10060.0,            # design gross weight [lb]
        "W_START_FRAC": 0.955,       # weight at start of cruise / W0
        "CL_TARGET": 0.272,          # cruise lift coefficient the wing is trimmed to
        "NZ_LIMIT": 7.33, "NZ_ULT_FACTOR": 1.5,       # Raymer N_z = 1.5 * 7.33
        "CSW_FRAC": 0.20,            # control-surface area / wing area (Raymer S_csw)
        "KORN_KAPPA": 0.87,          # airfoil technology factor (0.87 = 64A series)
        "KORN_CHORD_FRAC": 0.5,      # sweep used in Korn: 0.5 = half-chord (local), 0.25 = quarter-chord
        "MDD_MIN": 0.78, "MDD_GOAL": 0.90,
        "TC_VARIABLE": True,         # t/c root and tip are design variables
        "TC_ROOT": 0.12, "TC_TIP": 0.10,              # used when TC_VARIABLE is false
        "BOUNDS": {"AR": [4.5, 6.5], "SWEEP_LE": [15.0, 40.0], "TAPER": [0.2, 0.6],
                   "WASHOUT": [0.0, 5.0], "TC_ROOT": [0.10, 0.14], "TC_TIP": [0.08, 0.12]},
        "AIRFOIL_ROOT": "system_files/presets/transonic/naca64a212.dat",
        "AIRFOIL_TIP":  "system_files/presets/transonic/naca64a210.dat",
        "POLAR_FILE":   "system_files/presets/transonic/polars.json",
        "POLAR_RE": 11.56e6,         # XFOIL Reynolds number of the section polars (transition fixed at 5 %)
        "POLAR_MACH": 0.6,           # highest Mach XFOIL handles reliably
        "RE_EXPONENT": -0.2,         # cd ~ Re^-0.2 to scale the polar to each strip's chord
        "STALL_MACH": 0.15,          # sea-level CL_max condition
        "CLMAX_ROOT": 1.79, "CLMAX_TIP": 1.5,         # 1.5 = conservative tip value (XFOIL gives 1.72)
        "STALL_ETA_OUT": 0.65,       # first stall must occur inboard of this span fraction: strips outboard of it...
        "STALL_MARGIN": 1.0,         # optional: outboard cl/clmax must stay below this when the first strip stalls (1.0 = off;
                                     # the VLM loading is flat there, so values < 0.99 are rarely attainable)
        "FUEL_TOTAL_LB": 3296.0, "FUEL_TOTAL_M3": 1.84, "FUEL_SHARE_MIN": 0.5,
        "CM_MAX": 0.35,
        # Raymer Fig. 4.21 'subsonic' pitch-up boundary: max AR vs quarter-chord sweep [deg].
        # PLACEHOLDER (figure not digitized): the Study-3 chart wing (AR 5.25, 25.4 deg) lies inside it.
        # Replace with points read off your copy of the figure.
        "PITCHUP_ENABLED": True,
        "PITCHUP_SWEEP_DEG": [0.0, 10.0, 20.0, 30.0, 40.0, 50.0],
        "PITCHUP_AR_MAX":    [9.0, 8.0, 7.0, 6.0, 5.0, 4.0],
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

    ("geometry",    "FUSELAGE_Y",    "Half fuselage width [m]",   "float", "Removed from the gross half-span"),
    ("geometry",    "HALF_SPAN_MAX", "Max gross half-span [m]",   "float", "Span cap"),
    ("geometry",    "C_ROOT_MIN",    "Min root chord [m]",        "float", "Smaller root chords are rejected"),
    ("geometry",    "C_ROOT_MAX",    "Max root chord [m]",        "float", "Larger root chords are rejected"),
    ("geometry",    "N_SECTIONS",    "Spanwise airfoil stations", "int",   ">=3 (root/kink/tip + extra stations)"),

    ("mesh",        "TESS_W",        "Chordwise tessellation",    "int",   "VLM chordwise resolution"),
    ("mesh",        "TESS_U",        "Spanwise tessellation",     "int",   "Total over the half-span"),
    ("mesh",        "WAKE_ITERS",    "VSPAERO wake iterations",   "int",   "0 = fastest"),
    ("mesh",        "TIMEOUT_S",     "Per-wing timeout [s]",      "float", "Kill VSPAERO after this many seconds"),

    ("airfoil",     "MIN_TC_ROOT",   "Min t/c at root",           "float", "Thickness screen for the co-opt sweep"),
    ("airfoil",     "MIN_TC_MID",    "Min t/c mid-span",          "float", "Thickness screen for the co-opt sweep"),
    ("airfoil",     "MIN_TC_TIP",    "Min t/c at tip",            "float", "Thickness screen for the co-opt sweep"),
    ("airfoil",     "XFOIL_MACH",    "XFOIL Mach",                "float", "Compressibility correction in XFOIL"),
    ("airfoil",     "ALPHA_SPREAD",  "Alpha spread [deg]",        "float", "+/- around the local design alpha"),
    ("airfoil",     "CM_MIN",        "Min section Cm",            "float", "Reject more nose-down sections (conventional)"),
    ("airfoil",     "CM_MIN_BWB",    "Min section Cm (BWB)",      "float", "Reject more nose-down sections (BWB)"),
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
