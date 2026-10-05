# Setup

RAPID relies on three external pieces that are **not** stored in this repository
(they are large and platform-specific): a Python 3.13/3.11 environment, the
OpenVSP/VSPAERO engine, and the XFOIL solver. This guide gets a fresh clone
running on **Windows** (the solvers are Windows binaries).

## Quick start (one click)
Run **`SETUP.bat`** from the project root. It:

1. finds 64-bit **Python 3.13** (or 3.11) through the `py` launcher, and offers to
   install Python 3.13 for your user with `winget` if neither is present;
2. creates `.venv` and installs `requirements.txt`;
3. downloads **OpenVSP 3.47.0** (the build matching your Python) and installs its
   `openvsp` Python module into `.venv`;
4. downloads **XFOIL 6.99** and the full **UIUC** airfoil set (~1650 profiles);
5. runs `system_files\check_install.py` and prints PASS/FAIL per component.

`SETUP.bat /unattended` does the same without prompts or a final pause
(useful for scripted installs).

> **Why only 3.13 / 3.11?** OpenVSP 3.47.0 win64 ships its compiled `openvsp`
> module only for those two Python versions. **Python 3.12 and 3.14 will not work.**
> Other Pythons may stay installed; SETUP picks the right one via `py -3.13`.

## Verify an install at any time
```
.\.venv\Scripts\python system_files\check_install.py --solve
```
`--solve` also evaluates one wing with VSPAERO and one airfoil with XFOIL, using
the built-in defaults (your `rapid_config.json` is not touched).

---

## Manual steps (what SETUP.bat does)

### 1. Python + dependencies
```
py -3.13 -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```

### 2. OpenVSP 3.47.0
```
.\.venv\Scripts\python system_files\download_openvsp.py
```
It downloads `OpenVSP-3.47.0-win64-Python3.13.zip` (or `-Python3.11`) from
<https://openvsp.org/download_old.php>, extracts it to
`system_files\OpenVSP-3.47.0-win64\` and pip-installs the bundled
`openvsp_config`, `degen_geom` and `openvsp` packages from its `python\` folder.

### 3. XFOIL + airfoil database
```
.\.venv\Scripts\python system_files\download_xfoil.py
.\.venv\Scripts\python system_files\download_airfoils.py
```
The curated `airfoil_database\Low_Re | Medium_Re | High_Re | Reflexed`
catalogs ship with the repo; the full UIUC set is recommended for the airfoil
co-optimization ("UIUC_All" in the catalog menu).

### 4. Run
```
CONFIGURE.bat            # constraints / bounds / flight / geometry / mesh
RUN_OPTIMIZATION.bat     # conventional wing study
RUN_BWB.bat              # blended-wing-body / tailless study
RUN_AIRFOIL.bat          # airfoil optimization for a saved .vsp3 wing
RUN_PLOTTER.bat          # Pareto + planform dashboard
```
Launchers use `system_files\python_runtime\python.exe` if present, else `.venv`,
else `python` on PATH.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `import openvsp` fails (`DLL load failed`, `No module named openvsp`) | Python version mismatch. Check `.\.venv\Scripts\python --version` is 3.13/3.11, then `.\.venv\Scripts\python -m pip install system_files\OpenVSP-3.47.0-win64\python\openvsp_config system_files\OpenVSP-3.47.0-win64\python\degen_geom system_files\OpenVSP-3.47.0-win64\python\openvsp` |
| OpenVSP download fails | Download the **3.47.0 win64 Python 3.13** zip manually from <https://openvsp.org/download_old.php> and extract it so `vsp.exe` sits in `system_files\OpenVSP-3.47.0-win64\`. Re-run SETUP.bat. |
| Every wing is `REJECTED: worker failed ...` | The message shows the worker's error. Open `wing_models\<wing>\<wing>.vspaero.log` for VSPAERO's own output. |
| The run prints "Only 0.0% of the design space passes the geometry limits" | Your bounds/`S_REF` do not fit the geometry limits. Set `FUSELAGE_Y`, `C_ROOT_MIN/MAX` in CONFIGURE.bat → [GEOMETRY] for your aircraft's size. |
| Phase 2 says "No feasible profile converged" | The catalog is too small for the Cm floor / thickness screen. Use `UIUC_All`, or relax `airfoil.CM_MIN` / `MIN_TC_*`. |

---

## Maintainer note — trimming binaries from git history
The initial commits embedded `python_runtime/` and `OpenVSP-3.47.0-win64/`.
`.gitignore` excludes them now, but they remain in the history (~188 MB). To purge
them (smaller clones), use [`git filter-repo`](https://github.com/newren/git-filter-repo)
or start a fresh history. Both rewrite commits, so coordinate before force-pushing.
