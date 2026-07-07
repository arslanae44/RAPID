# Setup

RAPID relies on three external binaries that are **not** stored in this repository
(they are large and platform-specific): the Python runtime, the OpenVSP/VSPAERO
engine, and the XFOIL solver. This guide gets a fresh clone running on Windows.

## Quick start (one click)
Run **`SETUP.bat`** from the project root. It installs the Python dependencies
and downloads OpenVSP, XFOIL, and (optionally) the full airfoil database in one
go. The manual steps below are the same thing, broken out.

---

## 1. Python + dependencies
Install **Python 3.13 or 3.11** (64-bit) - OpenVSP 3.47.0 win64 is only built for
these two versions, and the `openvsp` module must match your interpreter. Then:

```
pip install -r requirements.txt
```

The launchers (`RUN_*.bat`, `CONFIGURE.bat`) automatically use the bundled
`system_files\python_runtime\python.exe` if it exists; otherwise they fall back
to the `python` on your PATH.

## 2. OpenVSP 3.47.0
Run the helper (also called by `SETUP.bat`):

```
python system_files\download_openvsp.py
```

It fetches and extracts the build to `system_files\OpenVSP-3.47.0-win64\`. If the
automatic download fails, grab the **OpenVSP 3.47.0 win64** zip from
<https://openvsp.org/download.php> and extract it there manually.

This package provides the `vsp.exe` / `vspaero.exe` solvers **and** the `openvsp`
Python module the optimizer imports. That module is a compiled extension, so it
must match your Python version — if `import openvsp` fails, install the wheel
from `OpenVSP-3.47.0-win64\python\` for your Python, or run RAPID with the Python
interpreter shipped inside the OpenVSP package.

## 3. XFOIL + airfoil database
From the project root:

```
python system_files\download_xfoil.py       # fetches XFOIL 6.99 into system_files\xfoil\
python system_files\download_airfoils.py     # fetches the full UIUC set (~1650 airfoils)
```

The curated `airfoil_database\Low_Re | Medium_Re | High_Re | Reflexed` catalogs
ship with the repo, but the full UIUC set is recommended: the airfoil
co-optimization can sweep the entire database for the best section per station.

## 4. Run
```
CONFIGURE.bat            # set constraints / bounds / flight conditions
RUN_OPTIMIZATION.bat     # conventional wing study
RUN_BWB.bat              # blended-wing-body / tailless study
```

---

## Maintainer note — trimming binaries from git history
The initial commits embedded `python_runtime/` and `OpenVSP-3.47.0-win64/`.
`.gitignore` now excludes them, but they remain in the existing history. To
untrack them for future commits (files stay on disk):

```
del .git\index.lock                                   & rem if a stale lock exists
git rm -r --cached system_files\python_runtime system_files\OpenVSP-3.47.0-win64 system_files\test_debug.py
git rm --cached system_files\altc130mod.adb.cases system_files\altc130mod.case.*.dat system_files\altc130mod.csf system_files\altc130mod.cuts system_files\altc130mod.group.* system_files\altc130mod.lod system_files\altc130mod.polar system_files\altc130mod.quad.cases system_files\altc130mod.slc system_files\altc130mod.vkey system_files\altc130mod.vspgeom
git commit -m "Exclude portable binaries and run artifacts from tracking"
```

To also purge them from past history (smaller clones), use
[`git filter-repo`](https://github.com/newren/git-filter-repo) or start a fresh
history — both rewrite commits, so coordinate before pushing.
