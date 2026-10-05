---
name: Bug report
about: Something in RAPID crashed or produced wrong results
labels: bug
---

**What happened**
<!-- What you ran (RUN_OPTIMIZATION.bat / RUN_BWB.bat / RUN_AIRFOIL.bat / ...) and what went wrong. -->

**Console output**
```
<!-- paste the last ~50 lines, including any traceback -->
```

**Environment**
- Windows version:
- Python version (`.\.venv\Scripts\python --version`):
- `import openvsp` works? (`.\.venv\Scripts\python -c "import openvsp; print(openvsp.GetVSPVersion())"`):
- Config: conventional / BWB, any custom `rapid_config.json` values:

**For a failed wing**
<!-- Attach params.json, result.json and <name>.vspaero.log from the wing_models/<wing>/ folder. -->
