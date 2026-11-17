"""
Launch vsp_worker.py (one OpenVSP/VSPAERO evaluation) in its own process.

Shared by planform_opti.py and transonic_opti.py. The optional `lock` serializes
process start-up so concurrent workers do not fight over Windows DLL scans.
"""

import os
import random
import subprocess
import sys
import sysconfig
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VSP_ROOT = os.path.join(SCRIPT_DIR, "OpenVSP-3.47.0-win64")
WORKER_SCRIPT = os.path.join(SCRIPT_DIR, "vsp_worker.py")

# Files worth keeping from a finished run; everything else VSPAERO wrote is bulky and regenerable.
KEEP_SUFFIXES = (".vsp3", ".polar", ".lod", ".vspaero", ".log", ".json")


def run_worker(run_dir, params_file, result_file, timeout, lock=None):
    """Run the worker; returns (ok, diagnostic)."""
    python_bin = getattr(sys, "_base_executable", sys.executable)
    env = os.environ.copy()
    env.update({"OMP_NUM_THREADS": "1", "PYTHONDONTWRITEBYTECODE": "1",
                "RAPID_EXTRA_PATHS": os.pathsep.join([sysconfig.get_path("purelib"), SCRIPT_DIR])})
    cmd = [python_bin, WORKER_SCRIPT, VSP_ROOT, run_dir, params_file, result_file]
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    diag = ""
    for _ in range(3):
        if lock is not None:
            lock.acquire()
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, env=env, creationflags=flags)
            time.sleep(0.25)   # serialize DLL loading under Windows scanners
        finally:
            if lock is not None:
                lock.release()
        try:
            out, err = proc.communicate(timeout=timeout + 60)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.communicate()
            return False, "worker timed out"
        if os.path.exists(result_file):
            return True, ""
        diag = (err or out or "").strip()[-600:]
        lowered = diag.lower()
        if "being used by another process" in lowered or "access is denied" in lowered or not diag:
            time.sleep(0.5 + random.random() * 1.5)   # transient lock: retry
            continue
        break
    return False, diag or "worker exited without a result"


def slim_run_dir(run_dir):
    """Delete the bulky solver by-products of a finished run."""
    for fn in os.listdir(run_dir):
        if not fn.lower().endswith(KEEP_SUFFIXES):
            try:
                os.remove(os.path.join(run_dir, fn))
            except OSError:
                pass
