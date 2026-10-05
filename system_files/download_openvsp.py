"""
Fetch OpenVSP 3.47.0 (win64) into system_files/OpenVSP-3.47.0-win64/.

The Windows build is published per Python version (3.11 or 3.13), so this
helper picks the archive matching the Python running it, downloads it from
openvsp.org, extracts the solver binaries, and tries to install the matching
openvsp Python module. If anything fails it prints manual instructions.

Only OpenVSP 3.47.0 is used on purpose: the optimizer is written against that
API and newer releases may behave differently.
"""

import io
import os
import sys
import zipfile
import subprocess
import urllib.request

VERSION = "3.47.0"
TARGET_NAME = "OpenVSP-3.47.0-win64"
SUPPORTED_PY = ("3.13", "3.11")
BASE_URL = "https://openvsp.org/download.php?file=zips/old/windows/OpenVSP-{v}-win64-Python{py}.zip"
MANUAL_PAGE = "https://openvsp.org/download_old.php"


def pick_py():
    mm = "{}.{}".format(sys.version_info.major, sys.version_info.minor)
    return mm if mm in SUPPORTED_PY else None


# Bundled packages in OpenVSP-.../python/, in dependency order. They require each
# other by name (openvsp -> degen_geom -> utilities) and none is on PyPI, so they
# must be installed in ONE pip call for pip to resolve them locally.
API_PACKAGES = ("utilities", "openvsp_config", "degen_geom", "openvsp")


def openvsp_importable(target_dir):
    """Check in a fresh interpreter (this process may have a stale import state)."""
    code = ("import os,sys; os.add_dll_directory(sys.argv[1]); "
            "import openvsp; print(openvsp.GetVSPVersion())")
    r = subprocess.run([sys.executable, "-c", code, target_dir], capture_output=True, text=True)
    return r.returncode == 0, (r.stdout or r.stderr).strip().splitlines()[-1:] or [""]


def install_python_api(target_dir):
    py_dir = os.path.join(target_dir, "python")
    pkgs = [os.path.join(py_dir, p) for p in API_PACKAGES if os.path.isdir(os.path.join(py_dir, p))]
    if not pkgs:
        print("[!] No bundled Python packages found in " + py_dir)
        return False
    print("[*] Installing the OpenVSP Python API: " + ", ".join(os.path.basename(p) for p in pkgs))
    r = subprocess.run([sys.executable, "-m", "pip", "install", "--disable-pip-version-check", *pkgs])
    if r.returncode != 0:
        return False
    copy_compiled_extensions(os.path.join(py_dir, "openvsp", "openvsp"))
    return True


def copy_compiled_extensions(src_pkg):
    """OpenVSP's MANIFEST.in lists _vsp.so / _vsp_g.so (Linux names), so on Windows
    pip installs the package WITHOUT _vsp.pyd / _vsp_g.pyd. Copy them in."""
    import shutil
    code = "import importlib.util; print(importlib.util.find_spec('openvsp').submodule_search_locations[0])"
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)   # locate without importing
    if r.returncode != 0 or not os.path.isdir(src_pkg):
        return
    dst_pkg = r.stdout.strip()
    for fn in os.listdir(src_pkg):
        if fn.lower().endswith((".pyd", ".dll", ".so")) and not os.path.exists(os.path.join(dst_pkg, fn)):
            shutil.copy2(os.path.join(src_pkg, fn), os.path.join(dst_pkg, fn))
            print("    [+] added missing compiled module " + fn)


def ensure_python_api(target_dir, py):
    ok, msg = openvsp_importable(target_dir)
    if not ok:
        install_python_api(target_dir)
        ok, msg = openvsp_importable(target_dir)
    if ok:
        print("[SUCCESS] 'import openvsp' works: " + msg[0])
        return True
    print("\n[!] OpenVSP binaries are installed, but 'import openvsp' still fails: " + msg[0])
    print("    Install the bundled packages for your Python {} in one command:".format(py))
    print('        "{}" -m pip install {}'.format(
        sys.executable, " ".join('"{}"'.format(os.path.join(target_dir, "python", p)) for p in API_PACKAGES)))
    return False


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_dir = os.path.join(base_dir, "system_files", TARGET_NAME)

    if os.path.exists(os.path.join(target_dir, "vsp.exe")):
        print("[=] OpenVSP already present at " + target_dir)
        # A previous run may have failed to install the Python module: (re)check it.
        return 0 if ensure_python_api(target_dir, pick_py() or "") else 1

    mm = "{}.{}".format(sys.version_info.major, sys.version_info.minor)
    py = pick_py()
    if py is None:
        print("[!] Your Python is {}. OpenVSP {} win64 is only built for {}.".format(
            mm, VERSION, " and ".join(SUPPORTED_PY)))
        print("    Install Python 3.13 (or 3.11) and re-run, or download manually from")
        print("    " + MANUAL_PAGE)
        return 1

    url = BASE_URL.format(v=VERSION, py=py)
    print("[*] Downloading OpenVSP {} (Python {}) from:\n    {}".format(VERSION, py, url))
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=120) as resp:
            data = resp.read()
        print("[+] Downloaded {} MB.".format(len(data) // (1024 * 1024)))
    except Exception as e:
        print("[-] Download failed: {}".format(e))
        print("    Get it manually from {} (pick the Python {} win64 build)".format(MANUAL_PAGE, py))
        print("    and extract so vsp.exe is at " + os.path.join(target_dir, "vsp.exe"))
        return 1

    tmp = os.path.join(base_dir, "system_files", "_openvsp_tmp")
    print("[*] Extracting...")
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        zf.extractall(tmp)

    src_root = None
    for root, _dirs, files in os.walk(tmp):
        if "vsp.exe" in files:
            src_root = root
            break
    if src_root is None:
        print("[!] vsp.exe not found in the archive; please extract manually.")
        return 1

    import shutil
    os.makedirs(target_dir, exist_ok=True)
    for item in os.listdir(src_root):
        dst = os.path.join(target_dir, item)
        if not os.path.exists(dst):
            shutil.move(os.path.join(src_root, item), dst)
    shutil.rmtree(tmp, ignore_errors=True)
    print("[+] OpenVSP ready at " + target_dir)
    return 0 if ensure_python_api(target_dir, py) else 1


if __name__ == "__main__":
    sys.exit(main())
