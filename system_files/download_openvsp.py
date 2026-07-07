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


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_dir = os.path.join(base_dir, "system_files", TARGET_NAME)

    if os.path.exists(os.path.join(target_dir, "vsp.exe")):
        print("[=] OpenVSP already present at " + target_dir)
        return

    mm = "{}.{}".format(sys.version_info.major, sys.version_info.minor)
    py = pick_py()
    if py is None:
        print("[!] Your Python is {}. OpenVSP {} win64 is only built for {}.".format(
            mm, VERSION, " and ".join(SUPPORTED_PY)))
        print("    Install Python 3.13 (or 3.11) and re-run, or download manually from")
        print("    " + MANUAL_PAGE)
        return

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
        return

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
        return

    import shutil
    os.makedirs(target_dir, exist_ok=True)
    for item in os.listdir(src_root):
        dst = os.path.join(target_dir, item)
        if not os.path.exists(dst):
            shutil.move(os.path.join(src_root, item), dst)
    shutil.rmtree(tmp, ignore_errors=True)
    print("[+] OpenVSP ready at " + target_dir)

    # Best-effort install of the bundled Python API (matches this Python build).
    py_dir = os.path.join(target_dir, "python")
    if os.path.isdir(py_dir):
        for pkg in ("openvsp_config", "degen_geom", "openvsp"):
            pkg_path = os.path.join(py_dir, pkg)
            if os.path.isdir(pkg_path):
                try:
                    subprocess.run([sys.executable, "-m", "pip", "install", pkg_path], check=True)
                except Exception as e:
                    print("    [i] pip install {} skipped: {}".format(pkg, e))

    try:
        import openvsp  # noqa: F401
        print("[SUCCESS] 'import openvsp' works.")
    except Exception:
        print("\n[i] OpenVSP binaries installed, but the Python module is not importable yet.")
        print("    Install it from the bundled folder for your Python {}:".format(py))
        print('        pip install "{}"'.format(os.path.join(py_dir, "openvsp")))
        print("    (install openvsp_config and degen_geom from the same folder first if needed).")


if __name__ == "__main__":
    main()
