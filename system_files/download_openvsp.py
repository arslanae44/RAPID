"""
Fetch the OpenVSP 3.47.0 win64 build into system_files/OpenVSP-3.47.0-win64/.

The binaries are not stored in the repository (they are large), so this helper
downloads and extracts them. It tries the known GitHub release assets and falls
back to printing manual instructions if the download cannot be completed.
"""

import io
import os
import zipfile
import urllib.request

VERSION = "3.47.0"
TARGET_NAME = f"OpenVSP-{VERSION}-win64"

CANDIDATE_URLS = [
    f"https://github.com/OpenVSP/OpenVSP/releases/download/OpenVSP_{VERSION}/OpenVSP-{VERSION}-win64.zip",
    f"https://github.com/OpenVSP/OpenVSP/releases/download/{VERSION}/OpenVSP-{VERSION}-win64.zip",
]
MANUAL_PAGE = "https://openvsp.org/download.php"


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    target_dir = os.path.join(base_dir, "system_files", TARGET_NAME)

    if os.path.exists(os.path.join(target_dir, "vsp.exe")):
        print(f"[=] OpenVSP already present at {target_dir}")
        return

    os.makedirs(os.path.dirname(target_dir), exist_ok=True)
    tmp_extract = os.path.join(base_dir, "system_files", "_openvsp_tmp")

    data = None
    for url in CANDIDATE_URLS:
        try:
            print(f"[*] Downloading OpenVSP {VERSION} from:\n    {url}")
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req, timeout=60) as resp:
                data = resp.read()
            print("[+] Download complete.")
            break
        except Exception as e:
            print(f"    [-] Failed: {e}")

    if data is None:
        print("\n[!] Automatic download failed. Install manually:")
        print(f"    1. Download the OpenVSP {VERSION} win64 zip from {MANUAL_PAGE}")
        print(f"    2. Extract it so that vsp.exe sits at:")
        print(f"       {os.path.join(target_dir, 'vsp.exe')}")
        return

    print("[*] Extracting...")
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        zf.extractall(tmp_extract)

    # The archive may extract directly or into a nested folder; locate vsp.exe.
    src_root = None
    for root, _dirs, files in os.walk(tmp_extract):
        if "vsp.exe" in files:
            src_root = root
            break

    if src_root is None:
        print("[!] vsp.exe not found in the archive; please extract manually.")
        return

    if os.path.abspath(src_root) != os.path.abspath(target_dir):
        os.replace(src_root, target_dir) if not os.path.exists(target_dir) else None
        if not os.path.exists(os.path.join(target_dir, "vsp.exe")):
            # fallback: move file-by-file
            import shutil
            os.makedirs(target_dir, exist_ok=True)
            for item in os.listdir(src_root):
                shutil.move(os.path.join(src_root, item), os.path.join(target_dir, item))

    print(f"[SUCCESS] OpenVSP ready at {target_dir}")
    print("\n[i] The 'openvsp' Python module lives in the extracted 'python' folder.")
    print("    If 'import openvsp' fails with your Python, install the bundled wheel")
    print("    from OpenVSP-*/python/ (match your Python version) or use the Python")
    print("    interpreter shipped inside the OpenVSP package.")


if __name__ == "__main__":
    main()
