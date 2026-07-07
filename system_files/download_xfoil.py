import urllib.request
import zipfile
import os
import io
import shutil

def download_and_setup_xfoil():
    url = "http://web.mit.edu/drela/Public/web/xfoil/XFOIL6.99.zip"
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    xfoil_dir = os.path.join(base_dir, "system_files", "xfoil")
    
    os.makedirs(xfoil_dir, exist_ok=True)
    
    print(f"[*] Downloading XFOIL 6.99 for Windows...")
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req, timeout=30) as response:
            with zipfile.ZipFile(io.BytesIO(response.read())) as zip_ref:
                zip_ref.extractall(xfoil_dir)
        print("[+] XFOIL successfully downloaded and extracted to system_files/xfoil!")
    except Exception as e:
        print(f"[-] Error during XFOIL download: {e}")
        return
        
    # Check if xfoil.exe exists in the extracted files
    exe_path = os.path.join(xfoil_dir, "xfoil.exe")
    if not os.path.exists(exe_path):
        # Check subdirectories
        for root, dirs, files in os.walk(xfoil_dir):
            for file in files:
                if file.lower() == "xfoil.exe":
                    shutil.move(os.path.join(root, file), exe_path)
                    print(f"[+] Moved xfoil.exe to primary directory.")
                    break

    if os.path.exists(exe_path):
        print("[SUCCESS] Portable XFOIL engine is fully installed and ready for use.")
    else:
        print("[-] Failed to locate xfoil.exe after extraction.")

if __name__ == "__main__":
    download_and_setup_xfoil()
