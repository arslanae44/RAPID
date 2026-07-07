import urllib.request
import zipfile
import os
import io
import shutil

def download_and_setup_db():
    url = "https://m-selig.ae.illinois.edu/ads/archives/coord_seligFmt.zip"
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    db_dir = os.path.join(base_dir, "airfoil_database")
    temp_extract = os.path.join(db_dir, "temp_extract")
    all_dir = os.path.join(db_dir, "UIUC_All")
    
    os.makedirs(db_dir, exist_ok=True)
    os.makedirs(all_dir, exist_ok=True)
    
    print(f"[*] Downloading UIUC Airfoil Coordinates Database (~1650 airfoils)...")
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req, timeout=30) as response:
            with zipfile.ZipFile(io.BytesIO(response.read())) as zip_ref:
                zip_ref.extractall(temp_extract)
        print("[+] Download and extraction successful!")
    except Exception as e:
        print(f"[-] Error during download: {e}")
        return
    
    # Move everything to UIUC_All and flatten
    extracted_subdir = os.path.join(temp_extract, "coord_seligFmt")
    if os.path.exists(extracted_subdir):
        for item in os.listdir(extracted_subdir):
            src = os.path.join(extracted_subdir, item)
            dst = os.path.join(all_dir, item)
            if os.path.isfile(src) and item.endswith(".dat"):
                shutil.copy2(src, dst)
        print(f"[+] Consolidated {len(os.listdir(all_dir))} Selig .dat files into UIUC_All.")
    
    # Clean up temp
    if os.path.exists(temp_extract):
        shutil.rmtree(temp_extract)
        
    # Create Curated Folders by Aerodynamic Regime (Reynolds numbers)
    regimes = {
        "Low_Re": [
            # Highly efficient low Reynolds number airfoils (UAVs, gliders, small craft)
            "sd7037.dat", "sd7062.dat", "ag35.dat", "ag04.dat", "dae51.dat",
            "mh32.dat", "mh45.dat", "sg6043.dat", "e387.dat", "e205.dat", "fx60126.dat"
        ],
        "Medium_Re": [
            # General aviation, regional transports (Re 500k to 3M)
            "n2412.dat", "n4412.dat", "clarky.dat", "n23012.dat", "n4415.dat", 
            "e374.dat", "fx63137.dat", "fx74cl5140.dat"
        ],
        "High_Re": [
            # Jet transports, high-speed regional, heavy transports
            "n64212.dat", "n65215.dat", "whitcomb.dat", "sc20714.dat", "sc20610.dat"
        ]
    }
    
    for regime, files in regimes.items():
        regime_path = os.path.join(db_dir, regime)
        os.makedirs(regime_path, exist_ok=True)
        
        copied_count = 0
        for filename in files:
            src = os.path.join(all_dir, filename)
            if os.path.exists(src):
                shutil.copy2(src, os.path.join(regime_path, filename))
                copied_count += 1
            else:
                # Case insensitive backup search
                for real_file in os.listdir(all_dir):
                    if real_file.lower() == filename.lower():
                        shutil.copy2(os.path.join(all_dir, real_file), os.path.join(regime_path, real_file))
                        copied_count += 1
                        break
                        
        print(f"[+] Created '{regime}' catalog with {copied_count} high-fidelity candidate shapes.")
        
    print("\n[SUCCESS] Setup complete! Local Airfoil Database is fully operational.")

if __name__ == "__main__":
    download_and_setup_db()
