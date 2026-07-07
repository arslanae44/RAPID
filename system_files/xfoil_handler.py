import os
import subprocess
import sys
import time

def analyze_airfoil(airfoil_path, reynolds, mach=0.0, alphas=[0.0], max_iter=100, work_dir=None, geom_factors=None, save_coords_path=None):
    """
    Runs viscous aerodynamic analysis on a given airfoil using the embedded XFOIL 6.99 engine.
    Optionally tweaks thickness/camber geometries and exports scaled profile coordinates.
    """
    import shutil
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xfoil_exe = os.path.join(script_dir, "xfoil", "xfoil.exe")
    
    if not os.path.exists(xfoil_exe):
        raise FileNotFoundError(f"XFOIL binary not found at: {xfoil_exe}")
        
    if not os.path.exists(airfoil_path):
        raise FileNotFoundError(f"Airfoil database file not found at: {airfoil_path}")
        
    # Generate an isolated workspace folder inside system temp or scripts dir to prevent space issues
    import uuid
    run_id = f"{os.getpid()}_{uuid.uuid4().hex[:8]}"
    
    # Define a safe root for runs
    runs_root = os.path.join(script_dir, "xfoil_runs")
    workspace = os.path.join(runs_root, f"run_{run_id}")
    os.makedirs(workspace, exist_ok=True)
    
    # Standardized local names inside the workspace
    local_coord_name = "input_af.dat"
    local_polar_name = "output_polar.txt"
    local_cmd_name   = "commands.txt"
    local_log_name   = "console.log"
    local_saved_af   = "modified_coords.dat"
    
    # Copy coordinate file into workspace to bypass absolute path constraints entirely
    local_coord_path = os.path.join(workspace, local_coord_name)
    shutil.copy(airfoil_path, local_coord_path)
    
    # ─── SANITIZE COORDINATES (Remove duplicates/dead-starts) ───────────────
    try:
        with open(local_coord_path, "r") as f:
            lines = f.readlines()
        
        header = lines[0]
        coords = []
        for line in lines[1:]:
            parts = line.strip().split()
            if len(parts) == 2:
                try:
                    coords.append([float(parts[0]), float(parts[1])])
                except: pass
        
        if coords:
            clean_coords = [coords[0]]
            for i in range(1, len(coords)):
                # Remove points that are too close to the previous one
                dist = ((coords[i][0]-coords[i-1][0])**2 + (coords[i][1]-coords[i-1][1])**2)**0.5
                if dist > 1e-7:
                    clean_coords.append(coords[i])
            
            with open(local_coord_path, "w") as f:
                f.write(header.strip() + "\n")
                for c in clean_coords:
                    f.write(f" {c[0]:.7f} {c[1]:.7f}\n")
    except Exception as e:
        print(f"[i] Coordinate sanitization skipped: {e}")

    # ─── WRITE XFOIL AUTOMATION COMMAND SCRIPT ───────────────────────────────
    commands = []
    
    # Disable XFOIL Graphics Plotting completely to prevent window popups & GDI deadlocks
    commands.append("PLOP")
    commands.append("G F") # Set Graphics flag to False explicitly
    commands.append("")    # Exit PLOP menu
    
    commands.append(f"LOAD {local_coord_name}")
    
    # Execute geometric refinement tweaks via GDES if scaling factors are requested
    if geom_factors is not None:
        t_fac, c_fac = geom_factors
        commands.append("GDES")
        commands.append(f"TFAC {t_fac} {c_fac}") # Multiplies thickness by t_fac, camber by c_fac
        commands.append("EXEC")
        commands.append("") # Return back to top level menu
        
    # Export scaled coordinates out if dynamic persistence is requested
    if save_coords_path is not None:
        commands.append(f"SAVE {local_saved_af}")
        commands.append("") # Enter for overwrite confirmation override
    
    # Pane buffer to ensure consistent and high-density panel counts
    commands.append("PCOP") # Ensure coordinates are set
    commands.append("PANE") 
    
    # Enter Operating menu
    commands.append("OPER")
    
    # Set Reynolds and switch to viscous solver
    commands.append("Visc")
    commands.append(f"{reynolds}")
    
    if mach > 0.01:
        commands.append(f"Mach {mach}")
        
    # Set convergence overrides
    # High Reynolds number stability tweaks
    if reynolds > 10000000:
        commands.append("VPAR")
        commands.append("N 11.0") # Increase Ncrit for more stable high-Re turbulent transition
        commands.append("XTR 0.05 0.05") # Force transition early (more realistic for transport scale)
        commands.append("")
        
    # Set convergence overrides
    commands.append(f"Iter {max_iter}")
    
    # Setup polar accumulation
    commands.append("PACC")
    commands.append(f"{local_polar_name}")
    commands.append("") # Empty string triggers the 'Enter' key (no dump file)
    
    # Start with a 0-deg 'warmup' point to help boundary layer settle
    commands.append("ALFA 0.0")
    
    # Run dynamic angles sequence
    for alpha in alphas:
        if abs(alpha) > 0.1: # Skip if already did 0.0
            commands.append(f"ALFA {alpha}")
        
    # Stop polar logging, return to top level, and exit cleanly
    commands.append("PACC") 
    commands.append("")
    commands.append("QUIT")
    
    cmd_path = os.path.join(workspace, local_cmd_name)
    with open(cmd_path, "w") as f:
        f.write("\n".join(commands) + "\n")
        
    # ─── EXECUTE HEADLESS SOLVER ─────────────────────────────────────────────
    stdout, stderr = "", ""
    try:
        si = None
        creationflags = 0
        if os.name == "nt":
            si = subprocess.STARTUPINFO()
            si.dwFlags |= subprocess.STARTF_USESHOWWINDOW  # keep XFOIL fully headless
            creationflags = subprocess.CREATE_NO_WINDOW
        proc = subprocess.Popen(
            [xfoil_exe],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            cwd=workspace,
            creationflags=creationflags,
            startupinfo=si,
        )
        
        with open(cmd_path, "r") as f:
            script_input = f.read()
            
        stdout, stderr = proc.communicate(input=script_input, timeout=35)
        
        with open(os.path.join(workspace, local_log_name), "w") as lf:
            lf.write("--- STDOUT ---\n" + (stdout or "") + "\n--- STDERR ---\n" + (stderr or ""))
            
    except subprocess.TimeoutExpired:
        proc.kill()
        print(f"[!] XFOIL simulation timed out in isolated workspace: {workspace}")
    except Exception as e:
        print(f"[!] Subprocess deployment failed: {e}")
        
    # Track local files for parsing logic
    polar_file   = os.path.join(workspace, local_polar_name)
    debug_log    = os.path.join(workspace, local_log_name)
    airfoil_name = os.path.basename(airfoil_path)
    work_dir     = workspace
    
    # If optimized coords were saved inside workspace, export them out BEFORE workspace deletion!
    if save_coords_path is not None:
        loc_saved_path = os.path.join(workspace, local_saved_af)
        if os.path.exists(loc_saved_path):
            try:
                os.makedirs(os.path.dirname(os.path.abspath(save_coords_path)), exist_ok=True)
                shutil.copyfile(loc_saved_path, save_coords_path)
            except Exception as e:
                print(f"[!] Failed to extract scaled coordinates to destination: {e}")

            
    # ─── PARSE POLAR FILE ────────────────────────────────────────────────────
    # ─── PARSE POLAR FILE ────────────────────────────────────────────────────
    results = {}
    if os.path.exists(polar_file):
        try:
            with open(polar_file, "r") as f:
                lines = f.readlines()
                
            data_mode = False
            for line in lines:
                if "---" in line:
                    data_mode = True
                    continue
                if data_mode:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        try:
                            alpha_val = float(parts[0])
                            cl = float(parts[1])
                            cd = float(parts[2])
                            cm = float(parts[4])
                            
                            # Filter physically impossible CD values (XFOIL artifacts/Inviscid leaks)
                            # At Re=50M, Cd should be at least ~0.0030 for a turbulent profile.
                            # If XFOIL returns something like 0.0005, it's a solver failure.
                            if cd < 0.0015:
                                continue
                            
                            ld = cl / cd
                            
                            # Drop physically impossible efficiencies rather than
                            # capping them: a 2D section pinned at an absurd L/D means
                            # the viscous solve diverged (blew up), so discard the point.
                            if ld > 250.0:
                                continue

                            results[alpha_val] = {
                                "CL": round(cl, 4),
                                "CD": round(cd, 5),
                                "CM": round(cm, 5),
                                "LD": round(ld, 2)
                            }
                        except ValueError:
                            pass
        except Exception as e:
             print(f"[!] Parsing failed for polar file: {e}")
    else:
        # Print raw log to understand failure
        try:
            if os.path.exists(debug_log):
                with open(debug_log, "r") as lf:
                    log_content = lf.read()
                print("\n[!] XFOIL failed to generate polar. Console Dump:")
                print("-" * 60)
                print(log_content)
                print("-" * 60)
        except Exception as e:
            print(f"Error displaying debug log: {e}")
            
    # ─── FINAL WORKSPACE CLEANUP ─────────────────────────────────────────────
    # Erase the entire temporary run folder to keep disk space lean
    try:
        import shutil
        shutil.rmtree(workspace, ignore_errors=True)
    except:
        pass
            
    return results

# ─── RUN BUILT-IN DIAGNOSTIC TEST ─────────────────────────────────────────────
if __name__ == "__main__":
    print("[*] Running self-diagnostic on xfoil_handler.py...")
    
    # Locating NACA 2412 file from curated Medium_Re
    script_dir = os.path.dirname(os.path.abspath(__file__))
    proj_root = os.path.dirname(script_dir)
    test_airfoil = os.path.join(proj_root, "airfoil_database", "Medium_Re", "naca2412.dat")
    
    if not os.path.exists(test_airfoil):
        # Fallback to UIUC_All search
        test_airfoil = os.path.join(proj_root, "airfoil_database", "UIUC_All", "naca2412.dat")
        
    if not os.path.exists(test_airfoil):
        print(f"[!] Test airfoil missing. Make sure database is set up.")
        sys.exit(1)
        
    print(f"[i] Using test shape: {os.path.basename(test_airfoil)}")
    
    # Test condition: Typical medium Reynolds number, swept at 3 key angles
    test_Re = 500000
    test_alphas = [1.0, 3.0, 5.0]
    
    print(f"[*] Running analysis at Re = {test_Re:,} for alpha in {test_alphas}...")
    t0 = time.time()
    res = analyze_airfoil(test_airfoil, test_Re, mach=0.1, alphas=test_alphas, work_dir=script_dir)
    t1 = time.time()
    
    if res:
        print(f"[SUCCESS] XFOIL returned convergence in {t1-t0:.2f} seconds!")
        print("\n" + "=" * 60)
        print(f" {os.path.basename(test_airfoil).upper()} POLAR REPORT — Re={test_Re:,}")
        print("=" * 60)
        print(f"{'Alpha':^10} | {'CL':^10} | {'CD':^10} | {'CM':^10} | {'L/D':^10}")
        print("-" * 60)
        for a in sorted(res.keys()):
            d = res[a]
            print(f"{a:^10.1f} | {d['CL']:^10.4f} | {d['CD']:^10.5f} | {d['CM']:^10.5f} | {d['LD']:^10.2f}")
        print("=" * 60)
    else:
        print("[!] Analysis returned zero valid converging points. XFOIL failed solver.")
