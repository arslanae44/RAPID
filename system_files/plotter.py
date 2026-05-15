import os, json, glob
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

# ─── PATHS (PORTABLE & DYNAMIC) ───────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORK_DIR   = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR = os.path.join(WORK_DIR, "wing_models")
JSON_PATH  = os.path.join(OUTPUT_DIR, "pareto_results.json")

# Create PLOTS directory safely
PLOTS_DIR  = os.path.join(WORK_DIR, "PLOTS")
os.makedirs(PLOTS_DIR, exist_ok=True)

def get_incremental_save_path():
    """Returns sequential filenames for incremental plot persistence."""
    counter = 1
    while True:
        path = os.path.join(PLOTS_DIR, f"plot_{counter}.png")
        if not os.path.exists(path):
            return path
        counter += 1

# ─── GEOMETRY PARAMETERS ──────────────────────────────────────────────────────
S_REF         = 210.0
FUSELAGE_Y    = 2.2
HALF_SPAN_MAX = 25.0

# ─── DATA LOADING UTILITIES ───────────────────────────────────────────────────
def load_pareto():
    if not os.path.exists(JSON_PATH):
        print(f"[ERROR] File not found: {JSON_PATH}")
        return []
    with open(JSON_PATH, "r") as f:
        data = json.load(f)
    return [d for d in data if 0 < d["LD"] < 60 and d["CL"] > 0]

def load_all_history():
    """Extracts all historical design points from output directories."""
    all_points = []
    result_files = glob.glob(os.path.join(OUTPUT_DIR, "*", "result.json"))
    
    for path in result_files:
        try:
            with open(path, "r") as f:
                res = json.load(f)
            
            if res.get("error") is None and res.get("CL") is not None:
                cl = res["CL"]
                ld = res["LD"]
                cm = abs(res.get("CM", 0.0))
                
                # Eliminate outlier points
                if 0.0 < cl < 3.0 and -5.0 < ld < 60.0:
                    is_feas = (0.15 <= cl) and (ld >= 15.0) and (cm <= 0.35)
                    all_points.append({"CL": cl, "LD": ld, "feasible": is_feas})
        except Exception:
            pass
            
    return all_points

# ─── PLANFORM POLYGON ─────────────────────────────────────────────────────────
def calc_wing_polygon(d):
    AR        = d["AR"]
    kink_frac = d["kink_frac"]
    t_inner   = d["t_inner"]
    t_outer   = d["t_outer"]
    sw_in     = np.radians(d["sweep_inner"])
    sw_out    = np.radians(d["sweep_outer"])

    b_half    = min(np.sqrt(AR * S_REF) / 2.0, HALF_SPAN_MAX)
    b_exposed = b_half - FUSELAGE_Y
    b_kink    = kink_frac * b_exposed
    b_tip     = b_exposed - b_kink

    factor = 0.5*(1+t_inner)*b_kink + 0.5*t_inner*(1+t_outer)*b_tip
    c_root = (S_REF / 2.0) / factor
    c_kink = t_inner * c_root
    c_tip  = t_outer * c_kink

    y_root = FUSELAGE_Y
    y_kink = FUSELAGE_Y + b_kink
    y_tip  = b_half

    x_le_root = 0.0
    x_le_kink = x_le_root + b_kink * np.tan(sw_in)
    x_le_tip  = x_le_kink + b_tip  * np.tan(sw_out)

    rx = [y_root, y_kink, y_tip,            y_tip,              y_kink,              y_root            ]
    ry = [-x_le_root, -x_le_kink, -x_le_tip, -(x_le_tip+c_tip), -(x_le_kink+c_kink), -(x_le_root+c_root)]

    lx = [-v for v in rx[::-1]]
    ly = ry[::-1]

    return lx + rx, ly + ry

# ─── DASHBOARD PLOTTER ────────────────────────────────────────────────────────
def plot_dashboard(pareto, all_pop):
    N = len(pareto)
    if N == 0:
        print("[WARNING] No Pareto solutions found to display.")
        return

    colors = plt.cm.turbo(np.linspace(0.1, 0.9, N))
    
    # Interactive tracking collections
    scatter_elements = []
    wing_axes = []
    info_text_objects = []

    n_cols = 8
    n_rows = int(np.ceil(N / n_cols))

    fig_height = 7.0 + (3.2 * n_rows)
    fig = plt.figure(figsize=(24, fig_height), facecolor="white")
    
    height_ratios = [2.5] + [1.0] * n_rows
    gs = gridspec.GridSpec(1 + n_rows, n_cols, height_ratios=height_ratios, hspace=0.4, wspace=0.25)

    ax_pareto = fig.add_subplot(gs[0, :])

    # Plot broad candidate cloud
    if all_pop:
        feas_cl = [p["CL"] for p in all_pop if p.get("feasible", True)]
        feas_ld = [p["LD"] for p in all_pop if p.get("feasible", True)]
        
        infeas_cl = [p["CL"] for p in all_pop if not p.get("feasible", True)]
        infeas_ld = [p["LD"] for p in all_pop if not p.get("feasible", True)]
        
        if infeas_cl:
            ax_pareto.scatter(
                infeas_cl, infeas_ld,
                color="#ffa8a8", s=25, zorder=1, edgecolors="none", alpha=0.35,
                label=f"Violated Constraints ({len(infeas_cl)})"
            )
        
        if feas_cl:
            ax_pareto.scatter(
                feas_cl, feas_ld,
                color="#a3e2a3", s=30, zorder=1, edgecolors="none", alpha=0.50,
                label=f"Feasible History ({len(feas_cl)})"
            )

    # 2. Pareto cephesi bağlantı çizgisi
    sorted_p = sorted(pareto, key=lambda x: x["CL"])
    ax_pareto.plot(
        [d["CL"] for d in sorted_p],
        [d["LD"] for d in sorted_p],
        color="black", linestyle="--", linewidth=1.5, alpha=0.5, zorder=2
    )

    # 3. Pareto şampiyonlarını renkli ve büyük çiz
    for i, (d, color) in enumerate(zip(pareto, colors)):
        sc = ax_pareto.scatter(
            d["CL"], d["LD"],
            color=color, s=90, zorder=3, edgecolors="black", linewidth=1.0,
            label=f"Optimum-{d['solution_id']} (AR={d['AR']:.1f}, sw_in={d['sweep_inner']:.0f}°)"
        )
        scatter_elements.append(sc)
        ax_pareto.annotate(
            f"#{d['solution_id']}",
            (d["CL"], d["LD"]),
            textcoords="offset points",
            xytext=(4, 4),
            color="black", fontsize=8, fontweight="normal", va="bottom", ha="left"
        )

    ax_pareto.axhline(15.0, color="red", linestyle=":", linewidth=1.5, alpha=0.8, label="Min L/D = 15 Boundary")
    ax_pareto.set_facecolor("#fcfcfc")
    ax_pareto.set_title("Pareto Front & Optimization History",
                         fontsize=15, fontweight="bold", color="black", pad=15)
    ax_pareto.set_xlabel("CL", fontsize=13, color="black", fontweight="bold")
    ax_pareto.set_ylabel("L/D", fontsize=13, color="black", fontweight="bold")
    ax_pareto.tick_params(colors="black", labelsize=11)
    
    for spine in ax_pareto.spines.values():
        spine.set_edgecolor("black")
        spine.set_linewidth(1.2)
        
    # Tight boundary zoom onto dynamic Pareto sets
    if pareto:
        p_cl = [d["CL"] for d in pareto]
        p_ld = [d["LD"] for d in pareto]
        
        min_cl, max_cl = min(p_cl), max(p_cl)
        min_ld, max_ld = min(p_ld), max(p_ld)
        
        span_cl = max_cl - min_cl if len(pareto) > 1 else 0.1
        span_ld = max_ld - min_ld if len(pareto) > 1 else 5.0
        
        pad_cl = max(0.015, span_cl * 0.15)
        pad_ld = max(0.8, span_ld * 0.15)
        
        ax_pareto.set_xlim(left=min_cl - pad_cl, right=max_cl + pad_cl)
        ax_pareto.set_ylim(bottom=min_ld - pad_ld, top=max_ld + pad_ld)

    ax_pareto.grid(True, linestyle="--", alpha=0.6, color="#cccccc")

    # ── Planform Subpanels ──
    for i, (d, color) in enumerate(zip(pareto, colors)):
        r = 1 + (i // n_cols)
        c = i % n_cols
        ax = fig.add_subplot(gs[r, c])
        wing_axes.append(ax)
        ax.set_facecolor("white")

        x, y = calc_wing_polygon(d)
        ax.fill(x, y, color=color, alpha=0.8, edgecolor="black", linewidth=1.5)

        # Fuselage centerline block
        ax.fill(
            [-FUSELAGE_Y, FUSELAGE_Y, FUSELAGE_Y, -FUSELAGE_Y],
            [0, 0, -8, -8],
            color="#e0e0e0", alpha=0.8, edgecolor="#888888", linewidth=1.2
        )

        b_half = min(np.sqrt(d["AR"] * S_REF) / 2.0, HALF_SPAN_MAX)
        ax.set_xlim(-b_half - 2, b_half + 2)
        ax.set_ylim(-14.5, 2)
        ax.set_aspect("equal")
        ax.grid(True, linestyle=":", alpha=0.5, color="gray")
        ax.tick_params(colors="black", labelsize=8)
        
        for spine in ax.spines.values():
            spine.set_edgecolor("gray")

        # Structural/Aerodynamic text captures
        factor = 0.5*(1+d["t_inner"])*(d["kink_frac"]*(b_half-FUSELAGE_Y)) + \
                 0.5*d["t_inner"]*(1+d["t_outer"])*((1-d["kink_frac"])*(b_half-FUSELAGE_Y))
        c_root = (S_REF / 2.0) / factor
        c_kink = d["t_inner"] * c_root
        c_tip  = d["t_outer"] * c_kink
        
        b_ref      = 2.0 * b_half
        ar_val     = int(round(d["AR"] * 10))
        span_val   = int(round(b_ref))
        kink_val   = int(round(d["kink_frac"] * 10))
        sw_in_val  = int(round(d["sweep_inner"]))
        sw_out_val = int(round(d["sweep_outer"]))
        t_in_val   = int(round(d["t_inner"] * 10))
        t_out_val  = int(round(d["t_outer"] * 10))
        
        pretty_name = f"W{ar_val}/{span_val}/{kink_val}/{sw_in_val}-{sw_out_val}/{t_in_val}-{t_out_val}"

        info = (f"{pretty_name}\n"
                f"AR={d['AR']:.2f}  b={b_ref:.1f}m\n"
                f"c_root={c_root:.2f}m  c_kink={c_kink:.2f}m  c_tip={c_tip:.2f}m\n"
                f"Λ_in={d['sweep_inner']:.0f}°  Λ_out={d['sweep_outer']:.0f}°")
        t_box = ax.text(0.5, -0.22, info, transform=ax.transAxes,
                fontsize=8, color="black", va="top", ha="center",
                bbox=dict(facecolor="#fdfdfd", alpha=0.95, edgecolor="lightgray", boxstyle="round,pad=0.4"))
        info_text_objects.append((t_box, pretty_name))

        ax.set_title(
            f"#{d['solution_id']}  CL={d['CL']:.4f}  L/D={d['LD']:.2f}",
            fontsize=10, fontweight="bold", color="black", pad=4
        )

    # ─── INTERACTIVE HOVER DYNAMICS ───────────────────────────────────────────
    def on_motion(event):
        changed = False
        
        # 1. Revert states
        for idx, sc in enumerate(scatter_elements):
            if sc.get_sizes()[0] != 90:
                sc.set_sizes([90])
                sc.set_linewidths([1.0])
                sc.set_edgecolor("black")
                sc.set_zorder(3)
                changed = True
        for ax in wing_axes:
            if ax.spines['bottom'].get_edgecolor() != "gray":
                for spine in ax.spines.values():
                    spine.set_edgecolor("gray")
                    spine.set_linewidth(0.8)
                changed = True
        
        if event.inaxes is None:
            if changed:
                fig.canvas.draw_idle()
            return
        
        # Highlight on hovering wing subpanel
        if event.inaxes in wing_axes:
            idx = wing_axes.index(event.inaxes)
            sc = scatter_elements[idx]
            sc.set_sizes([400])
            sc.set_edgecolor("gold")
            sc.set_linewidths([4.0])
            sc.set_zorder(100)
            
            for spine in event.inaxes.spines.values():
                spine.set_edgecolor("gold")
                spine.set_linewidth(2.5)
            changed = True
            
        # Highlight on hovering scatter node
        elif event.inaxes == ax_pareto:
            for idx, sc in enumerate(scatter_elements):
                cont, ind = sc.contains(event)
                if cont:
                    sc.set_sizes([400])
                    sc.set_edgecolor("gold")
                    sc.set_linewidths([4.0])
                    sc.set_zorder(100)
                    
                    target_ax = wing_axes[idx]
                    for spine in target_ax.spines.values():
                        spine.set_edgecolor("gold")
                        spine.set_linewidth(2.5)
                    changed = True
                    break
        
        if changed:
            fig.canvas.draw_idle()
            
    fig.canvas.mpl_connect("motion_notify_event", on_motion)

    save_path = get_incremental_save_path()
    plt.savefig(save_path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[✓] Chart saved: {save_path}")
    
    # Condense text displays to type only prior to window rendering to protect visibility
    for t_box, pretty_name in info_text_objects:
        t_box.set_text(pretty_name)
        t_box.set_fontsize(9)
        
    plt.show()

# ─── MAIN EXECUTION ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    pareto_data = load_pareto()
    all_population_data = load_all_history()
    
    if pareto_data:
        print(f"[✓] Loaded {len(pareto_data)} Pareto solutions.")
        if all_population_data:
            print(f"[✓] Loaded {len(all_population_data)} historical candidate evaluations.")
        plot_dashboard(pareto_data, all_population_data)
    else:
        print("[!] No valid Pareto data available. Has optimization completed?")
