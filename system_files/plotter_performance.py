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
    """Returns plot_perf_1.png, plot_perf_2.png etc without overwriting."""
    counter = 1
    while True:
        path = os.path.join(PLOTS_DIR, f"performance_plot_{counter}.png")
        if not os.path.exists(path):
            return path
        counter += 1

# ─── GEOMETRY PARAMETERS ──────────────────────────────────────────────────────
S_REF         = 210.0
FUSELAGE_Y    = 2.2
HALF_SPAN_MAX = 25.0

# ─── DATA LOADING & PROCESSING ───────────────────────────────────────────────
def load_pareto():
    if not os.path.exists(JSON_PATH):
        print(f"[ERROR] File not found: {JSON_PATH}")
        return []
    with open(JSON_PATH, "r") as f:
        data = json.load(f)
    valid = []
    for d in data:
        if 0 < d["LD"] < 60 and d["CL"] > 0:
            # Compute performance metrics
            d["EnduranceFactor"] = np.sqrt(d["CL"]) * d["LD"]  # CL^1.5 / CD
            d["RangeFactor"]     = d["LD"]                     # CL / CD
            valid.append(d)
    return valid

def load_all_history():
    """Retrieves data subsets and derives historical performance factors."""
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
                
                if 0.0 < cl < 3.0 and -5.0 < ld < 60.0:
                    is_feas = (0.15 <= cl) and (ld >= 15.0) and (cm <= 0.35)
                    all_points.append({
                        "CL": cl, "LD": ld, "feasible": is_feas,
                        "RangeFactor": ld,
                        "EnduranceFactor": np.sqrt(cl) * ld
                    })
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

# ─── PERFORMANCE PLOTTER ──────────────────────────────────────────────────────
def plot_performance_dashboard(pareto, all_pop):
    N = len(pareto)
    if N == 0:
        print("[WARNING] No Pareto solutions found to display.")
        return

    colors = plt.cm.turbo(np.linspace(0.1, 0.9, N))
    scatter_elements = []
    wing_axes = []
    info_text_objects = []  # To clean up for interactive show()

    n_cols = 8
    n_rows = int(np.ceil(N / n_cols))

    fig_height = 7.0 + (3.2 * n_rows)
    fig = plt.figure(figsize=(24, fig_height), facecolor="white")
    
    height_ratios = [2.5] + [1.0] * n_rows
    gs = gridspec.GridSpec(1 + n_rows, n_cols, height_ratios=height_ratios, hspace=0.4, wspace=0.25)

    ax_pareto = fig.add_subplot(gs[0, :])

    # 1. Plot entire history
    if all_pop:
        feas_x = [p["RangeFactor"] for p in all_pop if p.get("feasible", True)]
        feas_y = [p["EnduranceFactor"] for p in all_pop if p.get("feasible", True)]
        
        infeas_x = [p["RangeFactor"] for p in all_pop if not p.get("feasible", True)]
        infeas_y = [p["EnduranceFactor"] for p in all_pop if not p.get("feasible", True)]
        
        if infeas_x:
            ax_pareto.scatter(
                infeas_x, infeas_y,
                color="#ffa8a8", s=25, zorder=1, edgecolors="none", alpha=0.30,
                label="Violated Constraints"
            )
        
        if feas_x:
            ax_pareto.scatter(
                feas_x, feas_y,
                color="#a3e2a3", s=30, zorder=1, edgecolors="none", alpha=0.45,
                label="Feasible History Designs"
            )

    # 2. Connective boundary
    sorted_p = sorted(pareto, key=lambda x: x["RangeFactor"])
    ax_pareto.plot(
        [d["RangeFactor"] for d in sorted_p],
        [d["EnduranceFactor"] for d in sorted_p],
        color="black", linestyle="--", linewidth=1.5, alpha=0.5, zorder=2
    )

    # 3. Optimal nodes
    for i, (d, color) in enumerate(zip(pareto, colors)):
        sc = ax_pareto.scatter(
            d["RangeFactor"], d["EnduranceFactor"],
            color=color, s=90, zorder=3, edgecolors="black", linewidth=1.0
        )
        scatter_elements.append(sc)
        ax_pareto.annotate(
            f"#{d['solution_id']}",
            (d["RangeFactor"], d["EnduranceFactor"]),
            textcoords="offset points",
            xytext=(4, 4),
            color="black", fontsize=8, fontweight="normal", va="bottom", ha="left"
        )

    ax_pareto.set_facecolor("#fcfcfc")
    ax_pareto.set_title("Aircraft Aerodynamic Performance Map",
                         fontsize=15, fontweight="bold", color="black", pad=15)
    ax_pareto.set_xlabel("Range Factor (L/D)", fontsize=13, color="black", fontweight="bold")
    ax_pareto.set_ylabel(r"Endurance Factor ($C_L^{1.5}/C_D$)", fontsize=13, color="black", fontweight="bold")
    ax_pareto.tick_params(colors="black", labelsize=11)
    
    for spine in ax_pareto.spines.values():
        spine.set_edgecolor("black")
        spine.set_linewidth(1.2)
        
    # Tight dynamic viewport focus
    if pareto:
        p_x = [d["RangeFactor"] for d in pareto]
        p_y = [d["EnduranceFactor"] for d in pareto]
        
        min_x, max_x = min(p_x), max(p_x)
        min_y, max_y = min(p_y), max(p_y)
        
        span_x = max_x - min_x if len(pareto) > 1 else 5.0
        span_y = max_y - min_y if len(pareto) > 1 else 5.0
        
        pad_x = max(0.8, span_x * 0.15)
        pad_y = max(0.8, span_y * 0.15)
        
        ax_pareto.set_xlim(left=min_x - pad_x, right=max_x + pad_x)
        ax_pareto.set_ylim(bottom=min_y - pad_y, top=max_y + pad_y)

    ax_pareto.grid(True, linestyle="--", alpha=0.6, color="#cccccc")

    # ── Planform subpanels ──
    for i, (d, color) in enumerate(zip(pareto, colors)):
        r = 1 + (i // n_cols)
        c = i % n_cols
        ax = fig.add_subplot(gs[r, c])
        wing_axes.append(ax)
        ax.set_facecolor("white")

        x, y = calc_wing_polygon(d)
        ax.fill(x, y, color=color, alpha=0.8, edgecolor="black", linewidth=1.5)

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
                f"Endurance={d['EnduranceFactor']:.2f}\n"
                f"Range (L/D)={d['RangeFactor']:.2f}\n"
                f"AR={d['AR']:.2f}  CL={d['CL']:.3f}")
        t_box = ax.text(0.5, -0.22, info, transform=ax.transAxes,
                fontsize=8, color="black", va="top", ha="center",
                bbox=dict(facecolor="#fdfdfd", alpha=0.95, edgecolor="lightgray", boxstyle="round,pad=0.4"))
        info_text_objects.append((t_box, pretty_name))

        ax.set_title(
            f"#{d['solution_id']}  Eff={d['EnduranceFactor']:.2f}",
            fontsize=10, fontweight="bold", color="black", pad=4
        )

    # ─── INTERACTIVE HOVER DYNAMICS ───
    def on_motion(event):
        changed = False
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
    print(f"[✓] Performance map saved: {save_path}")
    
    # Filter displays for interactive visibility
    for t_box, pretty_name in info_text_objects:
        t_box.set_text(pretty_name)
        t_box.set_fontsize(9)
        
    plt.show()

if __name__ == "__main__":
    pareto_data = load_pareto()
    all_population_data = load_all_history()
    
    if pareto_data:
        print(f"[✓] Loaded {len(pareto_data)} performance solutions.")
        plot_performance_dashboard(pareto_data, all_population_data)
    else:
        print("[!] No valid Pareto data available.")
