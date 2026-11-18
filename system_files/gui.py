"""
RAPID GUI: a minimal local web page (standard library only).

    python system_files/gui.py [--port 8765] [--no-browser]

Modes: Transonic, Conventional, BWB, Airfoil, Check. Forms are generated from the
config (rapid_config.py + presets), so every parameter is editable. Each run is a
normal subprocess of the existing scripts; the page streams its log and shows results.
Binds to 127.0.0.1 only; POST requests need the per-session token embedded in the page.
"""

import argparse
import copy
import glob
import json
import math
import mimetypes
import os
import shutil
import secrets
import subprocess
import sys
import threading
import time
import urllib.parse
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)
from rapid_config import DEFAULTS, _deep_merge, load_raw_config  # noqa: E402

PY = sys.executable
TOKEN = secrets.token_hex(8)
WM = os.path.join(ROOT, "wing_models")
LOG = os.path.join(WM, "gui_last.log")
SERVED_DIRS = ("wing_models", "PLOTS", "docs")
STATE = {"proc": None, "mode": None, "label": "", "started": 0.0}
LOCK = threading.Lock()


# ─── SAVED SETTINGS ──────────────────────────────────────────────────────────
SETTINGS = os.path.join(SCRIPT_DIR, "gui_settings.json")


def load_settings():
    try:
        with open(SETTINGS, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_settings(mode, config, options):
    data = load_settings()
    data[mode] = {"config": config, "options": {k: v for k, v in options.items() if k != "fresh"}}
    with open(SETTINGS, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return {"ok": True}


def forget_settings(mode):
    data = load_settings()
    if data.pop(mode, None) is not None:
        with open(SETTINGS, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)


# ─── CONFIGS PER MODE ────────────────────────────────────────────────────────
def mode_config(mode):
    base = copy.deepcopy(DEFAULTS)
    if mode in ("conventional", "airfoil"):                    # start from the user's own rapid_config.json
        base = _deep_merge(base, load_raw_config())
    if mode == "bwb":
        base = _deep_merge(base, load_raw_config(os.path.join(SCRIPT_DIR, "bwb_config.json")))
    elif mode == "transonic":
        base = _deep_merge(base, load_raw_config(os.path.join(SCRIPT_DIR, "navy_trainer_config.json")))
    if mode == "airfoil":
        return {"flight": base["flight"], "airfoil": base["airfoil"]}
    if mode == "transonic":
        # airfoil settings come from the user's current Airfoil-tab settings; the preset only supplies a sensible max t/c
        af = copy.deepcopy(DEFAULTS["airfoil"])
        af.update(load_raw_config().get("airfoil", {}))
        af.update(load_settings().get("airfoil", {}).get("config", {}).get("airfoil", {}))
        for k in ("MAX_TC_ROOT", "MAX_TC_MID", "MAX_TC_TIP"):
            if float(af.get(k, 1.0)) >= 1.0:
                af[k] = base["airfoil"][k]
        return {"transonic": base["transonic"], "mesh": base["mesh"], "airfoil": af}
    if mode == "check":
        return {}
    return base


def outdir(mode):
    return os.path.join(WM, "transonic") if mode == "transonic" else os.path.join(WM, f"gui_{mode}")


# ─── RUNNING ─────────────────────────────────────────────────────────────────
def running():
    p = STATE["proc"]
    return p is not None and p.poll() is None


def start(req):
    mode, action, opt = req["mode"], req.get("action", "run"), req.get("options", {})
    with LOCK:
        if running():
            return {"error": "a run is already active"}
        os.makedirs(WM, exist_ok=True)
        out = outdir(mode)
        os.makedirs(out, exist_ok=True)
        env = os.environ.copy()
        env["RAPID_OUTPUT_DIR"] = out
        env["PYTHONUNBUFFERED"] = "1"          # a redirected stdout is block-buffered otherwise: no live log
        env.pop("RAPID_ACTIVE_AIRFOILS", None)
        cfg_path = os.path.join(out, "gui_config.json")
        with open(cfg_path, "w", encoding="utf-8") as f:
            json.dump(req.get("config", {}), f, indent=2)
        env["RAPID_CONFIG_FILE"] = cfg_path
        s = lambda k, d="": str(opt.get(k, d)).strip()          # noqa: E731
        cmd = [PY]
        if mode == "transonic":
            cmd += [os.path.join(SCRIPT_DIR, "transonic_opti.py"), "--pop", s("pop", 40), "--gens", s("gens", 30),
                    "--seed", s("seed", 1), "--select", s("select", "range")]
            if opt.get("fresh"):
                cmd.append("--fresh")
            if opt.get("no_tc"):
                cmd.append("--no-tc")
            if opt.get("airfoil"):
                cmd += ["--airfoil", "--catalog", s("catalog", "UIUC_All")]
            if action == "report":
                cmd.append("--report-only")
            if s("cores"):
                cmd += ["--cores", s("cores")]
        elif mode in ("conventional", "bwb"):
            if action == "plot":
                env["MPLBACKEND"] = "Agg"
                cmd += [os.path.join(SCRIPT_DIR, "plotter.py")]
            else:
                cmd += [os.path.join(SCRIPT_DIR, "planform_opti.py"), "--yes", "--pop", s("pop", 32), "--gens", s("gens", 20)]
                if s("cores"):
                    cmd += ["--cores", s("cores")]
                if opt.get("fresh"):
                    cmd.append("--fresh")
                if opt.get("airfoil"):
                    cmd.append("--airfoil")
        elif mode == "airfoil":
            acfg = os.path.join(out, "airfoil_config.json")
            with open(acfg, "w", encoding="utf-8") as f:
                json.dump({"vsp3": s("vsp3", "system_files/presets/example_transport_wing.vsp3"),
                           "wing_name": None, "n_sections": int(s("n_sections", 5) or 5), "catalog": s("catalog", "Medium_Re"),
                           "cores": int(s("cores")) if s("cores") else None, "refine": bool(opt.get("refine")),
                           "write_active": True}, f, indent=2)
            env["MPLBACKEND"] = "Agg"
            cmd += [os.path.join(SCRIPT_DIR, "airfoil_opti.py"), "--yes", "--config", acfg]
        elif mode == "check":
            cmd += [os.path.join(SCRIPT_DIR, "check_install.py"), "--solve"]
        else:
            return {"error": "unknown mode"}
        logf = open(LOG, "wb")
        flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        STATE.update(proc=subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=logf, stderr=subprocess.STDOUT, creationflags=flags),
                     mode=mode, label=f"{mode} {action}", started=time.time())
        return {"ok": True}


def stop():
    with LOCK:
        p = STATE["proc"]
        if p is None or p.poll() is not None:
            return {"ok": True}
        if os.name == "nt":                       # kill the whole tree (pool workers, VSPAERO)
            subprocess.run(["taskkill", "/PID", str(p.pid), "/T", "/F"], capture_output=True)
        else:
            p.terminate()
        return {"ok": True}


def read_log(offset):
    try:
        with open(LOG, "rb") as f:
            f.seek(offset)
            data = f.read()
        return data.decode("utf-8", errors="replace"), offset + len(data)
    except OSError:
        return "", 0


# ─── PARETO CARDS ────────────────────────────────────────────────────────────
def poly_trapezoid(b, cr, ct, sweep_le):
    """Right half-wing outline as [y, x] points (x aft of the root leading edge)."""
    t, y = math.tan(math.radians(sweep_le)), b / 2.0
    return [[0, 0], [y, y * t], [y, y * t + ct], [0, cr]]


def poly_cranked(r, cfg):
    import geometry
    geo = cfg["geometry"]
    g = geometry.chord_from_geometry(r["AR"], r["kink_frac"], r["t_inner"], r["t_outer"], float(cfg["flight"]["S_REF"]), geo)
    if g is None:
        return None
    c_root, c_kink, c_tip, b_half, b_kink, _ = g
    fy = float(geo["FUSELAGE_Y"])
    xk = b_kink * math.tan(math.radians(r["sweep_inner"]))
    xt = xk + (b_half - b_kink) * math.tan(math.radians(r["sweep_outer"]))
    return [[fy, 0], [fy + b_kink, xk], [fy + b_half, xt], [fy + b_half, xt + c_tip], [fy + b_kink, xk + c_kink], [fy, c_root]]


def pareto_transonic(out):
    rep = os.path.join(out, "report")
    pj = os.path.join(rep, "pareto.json")
    if not os.path.exists(pj):
        return None
    rows = sorted(json.load(open(pj, encoding="utf-8")), key=lambda r: r["CD"])
    chosen = None
    cw = os.path.join(rep, "chosen_wing.json")
    if os.path.exists(cw):
        chosen = json.load(open(cw, encoding="utf-8"))["design"]

    def pt(r, tag=""):
        return {"x": round(r["CD"] * 1e4, 2), "y": round(r["W_wing_lb"]), "tag": tag,
                "poly": poly_trapezoid(r["b"], r["cr"], r["ct"], r["SWEEP_LE"]),
                "lines": [f"{r['CD']*1e4:.1f} counts · {r['W_wing_lb']:.0f} lb", f"AR {r['AR']:.2f} · sweep {r['SWEEP_LE']:.1f}° · taper {r['TAPER']:.2f}",
                          f"washout {r['WASHOUT']:.1f}° · M_dd {r['Mdd_min']:.3f}"]}
    pts = []
    for r in rows:
        is_ch = chosen is not None and abs(r["AR"] - chosen["AR"]) < 1e-6 and abs(r["SWEEP_LE"] - chosen["SWEEP_LE"]) < 1e-6
        pts.append(pt(r, "chosen" if is_ch else ""))
    cj = os.path.join(rep, "chart_wing.json")
    if os.path.exists(cj):
        c = json.load(open(cj, encoding="utf-8"))
        d, m = c["design"], c["metrics"]
        pts.append(pt({"CD": m["CD"], "W_wing_lb": m["W_wing_lb"], "b": m["b"], "cr": m["cr"], "ct": m["ct"], "SWEEP_LE": d["SWEEP_LE"],
                       "AR": d["AR"], "TAPER": d["TAPER"], "WASHOUT": d["WASHOUT"], "Mdd_min": m["Mdd_min"]}, "chart"))
    return {"xlabel": "Cruise drag C_D (counts)  →  worse", "ylabel": "Wing weight (lb)  →  heavier",
            "hint": "Each dot is one optimal wing: no other wing found is both lighter AND lower-drag. Lower-left is better; moving along the "
                    "curve trades drag for weight. Numbers match the planform cards below. Gold = chosen wing, red = chart wing (dominated).",
            "points": pts}


def pareto_planform(mode, out):
    pj = os.path.join(out, "pareto_results.json")
    if not os.path.exists(pj):
        return None
    from rapid_config import load_effective_config
    cp = os.path.join(out, "gui_config.json")
    cfg = load_effective_config(cp if os.path.exists(cp) else None)
    rows = sorted(json.load(open(pj, encoding="utf-8")), key=lambda r: r["CL"])
    pts = []
    for r in rows:
        poly = poly_cranked(r, cfg)
        if poly is None:
            continue
        lines = [f"CL {r['CL']:.3f} · L/D {r['LD']:.1f}", f"AR {r['AR']:.2f} · kink {r['kink_frac']:.2f} · taper {r['t_inner']:.2f}/{r['t_outer']:.2f}",
                 f"sweep {r['sweep_inner']:.0f}°/{r['sweep_outer']:.0f}°" + (f" · SM {r['SM']*100:.0f}%" if "SM" in r else "")]
        pts.append({"x": r["CL"], "y": r["LD"], "tag": "", "poly": poly, "lines": lines})
    return {"xlabel": "Lift coefficient CL  →  more lift", "ylabel": "Lift-to-drag L/D  →  more efficient",
            "hint": "Each dot is one optimal wing: no other wing found has both higher CL AND higher L/D. Upper-right is better; moving along the "
                    "curve trades CL for L/D. Numbers match the planform cards below.", "points": pts}


def pareto_airfoil(out):
    pj = os.path.join(out, "airfoil_results.json")
    if not os.path.exists(pj):
        return None
    d = json.load(open(pj, encoding="utf-8"))
    delta = d.get("stall_delta", 2.0)
    res = []

    def lines(e):
        shape = f"t/c {e['tc']*100:.1f} % · camber {e['camber']*100:.1f} %" + (" · reflexed" if e["reflexed"] else "")
        stall = "stall: no post-stall data" if e.get("stall_drop") is None else f"stall: -{e['stall_drop']*100:.0f} % CL in {delta:g}° past CLmax"
        return [e["family"], shape, f"L/D {e['ld']:.1f} · Cm {e['cm']:+.3f}", stall]

    for sec in d["sections"]:
        pts, win = [], sec["winner"]
        for p in sec["points"]:
            if p["pareto"]:
                pts.append({"x": p["ld"], "y": p["cm"], "tag": "chosen" if p["name"] == win["name"] else "", "poly": p["coords"],
                            "title": p["name"].upper(), "lines": lines(p)})
            else:
                pts.append({"x": p["ld"], "y": p["cm"], "tag": "", "nocard": True, "poly": []})
        if not any(q["tag"] == "chosen" for q in pts):            # refined / off-front winner still gets a card
            pts.append({"x": win["ld"], "y": win["cm"], "tag": "chosen", "poly": win["coords"], "title": win["name"].upper(), "lines": lines(win)})
        res.append({"title": f"Station {sec['id']} · {sec['zone']} · span {sec['eta']*100:.0f} % · Re {sec['Re']/1e6:.1f}M · α {sec['alpha']}°",
                    "shape": "airfoil", "xlabel": "Average L/D  →  more efficient", "ylabel": "Pitching moment Cm  →  less nose-down",
                    "hint": f"Each dot is a catalog section ({d['catalog']}) at this station's Reynolds number and angle. Coloured dots are Pareto-optimal: "
                            "no other section has both higher L/D and a less nose-down Cm. Grey = dominated or rejected by the stall check. "
                            "Gold = the section the optimizer picked. Cards show the shape and type.", "points": pts})
    return res


def reset(mode):
    with LOCK:
        if running():
            return {"error": "stop the active run first"}
        out = os.path.abspath(outdir(mode))      # results + checkpoint only; saved settings are kept
        if not out.startswith(os.path.abspath(WM) + os.sep) or not os.path.isdir(out):
            return {"ok": True}
        shutil.rmtree(out, ignore_errors=True)
        return {"ok": True}


# ─── WINGS THE AIRFOIL TAB CAN WORK ON ───────────────────────────────────────
def list_wings():
    """[[relative .vsp3 path, label], ...]: the wings this GUI has produced, plus the example preset."""
    items = []

    def add(path, label):
        if os.path.exists(os.path.join(ROOT, path)):
            items.append([path.replace("\\", "/"), label])

    tdir = os.path.join(WM, "transonic")
    rep = os.path.join(tdir, "report")
    try:
        front = json.load(open(os.path.join(rep, "pareto.json"), encoding="utf-8"))
        ch = json.load(open(os.path.join(rep, "chosen_wing.json"), encoding="utf-8"))["design"]
        pick = lambda r: f"wing_models/transonic/{r['name']}/{r['name']}.vsp3"                       # noqa: E731
        for r in front:
            if abs(r["AR"] - ch["AR"]) < 1e-6 and abs(r["SWEEP_LE"] - ch["SWEEP_LE"]) < 1e-6:
                add(pick(r), "Transonic: chosen wing")
        add(pick(min(front, key=lambda r: r["CD"])), "Transonic: lowest-drag Pareto wing")
        add(pick(min(front, key=lambda r: r["W_wing_lb"])), "Transonic: lightest Pareto wing")
    except (OSError, ValueError, KeyError):
        pass
    add("wing_models/transonic/chart_wing/chart_wing.vsp3", "Transonic: chart wing")
    for mode in ("conventional", "bwb"):
        try:
            for r in json.load(open(os.path.join(WM, f"gui_{mode}", "pareto_results.json"), encoding="utf-8")):
                n = r.get("folder_name")
                if n:
                    add(f"wing_models/gui_{mode}/{n}/{n}.vsp3", f"{mode} #{r['solution_id']} (CL {r['CL']:.2f}, L/D {r['LD']:.1f})")
        except (OSError, ValueError):
            pass
    add("system_files/presets/example_transport_wing.vsp3", "Example transport wing (preset, 210 m^2)")
    return items


# ─── RESULTS ─────────────────────────────────────────────────────────────────
def rel(p):
    return "/file?p=" + urllib.parse.quote(os.path.relpath(p, ROOT).replace("\\", "/"))


def results(mode):
    out, table, images = outdir(mode), None, []
    if mode == "transonic":
        rep = os.path.join(out, "report")
        pj = os.path.join(rep, "pareto.json")
        if os.path.exists(pj):
            rows = json.load(open(pj, encoding="utf-8"))
            cols = ["CD (cts)", "W (lb)", "L/D", "AR", "LE sweep", "taper", "washout", "t/c r", "t/c t", "M_dd"]
            table = {"columns": cols, "rows": [[round(r["CD"] * 1e4, 1), round(r["W_wing_lb"]), round(r["LD"], 2), round(r["AR"], 2),
                                                round(r["SWEEP_LE"], 1), round(r["TAPER"], 3), round(r["WASHOUT"], 2),
                                                round(r["TC_ROOT"], 3), round(r["TC_TIP"], 3), round(r["Mdd_min"], 3)] for r in rows]}
        images = [rel(os.path.join(rep, f)) for f in ("pareto.png", "spanwise.png", "planform.png") if os.path.exists(os.path.join(rep, f))]
    elif mode in ("conventional", "bwb"):
        pj = os.path.join(out, "pareto_results.json")
        if os.path.exists(pj):
            rows = json.load(open(pj, encoding="utf-8"))
            keys = ["solution_id", "CL", "LD", "AR", "kink_frac", "t_inner", "t_outer", "sweep_inner", "sweep_outer", "SM", "CM_trim"]
            keys = [k for k in keys if any(k in r for r in rows)]
            table = {"columns": keys, "rows": [[r.get(k, "") for k in keys] for r in rows]}
        plots = sorted(glob.glob(os.path.join(ROOT, "PLOTS", "plot_*.png")), key=os.path.getmtime)
        images = [rel(plots[-1])] if plots else []
    elif mode == "airfoil":
        p = os.path.join(ROOT, "PLOTS", "airfoil_winners.png")
        images = [rel(p)] if os.path.exists(p) else []
        aa = os.path.join(SCRIPT_DIR, "active_airfoils.json")
        if os.path.exists(aa):
            st = json.load(open(aa, encoding="utf-8")).get("stations", [])
            table = {"columns": ["eta", "airfoil"], "rows": [[s["eta"], os.path.basename(str(s["airfoil"]))] for s in st]}
    pareto = pareto_transonic(out) if mode == "transonic" else (pareto_planform(mode, out) if mode in ("conventional", "bwb") else None)
    paretos = pareto_airfoil(out) if mode in ("airfoil", "transonic") else None
    return {"table": table, "images": images, "pareto": pareto, "paretos": paretos}


# ─── HTTP ────────────────────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        if u.path == "/":
            html = open(os.path.join(SCRIPT_DIR, "gui", "index.html"), encoding="utf-8").read().replace("__TOKEN__", TOKEN)
            return self._send(200, html, "text/html; charset=utf-8")
        if u.path == "/api/config":
            mode = q["mode"][0]
            saved = load_settings().get(mode, {})
            return self._send(200, {"config": _deep_merge(mode_config(mode), saved.get("config", {})), "options": saved.get("options", {})})
        if u.path == "/api/log":
            text, off = read_log(int(q.get("offset", ["0"])[0]))
            code = None if running() or STATE["proc"] is None else STATE["proc"].returncode
            return self._send(200, {"text": text, "offset": off, "running": running(), "label": STATE["label"], "code": code,
                                    "secs": round(time.time() - STATE["started"]) if running() else 0,
                                    "idle": round(time.time() - os.path.getmtime(LOG)) if running() and os.path.exists(LOG) else 0})
        if u.path == "/api/wings":
            return self._send(200, list_wings())
        if u.path == "/api/results":
            return self._send(200, results(q["mode"][0]))
        if u.path == "/file":
            p = os.path.normpath(os.path.join(ROOT, q["p"][0]))
            if not any(p.startswith(os.path.join(ROOT, d) + os.sep) for d in SERVED_DIRS) or not os.path.isfile(p):
                return self._send(404, {"error": "not found"})
            return self._send(200, open(p, "rb").read(), mimetypes.guess_type(p)[0] or "application/octet-stream")
        self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.headers.get("X-Token") != TOKEN:
            return self._send(403, {"error": "bad token"})
        n = int(self.headers.get("Content-Length", "0"))
        req = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/api/run":
            return self._send(200, start(req))
        if self.path == "/api/stop":
            return self._send(200, stop())
        if self.path == "/api/save":
            return self._send(200, save_settings(req["mode"], req.get("config", {}), req.get("options", {})))
        if self.path == "/api/reset":
            return self._send(200, reset(req["mode"]))
        self._send(404, {"error": "not found"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://127.0.0.1:{a.port}/"
    print(f"RAPID GUI running at {url}  (Ctrl+C to quit)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop()


if __name__ == "__main__":
    main()
