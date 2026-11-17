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
import mimetypes
import os
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


# ─── CONFIGS PER MODE ────────────────────────────────────────────────────────
def mode_config(mode):
    base = copy.deepcopy(DEFAULTS)
    if mode == "bwb":
        base = _deep_merge(base, load_raw_config(os.path.join(SCRIPT_DIR, "bwb_config.json")))
    elif mode == "transonic":
        base = _deep_merge(base, load_raw_config(os.path.join(SCRIPT_DIR, "navy_trainer_config.json")))
    if mode == "airfoil":
        return {"flight": base["flight"], "airfoil": base["airfoil"]}
    if mode == "transonic":
        return {"transonic": base["transonic"], "mesh": base["mesh"]}
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
    return {"table": table, "images": images}


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
            return self._send(200, mode_config(q["mode"][0]))
        if u.path == "/api/log":
            text, off = read_log(int(q.get("offset", ["0"])[0]))
            code = None if running() or STATE["proc"] is None else STATE["proc"].returncode
            return self._send(200, {"text": text, "offset": off, "running": running(), "label": STATE["label"], "code": code,
                                    "secs": round(time.time() - STATE["started"]) if running() else 0})
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
