"""
RAPID planform geometry (single source of truth for the optimizer, the VSP
worker parameters, the airfoil module, and the plotters).

The 6 design variables x = [AR, kink_frac, t_inner, t_outer, sweep_inner, sweep_outer]
describe a cranked (root / kink / tip) half-wing whose exposed panels start at
the centerline of the VSPAERO model. Spanwise stations can be densified to any
N >= 3 without changing the planform: extra stations are distributed along the
inner and outer panels in proportion to their span.
"""

import math

import numpy as np

# Built-in fallbacks (C-130-class transport); normally provided by rapid_config "geometry".
GEOMETRY_DEFAULTS = {
    "FUSELAGE_Y":    2.2,    # half fuselage width removed from the gross half-span [m]
    "HALF_SPAN_MAX": 25.0,   # gross half-span cap [m]
    "C_ROOT_MIN":    4.0,    # admissible root chord range [m]; designs outside are rejected
    "C_ROOT_MAX":    9.5,
}


def _g(geo_cfg, key):
    return float((geo_cfg or {}).get(key, GEOMETRY_DEFAULTS[key]))


def half_span_from_ar(AR, S_REF, geo_cfg=None):
    """Gross half-span from aspect ratio, capped at HALF_SPAN_MAX."""
    return min(math.sqrt(AR * S_REF) / 2.0, _g(geo_cfg, "HALF_SPAN_MAX"))


def chord_from_geometry(AR, kink_frac, t_inner, t_outer, S_REF, geo_cfg=None):
    """Return (c_root, c_kink, c_tip, b_half, b_kink, mac) or None if infeasible.

    b_half is the exposed half-span (gross half-span minus FUSELAGE_Y), and the
    chords are scaled so that both exposed panels together have area S_REF.
    """
    b_half = half_span_from_ar(AR, S_REF, geo_cfg) - _g(geo_cfg, "FUSELAGE_Y")
    if b_half <= 0.0:
        return None
    b_kink = kink_frac * b_half
    b_tip = b_half - b_kink
    factor = 0.5 * (1 + t_inner) * b_kink + 0.5 * t_inner * (1 + t_outer) * b_tip
    c_root = (S_REF / 2.0) / factor
    if not (_g(geo_cfg, "C_ROOT_MIN") <= c_root <= _g(geo_cfg, "C_ROOT_MAX")):
        return None
    c_kink = t_inner * c_root
    c_tip = t_outer * c_kink
    S_i = 0.5 * (c_root + c_kink) * b_kink
    S_o = 0.5 * (c_kink + c_tip) * b_tip
    mi = (2 / 3) * c_root * (1 + t_inner + t_inner ** 2) / (1 + t_inner)
    mo = (2 / 3) * c_kink * (1 + t_outer + t_outer ** 2) / (1 + t_outer)
    mac = (mi * S_i + mo * S_o) / (S_i + S_o)
    return c_root, c_kink, c_tip, b_half, b_kink, mac


def _split_counts(n_sections, b_kink, b_half):
    """Number of extra (non root/kink/tip) stations in the inner / outer panel."""
    extra = max(int(n_sections) - 3, 0)
    n_in = int(round(extra * b_kink / b_half)) if b_half > 0 else 0
    return n_in, extra - n_in


def planform_stations(x, S_REF, n_sections=3, incidence=0.0, geo_cfg=None):
    """Spanwise stations (root -> tip) for design x, or None if infeasible.

    Each station: {y, eta, chord, x_le, twist, zone}. Twist is 0 on the inner
    panel and washes out linearly to -incidence at the tip (outer panel).
    """
    AR, kink_frac, t_inner, t_outer, sw_in, sw_out = [float(v) for v in x]
    geo = chord_from_geometry(AR, kink_frac, t_inner, t_outer, S_REF, geo_cfg)
    if geo is None:
        return None
    c_root, c_kink, c_tip, b_half, b_kink, _mac = geo
    n_in, n_out = _split_counts(n_sections, b_kink, b_half)

    ys = list(np.linspace(0.0, b_kink, n_in + 2)) + list(np.linspace(b_kink, b_half, n_out + 2))[1:]
    tan_in, tan_out = math.tan(math.radians(sw_in)), math.tan(math.radians(sw_out))
    x_kink = b_kink * tan_in

    stations = []
    for i, y in enumerate(ys):
        if y <= b_kink + 1e-12:
            f = y / b_kink if b_kink > 0 else 0.0
            chord = c_root + (c_kink - c_root) * f
            x_le = y * tan_in
            twist = 0.0
        else:
            f = (y - b_kink) / (b_half - b_kink)
            chord = c_kink + (c_tip - c_kink) * f
            x_le = x_kink + (y - b_kink) * tan_out
            twist = -incidence * f
        if i == 0:
            zone = "ROOT"
        elif i == len(ys) - 1:
            zone = "TIP"
        elif abs(y - b_kink) < 1e-9:
            zone = "KINK"
        else:
            zone = "INNER" if y < b_kink else "OUTER"
        stations.append({"y": float(y), "eta": float(y / b_half), "chord": float(chord),
                         "x_le": float(x_le), "twist": float(twist), "zone": zone})
    return stations


def mac_reference(stations):
    """(MAC, x_LE of the MAC) of a piecewise-linear half-wing, integrated exactly
    per trapezoidal panel (chord and x_le are linear in y on each panel)."""
    area = c2 = cx = 0.0
    for a, b in zip(stations[:-1], stations[1:]):
        dy = b["y"] - a["y"]
        c0, c1 = a["chord"], b["chord"]
        x0, x1 = a["x_le"], b["x_le"]
        area += 0.5 * (c0 + c1) * dy
        c2 += dy * (c0 * c0 + c0 * c1 + c1 * c1) / 3.0
        cx += dy * (2 * c0 * x0 + c0 * x1 + c1 * x0 + 2 * c1 * x1) / 6.0
    return c2 / area, cx / area


def panels_from_stations(stations, sweeps):
    """OpenVSP wing-section list between consecutive stations.

    sweeps = (sweep_inner, sweep_outer) in degrees (leading-edge)."""
    b_kink = next((s["y"] for s in stations if s["zone"] == "KINK"), stations[-1]["y"])
    panels = []
    for a, b in zip(stations[:-1], stations[1:]):
        inner = b["y"] <= b_kink + 1e-9
        panels.append({
            "span": b["y"] - a["y"],
            "root_chord": a["chord"],
            "tip_chord": b["chord"],
            "sweep": float(sweeps[0] if inner else sweeps[1]),
            "twist": b["twist"],
        })
    return panels


def spanwise_tess(panels, tess_u_total, min_per_panel=3):
    """Distribute a total spanwise tessellation count over panels by span."""
    total_span = sum(p["span"] for p in panels)
    return [max(min_per_panel, int(round(tess_u_total * p["span"] / total_span))) for p in panels]
