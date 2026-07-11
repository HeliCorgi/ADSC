#!/usr/bin/env python3
"""WP7a Visualization Pack: turn the committed WP5/WP6/WP3 CSVs into static SVG
figures + a static report page.

Python 3 standard library only (spec v4.2 R9 exception) -- no third-party deps,
no plotting library, no CDN/JS/external fonts; SVG is generated directly.

Discipline (spec v4.2):
  * No wall-clock timestamp or run time is embedded in any output (R6). Runtime,
    if any, is printed to stdout by the caller.
  * Every numeric value is read from a committed CSV -- nothing is hand-written.
  * Every figure caption carries: source CSV name, master seed, schema_version.
  * All coordinates/values use fixed-decimal formatting so the byte output is
    stable across platforms (the CI "regeneration == committed" SHA check).

Usage:  make_viz.py <repo_root> [out_dir]
        out_dir defaults to <repo_root>/generated/viz
"""
import csv
import os
import sys

# ---- fixed formatting (platform-stable) -----------------------------------
def fx(v, dp=2):
    return f"{float(v):.{dp}f}"

def esc(s):
    return (str(s).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))

# ---- fixed palette ---------------------------------------------------------
COL = {
    "A": "#2f6db3", "B": "#c76a2a",
    "ok": "#3a8f4e", "warn": "#c9a227", "bad": "#b23b3b", "muted": "#7a7a7a",
    "grid": "#dddddd", "axis": "#444444", "band": "#9fbfe0", "band2": "#e0b89f",
    "ink": "#222222", "bg": "#ffffff", "accent": "#6a4fb3",
}
OUTCOME_COL = {
    "completed": "#3a8f4e", "kit_exhausted": "#69a03a",
    "dv_exhausted": "#c9a227", "keep_out_violation": "#b23b3b", "other": "#7a7a7a",
    "sync_timeout": "#8a6d3b",
}

# ---- tiny SVG builder ------------------------------------------------------
class Fig:
    def __init__(self, w, h, title):
        self.w, self.h = w, h
        self.parts = []
        self.parts.append(
            f'<rect x="0" y="0" width="{w}" height="{h}" fill="{COL["bg"]}"/>')
        self.text(20, 30, title, size=18, weight="bold")

    def rect(self, x, y, w, h, fill, stroke="none", sw=0, op=1.0):
        self.parts.append(
            f'<rect x="{fx(x)}" y="{fx(y)}" width="{fx(w)}" height="{fx(h)}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{fx(sw)}" '
            f'fill-opacity="{fx(op)}"/>')

    def line(self, x1, y1, x2, y2, stroke, sw=1.0, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{fx(x1)}" y1="{fx(y1)}" x2="{fx(x2)}" y2="{fx(y2)}" '
            f'stroke="{stroke}" stroke-width="{fx(sw)}"{d}/>')

    def polyline(self, pts, stroke, sw=2.0):
        p = " ".join(f"{fx(x)},{fx(y)}" for x, y in pts)
        self.parts.append(
            f'<polyline points="{p}" fill="none" stroke="{stroke}" '
            f'stroke-width="{fx(sw)}"/>')

    def polygon(self, pts, fill, op=1.0):
        p = " ".join(f"{fx(x)},{fx(y)}" for x, y in pts)
        self.parts.append(
            f'<polygon points="{p}" fill="{fill}" fill-opacity="{fx(op)}"/>')

    def circle(self, x, y, r, fill):
        self.parts.append(
            f'<circle cx="{fx(x)}" cy="{fx(y)}" r="{fx(r)}" fill="{fill}"/>')

    def text(self, x, y, s, size=13, fill=None, anchor="start", weight="normal"):
        fill = fill or COL["ink"]
        self.parts.append(
            f'<text x="{fx(x)}" y="{fx(y)}" font-family="sans-serif" '
            f'font-size="{fx(size)}" fill="{fill}" text-anchor="{anchor}" '
            f'font-weight="{weight}">{esc(s)}</text>')

    def caption(self, lines):
        y = self.h - 8 - 15 * (len(lines) - 1)
        for ln in lines:
            self.text(20, y, ln, size=11, fill=COL["muted"])
            y += 15

    def svg(self):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" '
                f'height="{self.h}" viewBox="0 0 {self.w} {self.h}">')
        return head + "".join(self.parts) + "</svg>\n"


def write_text(path, s):
    # newline="\n" forces LF on every platform so the committed bytes are stable.
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(s)


# ---- CSV loaders -----------------------------------------------------------
def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def seed_hex(dec_str):
    return "0x%X" % int(dec_str)


class Data:
    """Reads the committed CSVs; exposes only values that appear in them."""
    def __init__(self, gen):
        self.gen = gen
        self.wp5 = load_rows(os.path.join(gen, "wp5_campaign_summary.csv"))
        self.wp6 = load_rows(os.path.join(gen, "wp6_cost_summary.csv"))
        wp3p = os.path.join(gen, "wp3_decay_trade.csv")
        self.wp3 = load_rows(wp3p) if os.path.exists(wp3p) else []
        # shared campaign identity (from WP5 CSV)
        self.seed = seed_hex(self.wp5[0]["master_seed"])
        self.wp5_schema = self.wp5[0]["schema_version"]
        self.wp6_schema = self.wp6[0]["schema_version"] if self.wp6 else "?"
        self.n_runs = self.wp5[0]["n_runs"]
        self.catalogs = []
        for r in self.wp5:
            if r["catalog"] not in self.catalogs:
                self.catalogs.append(r["catalog"])

    def wp5_metric(self, catalog, metric):
        for r in self.wp5:
            if r["catalog"] == catalog and r["metric"] == metric:
                return r
        return None

    def wp6_rows(self, catalog=None, record_type=None, metric=None):
        out = []
        for r in self.wp6:
            if catalog is not None and r["catalog"] != catalog:
                continue
            if record_type is not None and r["record_type"] != record_type:
                continue
            if metric is not None and r["metric"] != metric:
                continue
            out.append(r)
        return out


def short(catalog):
    return catalog.split("/")[0].strip()


# ---- charts ================================================================
def axes(fig, x0, y0, x1, y1):
    fig.line(x0, y0, x0, y1, COL["axis"], 1.2)   # y axis
    fig.line(x0, y1, x1, y1, COL["axis"], 1.2)   # x axis


def chart_outcomes(d, out):
    """Per-catalog stacked proportion of the run-terminal outcomes (sums to N).
    Every category is shown even at zero. success = completed + kit_exhausted."""
    fig = Fig(680, 420, "WP5 mission outcomes (terminal, per run)")
    cats = ["completed", "kit_exhausted", "dv_exhausted", "keep_out_violation", "other"]
    x0, x1 = 40, 640
    row_h, gap = 70, 40
    y = 70
    for c in d.catalogs:
        n = float(d.n_runs)
        counts = [float(d.wp5_metric(c, k)["estimate"]) for k in cats]
        fig.text(x0, y - 8, short(c), size=13, weight="bold")
        xcur = x0
        width = x1 - x0
        for k, cnt in zip(cats, counts):
            w = width * (cnt / n) if n else 0.0
            if w > 0:
                fig.rect(xcur, y, w, row_h - 20, OUTCOME_COL[k], stroke="#fff", sw=1)
                if w > 46:
                    fig.text(xcur + w / 2, y + (row_h - 20) / 2 + 4,
                             "%d" % int(cnt), size=11, fill="#fff", anchor="middle")
            xcur += w
        y += row_h + gap - 20
    # legend (shows zero categories too)
    lx, ly = x0, y + 6
    for k in cats:
        fig.rect(lx, ly - 10, 12, 12, OUTCOME_COL[k])
        fig.text(lx + 17, ly, k, size=11)
        lx += 120
    su = d.wp5_metric(d.catalogs[0], "success_rate")
    fig.caption([
        "success (productive end) = completed + kit_exhausted; bars sum to N per catalog. "
        "gate_abort/sync_timeout are per-target events (see other figures).",
        f"source: wp5_campaign_summary.csv  schema {d.wp5_schema}  seed {d.seed}  "
        f"N={d.n_runs}  (e.g. {short(d.catalogs[0])} success_rate={fx(float(su['estimate']),3)})",
    ])
    write_text(out, fig.svg())


def chart_percentiles(d, metric, title, unit, out, dp=1):
    """p05/p50/p95 range bars per catalog (never mean-only)."""
    fig = Fig(560, 420, title)
    x0, y0, x1, y1 = 70, 60, 520, 330
    axes(fig, x0, y0, x1, y1)
    rows = [(c, d.wp5_metric(c, metric)) for c in d.catalogs]
    vmax = max(float(r["p95"]) for _, r in rows) * 1.15 or 1.0
    def py(v):
        return y1 - (y1 - y0) * (v / vmax)
    # y ticks
    for i in range(5):
        v = vmax * i / 4
        yy = py(v)
        fig.line(x0, yy, x1, yy, COL["grid"], 1)
        fig.text(x0 - 8, yy + 4, fx(v, dp), size=10, fill=COL["muted"], anchor="end")
    slots = len(rows)
    for i, (c, r) in enumerate(rows):
        cx = x0 + (x1 - x0) * (i + 0.5) / slots
        p05, p50, p95 = float(r["p05"]), float(r["p50"]), float(r["p95"])
        col = COL["A"] if i == 0 else COL["B"]
        bw = 44
        fig.rect(cx - bw / 2, py(p95), bw, py(p05) - py(p95), col, op=0.30)
        fig.line(cx - bw / 2, py(p50), cx + bw / 2, py(p50), col, 3)   # p50
        fig.line(cx, py(p95), cx, py(p05), col, 1.5)                   # whisker
        fig.text(cx, y1 + 18, short(c), size=12, anchor="middle")
        fig.text(cx, py(p95) - 8, "p95 " + fx(p95, dp), size=10, anchor="middle", fill=COL["muted"])
        fig.text(cx + bw / 2 + 6, py(p50) + 4, "p50 " + fx(p50, dp), size=11)
        fig.text(cx, py(p05) + 16, "p05 " + fx(p05, dp), size=10, anchor="middle", fill=COL["muted"])
    fig.text(x0 - 52, (y0 + y1) / 2, unit, size=11, fill=COL["muted"], anchor="middle")
    fig.caption([
        "box = p05..p95, thick line = p50 (distribution, not mean).",
        f"source: wp5_campaign_summary.csv  schema {d.wp5_schema}  seed {d.seed}  N={d.n_runs}",
    ])
    write_text(out, fig.svg())


def chart_keepout_rate(d, out):
    """Rate panel (rate + Wilson 95% CI + N). No trajectory is drawn -- the WP5
    CSV has none; a trajectory-looking figure would be dishonest."""
    fig = Fig(560, 420, "WP5 keep-out violation rate (+ Wilson 95% CI)")
    x0, y0, x1, y1 = 90, 70, 520, 320
    axes(fig, x0, y0, x1, y1)
    rows = [(c, d.wp5_metric(c, "keep_out_violation_rate")) for c in d.catalogs]
    vmax = max(float(r["wilson_high"]) for _, r in rows) * 1.4 or 0.05
    def px(v):
        return x0 + (x1 - x0) * (v / vmax)
    for i in range(6):
        v = vmax * i / 5
        xx = px(v)
        fig.line(xx, y0, xx, y1, COL["grid"], 1)
        fig.text(xx, y1 + 16, fx(v, 3), size=10, fill=COL["muted"], anchor="middle")
    for i, (c, r) in enumerate(rows):
        est = float(r["estimate"]); lo = float(r["wilson_low"]); hi = float(r["wilson_high"])
        cy = y0 + (y1 - y0) * (i + 0.5) / len(rows)
        col = COL["A"] if i == 0 else COL["B"]
        fig.line(px(lo), cy, px(hi), cy, col, 2)
        fig.line(px(lo), cy - 7, px(lo), cy + 7, col, 2)
        fig.line(px(hi), cy - 7, px(hi), cy + 7, col, 2)
        fig.circle(px(est), cy, 5, col)
        fig.text(x0 - 10, cy + 4, short(c), size=12, anchor="end")
        fig.text(px(hi) + 8, cy + 4,
                 f"{fx(est,3)}  [{fx(lo,3)}, {fx(hi,3)}]", size=11)
    fig.text((x0 + x1) / 2, y1 + 34, "violation rate (fraction of runs)",
             size=11, fill=COL["muted"], anchor="middle")
    fig.rect(90, 344, 430, 20, "#fbeeee", stroke=COL["bad"], sw=1)
    fig.text(98, 358, "Simplified research visualization, not a flight safety certificate.",
             size=11, fill=COL["bad"])
    fig.caption([
        "Wilson 95% CI (z=1.959963984540054); rate reported even at zero.",
        f"source: wp5_campaign_summary.csv  schema {d.wp5_schema}  seed {d.seed}  N={d.n_runs}",
    ])
    write_text(out, fig.svg())


def chart_amortization(d, out):
    """cost/removal vs kits carried N; the N=4 Delta-v-limited minimum + turn is
    the headline installer-argument figure."""
    fig = Fig(680, 460, "WP6 amortization: cost/removal vs kits carried N")
    x0, y0, x1, y1 = 70, 60, 620, 350
    axes(fig, x0, y0, x1, y1)
    series = {}
    ns = set()
    for c in d.catalogs:
        pts = {}
        for r in d.wp6_rows(c, "amortization", "cost_per_removal"):
            n = int(r["n_kits"]); ns.add(n)
            pts[n] = (float(r["p05"]), float(r["p50"]), float(r["p95"]))
        series[c] = pts
    ns = sorted(ns)
    vmax = max(v[2] for c in d.catalogs for v in series[c].values()) * 1.1
    def X(n):
        return x0 + (x1 - x0) * (n - ns[0]) / (ns[-1] - ns[0])
    def Y(v):
        return y1 - (y1 - y0) * (v / vmax)
    for i in range(6):
        v = vmax * i / 5
        fig.line(x0, Y(v), x1, Y(v), COL["grid"], 1)
        fig.text(x0 - 8, Y(v) + 4, fx(v, 0), size=10, fill=COL["muted"], anchor="end")
    for n in ns:
        fig.text(X(n), y1 + 18, str(n), size=11, anchor="middle")
    fig.text((x0 + x1) / 2, y1 + 36, "kits carried N", size=11, fill=COL["muted"], anchor="middle")
    fig.text(x0 - 46, (y0 + y1) / 2, "CU / removal", size=11, fill=COL["muted"], anchor="middle")
    # p05..p95 band for catalog A, then p50 lines for both.
    a = d.catalogs[0]
    band_top = [(X(n), Y(series[a][n][2])) for n in ns]
    band_bot = [(X(n), Y(series[a][n][0])) for n in reversed(ns)]
    fig.polygon(band_top + band_bot, COL["band"], op=0.35)
    for i, c in enumerate(d.catalogs):
        col = COL["A"] if i == 0 else COL["B"]
        fig.polyline([(X(n), Y(series[c][n][1])) for n in ns], col, 2.4)
        for n in ns:
            fig.circle(X(n), Y(series[c][n][1]), 2.6, col)
    # mark the minimum (turn) for catalog A.
    nmin = min(ns, key=lambda n: series[a][n][1])
    fig.line(X(nmin), y0, X(nmin), y1, COL["accent"], 1.3, dash="5,4")
    vmin = series[a][nmin][1]
    ratio = None
    for r in d.wp6_rows(a, "amortization", "cost_per_removal_ratio_to_n1"):
        if int(r["n_kits"]) == nmin:
            ratio = float(r["p50"])
    fig.text(X(nmin) + 6, y0 + 18,
             f"N={nmin}: min {fx(vmin,1)} CU"
             + (f" ({fx(ratio,3)}x of N={ns[0]})" if ratio is not None else ""),
             size=11, fill=COL["accent"])
    fig.text(X(nmin) + 6, y0 + 34, "Delta-v-limited; curve turns up beyond",
             size=10, fill=COL["muted"])
    # legend
    for i, c in enumerate(d.catalogs):
        col = COL["A"] if i == 0 else COL["B"]
        fig.rect(x0 + 10 + i * 190, y0 + 6, 12, 12, col)
        fig.text(x0 + 28 + i * 190, y0 + 16, short(c), size=11)
    fig.caption([
        "line = p50 cost/removal; shaded = catalog-A p05..p95. Ratio-to-N1 read from CSV.",
        f"source: wp6_cost_summary.csv  schema {d.wp6_schema}  seed {d.seed}  "
        f"(WP6 re-runs the WP5 engine; relative cost units CU)",
    ])
    write_text(out, fig.svg())


def chart_tornado(d, out):
    """Cost/removal sensitivity, ranked (already sorted in the CSV)."""
    fig = Fig(680, 420, "WP6 tornado: cost/removal sensitivity (+/-30%)")
    rows = d.wp6_rows(record_type="tornado", metric="cost_per_removal_swing")
    catname = rows[0]["catalog"] if rows else ""
    x0, y1 = 220, 350
    xmax = 600
    vmax = max(float(r["estimate"]) for r in rows) * 1.15 or 1.0
    bh, gap = 34, 16
    y = 70
    for r in rows:
        w = (xmax - x0) * (float(r["estimate"]) / vmax)
        fig.rect(x0, y, w, bh, COL["accent"], op=0.85)
        fig.text(x0 - 8, y + bh / 2 + 4, r["param"], size=12, anchor="end")
        fig.text(x0 + w + 6, y + bh / 2 + 4, fx(float(r["estimate"]), 2) + " CU", size=11)
        y += bh + gap
    fig.line(x0, 60, x0, y1, COL["axis"], 1.2)
    fig.text(x0, y1 + 18, "0", size=10, fill=COL["muted"], anchor="middle")
    fig.text(xmax, y1 + 18, "swing [CU/removal]", size=10, fill=COL["muted"], anchor="end")
    fig.caption([
        "one-at-a-time +/-30% perturbation of each cost parameter; ranked by swing.",
        f"source: wp6_cost_summary.csv  schema {d.wp6_schema}  seed {d.seed}  "
        f"catalog {short(catname)}",
    ])
    write_text(out, fig.svg())


def chart_fom_weightings(d, out):
    """FoM under two weightings + the band-weight flip (T5)."""
    fig = Fig(680, 460, "WP6 FoM under two weightings (T5 band-priority flip)")
    weightings = ["spatial_density", "criticality"]
    x0, y0, x1, y1 = 70, 60, 620, 320
    axes(fig, x0, y0, x1, y1)
    fom = {}
    bw = {}
    for c in d.catalogs:
        for w in weightings:
            for r in d.wp6_rows(c, "fom", "fom"):
                if r["weighting"] == w:
                    fom[(c, w)] = (float(r["p05"]), float(r["p50"]), float(r["p95"]))
            for r in d.wp6_rows(c, "fom", "band_weight"):
                if r["weighting"] == w:
                    bw[(c, w)] = float(r["estimate"])
    vmax = max(v[2] for v in fom.values()) * 1.15
    def Y(v):
        return y1 - (y1 - y0) * (v / vmax)
    for i in range(6):
        v = vmax * i / 5
        fig.line(x0, Y(v), x1, Y(v), COL["grid"], 1)
        fig.text(x0 - 8, Y(v) + 4, fx(v, 0), size=10, fill=COL["muted"], anchor="end")
    fig.text(x0 - 46, (y0 + y1) / 2, "FoM [kg/CU]", size=11, fill=COL["muted"], anchor="middle")
    gw = (x1 - x0) / len(weightings)
    for gi, w in enumerate(weightings):
        gx = x0 + gw * gi
        fig.text(gx + gw / 2, y1 + 18, w, size=12, anchor="middle")
        for ci, c in enumerate(d.catalogs):
            p05, p50, p95 = fom[(c, w)]
            col = COL["A"] if ci == 0 else COL["B"]
            bx = gx + gw * (0.25 + 0.4 * ci)
            wbar = gw * 0.28
            fig.rect(bx - wbar / 2, Y(p50), wbar, y1 - Y(p50), col, op=0.85)
            fig.line(bx, Y(p95), bx, Y(p05), COL["ink"], 1.3)
            fig.text(bx, Y(p50) - 6, fx(p50, 1), size=10, anchor="middle")
            fig.text(bx, Y(p50) - 19, "w=" + fx(bw[(c, w)], 2), size=10,
                     anchor="middle", fill=COL["accent"])
    # legend
    for ci, c in enumerate(d.catalogs):
        col = COL["A"] if ci == 0 else COL["B"]
        fig.rect(x0 + 10 + ci * 200, y0 + 6, 12, 12, col)
        fig.text(x0 + 28 + ci * 200, y0 + 16, short(c), size=11)
    a, b = d.catalogs[0], d.catalogs[1]
    fig.caption([
        f"bar = FoM p50, whisker = p05..p95; w = band weight w(h). T5 flip: spatial ranks "
        f"{short(b)} band > {short(a)} band (w {fx(bw[(b,'spatial_density')],2)} > "
        f"{fx(bw[(a,'spatial_density')],2)}), criticality flips it "
        f"({fx(bw[(a,'criticality')],2)} > {fx(bw[(b,'criticality')],2)}).",
        f"absolute FoM is mass-dominated ({short(a)} > {short(b)} under both). weightings are illustrative (cite on fill).",
        f"source: wp6_cost_summary.csv  schema {d.wp6_schema}  seed {d.seed}",
    ])
    write_text(out, fig.svg())


def chart_decay(d, out):
    """WP3 sail area vs decay years, catalog A/B, solar min..max band, 25-yr line."""
    fig = Fig(700, 470, "WP3 sail-only decay: area vs years (solar min..max band)")
    if not d.wp3:
        fig.text(40, 240, "wp3_decay_trade.csv not found (run decay_trade).", size=13, fill=COL["bad"])
        write_text(out, fig.svg())
        return
    schema = d.wp3[0]["schema_version"]
    x0, y0, x1, y1 = 80, 60, 640, 360
    axes(fig, x0, y0, x1, y1)
    import math
    series = {}
    guideline = 25.0
    area25 = {}
    for r in d.wp3:
        if r["record_type"] == "decay_years":
            series.setdefault(r["catalog"], []).append(
                (float(r["sail_area_m2"]), float(r["value_solar_max"]), float(r["value_solar_min"])))
        elif r["record_type"] == "guideline":
            guideline = float(r["value_solar_max"])
        elif r["record_type"] == "area_for_25yr":
            area25[r["catalog"]] = (float(r["value_solar_max"]), float(r["value_solar_min"]))
    for c in series:
        series[c].sort()
    areas = sorted({a for c in series for a, _, _ in series[c]})
    ymax = max(v for c in series for _, _, v in series[c])
    amin, amax = areas[0], areas[-1]
    def X(a):
        return x0 + (x1 - x0) * (math.log10(a) - math.log10(amin)) / (math.log10(amax) - math.log10(amin))
    def Y(v):
        return y1 - (y1 - y0) * (math.log10(max(v, 0.1)) - math.log10(0.1)) / (math.log10(ymax) - math.log10(0.1))
    # log y gridlines at decades
    dec = 0.1
    while dec <= ymax:
        fig.line(x0, Y(dec), x1, Y(dec), COL["grid"], 1)
        fig.text(x0 - 8, Y(dec) + 4, ("%g" % dec), size=10, fill=COL["muted"], anchor="end")
        dec *= 10
    for a in areas:
        fig.text(X(a), y1 + 16, ("%g" % a), size=10, fill=COL["muted"], anchor="middle")
    fig.text((x0 + x1) / 2, y1 + 34, "sail area [m^2] (log)", size=11, fill=COL["muted"], anchor="middle")
    fig.text(x0 - 54, (y0 + y1) / 2, "years to 180 km (log)", size=11, fill=COL["muted"], anchor="middle")
    # 25-year guideline line
    fig.line(x0, Y(guideline), x1, Y(guideline), COL["bad"], 1.6, dash="6,4")
    fig.text(x1 - 4, Y(guideline) - 5, f"{fx(guideline,0)}-yr guideline", size=11,
             fill=COL["bad"], anchor="end")
    for i, c in enumerate(d.catalogs if d.catalogs[0] in series else list(series)):
        if c not in series:
            continue
        col = COL["A"] if i == 0 else COL["B"]
        bcol = COL["band"] if i == 0 else COL["band2"]
        top = [(X(a), Y(vmn)) for a, _, vmn in series[c]]      # solar min = slow = many yrs
        bot = [(X(a), Y(vmx)) for a, vmx, _ in reversed(series[c])]  # solar max = fast
        fig.polygon(top + bot, bcol, op=0.35)
        fig.polyline([(X(a), Y(vmx)) for a, vmx, _ in series[c]], col, 1.6)
        fig.polyline([(X(a), Y(vmn)) for a, _, vmn in series[c]], col, 1.6)
    # legend + area-for-25yr annotation
    lx = x0 + 10
    for i, c in enumerate(d.catalogs):
        if c not in series:
            continue
        col = COL["A"] if i == 0 else COL["B"]
        fig.rect(lx, y0 + 6, 12, 12, col)
        note = short(c)
        if c in area25:
            note += f"  (25-yr area {fx(area25[c][0],0)}..{fx(area25[c][1],0)} m^2)"
        fig.text(lx + 17, y0 + 16, note, size=11)
        lx += 300
    fig.caption([
        "band = solar max (fast) .. solar min (slow); the heavy high SL-16 class needs an "
        "impractically large sail to meet 25 yr while the light SL-8 class closes (motivates T1).",
        f"source: wp3_decay_trade.csv  schema {schema}  seed n/a (deterministic decay model)",
    ])
    write_text(out, fig.svg())


def make_dashboard(d, out_dir, files):
    rows = "\n".join(
        f'    <figure><img src="{fn}" alt="{esc(title)}" width="100%"/>'
        f'<figcaption>{esc(title)}</figcaption></figure>'
        for title, fn in files)
    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"/>
<title>ADSC - WP7a static report page</title>
<style>
  body {{ font-family: sans-serif; color: #222; margin: 24px; background:#fafafa; }}
  h1 {{ font-size: 20px; }} p {{ color:#555; max-width:60em; }}
  .grid {{ display:flex; flex-wrap:wrap; gap:20px; }}
  figure {{ margin:0; width:min(680px,100%); background:#fff; border:1px solid #e2e2e2;
            border-radius:8px; padding:10px; box-sizing:border-box; }}
  figcaption {{ font-size:13px; color:#444; margin-top:6px; }}
  .meta {{ font-size:12px; color:#777; }}
  .warn {{ color:#b23b3b; }}
</style></head><body>
<h1>ADSC - Visualization Pack (WP7a) - static report page</h1>
<p class="meta">Regenerated from committed CSVs by <code>tools/viz/make_viz.py</code>
(Python 3 stdlib only). Source: wp5_campaign_summary.csv (schema {d.wp5_schema}),
wp6_cost_summary.csv (schema {d.wp6_schema}), wp3_decay_trade.csv. Master seed {d.seed},
N={d.n_runs}. No timestamps; every number read from the CSVs.</p>
<p class="warn">Keep-out figure: simplified research visualization, not a flight safety certificate.</p>
<div class="grid">
{rows}
</div>
</body></html>
"""
    write_text(os.path.join(out_dir, "wp5_dashboard.html"), html)


def main():
    if len(sys.argv) < 2:
        print("usage: make_viz.py <repo_root> [out_dir]", file=sys.stderr)
        return 2
    repo = sys.argv[1]
    gen = os.path.join(repo, "generated")
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(gen, "viz")
    os.makedirs(out_dir, exist_ok=True)
    d = Data(gen)

    def p(name):
        return os.path.join(out_dir, name)

    chart_outcomes(d, p("wp5_outcomes.svg"))
    chart_percentiles(d, "dv_used_m_per_s", "WP5 Delta-v used per mission (p05/p50/p95)",
                      "m/s", p("wp5_dv_percentiles.svg"), dp=1)
    chart_percentiles(d, "removals_per_mission", "WP5 removals per mission (p05/p50/p95)",
                      "count", p("wp5_removals_percentiles.svg"), dp=2)
    chart_percentiles(d, "sync_arrival_time_s", "WP5 sync arrival time (p05/p50/p95)",
                      "s", p("wp5_sync_time_percentiles.svg"), dp=2)
    chart_keepout_rate(d, p("wp5_keepout_rate.svg"))
    chart_amortization(d, p("wp6_amortization.svg"))
    chart_tornado(d, p("wp6_tornado.svg"))
    chart_fom_weightings(d, p("wp6_fom_weightings.svg"))
    chart_decay(d, p("wp3_decay_trade.svg"))

    files = [
        ("WP5 mission outcomes", "wp5_outcomes.svg"),
        ("WP5 Delta-v used (p05/p50/p95)", "wp5_dv_percentiles.svg"),
        ("WP5 removals per mission (p05/p50/p95)", "wp5_removals_percentiles.svg"),
        ("WP5 sync arrival time (p05/p50/p95)", "wp5_sync_time_percentiles.svg"),
        ("WP5 keep-out violation rate (+ Wilson 95% CI)", "wp5_keepout_rate.svg"),
        ("WP6 amortization curve", "wp6_amortization.svg"),
        ("WP6 tornado sensitivity", "wp6_tornado.svg"),
        ("WP6 FoM under two weightings (T5)", "wp6_fom_weightings.svg"),
        ("WP3 sail-only decay trade", "wp3_decay_trade.svg"),
    ]
    make_dashboard(d, out_dir, files)
    print("[WP7a] wrote %d SVGs + wp5_dashboard.html to %s" % (len(files), out_dir))


if __name__ == "__main__":
    sys.exit(main())
