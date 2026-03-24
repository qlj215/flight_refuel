#!/usr/bin/env python3
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parent
REPORT_PATH = ROOT / "ab_report.json"


def read_json(p: Path):
    return json.loads(p.read_text(encoding="utf-8"))


def parse_kv_file(p: Path) -> Dict[str, float]:
    out = {}
    for ln in p.read_text(encoding="utf-8").splitlines():
        if "=" in ln:
            k, v = ln.split("=", 1)
            k = k.strip()
            v = v.strip()
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def read_csv_xyz(p: Path) -> List[Tuple[float, float, float]]:
    lines = p.read_text(encoding="utf-8").splitlines()
    pts = []
    for ln in lines[1:]:
        if not ln.strip():
            continue
        x, y, z = ln.split(",")
        pts.append((float(x), float(y), float(z)))
    return pts


def svg_header(w: int, h: int) -> str:
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">\n'
        f'<rect x="0" y="0" width="{w}" height="{h}" fill="white"/>\n'
    )


def svg_footer() -> str:
    return "</svg>\n"


def esc(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def fmt_e(x: float) -> str:
    return f"{x:.3e}"


def color_from_log(v: float, vmin: float = -12.0, vmax: float = -6.0) -> str:
    # Blue (small) -> Red (larger)
    lv = math.log10(max(v, 1e-16))
    t = 0.0 if vmax <= vmin else (lv - vmin) / (vmax - vmin)
    t = max(0.0, min(1.0, t))
    r = int(40 + 210 * t)
    g = int(120 + 80 * (1 - t))
    b = int(220 - 170 * t)
    return f"rgb({r},{g},{b})"


def write_max_diff_bar(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    cases = [r["case"] for r in ok]
    vals = [float(r["max_abs_diff"]) for r in ok]

    w, h = 1100, 460
    ml, mr, mt, mb = 90, 40, 60, 110
    cw, ch = w - ml - mr, h - mt - mb

    # log scale baseline
    y_min_log = -12
    y_max_log = max(-6, int(math.ceil(max(math.log10(max(v, 1e-16)) for v in vals))))

    def y_of(v: float) -> float:
        lv = math.log10(max(v, 1e-16))
        t = (lv - y_min_log) / (y_max_log - y_min_log)
        return mt + ch * (1 - t)

    bar_w = cw / max(1, len(cases)) * 0.55
    gap = cw / max(1, len(cases))

    s = [svg_header(w, h)]
    s.append(f'<text x="{w/2}" y="34" text-anchor="middle" font-size="20" font-family="Arial">A/B max_abs_diff by case (log scale)</text>\n')

    # axes
    s.append(f'<line x1="{ml}" y1="{mt}" x2="{ml}" y2="{mt+ch}" stroke="black"/>\n')
    s.append(f'<line x1="{ml}" y1="{mt+ch}" x2="{ml+cw}" y2="{mt+ch}" stroke="black"/>\n')

    # y ticks
    for e in range(y_min_log, y_max_log + 1):
        y = mt + ch * (1 - (e - y_min_log) / (y_max_log - y_min_log))
        s.append(f'<line x1="{ml-5}" y1="{y:.2f}" x2="{ml+cw}" y2="{y:.2f}" stroke="#e6e6e6"/>\n')
        s.append(f'<text x="{ml-10}" y="{y+4:.2f}" text-anchor="end" font-size="11" font-family="Consolas">1e{e}</text>\n')

    # bars
    for i, (c, v) in enumerate(zip(cases, vals)):
        x = ml + gap * i + (gap - bar_w) / 2
        y = y_of(v)
        hbar = (mt + ch) - y
        s.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{bar_w:.2f}" height="{hbar:.2f}" fill="#4e79a7"/>\n')
        s.append(f'<text x="{x+bar_w/2:.2f}" y="{mt+ch+22}" text-anchor="middle" font-size="11" font-family="Arial" transform="rotate(25 {x+bar_w/2:.2f} {mt+ch+22})">{esc(c)}</text>\n')
        s.append(f'<text x="{x+bar_w/2:.2f}" y="{y-6:.2f}" text-anchor="middle" font-size="10" font-family="Consolas">{fmt_e(v)}</text>\n')

    s.append(svg_footer())
    out_path.write_text("".join(s), encoding="utf-8")


def write_heatmap(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    cases = [r["case"] for r in ok]
    metrics = list(ok[0]["diffs"].keys()) if ok else []

    w = 1320
    cell_w = 170
    cell_h = 30
    ml = 260
    mt = 80
    mr = 30
    mb = 40
    h = mt + cell_h * len(metrics) + mb

    s = [svg_header(w, h)]
    s.append(f'<text x="{w/2}" y="34" text-anchor="middle" font-size="20" font-family="Arial">Absolute difference heatmap (model vs project)</text>\n')

    # col headers
    for j, c in enumerate(cases):
        x = ml + j * cell_w + cell_w / 2
        s.append(f'<text x="{x:.2f}" y="{mt-15}" text-anchor="middle" font-size="12" font-family="Arial">{esc(c)}</text>\n')

    # rows
    for i, m in enumerate(metrics):
        y = mt + i * cell_h
        s.append(f'<text x="{ml-10}" y="{y+20:.2f}" text-anchor="end" font-size="12" font-family="Consolas">{esc(m)}</text>\n')
        for j, c in enumerate(cases):
            v = float(next(r for r in ok if r["case"] == c)["diffs"][m]["abs_diff"])
            x = ml + j * cell_w
            fill = color_from_log(v)
            s.append(f'<rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" fill="{fill}" stroke="white"/>\n')
            txt_color = "black"
            s.append(f'<text x="{x+cell_w/2:.2f}" y="{y+20:.2f}" text-anchor="middle" font-size="10" font-family="Consolas" fill="{txt_color}">{fmt_e(v)}</text>\n')

    # legend
    lx, ly = 70, h - 28
    s.append(f'<text x="{lx}" y="{ly-8}" font-size="11" font-family="Arial">color ~ log10(abs_diff)</text>\n')
    for k in range(0, 121):
        t = k / 120
        lv = -12 + t * 6
        v = 10 ** lv
        s.append(f'<rect x="{lx + k*2}" y="{ly}" width="2" height="12" fill="{color_from_log(v)}" stroke="none"/>\n')
    s.append(f'<text x="{lx}" y="{ly+26}" font-size="10" font-family="Consolas">1e-12</text>\n')
    s.append(f'<text x="{lx+240}" y="{ly+26}" text-anchor="end" font-size="10" font-family="Consolas">1e-6</text>\n')

    s.append(svg_footer())
    out_path.write_text("".join(s), encoding="utf-8")


def map_xy(points: List[Tuple[float, float]], W: int, H: int, pad: int = 50):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    dx = max(1.0, xmax - xmin)
    dy = max(1.0, ymax - ymin)
    span = max(dx, dy)
    cx = (xmin + xmax) / 2
    cy = (ymin + ymax) / 2
    half = span / 2 * 1.08

    def f(p):
        x, y = p
        nx = (x - (cx - half)) / (2 * half)
        ny = (y - (cy - half)) / (2 * half)
        sx = pad + nx * (W - 2 * pad)
        sy = H - (pad + ny * (H - 2 * pad))
        return sx, sy

    return f


def polyline_svg(points: List[Tuple[float, float]], color: str, width: float = 2.0) -> str:
    d = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="{width}"/>\n'


def marker_circle(x: float, y: float, color: str, r: float = 4.0) -> str:
    return f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{color}" stroke="white" stroke-width="1"/>\n'


def marker_cross(x: float, y: float, color: str, s: float = 5.0) -> str:
    return (
        f'<line x1="{x-s:.2f}" y1="{y-s:.2f}" x2="{x+s:.2f}" y2="{y+s:.2f}" stroke="{color}" stroke-width="2"/>\n'
        f'<line x1="{x-s:.2f}" y1="{y+s:.2f}" x2="{x+s:.2f}" y2="{y-s:.2f}" stroke="{color}" stroke-width="2"/>\n'
    )


def write_case_trajectory(case_dir: Path, case_name: str, max_abs: float):
    out_json = read_json(case_dir / "out_proj" / "output.json")
    model_kv = parse_kv_file(case_dir / "output_model.txt")

    tanker_pts = read_csv_xyz(case_dir / "out_proj" / "tanker01.csv")
    recv_pts = read_csv_xyz(case_dir / "out_proj" / "receiver01.csv")

    tanker_xy = [(x, y) for x, y, _ in tanker_pts]
    recv_xy = [(x, y) for x, y, _ in recv_pts]

    proj_meet = tuple(out_json["meeting_point"])
    proj_tm = tuple(out_json["phase"]["tanker_midpoint"])
    proj_rm = tuple(out_json["phase"]["receiver_midpoint"])

    model_meet = (float(model_kv["meet_x_m"]), float(model_kv["meet_y_m"]))
    model_tm = (float(model_kv["phase1_tanker_end_x_m"]), float(model_kv["phase1_tanker_end_y_m"]))
    model_rm = (float(model_kv["phase1_receiver_end_x_m"]), float(model_kv["phase1_receiver_end_y_m"]))

    all_xy = tanker_xy + recv_xy + [proj_meet, proj_tm, proj_rm, model_meet, model_tm, model_rm]

    W, H = 980, 660
    tf = map_xy(all_xy, W, H, pad=70)

    s = [svg_header(W, H)]
    s.append(f'<text x="{W/2}" y="32" text-anchor="middle" font-size="20" font-family="Arial">Trajectory view: {esc(case_name)}</text>\n')
    s.append(f'<text x="{W/2}" y="54" text-anchor="middle" font-size="12" font-family="Consolas">max_abs_diff={fmt_e(max_abs)}</text>\n')

    s.append(polyline_svg([tf(p) for p in tanker_xy], "#1f77b4", 2.2))
    s.append(polyline_svg([tf(p) for p in recv_xy], "#2ca02c", 2.2))

    # starts
    s.append(marker_circle(*tf(tanker_xy[0]), "#1f77b4", 4.8))
    s.append(marker_circle(*tf(recv_xy[0]), "#2ca02c", 4.8))

    # project markers
    s.append(marker_circle(*tf(proj_meet), "#d62728", 5.2))
    s.append(marker_circle(*tf(proj_tm), "#9467bd", 4.2))
    s.append(marker_circle(*tf(proj_rm), "#17becf", 4.2))

    # model markers
    s.append(marker_cross(*tf(model_meet), "#ff7f0e", 5.2))
    s.append(marker_cross(*tf(model_tm), "#8c564b", 4.3))
    s.append(marker_cross(*tf(model_rm), "#e377c2", 4.3))

    # legend
    lx, ly = 70, H - 84
    legend = [
        ("#1f77b4", "line", "project tanker trajectory"),
        ("#2ca02c", "line", "project receiver trajectory"),
        ("#d62728", "dot", "project meeting point"),
        ("#ff7f0e", "cross", "model meeting point"),
        ("#9467bd", "dot", "project tanker phase midpoint"),
        ("#8c564b", "cross", "model tanker phase midpoint"),
    ]
    y = ly
    for color, kind, name in legend:
        if kind == "line":
            s.append(f'<line x1="{lx}" y1="{y}" x2="{lx+22}" y2="{y}" stroke="{color}" stroke-width="2"/>\n')
        elif kind == "dot":
            s.append(f'<circle cx="{lx+11}" cy="{y}" r="4" fill="{color}"/>\n')
        else:
            s.append(marker_cross(lx + 11, y, color, 4))
        s.append(f'<text x="{lx+30}" y="{y+4}" font-size="11" font-family="Arial">{esc(name)}</text>\n')
        y += 16

    s.append(svg_footer())
    (ROOT / f"viz_trajectory_{case_name}.svg").write_text("".join(s), encoding="utf-8")


def write_summary_md(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    lines = []
    lines.append("# A/B Visualization Summary\n")
    lines.append("\n")
    lines.append("- Source: `ab_report.json`\n")
    lines.append("- Cases: {}\n".format(len(ok)))
    lines.append("\n")
    lines.append("## Overview\n")
    lines.append("\n")
    lines.append("![](viz_max_abs_diff.svg)\n")
    lines.append("\n")
    lines.append("![](viz_abs_diff_heatmap.svg)\n")
    lines.append("\n")
    lines.append("## Case table\n")
    lines.append("\n")
    lines.append("| case | method(model/project) | method_match | max_abs_diff |\n")
    lines.append("|---|---|---:|---:|\n")
    for r in ok:
        lines.append(
            f"| {r['case']} | {r['meeting_method_model']}/{r['meeting_method_project']} | {str(r['method_match']).lower()} | {fmt_e(float(r['max_abs_diff']))} |\n"
        )
    lines.append("\n")
    lines.append("## Trajectory visualizations\n\n")
    for r in ok:
        c = r["case"]
        lines.append(f"### {c}\n\n")
        lines.append(f"![](viz_trajectory_{c}.svg)\n\n")

    out_path.write_text("".join(lines), encoding="utf-8")


def main():
    report = read_json(REPORT_PATH)
    ok = [r for r in report if r.get("status") == "ok"]
    if not ok:
        raise SystemExit("No successful cases in report.")

    write_max_diff_bar(report, ROOT / "viz_max_abs_diff.svg")
    write_heatmap(report, ROOT / "viz_abs_diff_heatmap.svg")

    for r in ok:
        write_case_trajectory(ROOT / r["case"], r["case"], float(r["max_abs_diff"]))

    write_summary_md(report, ROOT / "viz_summary.md")
    print("Visualization generated in", ROOT)


if __name__ == "__main__":
    main()
