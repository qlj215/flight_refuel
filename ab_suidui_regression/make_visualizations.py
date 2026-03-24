#!/usr/bin/env python3
import json
import math
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parent
REPORT_PATH = ROOT / "ab_report.json"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv_xyz(path: Path) -> List[Tuple[float, float, float]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    pts = []
    for line in lines[1:]:
        if not line.strip():
            continue
        x, y, z = line.split(",")
        pts.append((float(x), float(y), float(z)))
    return pts


def fmt_e(v: float) -> str:
    return f"{v:.3e}"


def esc(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def svg_header(w: int, h: int) -> str:
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" '
        f'viewBox="0 0 {w} {h}">\n'
        f'<rect x="0" y="0" width="{w}" height="{h}" fill="white"/>\n'
    )


def svg_footer() -> str:
    return "</svg>\n"


def color_from_log(v: float, vmin: float = -12.0, vmax: float = 1.0) -> str:
    lv = math.log10(max(v, 1e-16))
    t = 0.0 if vmax <= vmin else (lv - vmin) / (vmax - vmin)
    t = max(0.0, min(1.0, t))
    r = int(40 + 210 * t)
    g = int(120 + 80 * (1 - t))
    b = int(220 - 170 * t)
    return f"rgb({r},{g},{b})"


def map_xy(points: List[Tuple[float, float]], w: int, h: int, pad: int = 55):
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

    def tf(p):
        x, y = p
        nx = (x - (cx - half)) / (2 * half)
        ny = (y - (cy - half)) / (2 * half)
        sx = pad + nx * (w - 2 * pad)
        sy = h - (pad + ny * (h - 2 * pad))
        return sx, sy

    return tf


def write_max_diff_bar(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    cases = [r["case"] for r in ok]
    vals = [float(r["max_abs_diff"]) for r in ok]

    w, h = 1100, 460
    ml, mr, mt, mb = 90, 40, 60, 110
    cw, ch = w - ml - mr, h - mt - mb

    y_min_log = -12
    y_max_log = max(1, int(math.ceil(max(math.log10(max(v, 1e-16)) for v in vals))))

    def y_of(v: float) -> float:
        lv = math.log10(max(v, 1e-16))
        t = (lv - y_min_log) / (y_max_log - y_min_log)
        return mt + ch * (1 - t)

    bar_w = cw / max(1, len(cases)) * 0.55
    gap = cw / max(1, len(cases))

    s = [svg_header(w, h)]
    s.append(
        f'<text x="{w/2}" y="34" text-anchor="middle" font-size="20" font-family="Arial">'
        f'Suidui A/B max_abs_diff by case (log scale)</text>\n'
    )

    s.append(f'<line x1="{ml}" y1="{mt}" x2="{ml}" y2="{mt+ch}" stroke="black"/>\n')
    s.append(f'<line x1="{ml}" y1="{mt+ch}" x2="{ml+cw}" y2="{mt+ch}" stroke="black"/>\n')

    for e in range(y_min_log, y_max_log + 1):
        y = mt + ch * (1 - (e - y_min_log) / (y_max_log - y_min_log))
        s.append(f'<line x1="{ml-5}" y1="{y:.2f}" x2="{ml+cw}" y2="{y:.2f}" stroke="#e6e6e6"/>\n')
        s.append(
            f'<text x="{ml-10}" y="{y+4:.2f}" text-anchor="end" '
            f'font-size="11" font-family="Consolas">1e{e}</text>\n'
        )

    for i, (c, v) in enumerate(zip(cases, vals)):
        x = ml + gap * i + (gap - bar_w) / 2
        y = y_of(v)
        hbar = (mt + ch) - y
        s.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{bar_w:.2f}" height="{hbar:.2f}" fill="#4e79a7"/>\n')
        s.append(
            f'<text x="{x+bar_w/2:.2f}" y="{mt+ch+22}" text-anchor="middle" '
            f'font-size="11" font-family="Arial" transform="rotate(25 {x+bar_w/2:.2f} {mt+ch+22})">{esc(c)}</text>\n'
        )
        s.append(
            f'<text x="{x+bar_w/2:.2f}" y="{y-6:.2f}" text-anchor="middle" '
            f'font-size="10" font-family="Consolas">{fmt_e(v)}</text>\n'
        )

    s.append(svg_footer())
    out_path.write_text("".join(s), encoding="utf-8")


def write_heatmap(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    cases = [r["case"] for r in ok]
    metrics = list(ok[0]["diffs"].keys()) if ok else []

    cell_w = 165
    cell_h = 30
    ml = 280
    mt = 85
    w = ml + cell_w * len(cases) + 40
    h = mt + cell_h * len(metrics) + 50

    s = [svg_header(w, h)]
    s.append(
        f'<text x="{w/2}" y="34" text-anchor="middle" font-size="20" font-family="Arial">'
        f'Suidui absolute difference heatmap (model vs project)</text>\n'
    )

    for j, c in enumerate(cases):
        x = ml + j * cell_w + cell_w / 2
        s.append(f'<text x="{x:.2f}" y="{mt-16}" text-anchor="middle" font-size="12" font-family="Arial">{esc(c)}</text>\n')

    for i, m in enumerate(metrics):
        y = mt + i * cell_h
        s.append(f'<text x="{ml-10}" y="{y+20:.2f}" text-anchor="end" font-size="12" font-family="Consolas">{esc(m)}</text>\n')
        for j, c in enumerate(cases):
            v = float(next(r for r in ok if r["case"] == c)["diffs"][m]["abs_diff"])
            x = ml + j * cell_w
            s.append(f'<rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" fill="{color_from_log(v)}" stroke="white"/>\n')
            s.append(
                f'<text x="{x+cell_w/2:.2f}" y="{y+20:.2f}" text-anchor="middle" '
                f'font-size="10" font-family="Consolas">{fmt_e(v)}</text>\n'
            )

    lx, ly = 72, h - 26
    s.append(f'<text x="{lx}" y="{ly-8}" font-size="11" font-family="Arial">color ~ log10(abs_diff)</text>\n')
    for k in range(0, 121):
        t = k / 120
        lv = -12 + t * 13
        v = 10 ** lv
        s.append(f'<rect x="{lx + k*2}" y="{ly}" width="2" height="12" fill="{color_from_log(v)}"/>\n')
    s.append(f'<text x="{lx}" y="{ly+24}" font-size="10" font-family="Consolas">1e-12</text>\n')
    s.append(f'<text x="{lx+240}" y="{ly+24}" text-anchor="end" font-size="10" font-family="Consolas">1e1</text>\n')

    s.append(svg_footer())
    out_path.write_text("".join(s), encoding="utf-8")


def polyline_svg(points: List[Tuple[float, float]], color: str, width: float = 2.0) -> str:
    pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}"/>\n'


def dot(x: float, y: float, color: str, r: float = 4.0) -> str:
    return f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{color}" stroke="white" stroke-width="1"/>\n'


def cross(x: float, y: float, color: str, s: float = 5.0) -> str:
    return (
        f'<line x1="{x-s:.2f}" y1="{y-s:.2f}" x2="{x+s:.2f}" y2="{y+s:.2f}" stroke="{color}" stroke-width="2"/>\n'
        f'<line x1="{x-s:.2f}" y1="{y+s:.2f}" x2="{x+s:.2f}" y2="{y-s:.2f}" stroke="{color}" stroke-width="2"/>\n'
    )


def write_case_trajectory(report_item: dict):
    case = report_item["case"]
    case_dir = ROOT / case

    out_json = read_json(case_dir / "out_proj" / "output.json")
    one = out_json["results"][0] if "results" in out_json else out_json

    tanker_pts = read_csv_xyz(case_dir / "out_proj" / "tanker01.csv")
    recv_pts = read_csv_xyz(case_dir / "out_proj" / "receiver01.csv")

    tanker_xy = [(x, y) for x, y, _ in tanker_pts]
    recv_xy = [(x, y) for x, y, _ in recv_pts]

    proj_meet = tuple(one["meeting_point"])
    proj_refuel = tuple(one["debug"]["refuel_point"])
    model_meet = (
        float(report_item["diffs"]["meeting_x"]["model"]),
        float(report_item["diffs"]["meeting_y"]["model"]),
    )
    model_refuel = (
        float(report_item["diffs"]["refuel_x"]["model"]),
        float(report_item["diffs"]["refuel_y"]["model"]),
    )

    goal = tuple(one["debug"].get("goal_point", recv_xy[-1]))

    all_xy = tanker_xy + recv_xy + [proj_meet, proj_refuel, model_meet, model_refuel, goal]
    tf = map_xy(all_xy, 980, 660, pad=70)

    s = [svg_header(980, 660)]
    s.append(f'<text x="490" y="32" text-anchor="middle" font-size="20" font-family="Arial">Suidui trajectory: {esc(case)}</text>\n')
    s.append(f'<text x="490" y="54" text-anchor="middle" font-size="12" font-family="Consolas">max_abs_diff={fmt_e(float(report_item["max_abs_diff"]))}</text>\n')

    s.append(polyline_svg([tf(p) for p in tanker_xy], "#1f77b4", 2.2))
    s.append(polyline_svg([tf(p) for p in recv_xy], "#2ca02c", 2.2))

    s.append(dot(*tf(tanker_xy[0]), "#1f77b4", 4.6))
    s.append(dot(*tf(recv_xy[0]), "#2ca02c", 4.6))
    s.append(dot(*tf(goal), "#111111", 4.8))

    s.append(dot(*tf(proj_meet), "#d62728", 5.2))
    s.append(cross(*tf(model_meet), "#ff7f0e", 5.2))
    s.append(dot(*tf(proj_refuel), "#9467bd", 4.8))
    s.append(cross(*tf(model_refuel), "#8c564b", 4.8))

    lx, ly = 70, 560
    legend = [
        ("line", "#1f77b4", "project tanker trajectory"),
        ("line", "#2ca02c", "project receiver trajectory"),
        ("dot", "#d62728", "project meeting point"),
        ("cross", "#ff7f0e", "model meeting point"),
        ("dot", "#9467bd", "project refuel point"),
        ("cross", "#8c564b", "model refuel point"),
        ("dot", "#111111", "goal point"),
    ]
    y = ly
    for kind, color, text in legend:
        if kind == "line":
            s.append(f'<line x1="{lx}" y1="{y}" x2="{lx+22}" y2="{y}" stroke="{color}" stroke-width="2"/>\n')
        elif kind == "dot":
            s.append(dot(lx + 11, y, color, 4.0))
        else:
            s.append(cross(lx + 11, y, color, 4.0))
        s.append(f'<text x="{lx+30}" y="{y+4}" font-size="11" font-family="Arial">{esc(text)}</text>\n')
        y += 16

    s.append(svg_footer())
    (ROOT / f"viz_trajectory_{case}.svg").write_text("".join(s), encoding="utf-8")


def write_summary_md(report: List[dict], out_path: Path):
    ok = [r for r in report if r.get("status") == "ok"]
    lines = []
    lines.append("# Suidui A/B Visualization Summary\n\n")
    lines.append("- Source: `ab_report.json`\n")
    lines.append(f"- Cases: {len(ok)}\n\n")

    lines.append("## Overview\n\n")
    lines.append("![](viz_max_abs_diff.svg)\n\n")
    lines.append("![](viz_abs_diff_heatmap.svg)\n\n")

    lines.append("## Case table\n\n")
    lines.append("| case | max_abs_diff |\n")
    lines.append("|---|---:|\n")
    for r in ok:
        lines.append(f"| {r['case']} | {fmt_e(float(r['max_abs_diff']))} |\n")
    lines.append("\n")

    lines.append("## Trajectory visualizations\n\n")
    for r in ok:
        case = r["case"]
        lines.append(f"### {case}\n\n")
        lines.append(f"![](viz_trajectory_{case}.svg)\n\n")

    out_path.write_text("".join(lines), encoding="utf-8")


def main():
    report = read_json(REPORT_PATH)
    ok = [r for r in report if r.get("status") == "ok"]
    if not ok:
        raise SystemExit("No successful cases found in ab_report.json")

    write_max_diff_bar(report, ROOT / "viz_max_abs_diff.svg")
    write_heatmap(report, ROOT / "viz_abs_diff_heatmap.svg")

    for item in ok:
        write_case_trajectory(item)

    write_summary_md(report, ROOT / "viz_summary.md")
    print("Visualization generated in", ROOT)


if __name__ == "__main__":
    main()
