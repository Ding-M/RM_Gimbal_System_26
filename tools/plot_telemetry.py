#!/usr/bin/env python3
import csv
import html
import math
import sys
from pathlib import Path


WIDTH = 1100
HEIGHT = 560
LEFT = 82
RIGHT = 28
TOP = 54
BOTTOM = 74
COLORS = ["#2563eb", "#f97316", "#16a34a", "#dc2626"]


def latest_csv():
    candidates = sorted(Path("build/logs").glob("telemetry_*.csv"))
    if not candidates:
        raise FileNotFoundError("未找到 build/logs/telemetry_*.csv")
    return candidates[-1]


def read_rows(path):
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise ValueError(f"{path} 是空文件")
    return rows


def to_float(value):
    if value is None or value == "":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def series_from(rows, x_key, y_key, transform=lambda value: value, skip_negative_response=False):
    points = []
    for row in rows:
        x = to_float(row.get(x_key))
        y = to_float(row.get(y_key))
        if x is None or y is None:
            continue
        y = transform(y)
        if skip_negative_response and y < 0:
            continue
        points.append((x / 1000.0, y))
    return points


def bounds(series_list):
    xs = [x for series in series_list for x, _ in series]
    ys = [y for series in series_list for _, y in series]
    if not xs or not ys:
        return 0.0, 1.0, -1.0, 1.0

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_min == x_max:
        x_max = x_min + 1.0
    if y_min == y_max:
        y_min -= 1.0
        y_max += 1.0

    y_pad = max((y_max - y_min) * 0.12, 1.0)
    return x_min, x_max, y_min - y_pad, y_max + y_pad


def nice_ticks(min_value, max_value, count=6):
    if max_value <= min_value:
        return [min_value]
    raw_step = (max_value - min_value) / max(1, count - 1)
    magnitude = 10 ** math.floor(math.log10(raw_step))
    normalized = raw_step / magnitude
    if normalized <= 1:
        step = magnitude
    elif normalized <= 2:
        step = 2 * magnitude
    elif normalized <= 5:
        step = 5 * magnitude
    else:
        step = 10 * magnitude

    start = math.ceil(min_value / step) * step
    ticks = []
    value = start
    while value <= max_value + step * 0.5:
        ticks.append(value)
        value += step
    return ticks


def format_tick(value):
    if abs(value) >= 100 or abs(value - round(value)) < 1e-6:
        return str(int(round(value)))
    return f"{value:.1f}"


def polyline(points, x_min, x_max, y_min, y_max):
    plot_w = WIDTH - LEFT - RIGHT
    plot_h = HEIGHT - TOP - BOTTOM
    result = []
    for x, y in points:
        px = LEFT + (x - x_min) / (x_max - x_min) * plot_w
        py = TOP + (y_max - y) / (y_max - y_min) * plot_h
        result.append(f"{px:.1f},{py:.1f}")
    return " ".join(result)


def make_svg(title, x_label, y_label, named_series, output_path):
    non_empty = [(name, points) for name, points in named_series if points]
    x_min, x_max, y_min, y_max = bounds([points for _, points in non_empty])
    x_ticks = nice_ticks(x_min, x_max)
    y_ticks = nice_ticks(y_min, y_max)
    plot_w = WIDTH - LEFT - RIGHT
    plot_h = HEIGHT - TOP - BOTTOM

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{WIDTH / 2}" y="30" text-anchor="middle" font-family="Arial" font-size="22" font-weight="700">{html.escape(title)}</text>',
        f'<rect x="{LEFT}" y="{TOP}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#d4d4d8"/>',
    ]

    for tick in x_ticks:
        px = LEFT + (tick - x_min) / (x_max - x_min) * plot_w
        parts.append(f'<line x1="{px:.1f}" y1="{TOP}" x2="{px:.1f}" y2="{TOP + plot_h}" stroke="#e5e7eb"/>')
        parts.append(f'<text x="{px:.1f}" y="{HEIGHT - 46}" text-anchor="middle" font-family="Arial" font-size="13" fill="#52525b">{format_tick(tick)}</text>')

    for tick in y_ticks:
        py = TOP + (y_max - tick) / (y_max - y_min) * plot_h
        parts.append(f'<line x1="{LEFT}" y1="{py:.1f}" x2="{LEFT + plot_w}" y2="{py:.1f}" stroke="#e5e7eb"/>')
        parts.append(f'<text x="{LEFT - 12}" y="{py + 4:.1f}" text-anchor="end" font-family="Arial" font-size="13" fill="#52525b">{format_tick(tick)}</text>')

    parts.append(f'<text x="{WIDTH / 2}" y="{HEIGHT - 15}" text-anchor="middle" font-family="Arial" font-size="15" fill="#27272a">{html.escape(x_label)}</text>')
    parts.append(f'<text x="20" y="{HEIGHT / 2}" text-anchor="middle" transform="rotate(-90 20 {HEIGHT / 2})" font-family="Arial" font-size="15" fill="#27272a">{html.escape(y_label)}</text>')

    for index, (name, points) in enumerate(non_empty):
        color = COLORS[index % len(COLORS)]
        points_text = polyline(points, x_min, x_max, y_min, y_max)
        dash = ' stroke-dasharray="9 7"' if index == 1 else ""
        opacity = ' stroke-opacity="0.82"' if index == 1 else ""
        parts.append(f'<polyline fill="none" stroke="{color}" stroke-width="2.6"{dash}{opacity} points="{points_text}"/>')

    legend_x = LEFT + 16
    legend_y = TOP + 22
    for index, (name, _) in enumerate(non_empty):
        color = COLORS[index % len(COLORS)]
        y = legend_y + index * 24
        dash = ' stroke-dasharray="9 7"' if index == 1 else ""
        opacity = ' stroke-opacity="0.82"' if index == 1 else ""
        parts.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 28}" y2="{y}" stroke="{color}" stroke-width="3"{dash}{opacity}/>')
        parts.append(f'<text x="{legend_x + 38}" y="{y + 5}" font-family="Arial" font-size="14" fill="#27272a">{html.escape(name)}</text>')

    if not non_empty:
        parts.append(f'<text x="{WIDTH / 2}" y="{HEIGHT / 2}" text-anchor="middle" font-family="Arial" font-size="20" fill="#71717a">没有可绘制的数据</text>')

    parts.append("</svg>")
    output_path.write_text("\n".join(parts))


def write_index(output_dir, files):
    links = "\n".join(
        f'<li><a href="{html.escape(path.name)}">{html.escape(title)}</a></li>'
        for title, path in files
    )
    content = f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Telemetry Plots</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 28px; color: #27272a; }}
    iframe {{ display: block; width: 1120px; height: 580px; border: 1px solid #e5e7eb; margin: 18px 0 34px; }}
  </style>
</head>
<body>
  <h1>Telemetry Plots</h1>
  <ul>{links}</ul>
  {''.join(f'<h2>{html.escape(title)}</h2><iframe src="{html.escape(path.name)}"></iframe>' for title, path in files)}
</body>
</html>
"""
    (output_dir / "index.html").write_text(content)


def main():
    csv_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else latest_csv()
    rows = read_rows(csv_path)

    output_dir = csv_path.parent / f"plots_{csv_path.stem}"
    output_dir.mkdir(parents=True, exist_ok=True)

    yaw_svg = output_dir / "yaw.svg"
    pitch_svg = output_dir / "pitch.svg"
    response_svg = output_dir / "response.svg"

    make_svg(
        "Yaw Tracking Response",
        "time (s)",
        "yaw (deg)",
        [
            ("target_yaw_deg", series_from(rows, "time_ms", "target_yaw_deg")),
            ("-cmd_yaw_deg", series_from(rows, "time_ms", "cmd_yaw_deg", lambda value: -value)),
        ],
        yaw_svg,
    )

    make_svg(
        "Pitch Tracking Response",
        "time (s)",
        "pitch (deg)",
        [
            ("target_pitch_deg", series_from(rows, "time_ms", "target_pitch_deg")),
            ("cmd_pitch_deg", series_from(rows, "time_ms", "cmd_pitch_deg")),
        ],
        pitch_svg,
    )

    make_svg(
        "Response Time",
        "time (s)",
        "response (ms)",
        [
            ("yaw_response_ms", series_from(rows, "time_ms", "yaw_response_ms", skip_negative_response=True)),
            ("pitch_response_ms", series_from(rows, "time_ms", "pitch_response_ms", skip_negative_response=True)),
        ],
        response_svg,
    )

    files = [
        ("Yaw Tracking Response", yaw_svg),
        ("Pitch Tracking Response", pitch_svg),
        ("Response Time", response_svg),
    ]
    write_index(output_dir, files)

    print(f"输入 CSV: {csv_path}")
    print(f"输出目录: {output_dir}")
    print(f"总览页面: {output_dir / 'index.html'}")


if __name__ == "__main__":
    main()
