"""Regenerate the figures in docs/figures from fresh flights and sweeps.

    cargo build --release -p mission --bin lander
    python docs/plots.py

Standard library only: each figure is a small SVG line chart that follows the light or
dark color scheme of the page showing it.
"""

import json
import math
import os
import subprocess
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LANDER = os.path.join(ROOT, "target", "release", "lander")
OUT = os.path.join(ROOT, "docs", "figures")

# Categorical slots 1-3 (blue, orange, aqua), light and dark steps, in fixed order.
SERIES = [("#2a78d6", "#3987e5"), ("#eb6834", "#d95926"), ("#1baf7a", "#199e70")]
STYLE = """
.root { --surface: #fcfcfb; --text: #0b0b0b; --text-2: #52514e; --muted: #898781;
        --grid: #e1e0d9; --axis: #c3c2b7; --band: #f1f0ec; %s }
@media (prefers-color-scheme: dark) {
  .root { --surface: #1a1a19; --text: #f0efec; --text-2: #c3c2b7; --muted: #898781;
          --grid: #2c2c2a; --axis: #383835; --band: #232322; %s }
}
text { font-family: system-ui, -apple-system, 'Segoe UI', sans-serif; }
""" % (
    " ".join(f"--s{i}: {light};" for i, (light, _) in enumerate(SERIES)),
    " ".join(f"--s{i}: {dark};" for i, (_, dark) in enumerate(SERIES)),
)
WIDTH, HEIGHT = 720, 320
LEFT, RIGHT, TOP, BOTTOM = 56, 120, 48, 44


def fly(*args):
    """Fly one flight and return its log."""
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "log.json")
        subprocess.run([LANDER, "fly", *args, "--save", path], check=True, capture_output=True)
        with open(path) as f:
            return json.load(f)


def sweep(scenario, runs=200):
    """Landing rate (%) at each stress level, from `lander sweep`."""
    out = subprocess.run(
        [LANDER, "sweep", scenario, "--runs", str(runs)], check=True, capture_output=True, text=True
    ).stdout
    rows = [line.split() for line in out.splitlines()[2:] if line.strip()]
    return [float(r[0]) for r in rows], [float(r[1].rstrip("%")) for r in rows]


def nice_ticks(lo, hi, count=5):
    span = hi - lo or 1.0
    raw = span / count
    magnitude = 10 ** math.floor(math.log10(raw))
    step = min((m * magnitude for m in (1, 2, 2.5, 5, 10)), key=lambda s: abs(s - raw))
    start = math.floor(lo / step) * step
    ticks = []
    value = start
    while value <= hi + 1e-9:
        if value >= lo - 1e-9:
            ticks.append(round(value, 10))
        value += step
    return ticks


def fmt(value):
    return f"{value:g}" if abs(value) < 1000 else f"{value / 1000:g}K"


def line_chart(name, title, x_label, y_label, series, bands=(), markers=(), y_range=None):
    """series: [(label, xs, ys)]; bands: [(x0, x1, label)] shaded spans; markers:
    [(x, label)] thin vertical rules."""
    xs = [x for _, sx, _ in series for x in sx]
    ys = [y for _, _, sy in series for y in sy]
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = y_range or (min(0.0, min(ys)), max(ys) * 1.05)
    plot_w, plot_h = WIDTH - LEFT - RIGHT, HEIGHT - TOP - BOTTOM
    px = lambda x: LEFT + (x - x_lo) / (x_hi - x_lo) * plot_w
    py = lambda y: TOP + plot_h - (y - y_lo) / (y_hi - y_lo) * plot_h
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" class="root" role="img">',
        f"<title>{title}</title><style>{STYLE}</style>",
        f'<rect width="{WIDTH}" height="{HEIGHT}" rx="8" fill="var(--surface)"/>',
        f'<text x="{LEFT}" y="26" font-size="15" font-weight="600" fill="var(--text)">{title}</text>',
    ]
    for x0, x1, label in bands:
        a, b = px(max(x0, x_lo)), px(min(x1, x_hi))
        parts.append(f'<rect x="{a:.1f}" y="{TOP}" width="{b - a:.1f}" height="{plot_h}" fill="var(--band)"/>')
        parts.append(
            f'<text x="{(a + b) / 2:.1f}" y="{TOP - 6}" font-size="10" text-anchor="middle" '
            f'fill="var(--muted)">{label}</text>'
        )
    for y in nice_ticks(y_lo, y_hi):
        parts.append(
            f'<line x1="{LEFT}" x2="{LEFT + plot_w}" y1="{py(y):.1f}" y2="{py(y):.1f}" '
            f'stroke="var(--grid)" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{LEFT - 8}" y="{py(y) + 4:.1f}" font-size="11" text-anchor="end" '
            f'fill="var(--muted)">{fmt(y)}</text>'
        )
    for x in nice_ticks(x_lo, x_hi, 6):
        parts.append(
            f'<text x="{px(x):.1f}" y="{TOP + plot_h + 18}" font-size="11" text-anchor="middle" '
            f'fill="var(--muted)">{fmt(x)}</text>'
        )
    parts.append(
        f'<line x1="{LEFT}" x2="{LEFT + plot_w}" y1="{TOP + plot_h}" y2="{TOP + plot_h}" '
        f'stroke="var(--axis)" stroke-width="1"/>'
    )
    for x, label in markers:
        parts.append(
            f'<line x1="{px(x):.1f}" x2="{px(x):.1f}" y1="{TOP}" y2="{TOP + plot_h}" '
            f'stroke="var(--text-2)" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{px(x) + 4:.1f}" y="{TOP + plot_h - 6}" font-size="11" fill="var(--text-2)">{label}</text>'
        )
    parts.append(
        f'<text x="{LEFT + plot_w / 2}" y="{HEIGHT - 8}" font-size="11" text-anchor="middle" '
        f'fill="var(--text-2)">{x_label}</text>'
    )
    parts.append(
        f'<text x="14" y="{TOP + plot_h / 2}" font-size="11" text-anchor="middle" fill="var(--text-2)" '
        f'transform="rotate(-90 14 {TOP + plot_h / 2})">{y_label}</text>'
    )
    # Direct labels at each line's end, placed top to bottom and nudged apart so they never
    # collide. A single line needs none: the title names it.
    label_y = {}
    for i in sorted(range(len(series)), key=lambda i: py(series[i][2][-1])):
        end_y = py(series[i][2][-1])
        while any(abs(end_y - other) < 14 for other in label_y.values()):
            end_y += 14
        label_y[i] = end_y
    for i, (label, sx, sy) in enumerate(series):
        points = " ".join(f"{px(x):.1f},{py(y):.1f}" for x, y in zip(sx, sy))
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="var(--s{i})" stroke-width="2" '
            f'stroke-linejoin="round" stroke-linecap="round"/>'
        )
        if len(series) == 1:
            continue
        end_y = label_y[i]
        parts.append(f'<circle cx="{px(sx[-1]):.1f}" cy="{py(sy[-1]):.1f}" r="4" fill="var(--s{i})" '
                     f'stroke="var(--surface)" stroke-width="2"/>')
        parts.append(
            f'<text x="{LEFT + plot_w + 10}" y="{end_y + 4:.1f}" font-size="12" fill="var(--text)">'
            f'<tspan fill="var(--s{i})">●</tspan> {label}</text>'
        )
    parts.append("</svg>")
    with open(os.path.join(OUT, name + ".svg"), "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    print("wrote", name)


def phase_bands(log, phases):
    """Shaded spans for the named phases, from the log's phase events."""
    times = [(e["t"], e["text"]) for e in log["events"]]
    bands = []
    for i, (t, text) in enumerate(times):
        if text in phases:
            end = next((t2 for t2, _ in times[i + 1:] if t2 > t), log["frames"][-1]["t"])
            bands.append((t, end, phases[text]))
    return bands


def main():
    os.makedirs(OUT, exist_ok=True)

    full = fly("full")
    frames = full["frames"][::25]
    line_chart(
        "full-return-altitude",
        "Full return: altitude",
        "flight time (s)",
        "altitude (km)",
        [("altitude", [f["t"] for f in frames], [f["pos"][2] / 1000 for f in frames])],
        bands=phase_bands(full, {"Boostback": "boostback", "Entry burn": "entry", "Landing burn": "landing"}),
    )

    burn = [f for f in full["frames"] if f["phase"] == "LandingBurn"][::5]
    line_chart(
        "landing-burn-throttle",
        "Landing burn: commanded vs. delivered throttle",
        "flight time (s)",
        "throttle (% of one engine)",
        [
            ("commanded", [f["t"] for f in burn], [f["throttle_cmd"] * 100 for f in burn]),
            ("delivered", [f["t"] for f in burn], [f["thrust_level"] * 100 for f in burn]),
        ],
        y_range=(0, 100),
    )

    windy = fly("landing", "--seed", "41", "--dispersed")
    frames = windy["frames"][::5]
    speed = lambda w: math.hypot(w[0], w[1])
    line_chart(
        "wind-estimate",
        "Wind at the booster: truth vs. flight computer",
        "flight time (s)",
        "wind speed (m/s)",
        [
            ("truth", [f["t"] for f in frames], [speed(f["wind"]) for f in frames]),
            ("estimate", [f["t"] for f in frames], [speed(f["est_wind"]) for f in frames]),
        ],
    )

    landing = fly("landing")
    standing = 16.8
    terminal = [f for f in landing["frames"] if f["phase"] == "LandingBurn" and f["pos"][2] - standing < 200]
    touchdown = terminal[-1]["t"]
    tilt = lambda q: math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2)))))
    line_chart(
        "terminal-tilt",
        "Terminal guidance: tilt over the last 200 m",
        "seconds before touchdown",
        "tilt from vertical (deg)",
        [("tilt", [f["t"] - touchdown for f in terminal], [tilt(f["att"]) for f in terminal])],
    )

    ship = fly("ship")
    swell = ship["ship"]
    roll, pitch = swell["roll"], swell["pitch"]
    at = lambda s, t: s["amplitude"] * math.sin(2 * math.pi / s["period"] * t + s["phase"])
    end = next(e["t"] for e in ship["events"] if e["text"].startswith("Touchdown"))
    times = [end - 20 + 0.1 * i for i in range(230)]
    line_chart(
        "deck-tilt",
        "Drone ship: deck tilt around the timed touchdown",
        "flight time (s)",
        "deck tilt (deg)",
        [("deck tilt", times, [math.degrees(math.hypot(at(roll, t), at(pitch, t))) for t in times])],
        markers=[(end, "touchdown")],
    )

    sweeps = [(label, *sweep(name)) for label, name in
              [("Landing", "landing"), ("Full return", "full"), ("Drone ship", "ship")]]
    for label, stress, landed in sweeps:
        print(f"{label}: " + ", ".join(f"{s:g} -> {l:g}%" for s, l in zip(stress, landed)))
    line_chart(
        "stress-sweep",
        "Landing rate as conditions get harsher",
        "stress level",
        "landed (%)",
        sweeps,
        y_range=(0, 100),
    )


if __name__ == "__main__":
    main()
