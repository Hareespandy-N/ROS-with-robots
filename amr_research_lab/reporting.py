"""Original SVG figures and machine-readable simulation results, using stdlib."""
from dataclasses import asdict
import csv
from html import escape
import json
from pathlib import Path
import re


def trace_svg(sim):
    scale = min(84, 740 / sim.world.width, 450 / sim.world.height)
    left, top = 45, 80
    def xy(x, y):
        return left + x * scale, top + (sim.world.height - y) * scale
    def polyline(points, color, width, dash=''):
        coordinates = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        return f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="{width}" stroke-dasharray="{dash}"/>'
    metrics = sim.metrics()
    out = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 880 760" role="img">',
           '<title>Original AMR simulator trajectory and speed plot</title>',
           '<rect width="880" height="760" fill="#0d1726"/>',
           '<g font-family="sans-serif" fill="#e5eef9">',
           f'<text x="40" y="34" font-size="23">{escape(sim.scenario.name)} / {escape(sim.controller)}</text>',
           '<text x="40" y="58" font-size="13" fill="#aabdd2">SIMULATION ONLY · blue = planned path · green = travelled path</text>',
           f'<rect x="{left}" y="{top}" width="{sim.world.width * scale}" height="{sim.world.height * scale}" fill="#17283a" stroke="#53708d"/>']
    for rect in sim.world.obstacles:
        x, y = xy(rect.x1, rect.y2)
        out.append(f'<rect x="{x}" y="{y}" width="{(rect.x2-rect.x1)*scale}" height="{(rect.y2-rect.y1)*scale}" fill="#8297ad"/>')
    if sim.scenario.blockage:
        r = sim.scenario.blockage
        x, y = xy(r.x1, r.y2)
        out.append(f'<rect x="{x}" y="{y}" width="{(r.x2-r.x1)*scale}" height="{(r.y2-r.y1)*scale}" fill="#eeb663" opacity="0.4"/>')
    out.append(polyline([xy(*p) for p in sim.path], '#61a8ff', 2, '5 4'))
    out.append(polyline([xy(s.x, s.y) for s in sim.trace], '#4bd8ad', 3))
    for pose, color in ((sim.scenario.start, '#f4c568'), (sim.goal, '#f58abb')):
        x, y = xy(pose.x, pose.y)
        out.append(f'<circle cx="{x}" cy="{y}" r="7" fill="{color}"/>')
    out += [f'<text x="40" y="570" font-size="17">{escape(sim.status)} · {metrics["duration_s"]:.2f} s · {metrics["distance_m"]:.2f} m</text>',
            f'<text x="40" y="594" font-size="13">Final XY error {metrics["final_xy_error_m"]*1000:.2f} mm (ideal simulation ground truth)</text>',
            '<text x="40" y="624" font-size="13">Linear speed (m/s) over simulation time</text>',
            '<path d="M45 638 V710 H810" fill="none" stroke="#53708d"/>']
    duration = max(sim.time, sim.limits.dt)
    out.append(polyline([(45 + s.t / duration * 760, 710 - s.v / sim.limits.max_v * 65)
                         for s in sim.trace], '#61a8ff', 2))
    out += ['<text x="40" y="742" font-size="12" fill="#aabdd2">Original generated figure. Physical accuracy: not proven yet.</text>', '</g></svg>']
    return '\n'.join(out) + '\n'


def write_report(sim, output_dir):
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', sim.scenario.name):
        raise ValueError('scenario name must be a filename-safe identifier')
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    stem = sim.scenario.name + '-' + sim.controller
    metrics_file = output / (stem + '.json')
    metrics_file.write_text(json.dumps(sim.metrics(), indent=2, allow_nan=False) + '\n', encoding='utf-8')
    csv_file = output / (stem + '.csv')
    with csv_file.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(sim.trace[0])))
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in sim.trace)
    figure = output / (stem + '.svg')
    figure.write_text(trace_svg(sim), encoding='utf-8')
    return metrics_file, csv_file, figure


def write_comparison(results, output_dir):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'benchmark.json').write_text(json.dumps(results, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    maximum = max(result['duration_s'] for result in results)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 560" role="img">',
           '<title>Original controller comparison: ideal simulation durations</title>',
           '<rect width="1000" height="560" fill="#0d1726"/>',
           '<g fill="#e5eef9" font-family="sans-serif">',
           '<text x="35" y="42" font-size="25">AMR Research Lab / controller comparison</text>',
           '<text x="35" y="72" font-size="14">Shared maps, goals, footprints, acceleration limits, and collision checks.</text>']
    rows = ['# Reproducible ideal-simulation results', '',
            'These are toy-map experiments, not a Nav2 benchmark or physical accuracy evidence.', '',
            '| Scenario | Controller | Outcome | Duration (s) | XY error (mm) | Min clearance (m) | Blocked (s) |',
            '|---|---|---|---:|---:|---:|---:|']
    for i, result in enumerate(results):
        y = 115 + i * 60
        color = '#61a8ff' if result['controller'] == 'baseline' else '#4bd8ad'
        label = result['scenario'] + ' / ' + result['controller']
        svg += [f'<text x="35" y="{y+17}" font-size="14">{escape(label)}</text>',
                f'<rect x="355" y="{y}" width="{result["duration_s"]/maximum*470:.2f}" height="25" rx="4" fill="{color}"/>',
                f'<text x="845" y="{y+18}" font-size="14">{result["duration_s"]:.2f} s</text>']
        rows.append(f'| {result["scenario"]} | {result["controller"]} | {result["status"]} | {result["duration_s"]:.2f} | {result["final_xy_error_m"]*1000:.2f} | {result["min_clearance_m"]:.3f} | {result["blocked_time_s"]:.2f} |')
    svg += ['<text x="35" y="520" font-size="13">Slower is not automatically worse: the regulated variant deliberately reduces speed near obstacles.</text>', '</g></svg>']
    (output / 'comparison.svg').write_text('\n'.join(svg) + '\n', encoding='utf-8')
    (output / 'README.md').write_text('\n'.join(rows) + '\n', encoding='utf-8')
