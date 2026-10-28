"""Build a standalone, offline report from benchmark_boundary run bundles."""

import argparse
import csv
from html import escape
import json
import math
import os
from pathlib import Path
from statistics import mean, median


CONTROLLERS = ("laguerre", "ltv")
NUMERIC = (
    "duration_s", "requested_duration_s", "laps", "required_laps",
    "path_error_rms_cm", "clearance_cm", "road_clearance_cm", "contacts",
    "road_departures", "fallback_steps", "solver_failures", "max_stopped_s",
    "variables", "solve_median_ms", "solve_p99_ms", "online_p99_ms", "step_p99_ms",
    "sensor_p99_ms", "planning_p99_ms", "validation_p99_ms", "monitor_p99_ms",
    "missed_online_deadlines", "missed_step_deadlines", "deadline_interventions",
    "replans", "ticks", "seed",
)


def logical(value):
    if value.lower() in ("1", "true"):
        return True
    if value.lower() in ("0", "false"):
        return False
    raise ValueError(f"Invalid logical value: {value!r}")


def resolve_run(directory, value):
    root = Path(directory).resolve()
    run = Path(value)
    if not run.is_absolute():
        run = root / run
    run = run.resolve()
    if not run.is_relative_to(root / "runs"):
        raise ValueError("Run link must stay inside the benchmark runs directory")
    return run


def load_benchmark(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "cases.json").read_text(encoding="utf-8"))
    cases = metadata["cases"]
    if isinstance(cases, dict):
        cases = [cases]
    definitions = {case["name"]: case for case in cases}
    if len(definitions) != len(cases):
        raise ValueError("Duplicate benchmark case definitions")
    with (directory / "summary.csv").open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    pairs = {name: {} for name in definitions}
    for row in rows:
        name, controller = row["case_name"], row["controller"]
        if name not in pairs or controller not in CONTROLLERS:
            raise ValueError("Unknown case or controller in summary")
        if controller in pairs[name]:
            raise ValueError("Duplicate case/controller result")
        for field in NUMERIC:
            row[field] = float(row[field])
            if not math.isfinite(row[field]):
                raise ValueError(f"Nonfinite benchmark field: {field}")
        row["success"] = logical(row["success"])
        row["safety_pass"] = logical(row["safety_pass"])
        if row["ticks"] <= 0 or row["requested_duration_s"] <= 0:
            raise ValueError("Run must contain ticks and a positive requested duration")
        if row["seed"] != definitions[name]["experiment"]["seed"]:
            raise ValueError("Run seed differs from predeclared experiment")
        run = resolve_run(directory, row["run_directory"])
        with (run / "results.csv").open(newline="", encoding="utf-8-sig") as handle:
            ticks = list(csv.DictReader(handle))
        if len(ticks) != row["ticks"]:
            raise ValueError("Saved tick count differs from summary")
        solves = [1000 * float(tick["solve_time"]) for tick in ticks
                  if logical(tick["solver_called"])]
        if any(not math.isfinite(value) or value < 0 for value in solves):
            raise ValueError("Invalid called-optimizer timing")
        row["optimizer_calls"] = len(solves)
        row["optimizer_median_ms"] = median(solves) if solves else None
        pairs[name][controller] = row
    if not rows:
        raise ValueError("Benchmark has no results")
    if metadata["status"] == "completed" and any(len(pair) != 2 for pair in pairs.values()):
        raise ValueError("Completed benchmark is missing a paired controller run")
    return metadata, rows, pairs


def comparison_rows(pairs):
    result = []
    for name, pair in pairs.items():
        if len(pair) != 2:
            continue
        a, b = pair["laguerre"], pair["ltv"]
        result.append({
            "case_name": name,
            "laguerre_success": a["success"], "ltv_success": b["success"],
            "laguerre_safety_pass": a["safety_pass"], "ltv_safety_pass": b["safety_pass"],
            "laguerre_laps": a["laps"], "ltv_laps": b["laps"],
            "laguerre_variables": a["variables"], "ltv_variables": b["variables"],
            "laguerre_optimizer_median_ms": a["optimizer_median_ms"],
            "ltv_optimizer_median_ms": b["optimizer_median_ms"],
            "laguerre_minus_ltv_optimizer_ms": (
                a["optimizer_median_ms"] - b["optimizer_median_ms"]
                if a["optimizer_median_ms"] is not None and b["optimizer_median_ms"] is not None
                else None),
            "laguerre_online_p99_ms": a["online_p99_ms"],
            "ltv_online_p99_ms": b["online_p99_ms"],
        })
    return result


def timing_chart(rows):
    width, left, scale = 1020, 245, 5
    largest = max(50, max(row["step_p99_ms"] for row in rows))
    scale = min(scale, (width - left - 140) / largest)
    elements = []
    for i, row in enumerate(rows):
        y = 28 + i * 27
        label = escape(f'{row["case_name"]} / {row["controller"]}')
        online, full = row["online_p99_ms"], row["step_p99_ms"]
        elements.append(f'<text x="0" y="{y + 10}" font-size="12">{label}</text>')
        elements.append(f'<rect x="{left}" y="{y}" width="{full * scale:.2f}" '
                        'height="13" fill="#bac4d2"/>')
        elements.append(f'<rect x="{left}" y="{y}" width="{online * scale:.2f}" '
                        'height="7" fill="#246fa8"/>')
        elements.append(f'<text x="{left + full * scale + 5:.2f}" y="{y + 11}" '
                        f'font-size="11">{online:.1f} / {full:.1f} ms</text>')
    height = 50 + len(rows) * 27
    deadline = left + 50 * scale
    elements.append(f'<path d="M {deadline:.2f} 18 V {height}" stroke="#c22" '
                    'stroke-dasharray="4 3"/>')
    return (f'<svg role="img" aria-label="Online and full-loop p99 timings" '
            f'viewBox="0 0 {width} {height}">'
            '<text x="245" y="12" font-size="12">Blue: online p99; grey: full-loop p99; '
            'red: 50 ms budget</text>' + "".join(elements) + "</svg>")


def build_report(directory, output=None):
    directory = Path(directory).resolve()
    output = Path(output).resolve() if output else directory / "dashboard.html"
    metadata, rows, pairs = load_benchmark(directory)
    links = {name: escape(Path(os.path.relpath(directory / name, output.parent)).as_posix(),
                          quote=True)
             for name in ("warmup.csv", "summary.csv", "comparison.csv", "cases.json")}
    paired = comparison_rows(pairs)
    comparison = directory / "comparison.csv"
    if paired:
        with comparison.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(paired[0]))
            writer.writeheader()
            writer.writerows(paired)
    totals = []
    for controller in CONTROLLERS:
        group = [row for row in rows if row["controller"] == controller]
        if not group:
            continue
        count = len(group)
        successes = sum(row["success"] for row in group)
        safe = sum(row["safety_pass"] for row in group)
        contacts = sum(row["contacts"] for row in group)
        total_ticks = sum(row["ticks"] for row in group)
        online_misses = sum(row["missed_online_deadlines"] for row in group)
        full_misses = sum(row["missed_step_deadlines"] for row in group)
        medians = [row["optimizer_median_ms"] for row in group
                   if row["optimizer_median_ms"] is not None]
        solve = f"{mean(medians):.2f}" if medians else "not called"
        totals.append(
            f'<tr><th>{controller}</th><td>{successes}/{count}</td><td>{safe}/{count}</td>'
            f'<td>{contacts:g}</td><td>{solve}</td>'
            f'<td>{online_misses:g}/{total_ticks:g} ({100 * online_misses / total_ticks:.2f}%)</td>'
            f'<td>{full_misses:g}/{total_ticks:g} ({100 * full_misses / total_ticks:.2f}%)</td></tr>')
    cells = []
    for row in rows:
        run = resolve_run(directory, row["run_directory"])
        link = escape(Path(os.path.relpath(run / "manifest.json", output.parent)).as_posix(), quote=True)
        status = "pass" if row["success"] else "fail"
        def number(field):
            return f'{row[field]:.2f}'
        optimizer = (f'{row["optimizer_median_ms"]:.2f}'
                     if row["optimizer_median_ms"] is not None else "not called")
        cells.append(
            f'<tr class="{status}"><th>{escape(row["case_name"])}</th>'
            f'<td>{escape(row["track"])}</td><td>{row["controller"]}</td>'
            f'<td>{"PASS" if row["success"] else "FAIL"}</td>'
            f'<td>{"PASS" if row["safety_pass"] else "FAIL"}</td>'
            f'<td>{escape(row["termination"])}</td>'
            f'<td>{number("duration_s")} / {number("requested_duration_s")}</td>'
            f'<td>{number("laps")} / {number("required_laps")}</td>'
            f'<td>{number("path_error_rms_cm")}</td><td>{number("clearance_cm")}</td>'
            f'<td>{number("road_clearance_cm")}</td>'
            f'<td>{row["contacts"]:g} / {row["road_departures"]:g}</td>'
            f'<td>{row["fallback_steps"]:g} / {row["solver_failures"]:g}</td>'
            f'<td>{number("max_stopped_s")}</td><td>{row["variables"]:g}</td>'
            f'<td>{optimizer} ({row["optimizer_calls"]} calls)</td>'
            f'<td>{number("solve_median_ms")} / {number("solve_p99_ms")}</td>'
            f'<td>{number("sensor_p99_ms")} / {number("planning_p99_ms")} / '
            f'{number("validation_p99_ms")} / {number("monitor_p99_ms")}</td>'
            f'<td>{row["replans"]:g}/{row["ticks"]:g}</td>'
            f'<td>{row["deadline_interventions"]:g}</td>'
            f'<td><a href="{link}">Manifest</a></td></tr>')
    case_details = escape(json.dumps(metadata["cases"], indent=2))
    page = f"""<!doctype html>
<html lang="en"><meta charset="utf-8"><title>MPC simulation robustness comparison</title>
<style>
body{{font:15px system-ui,sans-serif;margin:2rem;color:#172739}}
h1,h2{{color:#164d76}} table{{border-collapse:collapse;font-size:13px;width:100%}}
th,td{{padding:.5rem;border:1px solid #ccd4dd;text-align:left;white-space:nowrap}}
thead{{background:#dceaf5}}.fail{{background:#fff0dd}}.pass{{background:#eff8f1}}
.scroll{{overflow:auto}}svg{{max-width:1100px;width:100%}}pre{{white-space:pre-wrap}}
</style><h1>MPC simulation robustness comparison</h1>
<p>Profile: <b>{escape(metadata["profile"])}</b>; status:
<b>{escape(metadata["status"])}</b>; {len(rows)} recorded runs.
Failed and early-terminated runs are retained, not filtered.</p>
<p><b>Simulation only.</b> This finite, seeded matrix is not a safety certification or
a statistically estimated success rate. There is no trained policy: both controllers
solve online. ROS2, external worlds, camera perception, and hardware are out of scope.</p>
<p>Paired cases share physical world configuration, seed, fixed-index disturbance
draws, tuning, constraints, horizons, initial state, and acceptance criteria.
Their perceived observations can differ because their trajectories differ.
Controller order alternates; cold warm-up is recorded separately in
<a href="{links["warmup.csv"]}">warmup.csv</a>. Deadline outcomes depend on desktop load.</p>
<h2>Outcomes and timing</h2><div class="scroll"><table><thead><tr>
<th>Controller</th><th>Mission pass</th><th>Safety pass</th><th>Contact steps</th>
<th>Mean of called-optimizer run medians [ms]</th><th>Online misses</th><th>Full-loop misses</th>
</tr></thead><tbody>{"".join(totals)}</tbody></table></div>
<p>Mission pass requires completed duration, sufficient progress, no fallback and
the safety criteria. A stopped car is not automatically a successful mission.
Minimum progress is predeclared per case. Safety pass describes only the executed
portion, not what would happen after early termination.</p>
{timing_chart(rows)}
<p>Online time includes sensing, planning, solve and validation; full-loop time also
includes the separate physical truth monitor and simulator bookkeeping.
Per-stage percentiles are not additive. The watchdog checks elapsed work before
applying a control but cannot undo delay or guarantee its braking computation
meets 50 ms. QP iteration caps are not wall-clock guarantees.</p>
<p>Optimizer medians are recomputed from saved ticks with
<code>solver_called=true</code>; waiting ticks are excluded. The original summary's
solve-stage median/p99 includes every tick and is retained separately below.
Never-called optimizers are marked as such, not assigned a zero solve cost.</p>
<h2>Every run</h2><div class="scroll"><table><thead><tr>
<th>Case</th><th>Track</th><th>Controller</th><th>Mission</th><th>Safety</th>
<th>Termination</th><th>Actual/requested [s]</th><th>Laps/required</th>
<th>Path RMS [cm]</th><th>Obstacle gap [cm]</th><th>Road gap [cm]</th>
<th>Contact/departure steps</th><th>Fallback/solver failure</th><th>Longest stop [s]</th>
<th>Variables</th><th>Called optimizer median [ms]</th><th>All-tick solve stage median/p99 [ms]</th>
<th>Sensor/plan/validation/monitor p99 [ms]</th><th>Replans/ticks</th>
<th>Watchdog interventions</th><th>Saved run</th>
</tr></thead><tbody>{"".join(cells)}</tbody></table></div>
<p><a href="{links["summary.csv"]}">Full summary CSV</a> |
<a href="{links["comparison.csv"]}">Paired comparison CSV</a> |
<a href="{links["cases.json"]}">Predeclared cases</a></p>
<h2>Interpretation limits</h2>
<p>Ideal paired-line association, known pose, bounded noise, circular obstacle
footprints, and a stated maximum movable-object speed are assumptions.
No directional obstacle forecast is used. Complete, miss-free range observations
can exclude unseen objects only while a checked future stopping tube remains
inside the shrinking observed region. This is not a guarantee against arbitrary
new objects spawning inside that region. With missed detections, negative
confirmations are probabilistic; repeated misses can clear a real object.
Conservative stale data or unknown classifications may cause a failed/stalled
mission. Plant mismatch cases test limited wheelbase/acceleration errors with
margins, not robust stability for arbitrary model error.</p>
<p>Compare progress and failures alongside speed. Fewer decision variables do not
establish a runtime advantage; this matrix uses one basis/tuning and one seed,
not universal superiority or multi-seed statistical evidence.</p>
<details><summary>Exact experiment definitions</summary><pre>{case_details}</pre></details>
</html>"""
    output.write_text(page, encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(build_report(args.directory, args.output))


if __name__ == "__main__":
    main()
