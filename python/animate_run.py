"""Animate a self-contained MATLAB run, or an explicitly selected legacy CSV."""

import argparse
import json
from pathlib import Path
import warnings

import numpy as np
import pandas as pd


RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


def load_run(source, obstacle_file=None):
    source = Path(source)
    if not source.exists() and not source.is_absolute():
        source = RESULTS_DIR / source
    if source.is_dir():
        directory = source
        with (directory / "manifest.json").open(encoding="utf-8") as handle:
            manifest = json.load(handle)
        if manifest["schema_version"] != 1:
            raise ValueError("Unsupported run manifest schema")
        csv_file = directory / "results.csv"
        obstacle_path = directory / "obstacles.csv"
    else:
        directory = source.parent
        csv_file = source
        manifest = {
            "controller": "legacy",
            "scenario": source.stem,
            "parameters": {
                "car_front": 0.20,
                "car_rear": 0.08,
                "car_half_width": 0.075,
                "delta_max": np.deg2rad(22),
                "v_tgt": 1.2,
                "clearance": 0.05,
            },
        }
        obstacle_path = directory / obstacle_file if obstacle_file else None
        warnings.warn(
            "Legacy CSV has no run manifest; historical footprint dimensions "
            "are not certified. Use a new run directory for consistent geometry.",
            UserWarning,
            stacklevel=2,
        )
    trajectory = pd.read_csv(csv_file)
    track = pd.read_csv(directory / "track.csv")
    obstacles = (
        pd.read_csv(obstacle_path)
        if obstacle_path is not None
        else pd.DataFrame(columns=["X", "Y", "R", "spawn_time"])
    )
    required = {"t", "X", "Y", "psi", "v", "delta", "X_ref", "Y_ref"}
    if not required.issubset(trajectory.columns):
        raise ValueError(f"Trajectory is missing columns: {sorted(required - set(trajectory.columns))}")
    times = trajectory["t"].to_numpy()
    if len(times) < 2 or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError("Animation requires at least two strictly increasing finite timestamps")
    if not np.allclose(np.diff(times), times[1] - times[0], rtol=1e-5, atol=1e-9):
        raise ValueError("Animation requires uniformly sampled timestamps")
    if not {"X", "Y"}.issubset(track.columns):
        raise ValueError("Track must contain X and Y")
    if not {"X", "Y", "R"}.issubset(obstacles.columns):
        raise ValueError("Obstacles must contain X, Y, and R")
    if "spawn_time" not in obstacles:
        obstacles["spawn_time"] = 0.0
    if manifest.get("world", {}).get("mode") == "live_moving_world":
        history = pd.read_csv(directory / "obstacle_history.csv")
        required_history = {"t", "id", "X", "Y", "R", "known", "sensed",
                            "known_X", "known_Y", "last_seen"}
        if not required_history.issubset(history.columns):
            raise ValueError("Moving obstacle history is missing required measurements")
        if history.duplicated(["t", "id"]).any():
            raise ValueError("Duplicate moving obstacle history samples")
        for time, group in history.groupby("t", sort=True):
            expected_ids = set()
            for obstacle in obstacles.itertuples():
                removal = getattr(obstacle, "removal_time", np.nan)
                if obstacle.spawn_time <= time and (not np.isfinite(removal) or time < removal):
                    expected_ids.add(obstacle.id)
            if set(group["id"]) != expected_ids:
                raise ValueError(f"Incomplete moving obstacle history at {time}")
        if not np.array_equal(np.sort(history["t"].unique()), times):
            raise ValueError("Moving obstacle history timestamps must match the trajectory")
        if not np.isfinite(history[["X", "Y", "R"]].to_numpy()).all():
            raise ValueError("Moving obstacle truth must be finite")
        known_rows = history.loc[history["known"].astype(bool)]
        if not np.isfinite(known_rows[["known_X", "known_Y", "last_seen"]].to_numpy()).all():
            raise ValueError("Known moving obstacles must have finite measured positions and times")
        obstacles.attrs["motion_history"] = history
        measurement_path = directory / "measurement_history.csv"
        if measurement_path.exists():
            measurements = pd.read_csv(measurement_path)
            columns = {"t", "id", "X", "Y", "R", "last_seen", "speed_bound", "position_bound"}
            if not columns.issubset(measurements.columns):
                raise ValueError("Measurement memory is missing required columns")
            if measurements.duplicated(["t", "id"]).any():
                raise ValueError("Duplicate measurement memory samples")
            if not np.isfinite(measurements[list(columns)].to_numpy()).all():
                raise ValueError("Measurement memory must be finite")
            if not set(measurements["t"]).issubset(times) or not set(
                    measurements["id"]).issubset(obstacles["id"]):
                raise ValueError("Measurement memory has unknown timestamps or IDs")
            if (measurements["last_seen"] > measurements["t"] + 1e-9).any():
                raise ValueError("Measurement memory contains future observations")
            if (measurements[["R", "speed_bound", "position_bound"]] < 0).any().any():
                raise ValueError("Measurement uncertainty bounds must be nonnegative")
            obstacles.attrs["measurement_history"] = measurements
        if manifest["world"].get("track_sensor") == "local_paired_boundary_lines":
            lines = pd.read_csv(directory / "line_history.csv")
            columns = {"t", "left_X", "left_Y", "right_X", "right_Y"}
            if not columns.issubset(lines.columns) or not np.array_equal(
                    np.sort(lines["t"].unique()), times):
                raise ValueError("Local line measurements must cover every trajectory tick")
            if not np.isfinite(lines[list(columns)].to_numpy()).all():
                raise ValueError("Local line measurements must be finite")
            obstacles.attrs["line_history"] = lines
    return trajectory, track, obstacles, manifest, directory


def car_corners(x, y, heading, parameters):
    front = parameters["car_front"]
    rear = parameters["car_rear"]
    half_width = parameters["car_half_width"]
    points = np.array(
        [[front, half_width], [front, -half_width], [-rear, -half_width], [-rear, half_width]]
    )
    cosine, sine = np.cos(heading), np.sin(heading)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T + [x, y]


def obstacle_state(obstacle, time):
    spawned = np.isfinite(obstacle.X) and np.isfinite(obstacle.Y) and obstacle.spawn_time <= time
    detection_time = getattr(obstacle, "detection_time", obstacle.spawn_time)
    removal_time = getattr(obstacle, "removal_time", np.nan)
    cleared_time = getattr(obstacle, "removal_detection_time", np.nan)
    present = spawned and (not np.isfinite(removal_time) or time < removal_time)
    known = spawned and np.isfinite(detection_time) and detection_time <= time
    known = known and (not np.isfinite(cleared_time) or time < cleared_time)
    return present, known


def create_animation(trajectory, track, obstacles, manifest, stride=1, path_only=False, end_hold=0):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Polygon, Wedge
    from matplotlib.lines import Line2D

    if not np.isfinite(end_hold) or end_hold < 0:
        raise ValueError("End hold must be finite and nonnegative")
    p = manifest["parameters"]
    fig = plt.figure(figsize=(8, 7) if path_only else (13, 7))
    if path_only:
        map_axis = fig.add_subplot()
        data_axes = []
    else:
        grid = fig.add_gridspec(3, 2)
        map_axis = fig.add_subplot(grid[:, 0])
        data_axes = [fig.add_subplot(grid[i, 1]) for i in range(3)]
    map_axis.plot(track["X"], track["Y"], "k--", label="Track centerline")
    if {"left_X", "left_Y", "right_X", "right_Y"}.issubset(track.columns):
        map_axis.plot(track["left_X"], track["left_Y"], color="0.7")
        map_axis.plot(track["right_X"], track["right_Y"], color="0.7")
    elif "road_half_width" in p and "psi" in track:
        for side in (-1, 1):
            map_axis.plot(
                track["X"] - side * p["road_half_width"] * np.sin(track["psi"]),
                track["Y"] + side * p["road_half_width"] * np.cos(track["psi"]),
                color="0.7",
            )
    map_axis.set_aspect("equal")
    margin = p.get("road_half_width", 0.8) + 0.2
    map_axis.set_xlim(track["X"].min() - margin, track["X"].max() + margin)
    map_axis.set_ylim(track["Y"].min() - margin, track["Y"].max() + margin)
    map_axis.set(xlabel="X [m]", ylabel="Y [m]")
    map_axis.set_title(f'{manifest["controller"]} / {manifest["scenario"]}'
                       f' {manifest.get("world", {}).get("track_kind", "")}'.rstrip())
    trail, = map_axis.plot([], [], "b-", linewidth=1.7, zorder=3, label="Vehicle trail")
    reference, = map_axis.plot([], [], "g--", linewidth=1, zorder=2, label="Reference history")
    first = trajectory.iloc[0]
    body = Polygon(car_corners(first.X, first.Y, first.psi, p), color="tab:blue", alpha=0.6)
    map_axis.add_patch(body)
    world = manifest.get("world", {})
    sensor = None
    moving = world.get("mode") == "live_moving_world"
    if world.get("mode") in {"independent_random_world", "live_moving_world"}:
        config = world["parameters"]
        half_fov = np.rad2deg(config["sensor_fov"]) / 2
        heading = np.rad2deg(first.psi)
        sensor = Wedge(
            (first.X, first.Y), config["sensor_range"],
            heading - half_fov, heading + half_fov,
            facecolor="tab:blue", edgecolor="tab:blue", alpha=0.07,
            linestyle="--", zorder=0, label="Current sensor range",
        )
        map_axis.add_patch(sensor)
        map_axis.add_line(Line2D([], [], color="0.5", linestyle="--",
                                label="Outside sensing" if moving else "Spawned, not detected"))
        map_axis.add_line(Line2D([], [], color="red",
                                label="World object (in range)" if moving else "Detected / remembered"))
        if moving and world.get("memory") != "current_range_measurements_only":
            map_axis.add_line(Line2D([], [], color="tab:orange", linestyle=":",
                                    label="Last measured position"))
        if not moving and "removal_time" in obstacles and obstacles["removal_time"].notna().any():
            map_axis.add_line(Line2D([], [], color="tab:orange", linestyle=":",
                                    label="Removed, still remembered"))
    patches = []
    for row in obstacles.itertuples():
        patch = Circle((row.X, row.Y), row.R, fill=False, edgecolor="red", visible=False)
        map_axis.add_patch(patch)
        patches.append(patch)
    memory_patches = []
    motion_frames = {}
    measurement_frames = None
    line_frames = {}
    sensed_lines = []
    if moving:
        history = obstacles.attrs.get("motion_history")
        if history is None:
            raise ValueError("Moving-world animation requires recorded position history")
        motion_frames = {time: group.set_index("id") for time, group in history.groupby("t")}
        if "measurement_history" in obstacles.attrs:
            measurement_frames = {time: group.set_index("id") for time, group in
                                  obstacles.attrs["measurement_history"].groupby("t")}
        for row in obstacles.itertuples():
            patch = Circle((row.X, row.Y), row.R, fill=False, edgecolor="tab:orange",
                           linestyle=":", visible=False)
            map_axis.add_patch(patch)
            memory_patches.append(patch)
        if "line_history" in obstacles.attrs:
            line_frames = {time: group for time, group in
                           obstacles.attrs["line_history"].groupby("t")}
            for side, color in (("left", "teal"), ("right", "magenta")):
                line, = map_axis.plot([], [], color=color, linewidth=2,
                                      label=f"Sensed {side} line", zorder=2)
                sensed_lines.append(line)
    text = fig.text(0.03, 0.98, "", va="top", fontsize=9)
    map_axis.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12),
                    ncol=3, fontsize=7)
    data_lines = []
    times = trajectory["t"].to_numpy()
    if data_axes:
        series = [
            (trajectory["v"].to_numpy(), "Speed [m/s]", p["v_tgt"]),
            (np.rad2deg(trajectory["delta"].to_numpy()), "Steering [deg]", np.rad2deg(p["delta_max"])),
            (
                trajectory["min_clearance"].to_numpy()
                if "min_clearance" in trajectory else np.full(len(times), np.nan),
                "World body clearance [m]",
                p["clearance"],
            ),
        ]
        for axis, (values, label, limit) in zip(data_axes, series):
            line, = axis.plot([], [], label=label)
            axis.axhline(limit, color="red", linestyle="--")
            if label == "Steering [deg]":
                axis.axhline(-limit, color="red", linestyle="--")
            finite = values[np.isfinite(values)]
            low = min(0, finite.min()) if finite.size else 0
            high = max(limit, finite.max()) if finite.size else max(limit, 1)
            pad = max(0.1, 0.1 * (high - low))
            axis.set(xlim=(times[0], times[-1]), ylim=(low - pad, high + pad), ylabel=label)
            axis.grid(True, alpha=0.3)
            data_lines.append((line, values))
        if "observed_clearance" in trajectory:
            known_values = trajectory["observed_clearance"].to_numpy()
            known_line, = data_axes[-1].plot([], [], "--", color="tab:orange", label="Known-map clearance")
            data_lines.append((known_line, known_values))
            data_axes[-1].legend(loc="upper right", fontsize=8)
        data_axes[-1].set_xlabel("Time [s]")
    map_axis.grid(True, alpha=0.3)
    detection_times = (
        obstacles["detection_time"].to_numpy()
        if "detection_time" in obstacles
        else np.array([])
    )
    removal_times = obstacles.get("removal_time", pd.Series(dtype=float)).to_numpy()
    cleared_times = obstacles.get("removal_detection_time", pd.Series(dtype=float)).to_numpy()
    spawn_times = obstacles["spawn_time"].to_numpy()

    def recent(event_times, time):
        return np.any(np.isfinite(event_times) & (event_times <= time) & (event_times > time - 0.8))

    def update(index):
        row = trajectory.iloc[index]
        body.set_xy(car_corners(row.X, row.Y, row.psi, p))
        trail.set_data(trajectory["X"].iloc[:index + 1], trajectory["Y"].iloc[:index + 1])
        reference.set_data(trajectory["X_ref"].iloc[:index + 1], trajectory["Y_ref"].iloc[:index + 1])
        for patch, obstacle in zip(patches, obstacles.itertuples()):
            if moving:
                if obstacle.id not in motion_frames[row.t].index:
                    patch.set_visible(False)
                    continue
                sample = motion_frames[row.t].loc[obstacle.id]
                patch.set_center((sample.X, sample.Y))
                patch.set_visible(True)
                patch.set_edgecolor("red" if sample.sensed else "0.5")
                patch.set_linestyle("-" if sample.sensed else "--")
                continue
            present, known = obstacle_state(obstacle, row.t)
            just_spawned = present and row.t - obstacle.spawn_time < 0.6
            patch.set_visible(present or known)
            patch.set_edgecolor("goldenrod" if just_spawned else "tab:orange" if not present
                                else "red" if known else "0.5")
            patch.set_linestyle(":" if not present else "-" if known else "--")
            patch.set_linewidth(2.5 if just_spawned else 1.5)
        if moving:
            for patch, obstacle in zip(memory_patches, obstacles.itertuples()):
                if measurement_frames is not None:
                    frame = measurement_frames.get(row.t)
                    if frame is None or obstacle.id not in frame.index:
                        patch.set_visible(False)
                        continue
                    sample = frame.loc[obstacle.id]
                    patch.set_visible(True)
                    patch.set_center((sample.X, sample.Y))
                    patch.set_edgecolor("teal" if sample.get("outside_visible", False)
                                        else "tab:orange")
                    patch.set_radius(sample.R + sample.position_bound)
                    continue
                if obstacle.id not in motion_frames[row.t].index:
                    patch.set_visible(False)
                    continue
                sample = motion_frames[row.t].loc[obstacle.id]
                patch.set_visible(bool(sample.known) and not bool(sample.sensed))
                if sample.known:
                    patch.set_center((sample.known_X, sample.known_Y))
        if sensed_lines:
            sample = line_frames[row.t]
            for line, side in zip(sensed_lines, ("left", "right")):
                line.set_data(sample[f"{side}_X"], sample[f"{side}_Y"])
        if sensor is not None:
            heading = np.rad2deg(row.psi)
            sensor.set_center((row.X, row.Y))
            sensor.set_theta1(heading - half_fov)
            sensor.set_theta2(heading + half_fov)
        status = row.get("status", "historical")
        counts = ""
        if "world_obstacles" in trajectory:
            label = "Active map" if measurement_frames is not None else "Known"
            counts = f'\nWorld: {int(row.world_obstacles)}   {label}: {int(row.visible_obstacles)}'
        if "total_spawned" in trajectory and world.get("mode") == "independent_random_world":
            counts += f'\nTotal spawned: {int(row.total_spawned)}   Removed: {int(row.total_removed)}'
        notices = []
        if moving:
            notices.append("Timestamped measurements; bounded motion" if measurement_frames is not None
                           else "Live positions; bounded motion" if line_frames
                           else "Live positions only; no motion forecast")
        if "line_age" in trajectory:
            notices.append(f"Line age: {row.line_age:.2f} s; object age: {row.obstacle_age:.2f} s")
            notices.append("Orange: measured memory; teal: outside complete scan")
        if row.get("yielding", False):
            notices.append("YIELD: checked braking / waiting")
        if recent(spawn_times, row.t):
            notices.append("WORLD: obstacle appeared")
        if recent(removal_times, row.t):
            notices.append("WORLD: obstacle disappeared")
        if row.get("new_detections", 0) > 0 or recent(detection_times, row.t):
            notices.append("SENSOR: obstacle detected")
        if row.get("removal_detections", 0) > 0 or recent(cleared_times, row.t):
            notices.append("SENSOR: absence confirmed")
        if not row.get("applied", True):
            notices.append("SAFETY STOP: command not applied")
        if row.get("confirmed_contact", False):
            notices.append("CONTACT: simulation stopped")
        notice = "".join(f"\n{message}" for message in notices)
        text.set_text(f"t = {row.t:.2f} s   v = {row.v:.2f} m/s\n{status}{counts}{notice}")
        for line, values in data_lines:
            line.set_data(times[:index + 1], values[:index + 1])
        return (trail, reference, body, text, *patches, *memory_patches, *sensed_lines,
                *(line for line, _ in data_lines))

    dt = times[1] - times[0]
    frames = list(range(0, len(trajectory), stride))
    frames.extend([len(trajectory) - 1] * round(end_hold / (dt * stride)))
    animation = FuncAnimation(fig, update, frames=frames, interval=1000 * dt * stride, blit=False)
    fig.tight_layout(rect=(0, 0.06, 1, 0.80))
    return fig, animation, 1 / (dt * stride)


def main(default_source=None, default_obstacles=None, path_only=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", nargs="?", default=default_source, help="Run directory or legacy CSV")
    parser.add_argument("obstacles", nargs="?", default=default_obstacles, help="Legacy obstacle CSV")
    parser.add_argument("--save", action="store_true", help="Export MP4, or GIF when ffmpeg is unavailable")
    parser.add_argument("--no-show", action="store_true", help="Use a noninteractive backend")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--hold-end", type=float, default=0,
                        help="Hold the terminal observation for this many extra seconds")
    args = parser.parse_args()
    if args.source is None or args.stride < 1:
        parser.error("Provide a run directory and a positive stride")
    if args.no_show and not args.save:
        parser.error("--no-show requires --save")
    if not np.isfinite(args.hold_end) or args.hold_end < 0:
        parser.error("--hold-end must be finite and nonnegative")
    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from save_utils import save_animation

    trajectory, track, obstacles, manifest, directory = load_run(args.source, args.obstacles)
    if args.stride >= len(trajectory):
        parser.error("--stride must be smaller than the number of samples")
    fig, animation, fps = create_animation(
        trajectory, track, obstacles, manifest, args.stride, path_only, args.hold_end
    )
    if args.save:
        stem = "path_only" if path_only else "animation"
        output = directory / "videos" / f"{stem}.mp4"
        if not args.overwrite and (output.exists() or output.with_suffix(".gif").exists()):
            raise FileExistsError(f"Video already exists: {output}; pass --overwrite to replace it")
        save_animation(animation, str(output), fps=fps)
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
