import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
import pandas as pd

from animate_run import car_corners, create_animation, load_run, obstacle_state


class AnimationTests(unittest.TestCase):
    def test_local_lines_and_changing_active_obstacles_render(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle

        p = {"car_front": 0.29, "car_rear": 0.06, "car_half_width": 0.075,
             "delta_max": 0.38, "v_tgt": 0.8, "clearance": 0.05}
        manifest = {"schema_version": 1, "parameters": p, "controller": "laguerre",
                    "scenario": "boundary_crossing", "world": {
                        "mode": "live_moving_world", "track_sensor": "local_paired_boundary_lines",
                        "memory": "current_range_measurements_only",
                        "parameters": {"sensor_range": 3.5, "sensor_fov": 2 * np.pi}}}
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            pd.DataFrame({"t": [0, 0.05], "X": [0, 0.04], "Y": [0, 0], "psi": [0, 0],
                          "v": [0.8, 0.8], "delta": [0, 0], "X_ref": [0, 0.04],
                          "Y_ref": [0, 0], "yielding": [0, 1]}).to_csv(
                              directory / "results.csv", index=False)
            pd.DataFrame({"X": [0, 1, 2], "Y": [0, 0, 0]}).to_csv(
                directory / "track.csv", index=False)
            pd.DataFrame({"id": [1, 2], "X": [1, 2], "Y": [0, 0], "R": [0.1, 0.1],
                          "spawn_time": [0, 0.05], "removal_time": [0.05, np.inf]}).to_csv(
                              directory / "obstacles.csv", index=False)
            pd.DataFrame({"t": [0, 0.05], "id": [1, 2], "X": [1, 2], "Y": [0, 0],
                          "R": [0.1, 0.1], "sensed": [1, 1], "known": [1, 1],
                          "known_X": [1, 2], "known_Y": [0, 0], "last_seen": [0, 0.05]}).to_csv(
                              directory / "obstacle_history.csv", index=False)
            pd.DataFrame({"t": [0] * 3 + [0.05] * 3, "left_X": [0, 1, 2] * 2,
                          "left_Y": [0.8] * 6, "right_X": [0, 1, 2] * 2,
                          "right_Y": [-0.8] * 6}).to_csv(directory / "line_history.csv", index=False)
            pd.DataFrame({"t": [0, 0.05, 0.05], "id": [1, 1, 2], "X": [0.9, 0.9, 1.9],
                          "Y": [0.01] * 3, "R": [0.1] * 3, "last_seen": [0, 0, 0.05],
                          "speed_bound": [0.3] * 3, "position_bound": [0.02] * 3,
                          "outside_visible": [0, 1, 0]}).to_csv(
                              directory / "measurement_history.csv", index=False)
            trajectory, track, obstacles, loaded, _ = load_run(directory)
            fig, animation, fps = create_animation(trajectory, track, obstacles, loaded)
            animation._func(1)
            circles = [patch for patch in fig.axes[0].patches if isinstance(patch, Circle)]
            self.assertFalse(circles[0].get_visible())
            self.assertTrue(circles[1].get_visible())
            self.assertTrue(circles[2].get_visible())  # Removed object remains in measured memory.
            np.testing.assert_allclose(circles[2].center, [0.9, 0.01])
            self.assertAlmostEqual(circles[2].get_radius(), 0.12)
            lines = {line.get_label(): line for line in fig.axes[0].lines}
            np.testing.assert_allclose(lines["Sensed left line"].get_ydata(), [0.8] * 3)
            np.testing.assert_allclose(lines["Sensed right line"].get_ydata(), [-0.8] * 3)
            animation.save(directory / "lines.gif", writer="pillow", fps=fps)
            self.assertGreater((directory / "lines.gif").stat().st_size, 0)
            plt.close(fig)
            measurements = pd.read_csv(directory / "measurement_history.csv")
            measurements.loc[0, "last_seen"] = 1
            measurements.to_csv(directory / "measurement_history.csv", index=False)
            with self.assertRaisesRegex(ValueError, "future observations"):
                load_run(directory)

    def test_moving_obstacles_use_recorded_positions_and_stale_measurements(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle

        p = {"car_front": 0.29, "car_rear": 0.06, "car_half_width": 0.075,
             "delta_max": 0.38, "v_tgt": 1.2, "clearance": 0.05}
        manifest = {"schema_version": 1, "parameters": p, "controller": "laguerre",
                    "scenario": "moving", "world": {"mode": "live_moving_world",
                    "parameters": {"sensor_range": 2.5, "sensor_fov": 2 * np.pi}}}
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            pd.DataFrame({"t": [0, 0.05], "X": [2, 2], "Y": [0, 0.06],
                          "psi": [1.57] * 2, "v": [1.2] * 2, "delta": [0.2] * 2,
                          "X_ref": [2] * 2, "Y_ref": [0, 0.06]}).to_csv(
                              directory / "results.csv", index=False)
            pd.DataFrame({"X": [2, 0, -2], "Y": [0, 1.5, 0]}).to_csv(
                directory / "track.csv", index=False)
            pd.DataFrame({"id": [1], "X": [1.1], "Y": [1], "R": [0.12]}).to_csv(
                directory / "obstacles.csv", index=False)
            history = pd.DataFrame({"t": [0, 0.05], "id": [1, 1], "X": [1, 1.1],
                                    "Y": [1, 1], "R": [0.12, 0.12], "sensed": [1, 0],
                                    "known": [1, 1], "known_X": [1, 1], "known_Y": [1, 1],
                                    "last_seen": [0, 0]})
            history.to_csv(directory / "obstacle_history.csv", index=False)
            trajectory, track, obstacles, loaded, _ = load_run(directory)
            fig, animation, fps = create_animation(trajectory, track, obstacles, loaded)
            circles = [patch for patch in fig.axes[0].patches if isinstance(patch, Circle)]
            animation._func(1)
            np.testing.assert_allclose(circles[0].center, [1.1, 1])
            np.testing.assert_allclose(circles[1].center, [1, 1])
            self.assertTrue(circles[1].get_visible())
            animation.save(directory / "moving.gif", writer="pillow", fps=fps)
            self.assertGreater((directory / "moving.gif").stat().st_size, 0)
            plt.close(fig)
            history.iloc[:1].to_csv(directory / "obstacle_history.csv", index=False)
            with self.assertRaisesRegex(ValueError, "timestamps"):
                load_run(directory)

    def test_removal_requires_observed_absence_to_clear_memory(self):
        obstacle = SimpleNamespace(X=1, Y=2, spawn_time=2, detection_time=4,
                                   removal_time=6, removal_detection_time=8)
        self.assertEqual(obstacle_state(obstacle, 5), (True, True))
        self.assertEqual(obstacle_state(obstacle, 6), (False, True))
        self.assertEqual(obstacle_state(obstacle, 8), (False, False))
        obstacle.detection_time = np.nan
        self.assertEqual(obstacle_state(obstacle, 7), (False, False))

    def test_spawn_is_distinct_from_detection(self):
        obstacle = SimpleNamespace(X=1, Y=2, spawn_time=2, detection_time=4)
        self.assertEqual(obstacle_state(obstacle, 1), (False, False))
        self.assertEqual(obstacle_state(obstacle, 3), (True, False))
        self.assertEqual(obstacle_state(obstacle, 4), (True, True))
        obstacle.detection_time = np.nan
        self.assertEqual(obstacle_state(obstacle, 10), (True, False))

    def test_geometry_comes_from_manifest(self):
        p = {"car_front": 0.29, "car_rear": 0.06, "car_half_width": 0.075}
        corners = car_corners(2, 1, np.pi / 2, p)
        np.testing.assert_allclose(corners[0], [1.925, 1.29])
        np.testing.assert_allclose(corners[2], [2.075, 0.94])

    def test_run_loading_and_animation(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            p = {
                "car_front": 0.29, "car_rear": 0.06, "car_half_width": 0.075,
                "delta_max": 0.38, "v_tgt": 1.2, "clearance": 0.05,
            }
            manifest = {
                "schema_version": 1, "parameters": p, "controller": "laguerre", "scenario": "random",
                "world": {
                    "mode": "independent_random_world",
                    "parameters": {"sensor_range": 2.5, "sensor_fov": 2 * np.pi},
                },
            }
            (directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
            frame = pd.DataFrame({
                "t": [0, 0.05, 0.1, 0.15], "X": [2] * 4, "Y": [0, 0.06, 0.12, 0.18],
                "psi": [1.57] * 4, "v": [1.2] * 4, "delta": [0.2] * 4,
                "X_ref": [2] * 4, "Y_ref": [0, 0.06, 0.12, 0.18],
                "world_obstacles": [1, 1, 0, 0], "visible_obstacles": [0, 1, 1, 0],
                "total_spawned": [1] * 4, "total_removed": [0, 0, 1, 1],
                "removal_detections": [0, 0, 0, 1],
            })
            frame.to_csv(directory / "results.csv", index=False)
            pd.DataFrame({"X": [2, 0, -2], "Y": [0, 1.5, 0]}).to_csv(directory / "track.csv", index=False)
            pd.DataFrame({
                "X": [1], "Y": [1], "R": [0.1], "spawn_time": [0],
                "detection_time": [0.05], "removal_time": [0.1],
                "removal_detection_time": [0.15],
            }).to_csv(directory / "obstacles.csv", index=False)
            trajectory, track, obstacles, loaded, _ = load_run(directory)
            fig, animation, fps = create_animation(trajectory, track, obstacles, loaded, end_hold=0.1)
            self.assertAlmostEqual(fps, 20)
            self.assertEqual(list(animation.new_frame_seq()), [0, 1, 2, 3, 3, 3])
            from matplotlib.patches import Circle
            obstacle_patch = next(patch for patch in fig.axes[0].patches if isinstance(patch, Circle))
            animation._func(2)
            self.assertTrue(obstacle_patch.get_visible())
            self.assertEqual(obstacle_patch.get_linestyle(), ":")
            animation._func(3)
            self.assertFalse(obstacle_patch.get_visible())
            animation.save(directory / "test.gif", writer="pillow", fps=fps)
            self.assertGreater((directory / "test.gif").stat().st_size, 0)
            plt.close(fig)
            frame.loc[2, "t"] = 0.05
            frame.to_csv(directory / "results.csv", index=False)
            with self.assertRaisesRegex(ValueError, "strictly increasing"):
                load_run(directory)


if __name__ == "__main__":
    unittest.main()
