import csv
import json
from pathlib import Path
import tempfile
import unittest

from robustness_report import NUMERIC, build_report, comparison_rows, load_benchmark


class ReportTests(unittest.TestCase):
    def fixture(self, directory):
        metadata = {"profile": "test", "status": "completed", "cases": [
            {"name": "<loss>", "track": "kidney", "experiment": {"seed": 41}}]}
        (directory / "cases.json").write_text(json.dumps(metadata), encoding="utf-8")
        rows = []
        for controller, success in (("laguerre", 0), ("ltv", 1)):
            row = dict.fromkeys(NUMERIC, 1)
            row.update(case_name="<loss>", track="kidney", controller=controller,
                       success=success, safety_pass=1, termination="<stalled>",
                       seed=41, ticks=20, requested_duration_s=40,
                       run_directory=str(directory / "runs" / controller))
            run = Path(row["run_directory"])
            run.mkdir(parents=True)
            with (run / "results.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=["solver_called", "solve_time"])
                writer.writeheader()
                writer.writerows([{"solver_called": 0, "solve_time": 0}] * 18 + [
                    {"solver_called": 1, "solve_time": 0.01},
                    {"solver_called": 1, "solve_time": 0.02}])
            rows.append(row)
        self.write_rows(directory, rows)
        return rows

    def write_rows(self, directory, rows):
        with (directory / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def test_failed_runs_are_paired_preserved_and_escaped(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            self.fixture(directory)
            _, rows, pairs = load_benchmark(directory)
            self.assertEqual(len(rows), 2)
            self.assertFalse(comparison_rows(pairs)[0]["laguerre_success"])
            self.assertEqual(rows[0]["optimizer_median_ms"], 15)
            self.assertEqual(rows[0]["optimizer_calls"], 2)
            output = build_report(directory)
            html = output.read_text(encoding="utf-8")
            self.assertIn("&lt;loss&gt;", html)
            self.assertIn("&lt;stalled&gt;", html)
            self.assertNotIn("<stalled>", html)
            self.assertIn('class="fail"', html)
            self.assertIn("0/1", html)
            self.assertIn("runs/laguerre/manifest.json", html)
            with (directory / "comparison.csv").open(newline="") as handle:
                self.assertEqual(len(list(csv.DictReader(handle))), 1)

    def test_completed_matrix_rejects_missing_or_duplicate_pair(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            rows = self.fixture(directory)
            self.write_rows(directory, rows[:1])
            with self.assertRaisesRegex(ValueError, "missing a paired"):
                load_benchmark(directory)
            self.write_rows(directory, [rows[0], rows[0]])
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                load_benchmark(directory)

    def test_invalid_numeric_seed_or_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            rows = self.fixture(directory)
            rows[0]["solve_median_ms"] = "nan"
            self.write_rows(directory, rows)
            with self.assertRaisesRegex(ValueError, "Nonfinite"):
                load_benchmark(directory)
            rows[0]["solve_median_ms"] = 1
            rows[0]["seed"] = 42
            self.write_rows(directory, rows)
            with self.assertRaisesRegex(ValueError, "seed"):
                load_benchmark(directory)
            rows[0]["seed"] = 41
            rows[0]["run_directory"] = str(directory.parent)
            self.write_rows(directory, rows)
            with self.assertRaisesRegex(ValueError, "inside"):
                build_report(directory)

    def test_waiting_does_not_count_as_a_zero_cost_optimizer(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            rows = self.fixture(directory)
            run = Path(rows[0]["run_directory"])
            with (run / "results.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=["solver_called", "solve_time"])
                writer.writeheader()
                writer.writerows([{"solver_called": 0, "solve_time": 0}] * 20)
            _, parsed, pairs = load_benchmark(directory)
            self.assertIsNone(parsed[0]["optimizer_median_ms"])
            self.assertIsNone(comparison_rows(pairs)[0]["laguerre_minus_ltv_optimizer_ms"])
            self.assertIn("not called", build_report(directory).read_text(encoding="utf-8"))

    def test_published_relative_paths_are_resolved_from_the_benchmark(self):
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            rows = self.fixture(directory)
            for row in rows:
                row["run_directory"] = f'runs/{row["controller"]}'
            self.write_rows(directory, rows)
            _, parsed, _ = load_benchmark(directory)
            self.assertEqual(len(parsed), 2)
            self.assertIn("runs/ltv/manifest.json",
                          build_report(directory).read_text(encoding="utf-8"))
            rows[0]["run_directory"] = "../outside"
            self.write_rows(directory, rows)
            with self.assertRaisesRegex(ValueError, "inside"):
                load_benchmark(directory)


if __name__ == "__main__":
    unittest.main()
