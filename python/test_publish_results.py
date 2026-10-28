import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from publish_results import publish
from robustness_report import load_benchmark
import test_robustness_report as fixtures


class PublicationTests(unittest.TestCase):
    def source(self, root):
        source = root / "original"
        source.mkdir()
        helper = fixtures.ReportTests()
        rows = helper.fixture(source)
        metadata = json.loads((source / "cases.json").read_text())
        metadata["cases"][0]["name"] = "test_loss"
        (source / "cases.json").write_text(json.dumps(metadata))
        for row in rows:
            row["case_name"] = "test_loss"
        helper.write_rows(source, rows)
        (source / "warmup.csv").write_text("controller,cold_warmup_wall_s\nlaguerre,1\n")
        for row in rows:
            (Path(row["run_directory"]) / "manifest.json").write_text('{"scope":"test"}')
        return source

    def test_export_is_portable_and_preserves_original_data(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            source = self.source(root)
            original = (source / "summary.csv").read_bytes()
            target = publish(source, root / "public")
            _, rows, _ = load_benchmark(target / "robustness")
            self.assertEqual(len(rows), 2)
            self.assertTrue(all(row["run_directory"].startswith("runs/") for row in rows))
            self.assertEqual((source / "summary.csv").read_bytes(), original)
            provenance = json.loads((target / "provenance.json").read_text())
            for path, digest in provenance["sha256"].items():
                self.assertEqual(hashlib.sha256((target / path).read_bytes()).hexdigest(), digest)
            self.assertFalse(list(target.rglob("*.mat")))
            with self.assertRaises(FileExistsError):
                publish(source, target)

    def test_publication_cannot_overwrite_the_source(self):
        with tempfile.TemporaryDirectory() as name:
            source = self.source(Path(name))
            with self.assertRaisesRegex(ValueError, "separate"):
                publish(source, source / "export")


if __name__ == "__main__":
    unittest.main()
