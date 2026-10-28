"""Export portable CSV evidence without local binaries or duplicated snapshots."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import shutil

from robustness_report import build_report, load_benchmark, resolve_run


PRIVATE_PATH = re.compile(r"[A-Za-z]:[\\/]|/(?:Users|home)/")


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def publish(source, destination, legacy_summary=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination == source or destination.is_relative_to(source):
        raise ValueError("Publication destination must be separate from the source benchmark")
    if destination.exists():
        raise FileExistsError(f"Publication destination already exists: {destination}")
    metadata, _, _ = load_benchmark(source)
    cases = metadata["cases"]
    if isinstance(cases, dict):
        cases = [cases]
    if any(not re.fullmatch(r"[A-Za-z0-9_-]+", case["name"]) for case in cases):
        raise ValueError("Publication case names must be portable directory identifiers")
    destination.mkdir(parents=True)
    current = destination / "robustness"
    current.mkdir()
    for name in ("cases.json", "warmup.csv"):
        shutil.copy2(source / name, current / name)
    with (source / "summary.csv").open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    originals = {}
    for row in rows:
        original = resolve_run(source, row["run_directory"])
        name = f'{row["case_name"]}_{row["controller"]}'
        target = current / "runs" / name
        target.mkdir(parents=True)
        originals[name] = original.name
        for item in sorted(original.glob("*.csv")):
            shutil.copy2(item, target / item.name)
        manifest = original / "manifest.json"
        shutil.copy2(manifest, target / manifest.name)
        row["run_directory"] = f"runs/{name}"
    write_rows(current / "summary.csv", rows)
    build_report(current)
    if legacy_summary:
        legacy = destination / "legacy"
        legacy.mkdir()
        with Path(legacy_summary).open(newline="", encoding="utf-8-sig") as handle:
            historical = list(csv.DictReader(handle))
        snapshot = Path(historical[0]["run_directory"]) / "source"
        shutil.copytree(snapshot, legacy / "source")
        for row in historical:
            row["original_run_id"] = Path(row.pop("run_directory")).name
        write_rows(legacy / "summary.csv", historical)
    files = {}
    for item in sorted(destination.rglob("*")):
        if not item.is_file():
            continue
        if item.suffix in (".csv", ".json", ".html", ".m"):
            if PRIVATE_PATH.search(item.read_text(encoding="utf-8-sig")):
                raise ValueError(f"Local filesystem path found in publication: {item.name}")
        files[item.relative_to(destination).as_posix()] = hashlib.sha256(item.read_bytes()).hexdigest()
    provenance = {
        "scope": "simulation-only progress snapshot; not a safety certification",
        "source_benchmark_id": source.name,
        "original_run_ids": originals,
        "included": "all 32 run CSVs and manifests, case definitions, warmup and comparisons",
        "omitted": "MATLAB run.mat/benchmark.mat binaries and duplicated current source snapshots",
        "replay": "CSV animation works; MATLAB replay_mpc_run requires original run.mat or a new saved run",
        "source": "current MATLAB implementation in repository; legacy snapshot under legacy/source",
        "sha256": files,
    }
    (destination / "provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--legacy-summary", type=Path)
    args = parser.parse_args()
    print(publish(args.source, args.destination, args.legacy_summary))


if __name__ == "__main__":
    main()
