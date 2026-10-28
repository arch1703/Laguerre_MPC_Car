# Published experiment evidence

This is the portable, curated evidence for the 28 October 2026 progress snapshot.
The original local experiment folders were not modified.

## Current robustness matrix

- [Offline HTML dashboard](robustness/dashboard.html): download/open locally.
- [All 32 results](robustness/summary.csv); [16 paired comparisons](robustness/comparison.csv).
- [Predeclared cases](robustness/cases.json); [separate warm-up](robustness/warmup.csv).
- [Run CSVs and manifests](robustness/runs): all successes and failures, not only
  representative runs. Names are portable `case_controller` identifiers.

Each current run retains its state/control/timing log, physical object history,
timestamped measured memory, paired marking history, track, obstacle layout,
terminal state, final path, events when present, and parameter/source manifest.
No current history was downsampled or silently discarded.

Paths in the published summary are relative to its benchmark directory.
The report reader supports both these portable paths and the original local
absolute-path format. The dashboard recomputes optimizer medians from ticks
with `solver_called=true`, not waiting ticks.

`run.mat` and `benchmark.mat` are intentionally omitted, as are 32 repeated
copies of the current MATLAB source. Consequently these CSV exports work with
the Python animator but are **not** complete `replay_mpc_run` MAT bundles.
Run MATLAB again to create a full bundle for that API. The unchanged current
MATLAB implementation is committed under [matlab](../../matlab).

## Earlier controlled comparison

[legacy/summary.csv](legacy/summary.csv) is the distinct 23-case study.
Its `original_run_id` column preserves experiment identity without publishing
private filesystem paths. Per-run binaries/histories are not included for this
older study. [legacy/source](legacy/source) preserves one original implementation
snapshot; its `benchmark_mpc('full')` defines the sweep.

Do not pool this study with the newer matrix: horizons, tracks, sensing,
guards and acceptance criteria differ. Timing is desktop-dependent.

## Provenance and regeneration

[provenance.json](provenance.json) records original current run IDs, omissions,
and SHA-256 hashes of the exported evidence. Its checksums cover generated
dashboard/comparison files at export time; rebuilding them intentionally changes
those bytes if the generator changes.

From the repository root:

```powershell
python python\robustness_report.py results\published\robustness
python python\paper_figures.py
python python\animate_run.py results\published\robustness\runs\noisy_lines_laguerre
```

`python/publish_results.py` can curate an original local benchmark into a new
destination. It refuses to overwrite a destination or export inside its source,
validates inputs and rejects local filesystem paths in the output.

Committed videos are selected recordings, not new simulations. The latest noisy
video matches `robustness/runs/noisy_lines_laguerre`; other videos predate the
current robustness implementation and are labeled in the top-level README.
