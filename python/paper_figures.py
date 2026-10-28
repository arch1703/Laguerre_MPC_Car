"""Rebuild publication figures from the committed, portable experiment data."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from robustness_report import load_benchmark


def build_figures(source, output):
    _, rows, pairs = load_benchmark(source)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 2.8), layout="constrained")
    colors = ("#256eaa", "#d27a25")
    for i, controller in enumerate(("laguerre", "ltv")):
        values = [row["optimizer_median_ms"] for row in rows
                  if row["controller"] == controller and row["optimizer_median_ms"] is not None]
        axes[0].bar(i, np.mean(values), color=colors[i], alpha=0.65, width=0.5)
        axes[0].scatter(i + np.linspace(-0.15, 0.15, len(values)), values,
                        color=colors[i], s=10, edgecolors="black", linewidths=0.3)
    axes[0].set(xticks=[0, 1], xticklabels=["Laguerre", "Full-input LTV"],
                ylabel="Called-optimizer run median [ms]",
                title="16 run medians and their mean")
    names = ("baseline_chicane", "baseline_kidney", "baseline_oval")
    x = np.arange(len(names))
    for i, controller in enumerate(("laguerre", "ltv")):
        values = [pairs[name][controller]["step_p99_ms"] for name in names]
        axes[1].bar(x + (i - 0.5) * 0.34, values, width=0.34,
                    color=colors[i], label="Laguerre" if i == 0 else "LTV")
    axes[1].axhline(50, color="firebrick", linestyle="--", linewidth=1)
    axes[1].set(xticks=x, xticklabels=["Chicane", "Kidney", "Oval"],
                ylabel="Full simulator-loop p99 [ms]", title="Nominal-track cases")
    axes[1].legend(fontsize=7)
    for axis in axes:
        axis.tick_params(labelsize=8)
        axis.title.set_fontsize(9)
        axis.yaxis.label.set_fontsize(8)
        axis.grid(axis="y", alpha=0.2)
        axis.set_axisbelow(True)
    fig.savefig(output / "comparison.pdf")
    fig.savefig(output / "comparison.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("results") / "published" / "robustness")
    parser.add_argument("--output", type=Path, default=Path("docs") / "assets")
    args = parser.parse_args()
    build_figures(args.source, args.output)
