"""Plot weighted delay and replanning time from an existing result CSV."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--output", default="results/figures/strategy_metrics.png")
    args = parser.parse_args()
    data = pd.read_csv(args.csv)
    figure, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
    data.plot.bar(x="strategy", y="weighted_delay", ax=axes[0], legend=False)
    data.plot.bar(
        x="strategy", y="total_replanning_runtime", ax=axes[1], legend=False
    )
    axes[0].set_title("Weighted response delay")
    axes[1].set_title("Total replanning runtime")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(output.resolve())


if __name__ == "__main__":
    main()

