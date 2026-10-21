"""Shared JSON and CSV result writer for all analysis methods."""

import csv
import json
from pathlib import Path


def write_results(rows, result, cfg, root, output_json=None, output_csv=None,
                  default_json="results/results.json", default_csv="results/results.csv"):
    def resolve(value, fallback):
        path = Path(value) if value else Path(fallback)
        return path if path.is_absolute() else root / path

    out_json = resolve(output_json or cfg.get("solver", {}).get("output_json"), default_json)
    out_csv = resolve(output_csv or cfg.get("solver", {}).get("output_csv"), default_csv)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(result, indent=2), encoding="utf-8")
    with out_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return out_json, out_csv
