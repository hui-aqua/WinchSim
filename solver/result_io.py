"""Shared binary (.bin) and CSV result writer for all WinchSim analysis methods.

Stores high-density numerical simulation results in a packed Float32 binary container
(magic 'WSIM') with a single JSON metadata/scene header, achieving ~90-95% file size
reduction over plain JSON, with zero-copy TypedArray deserialization in web viewers.
"""

import csv
from datetime import datetime
import json
import logging
from numbers import Real
import os
from pathlib import Path
import struct
import tempfile
import time
import numpy as np


_REPLACEMENT_FALLBACKS = {}


def _atomic_path(filepath: Path):
    """Create a unique temporary file in the destination directory."""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix=f".{filepath.name}.", suffix=".tmp", dir=filepath.parent)
    os.close(fd)
    return Path(name)


def _replace_with_retry(temp_path: Path, filepath: Path) -> Path:
    """Atomically replace a result, recovering to a unique path if it is locked."""
    filepath.parent.mkdir(parents=True, exist_ok=True)
    target = _REPLACEMENT_FALLBACKS.get(filepath, filepath)
    delays = (0.0, 0.15, 0.3, 0.6, 1.2)

    for delay in delays:
        if delay:
            time.sleep(delay)
        try:
            os.replace(temp_path, target)
            return target
        except PermissionError as exc:
            last_error = exc

    if target == filepath:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        fallback = filepath.with_name(f"{filepath.stem}_recovered_{stamp}{filepath.suffix}")
        suffix = 1
        while fallback.exists():
            fallback = filepath.with_name(
                f"{filepath.stem}_recovered_{stamp}_{suffix}{filepath.suffix}"
            )
            suffix += 1
        _REPLACEMENT_FALLBACKS[filepath] = fallback
        try:
            os.replace(temp_path, fallback)
        except OSError:
            _REPLACEMENT_FALLBACKS.pop(filepath, None)
            raise
        logging.warning(
            "Could not replace locked result file %s (%s); saving this run to %s",
            filepath, last_error, fallback,
        )
        return fallback

    raise last_error


def write_binary_results(filepath: Path, result: dict, rows: list[dict]):
    """Write a packed Float32 binary simulation result file."""
    requested_fields = result.get("fields") or list(rows[0].keys())
    # The packed viewer format stores float32 columns only. Keep descriptive
    # text fields in CSV/JSON metadata, but exclude them from the numeric array.
    fields = [
        field for field in requested_fields
        if all(row.get(field) is None or isinstance(row.get(field), Real) for row in rows)
    ]
    num_rows = len(rows)
    num_cols = len(fields)

    # Metadata header excludes the large rows array to prevent duplication
    meta = {k: v for k, v in result.items() if k != "results"}
    meta["fields"] = fields
    meta["shape"] = [num_rows, num_cols]
    meta["dtype"] = "float32"
    meta_bytes = json.dumps(meta, separators=(',', ':')).encode("utf-8")

    # Contiguous little-endian float32 numerical array
    data_arr = np.zeros((num_rows, num_cols), dtype="<f4")
    for i, row in enumerate(rows):
        for j, fld in enumerate(fields):
            val = row.get(fld, 0.0)
            data_arr[i, j] = 0.0 if val is None else float(val)

    temp_path = _atomic_path(filepath)
    try:
        with temp_path.open("wb") as f:
            f.write(b"WSIM")
            f.write(struct.pack("<HI", 1, len(meta_bytes)))
            f.write(meta_bytes)
            f.write(data_arr.tobytes())
        filepath = _replace_with_retry(temp_path, filepath)
    finally:
        temp_path.unlink(missing_ok=True)
    return filepath


def read_binary_results(filepath: Path) -> dict:
    """Read a packed Float32 binary simulation result file into a result dictionary."""
    with open(filepath, "rb") as f:
        magic = f.read(4)
        if magic != b"WSIM":
            raise ValueError(f"Invalid WinchSim binary format: expected b'WSIM', got {magic!r}")
        version, header_len = struct.unpack("<HI", f.read(6))
        meta = json.loads(f.read(header_len).decode("utf-8"))
        num_rows, num_cols = meta["shape"]
        data = np.frombuffer(f.read(num_rows * num_cols * 4), dtype="<f4").reshape((num_rows, num_cols))

        fields = meta["fields"]
        rows = []
        for i in range(num_rows):
            rows.append({fields[j]: float(data[i, j]) for j in range(num_cols)})
        meta["results"] = rows
        meta["results_count"] = num_rows
        return meta


def write_results(rows, result, cfg, root, output_json=None, output_csv=None, output_bin=None,
                  default_json=None, default_csv="results/results.csv", default_bin="results/results.bin",
                  write_json=True):
    """Write simulation results to packed binary (.bin) and tabular CSV (.csv)."""
    def resolve(value, fallback):
        if not value and not fallback:
            return None
        path = Path(value) if value else Path(fallback)
        return path if path.is_absolute() else root / path

    solver_cfg = cfg.get("solver", {})

    # Determine binary path (preferred for 3D viewer)
    bin_target = output_bin or solver_cfg.get("output_bin")
    if not bin_target and not default_bin:
        bin_target = "results/results.bin"
    out_bin = resolve(bin_target, default_bin)

    # Determine CSV path (for Excel, pandas, and documentation)
    out_csv = resolve(output_csv or solver_cfg.get("output_csv"), default_csv)

    # Determine optional legacy JSON path
    json_target = output_json or solver_cfg.get("output_json")
    out_json = resolve(json_target, default_json)

    # 1. Write packed binary results
    if out_bin:
        out_bin = write_binary_results(out_bin, result, rows)

    # 2. Write CSV tabular results
    if out_csv and rows:
        temp_path = _atomic_path(out_csv)
        try:
            with temp_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)
            out_csv = _replace_with_retry(temp_path, out_csv)
        finally:
            temp_path.unlink(missing_ok=True)

    # 3. Write legacy JSON if explicitly configured or requested
    if out_json and write_json:
        out_json.parent.mkdir(parents=True, exist_ok=True)
        out_json.write_text(json.dumps(result, indent=2), encoding="utf-8")

    primary_out = out_bin or out_json
    return primary_out, out_csv
