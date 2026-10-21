"""Timestamped per-run solver logs with matching console output."""

from __future__ import annotations

from datetime import datetime
import logging
from pathlib import Path
import warnings


def configure_run_logging(input_file: str | Path, log_dir: Path) -> Path:
    """Start a timestamped log for one solver invocation and return its path."""
    timestamp = datetime.now().astimezone()
    stamp = timestamp.strftime("%Y%m%d_%H%M%S")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"{Path(input_file).stem}_{stamp}.log"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S%z",
        handlers=[logging.StreamHandler(), logging.FileHandler(log_path, encoding="utf-8")],
        force=True,
    )
    logging.captureWarnings(True)
    warnings.simplefilter("default")
    logging.info("WinchSim solver run started")
    logging.info("Input model: %s", Path(input_file).resolve())
    logging.info("Log file: %s", log_path.resolve())
    return log_path
