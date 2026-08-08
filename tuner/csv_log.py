"""Shared CSV-log parsing helpers for the tuner/ diagnostic scripts.

The FSDS telemetry/log CSVs all share one quirk this module exists to handle
once: a leading block of `#`-prefixed comment lines (e.g. a prepended score
header — see `telemetry_logger.py`) before the real header row, and
occasional short/malformed trailing rows to drop.
"""
import numpy as np


def read_data_lines(path):
    """Return non-comment lines from `path`, stripped of the trailing newline.

    Shared by every loader below: skips leading `#`-prefixed lines (metadata
    headers), nothing else.
    """
    with open(path) as fh:
        return [ln for ln in fh if not ln.startswith('#')]


def parse_rows(lines):
    """Split `lines` into (header, rows) on ',', keeping only well-formed rows.

    `lines[0]` is the header; every later line whose comma-split length
    doesn't match the header's is dropped (handles a truncated final row from
    a log that was still being written when read).
    """
    header = lines[0].strip().split(',')
    rows = [ln.strip().split(',') for ln in lines[1:]]
    rows = [r for r in rows if len(r) == len(header)]
    return header, rows


def load_columns(path, string_columns=()):
    """Load `path` into a dict of `np.array`, one entry per header column.

    Columns named in `string_columns` are kept as strings (e.g. a `'phase'`
    label column); every other column is parsed as float, with empty cells
    becoming `nan`.
    """
    header, rows = parse_rows(read_data_lines(path))
    cols = {}
    for i, name in enumerate(header):
        raw = [r[i] for r in rows]
        if name in string_columns:
            cols[name] = np.array(raw)
        else:
            cols[name] = np.array([float(x) if x else np.nan for x in raw])
    return cols


def medfilt(x, k=5):
    """Edge-padded median filter. `x` shorter than `k` (or `k < 3`) is a no-op."""
    x = np.asarray(x, float)
    if k < 3 or len(x) < k:
        return x
    xp = np.pad(x, k // 2, mode='edge')
    return np.array([np.median(xp[i:i + k]) for i in range(len(x))])
