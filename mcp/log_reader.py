"""WPILOG file reader for offline log analysis.

Reuses the DataLogReader pattern from analyze_turn_error.py.
"""

import os
from pathlib import Path
from datetime import datetime

import numpy as np
from wpiutil.log import DataLogReader

PROJECT_ROOT = Path(__file__).parent.parent
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs"


def list_logs(log_dir: str | None = None) -> list[dict]:
    """List .wpilog files in the log directory.

    Returns:
        List of {name, path, size_mb, modified} sorted by date (newest first).
    """
    d = Path(log_dir) if log_dir else DEFAULT_LOG_DIR
    if not d.exists():
        return []

    logs = []
    for f in d.glob("*.wpilog"):
        stat = f.stat()
        logs.append({
            "name": f.name,
            "path": str(f),
            "size_mb": round(stat.st_size / (1024 * 1024), 2),
            "modified": datetime.fromtimestamp(stat.st_mtime).isoformat(),
        })

    return sorted(logs, key=lambda x: x["modified"], reverse=True)


def _resolve_log_path(log_file: str) -> Path:
    """Resolve a log file name or path to an absolute Path."""
    p = Path(log_file)
    if p.is_absolute() and p.exists():
        return p
    # Try in default log dir
    candidate = DEFAULT_LOG_DIR / log_file
    if candidate.exists():
        return candidate
    # Try as relative to project root
    candidate = PROJECT_ROOT / log_file
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Log file not found: {log_file}")


def get_keys(log_file: str) -> list[str]:
    """List all signal keys in a .wpilog file.

    Args:
        log_file: Filename (looked up in logs/) or absolute path.

    Returns:
        Sorted list of entry names.
    """
    path = _resolve_log_path(log_file)
    reader = DataLogReader(str(path))

    keys = []
    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            keys.append(sd.name)
        elif record.isControl():
            continue
        else:
            break  # Once we hit data records, all starts have been seen

    # Actually, starts can be interleaved — scan all records
    keys = []
    reader = DataLogReader(str(path))
    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name not in keys:
                keys.append(sd.name)

    return sorted(keys)


def query(log_file: str, keys: list[str],
          start_sec: float | None = None,
          end_sec: float | None = None) -> dict:
    """Extract time series data from a .wpilog file.

    Args:
        log_file: Filename or path.
        keys: List of signal names to extract.
        start_sec: Start time in seconds (from log start). None = beginning.
        end_sec: End time in seconds. None = end.

    Returns:
        {key: {"timestamps": [...], "values": [...]}} where timestamps are in seconds.
    """
    path = _resolve_log_path(log_file)
    reader = DataLogReader(str(path))

    # Build entry_id -> name map, collect wanted entries
    entry_names = {}
    entry_types = {}
    wanted = set(keys)
    data = {k: ([], []) for k in keys}

    first_timestamp = None

    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            if sd.name in wanted:
                entry_names[sd.entry] = sd.name
                entry_types[sd.entry] = sd.type
            continue
        if record.isControl():
            continue

        eid = record.getEntry()
        if eid not in entry_names:
            continue

        ts_us = record.getTimestamp()
        if first_timestamp is None:
            first_timestamp = ts_us

        ts_sec = (ts_us - first_timestamp) / 1e6

        if start_sec is not None and ts_sec < start_sec:
            continue
        if end_sec is not None and ts_sec > end_sec:
            continue

        name = entry_names[eid]
        try:
            entry_type = entry_types.get(eid, "")
            if "double" in entry_type:
                val = record.getDouble()
            elif "int" in entry_type or "integer" in entry_type:
                val = record.getInteger()
            elif "boolean" in entry_type:
                val = record.getBoolean()
            elif "string" in entry_type:
                val = record.getString()
            elif "float" in entry_type:
                val = record.getFloat()
            else:
                val = record.getDouble()  # fallback

            data[name][0].append(ts_sec)
            data[name][1].append(val)
        except Exception:
            pass

    result = {}
    for k in keys:
        ts_list, val_list = data[k]
        result[k] = {
            "timestamps": ts_list,
            "values": val_list,
            "count": len(ts_list),
        }

    return result


def summary(log_file: str, keys: list[str] | None = None,
            max_keys: int = 50) -> dict:
    """Statistical summary of numeric signals in a .wpilog file.

    Args:
        log_file: Filename or path.
        keys: Specific keys to summarize. None = all numeric keys.
        max_keys: Max number of keys to summarize when keys=None.

    Returns:
        {key: {min, max, mean, std, count, duration_sec}} for numeric signals.
        Also includes top-level {total_keys, duration_sec, file}.
    """
    path = _resolve_log_path(log_file)
    reader = DataLogReader(str(path))

    # First pass: discover entries
    entry_info = {}  # entry_id -> {name, type}
    for record in reader:
        if record.isStart():
            sd = record.getStartData()
            entry_info[sd.entry] = {"name": sd.name, "type": sd.type}

    all_keys = [info["name"] for info in entry_info.values()]

    # Filter to requested keys or numeric-looking keys
    if keys is not None:
        target_keys = set(keys)
    else:
        # Take all keys that look numeric
        numeric_types = {"double", "float", "int64", "int32", "integer"}
        target_keys = set()
        for info in entry_info.values():
            if any(t in info["type"].lower() for t in numeric_types):
                target_keys.add(info["name"])
        # Cap at max_keys
        target_keys = set(sorted(target_keys)[:max_keys])

    # Build reverse map
    target_entries = {}
    for eid, info in entry_info.items():
        if info["name"] in target_keys:
            target_entries[eid] = info

    # Second pass: collect values
    data = {name: [] for name in target_keys}
    first_ts = None
    last_ts = None

    reader = DataLogReader(str(path))
    for record in reader:
        if record.isStart() or record.isControl():
            continue

        eid = record.getEntry()
        if eid not in target_entries:
            continue

        ts = record.getTimestamp()
        if first_ts is None:
            first_ts = ts
        last_ts = ts

        name = target_entries[eid]["name"]
        try:
            val = record.getDouble()
            data[name].append(val)
        except Exception:
            try:
                val = record.getInteger()
                data[name].append(float(val))
            except Exception:
                pass

    duration = (last_ts - first_ts) / 1e6 if first_ts and last_ts else 0

    result = {
        "_meta": {
            "file": path.name,
            "total_keys": len(all_keys),
            "summarized_keys": len(target_keys),
            "duration_sec": round(duration, 2),
        }
    }

    for name in sorted(data.keys()):
        vals = data[name]
        if not vals:
            result[name] = {"count": 0}
            continue

        arr = np.array(vals)
        result[name] = {
            "count": len(arr),
            "min": round(float(np.min(arr)), 6),
            "max": round(float(np.max(arr)), 6),
            "mean": round(float(np.mean(arr)), 6),
            "std": round(float(np.std(arr)), 6),
        }

    return result
