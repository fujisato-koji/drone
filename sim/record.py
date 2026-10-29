"""Read keyed values from the record (record/values.yaml) and stamp generated files.

The YAML reader is the one in tools/tdr.py, imported by path. Standard library only.
"""
from __future__ import annotations

import hashlib
import importlib.util
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VALUES_PATH = ROOT / "record" / "values.yaml"
_TDR_PATH = ROOT / "tools" / "tdr.py"

_cache: dict | None = None


def _load_tdr():
    spec = importlib.util.spec_from_file_location("_tdr_tool", _TDR_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _table() -> dict:
    global _cache
    if _cache is None:
        tdr = _load_tdr()
        data = tdr.load_yaml(VALUES_PATH.read_text(encoding="utf-8"))
        table = {}
        for item in data.get("items") or []:
            v = item.get("v")
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                table[str(item.get("key"))] = float(v)
        _cache = table
    return _cache


def value(key, default=None):
    """Number recorded under key; default if the key is absent; KeyError if absent and no default."""
    t = _table()
    if key in t:
        return t[key]
    if default is None:
        raise KeyError(key)
    return default


def values(prefix) -> dict:
    """All numeric record values whose key starts with prefix, as {key: number}."""
    return {k: v for k, v in _table().items() if k.startswith(prefix)}


def commit() -> str:
    """Short git commit of the repository, or "unknown"."""
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=str(ROOT),
                           capture_output=True, text=True, timeout=30)
        out = r.stdout.strip()
        return out if r.returncode == 0 and out else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def stamp(paths) -> str:
    """"sha-" and 12 hex of sha256 over (path, NUL, bytes with CRLF as LF) for each path, in order."""
    h = hashlib.sha256()
    for p in paths:
        p = Path(p)
        try:
            shown = p.resolve().relative_to(ROOT).as_posix()
        except ValueError:
            shown = p.as_posix()
        h.update(shown.encode("utf-8"))
        h.update(b"\0")
        h.update(p.read_bytes().replace(b"\r\n", b"\n"))
    return "sha-" + h.hexdigest()[:12]


def header(command, paths) -> str:
    """The tdr-run line for generated .js files (no trailing newline)."""
    return f"// tdr-run: {command} commit {commit()} stamp {stamp(paths)}"


def meta(command, paths) -> dict:
    """The same information as a dict for the "meta" block of generated .json files."""
    return {"tdr_run": command, "commit": commit(), "stamp": stamp(paths)}
