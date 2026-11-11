from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_json(path: str | Path) -> dict[str, Any]:
    file_path = Path(path)
    try:
        value = json.loads(file_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ValueError(f"configuration file not found: {file_path}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid JSON in {file_path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"configuration root must be an object: {file_path}")
    return value
