"""Stream previously recorded target JSONL at the original relative timing."""

from __future__ import annotations

import dataclasses
import json
import math
import time
import uuid

from .protocol import decode_target


class ReplaySource:
    def __init__(self, path, enabled=False):
        self.file = open(path, encoding="utf-8")
        self.enabled = enabled
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.started = time.monotonic()
        self.origin = None
        self.last_elapsed = -math.inf
        self.current = None
        self.next_entry = self._read()
        if self.next_entry is None:
            self.close()
            raise ValueError("recording contains no frames")
        self.last_error = "REPLAY INPUT: no physical headset verification"

    def _read(self):
        line = self.file.readline(65537)
        if not line:
            return None
        if len(line) > 65536:
            raise ValueError("recording line exceeds 64 KiB")
        entry = json.loads(line)
        elapsed = entry["elapsed_s"]
        if type(elapsed) not in (float, int) or not math.isfinite(elapsed) or elapsed < self.last_elapsed:
            raise ValueError("recording timestamps must be finite and ordered")
        self.last_elapsed = elapsed
        if self.origin is None:
            self.origin = elapsed
        frame = decode_target(json.dumps(entry["target"], allow_nan=False).encode())
        return elapsed - self.origin, frame

    def sample_target(self):
        elapsed = time.monotonic() - self.started
        consumed = False
        while self.next_entry is not None and self.next_entry[0] <= elapsed:
            self.current = self.next_entry[1]
            self.next_entry = self._read()
            consumed = True
        if self.current is None or (self.next_entry is None and not consumed):
            raise StopIteration
        result = dataclasses.replace(
            self.current, source="replay", session=self.session, seq=self.sequence,
            timestamp_unix_ns=time.time_ns(), enabled=bool(self.current.enabled and self.enabled),
        )
        self.sequence += 1
        return result

    def close(self):
        self.file.close()
