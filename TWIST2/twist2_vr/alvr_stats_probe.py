"""Read-only, bounded ALVR event capture for driver isolation experiments.

No ALVR settings are changed. Session/device/tracking payloads are not saved.
Only graph statistics, event counts and desync warning counts are summarized.
"""
from __future__ import annotations

import argparse
import asyncio
from collections import Counter
from datetime import datetime, timezone
import json
import math
import statistics
import time


FIELDS = (
    "total_pipeline_latency_s", "game_time_s", "server_compositor_s",
    "encoder_s", "network_s", "decoder_s", "decoder_queue_s",
    "client_compositor_s", "vsync_queue_s", "client_fps", "server_fps",
)


def summarize(events):
    counts = Counter()
    graphs = []
    desync = 0
    loss = []
    for event in events:
        item = event.get("event_type", {})
        kind, data = item.get("id"), item.get("data", {})
        counts[kind] += 1
        if kind == "GraphStatistics":
            graphs.append(data)
        elif kind == "Log" and "Desync detected" in data.get("content", ""):
            desync += 1
        elif kind == "StatisticsSummary":
            loss.append(data.get("packets_lost_per_sec", 0))
    fields = {}
    for field in FIELDS:
        values = [g[field] for g in graphs if isinstance(g.get(field), (int, float))
                  and math.isfinite(g[field])]
        if values:
            fields[field] = {"median": statistics.median(values),
                             "min": min(values), "max": max(values)}
    return {"event_counts": dict(counts), "desync_count": desync,
            "fresh_graph_statistics": bool(graphs), "fields": fields,
            "max_reported_packets_lost_per_sec": max(loss) if loss else None}


async def capture(duration):
    import websockets
    events = []
    start = time.monotonic()
    async with websockets.connect(
        "ws://127.0.0.1:8082/api/events",
        additional_headers={"X-ALVR": "true"},
        open_timeout=3, close_timeout=1, ping_interval=None,
    ) as websocket:
        while (remaining := duration - (time.monotonic() - start)) > 0:
            try:
                event = json.loads(await asyncio.wait_for(websocket.recv(), remaining))
            except asyncio.TimeoutError:
                break
            kind = event.get("event_type", {}).get("id")
            if kind in ("GraphStatistics", "StatisticsSummary", "Log"):
                events.append(event)
    return summarize(events)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8)
    parser.add_argument("--label", default="unspecified")
    args = parser.parse_args(argv)
    if not 0 < args.duration <= 30:
        parser.error("duration must be in (0, 30] seconds")
    start = datetime.now(timezone.utc).isoformat()
    result = asyncio.run(capture(args.duration))
    result.update(label=args.label, started_at_utc=start,
                  duration_s=args.duration, raw_events_saved=False)
    print(json.dumps(result, sort_keys=True), flush=True)
    return 0 if result["fresh_graph_statistics"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
