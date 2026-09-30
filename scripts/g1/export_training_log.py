#!/usr/bin/env python3
"""Export complete TensorBoard scalar histories to portable JSON and optional PNGs.

Examples:
  python scripts/g1/export_training_log.py runs/stand_2048_v2 --plot runs/stand_2048_v2/training_metrics.png
  python scripts/g1/export_training_log.py runs/run_a runs/run_b --plot /tmp/g1-training-plots

Each event file is copied and checked for concurrent changes before TensorBoard
reads it. Incomplete runs are explicitly labelled snapshots, even if the event
files happen to remain unchanged during export. No GPU or simulator is needed.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile


def signature(path):
    info = path.stat()
    return info.st_ino, info.st_size, info.st_mtime_ns


def finite_number(value):
    value = float(value)
    if math.isfinite(value):
        return value, None
    return None, "nan" if math.isnan(value) else ("positive_infinity" if value > 0 else "negative_infinity")


def snapshot_event(source, target):
    if source.is_symlink() or not source.is_file():
        raise ValueError(f"event source must be a regular non-symlink file: {source}")
    before = signature(source)
    shutil.copyfile(source, target)
    if signature(source) != before or target.stat().st_size != before[1]:
        raise ValueError(f"event file changed during snapshot; retry when its writer is quiescent: {source}")
    digest = hashlib.sha256()
    with target.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"size_bytes": before[1], "mtime_ns": before[2], "sha256": digest.hexdigest(),
            "_stat_signature": before}


def collect_history(run):
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    from tensorboard.util import tensor_util

    run = Path(run).resolve(strict=True)
    result_file = run / "result.json"
    result_bytes = result_file.read_bytes() if result_file.is_file() else None
    result_signature = signature(result_file) if result_bytes is not None else None
    result = json.loads(result_bytes) if result_bytes else {}
    if not isinstance(result, dict):
        raise ValueError(f"result.json must contain an object: {run}")
    event_files = sorted(run.rglob("events.out.tfevents.*"))
    if not event_files:
        raise FileNotFoundError(f"no TensorBoard event files found under {run}")
    histories = defaultdict(list)
    sources = []
    source_signatures = {}
    skipped_tensor_tags = set()
    with tempfile.TemporaryDirectory(prefix="g1-training-events-") as temporary:
        for index, source in enumerate(event_files):
            relative = source.relative_to(run).as_posix()
            if any(parent.is_symlink() for parent in source.parents if parent != run and run in parent.parents):
                raise ValueError(f"symlink parent in event path: {source}")
            snapshot = Path(temporary) / f"events.out.tfevents.snapshot.{index}"
            record = snapshot_event(source, snapshot)
            source_signatures[source] = record.pop("_stat_signature")
            record["path"] = relative
            sources.append(record)
            accumulator = EventAccumulator(str(snapshot), size_guidance={"scalars": 0, "tensors": 0},
                                           purge_orphaned_data=False).Reload()

            def append(tag, event, value, encoding):
                value, nonfinite = finite_number(value)
                wall_time, nonfinite_time = finite_number(event.wall_time)
                sample = {"wall_time": wall_time, "step": int(event.step), "value": value,
                          "source_event_file": relative, "encoding": encoding}
                if nonfinite:
                    sample["nonfinite_value"] = nonfinite
                if nonfinite_time:
                    sample["nonfinite_wall_time"] = nonfinite_time
                histories[tag].append(sample)

            for tag in accumulator.Tags()["scalars"]:
                for event in accumulator.Scalars(tag):
                    append(tag, event, event.value, "simple_value")
            # Newer writers can store scalar summaries as scalar-plugin tensors.
            for tag in accumulator.Tags()["tensors"]:
                metadata = accumulator.SummaryMetadata(tag)
                if metadata.plugin_data.plugin_name != "scalars":
                    skipped_tensor_tags.add(tag)
                    continue
                for event in accumulator.Tensors(tag):
                    value = tensor_util.make_ndarray(event.tensor_proto)
                    if value.size != 1 or value.dtype.kind not in "biuf":
                        raise ValueError(f"scalar-plugin tag has a nonnumeric/non-scalar tensor: {tag}")
                    append(tag, event, value.reshape(-1)[0], "tensor_scalar")
    # A result changing from running to finished while copying is not proof that
    # the snapshot contains the finished run's final events.
    after_result = result_file.read_bytes() if result_file.is_file() else None
    result_changed = result_bytes != after_result
    current_event_files = sorted(run.rglob("events.out.tfevents.*"))
    events_changed = current_event_files != event_files or any(
        not source.is_file() or signature(source) != saved for source, saved in source_signatures.items())
    events_newer_than_result = bool(result_signature and any(
        source["mtime_ns"] > result_signature[2] for source in sources))
    final_status = result.get("mode") == "train" and result.get("status") == "training_finished_evaluation_required"
    snapshot_only = not final_status or result_changed or events_changed or events_newer_than_result
    series = {}
    nonfinite_values = nonfinite_times = total = 0
    for tag, samples in sorted(histories.items()):
        # Preserve restarts and repeated steps; do not purge or deduplicate them.
        samples.sort(key=lambda sample: (sample["wall_time"] is None,
                                         sample["wall_time"] if sample["wall_time"] is not None else 0,
                                         sample["source_event_file"]))
        values = [sample["value"] for sample in samples if sample["value"] is not None]
        invalid_values = sum(sample["value"] is None for sample in samples)
        invalid_times = sum(sample["wall_time"] is None for sample in samples)
        series[tag] = {"count": len(samples), "finite_value_count": len(values),
                       "nonfinite_value_count": invalid_values, "nonfinite_wall_time_count": invalid_times,
                       "min_step": min(sample["step"] for sample in samples),
                       "max_step": max(sample["step"] for sample in samples),
                       "finite_min": min(values) if values else None,
                       "finite_max": max(values) if values else None, "samples": samples}
        nonfinite_values += invalid_values
        nonfinite_times += invalid_times
        total += len(samples)
    if not series:
        raise ValueError(f"no scalar summaries found under {run}")
    return {"schema": "g1.training_metrics.v1", "created_utc": datetime.now(timezone.utc).isoformat(),
            "source_run": str(run), "run_status": result.get("status", "result_not_available"),
            "run_completed_iterations": result.get("completed_iterations"),
            "snapshot_only": snapshot_only, "result_changed_during_export": result_changed,
            "events_changed_during_export": events_changed,
            "events_newer_than_completion_result": events_newer_than_result,
            "completion_note": ("Event snapshot of an unfinished or unconfirmed run; not a final training log."
                                if snapshot_only else "Completed training log; policy performance requires separate evaluation."),
            "event_files": sources, "scalar_tag_count": len(series), "sample_count": total,
            "nonfinite_value_count": nonfinite_values, "nonfinite_wall_time_count": nonfinite_times,
            "all_values_finite": nonfinite_values == 0,
            "all_numeric_fields_finite": nonfinite_values == 0 and nonfinite_times == 0,
            "nonfinite_encoding": "null numeric field with explicit nonfinite kind; samples are retained",
            "skipped_non_scalar_tensor_tags": sorted(skipped_tensor_tags), "scalars": series}


def render_plot(history, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    preferred = ["Train/mean_reward", "Train/mean_episode_length", "Loss/value_function", "Loss/surrogate",
                 "Policy/mean_noise_std", "Perf/total_fps"]
    tags = [tag for tag in preferred if tag in history["scalars"]]
    tags.extend([tag for tag in history["scalars"] if tag not in tags][:6 - len(tags)])
    columns = 2
    rows = (len(tags) + columns - 1) // columns
    figure, axes = plt.subplots(rows, columns, figsize=(12, 3.2 * rows), squeeze=False)
    for axis, tag in zip(axes.flat, tags):
        samples = history["scalars"][tag]["samples"]
        # NaNs create visible gaps instead of joining across invalid samples.
        axis.plot([sample["step"] for sample in samples],
                  [sample["value"] if sample["value"] is not None else float("nan") for sample in samples],
                  linewidth=1.1)
        axis.set_title(tag)
        axis.set_xlabel("TensorBoard step")
        axis.grid(alpha=0.25)
        invalid = history["scalars"][tag]["nonfinite_value_count"]
        if invalid:
            axis.text(0.02, 0.95, f"{invalid} nonfinite samples (gaps)", transform=axis.transAxes,
                      va="top", color="red")
    for axis in list(axes.flat)[len(tags):]:
        axis.set_visible(False)
    status = "unfinished snapshot" if history["snapshot_only"] else "completed training"
    figure.suptitle(f"{Path(history['source_run']).name} — {status}\n"
                   f"All tags retained in JSON; nonfinite values: {history['nonfinite_value_count']}")
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    figure.savefig(output, format="png", dpi=150)
    plt.close(figure)


def check_output(path, overwrite):
    if path.is_symlink():
        raise ValueError(f"refusing symlink output: {path}")
    if path.exists() and (not overwrite or not path.is_file()):
        raise FileExistsError(f"output exists; use --overwrite explicitly to replace a file: {path}")


def install_output(staged, destination, overwrite):
    check_output(destination, overwrite)
    if overwrite:
        os.replace(staged, destination)
    else:
        # Atomic no-clobber publication, including a file appearing after preflight.
        os.link(staged, destination)
        staged.unlink()


def export_run(run, plot=None, overwrite=False):
    run = Path(run).resolve(strict=True)
    output = run / "training_metrics.json"
    plot = Path(plot).absolute() if plot else None
    if plot and plot.suffix.lower() != ".png":
        raise ValueError("plot output must have a .png extension")
    check_output(output, overwrite)
    if plot:
        check_output(plot, overwrite)
    history = collect_history(run)
    with tempfile.TemporaryDirectory(prefix=".training-metrics-", dir=run) as temporary:
        staged = Path(temporary) / "training_metrics.json"
        staged.write_text(json.dumps(history, indent=2, allow_nan=False) + "\n")
        if plot:
            plot.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(prefix=".training-plot-", dir=plot.parent) as plot_temp:
                staged_plot = Path(plot_temp) / "plot.png"
                render_plot(history, staged_plot)
                install_output(staged_plot, plot, overwrite)
        install_output(staged, output, overwrite)
    return history


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--plot", type=Path, help="PNG path for one run; output directory for multiple runs")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.plot and len(args.runs) > 1 and args.plot.suffix.lower() == ".png":
        parser.error("--plot must be a directory for multiple runs")
    if len({run.resolve() for run in args.runs}) != len(args.runs):
        parser.error("run paths must be distinct")
    if args.plot and len(args.runs) > 1 and len({run.name for run in args.runs}) != len(args.runs):
        parser.error("multiple plotted runs must have distinct directory names")
    try:
        for run in args.runs:
            plot = args.plot / f"{run.name}.png" if args.plot and len(args.runs) > 1 else args.plot
            history = export_run(run, plot, args.overwrite)
            print(json.dumps({"run": str(run), "output": str(run / "training_metrics.json"),
                              "plot": str(plot) if plot else None, "snapshot_only": history["snapshot_only"],
                              "scalar_tag_count": history["scalar_tag_count"], "sample_count": history["sample_count"],
                              "nonfinite_value_count": history["nonfinite_value_count"],
                              "all_numeric_fields_finite": history["all_numeric_fields_finite"]}))
    except (OSError, ValueError, ImportError) as exc:
        parser.exit(1, f"Training log export failed: {exc}\n")


if __name__ == "__main__":
    main()
