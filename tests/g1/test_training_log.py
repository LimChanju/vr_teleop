"""Use real TensorBoard event files to check portable scalar export."""

import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tensorboard.compat.proto import event_pb2, summary_pb2
from tensorboard.summary.writer.event_file_writer import EventFileWriter
from tensorboard.util import tensor_util


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("training_log", ROOT / "scripts/g1/export_training_log.py")
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


class TrainingLogTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.run = Path(self.temp.name) / "run"
        self.run.mkdir()

    def write_events(self, values, suffix=""):
        writer = EventFileWriter(str(self.run), filename_suffix=suffix)
        for index, value in enumerate(values):
            writer.add_event(event_pb2.Event(wall_time=1000.0 + index, step=index,
                                            summary=summary_pb2.Summary(value=[
                                                summary_pb2.Summary.Value(tag="Train/mean_reward", simple_value=value)])))
        writer.add_event(event_pb2.Event(wall_time=1002.0, step=2, summary=summary_pb2.Summary(value=[
            summary_pb2.Summary.Value(tag="tensor_scalar", tensor=tensor_util.make_tensor_proto(0.75),
                metadata=summary_pb2.SummaryMetadata(plugin_data=summary_pb2.SummaryMetadata.PluginData(plugin_name="scalars")))])))
        writer.close()

    def completed_result(self):
        (self.run / "result.json").write_text(json.dumps({"mode": "train",
            "status": "training_finished_evaluation_required", "completed_iterations": 3}))

    def test_complete_history_and_plot_and_no_overwrite(self):
        self.write_events([1.0, 2.0, 3.0])
        self.completed_result()
        plot = self.run / "training_metrics.png"
        history = exporter.export_run(self.run, plot)
        self.assertFalse(history["snapshot_only"])
        self.assertTrue(history["all_numeric_fields_finite"])
        self.assertEqual(history["sample_count"], 4)
        self.assertEqual(history["scalars"]["Train/mean_reward"]["samples"][2]["wall_time"], 1002.0)
        self.assertEqual(history["scalars"]["tensor_scalar"]["samples"][0]["value"], 0.75)
        self.assertEqual(plot.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
        original = (self.run / "training_metrics.json").read_bytes()
        with self.assertRaises(FileExistsError):
            exporter.export_run(self.run, plot)
        self.assertEqual((self.run / "training_metrics.json").read_bytes(), original)
        exporter.export_run(self.run, plot, overwrite=True)

    def test_nonfinite_samples_and_repeated_steps_are_preserved(self):
        self.write_events([float("nan"), float("inf"), -float("inf")], ".first")
        self.write_events([7.0], ".resumed")
        history = exporter.export_run(self.run)
        self.assertTrue(history["snapshot_only"])
        self.assertFalse(history["all_values_finite"])
        self.assertEqual(history["nonfinite_value_count"], 3)
        reward = history["scalars"]["Train/mean_reward"]
        self.assertEqual(reward["count"], 4)
        self.assertEqual(reward["finite_value_count"], 1)
        self.assertEqual(sum(sample["step"] == 0 for sample in reward["samples"]), 2)
        invalid = [sample for sample in reward["samples"] if sample["value"] is None]
        self.assertEqual({sample["nonfinite_value"] for sample in invalid},
                         {"nan", "positive_infinity", "negative_infinity"})
        # Strict JSON readers can consume this: nonfinite tokens are never emitted.
        json.loads((self.run / "training_metrics.json").read_text(),
                   parse_constant=lambda value: self.fail(f"invalid JSON constant {value}"))

    def test_mutating_event_and_symlink_output_are_rejected(self):
        self.write_events([1.0])
        original_copy = exporter.shutil.copyfile

        def changed(source, target):
            result = original_copy(source, target)
            with source.open("ab") as stream:
                stream.write(b"changed")
            return result

        with patch.object(exporter.shutil, "copyfile", side_effect=changed):
            with self.assertRaisesRegex(ValueError, "changed during snapshot"):
                exporter.export_run(self.run)
        self.assertFalse((self.run / "training_metrics.json").exists())
        other = self.run / "preserve.json"
        other.write_text("preserve")
        (self.run / "training_metrics.json").symlink_to(other)
        with self.assertRaisesRegex(ValueError, "symlink"):
            exporter.export_run(self.run, overwrite=True)
        self.assertEqual(other.read_text(), "preserve")

    def test_completed_result_does_not_certify_newer_resumed_events(self):
        self.write_events([1.0], ".finished")
        self.completed_result()
        os.utime(self.run / "result.json", ns=(1, 1))
        self.write_events([2.0], ".resumed")
        history = exporter.collect_history(self.run)
        self.assertTrue(history["events_newer_than_completion_result"])
        self.assertTrue(history["snapshot_only"])


if __name__ == "__main__":
    unittest.main()
