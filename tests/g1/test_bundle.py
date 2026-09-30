"""Portable model bundle checks without importing Isaac Sim or allocating a GPU."""

import ast
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("g1_bundle", ROOT / "scripts/g1/bundle.py")
bundle = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bundle)


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo, self.run = self.root / "repo", self.root / "stand500"
        self.repo.mkdir()
        self.run.mkdir()
        (self.run / "model_final.pt").write_bytes(b"immutable checkpoint")
        (self.run / "policy.pt").write_bytes(b"immutable policy")
        self.metadata = {key: "same" for key in bundle.MODEL_CONTRACT_KEYS}
        self.metadata.update(observation_version="sparse_positions_v1", source_sha256={})
        self.write_metadata()
        (self.run / "nominal_targets.json").write_text("[]")

    def write_metadata(self):
        (self.run / "run_config.json").write_text(json.dumps(self.metadata))
        (self.run / "policy.json").write_text(json.dumps({
            **self.metadata, "sha256": bundle.sha256(self.run / "policy.pt"),
        }))

    def evidence(self, name="eval", checkpoint=None):
        folder = self.root / name
        folder.mkdir()
        (folder / "result.json").write_text(json.dumps({
            "status": "evaluated", "checkpoint": str(checkpoint or self.run / "model_final.pt"),
        }))
        (folder / "trace.npz").write_bytes(b"trace data")
        return folder

    def test_contract_covers_every_runtime_key(self):
        tree = ast.parse((ROOT / "scripts/g1/run.py").read_text())
        declaration = next(node for node in tree.body if isinstance(node, ast.Assign)
                           and any(isinstance(target, ast.Name) and target.id == "CONTRACT_KEYS"
                                   for target in node.targets))
        runtime_keys = ast.literal_eval(declaration.value)
        self.assertLessEqual(set(runtime_keys), set(bundle.MODEL_CONTRACT_KEYS))

    def test_legacy_default_and_physical_mismatches(self):
        legacy = dict(self.metadata)
        legacy.pop("observation_version")
        normalized, _ = bundle.validate_contract(legacy, self.metadata)
        self.assertEqual(normalized["observation_version"], "sparse_positions_v1")
        self.assertNotIn("observation_version", legacy)
        for key in ("body_joint_stiffness", "action_clip", "nominal_q", "coordinate_version", "observation_order"):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                bundle.validate_contract(self.metadata, {**self.metadata, key: "mismatched"})

    def test_v2_requires_estimator_and_matches_velocity_limits(self):
        richer = {**self.metadata, "observation_version": "sparse_tracking_v2"}
        with self.assertRaisesRegex(ValueError, "missing"):
            bundle.validate_contract(richer, richer)
        richer.update({key: {"setting": 1} for key in bundle.OPTIONAL_CONTRACT_KEYS})
        bundle.validate_contract(richer, richer)
        with self.assertRaisesRegex(ValueError, "target_velocity_estimator"):
            bundle.validate_contract(richer, {**richer, "target_velocity_estimator": {"setting": 2}})
        with self.assertRaisesRegex(ValueError, "training_velocity_limits"):
            bundle.validate_contract(richer, {**richer, "training_velocity_limits": [0.3, 0.0, 0.0]})

    def test_selected_evidence_legacy_docs_and_source_snapshot_roundtrip(self):
        for filename in ("README.md", "docs/WEBXR_PROBE.md", "quest3_probe.py", "scripts/setup.sh"):
            path = self.repo / filename
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("retained source")
        source = self.run / "source/g1_teleop/training.py"
        source.parent.mkdir(parents=True)
        source.write_text("original training source")
        self.metadata["source_sha256"] = {"g1_teleop/training.py": bundle.sha256(source)}
        self.metadata.pop("observation_version")
        self.write_metadata()
        evidence = self.evidence()
        (evidence / "model_999.pt").write_bytes(b"unselected checkpoint")
        (evidence / "input.jsonl").write_bytes(b"unselected large tracking log")
        for name in ("scenario.jsonl", "scenario_analysis.json", "scenario_analysis_tracking_v2.json"):
            (evidence / name).write_text("{}\n")
        for name in ("scenario_other.jsonl", "unrelated.json", "private.key", ".env.secret"):
            (evidence / name).write_text("must remain excluded")
        archive = self.root / "portable.tar.gz"
        descriptor = bundle.create_bundle(self.run, archive, repo=self.repo, evidence=[evidence])
        self.assertEqual(descriptor["observation_version"], "sparse_positions_v1")
        self.assertEqual(descriptor["training_source_snapshot_files"], 1)
        self.assertEqual(descriptor["evidence"][0]["checkpoint_relation"], "same_checkpoint")
        self.assertGreater(bundle.verify_bundle(archive), 8)
        unpacked = bundle.unpack_bundle(archive, self.root / "unpacked")
        self.assertEqual((unpacked / "models/stand500/source/g1_teleop/training.py").read_text(),
                         "original training source")
        self.assertTrue((unpacked / "docs/WEBXR_PROBE.md").is_file())
        self.assertTrue((unpacked / "evidence/eval/trace.npz").is_file())
        self.assertFalse((unpacked / "evidence/eval/model_999.pt").exists())
        self.assertFalse((unpacked / "evidence/eval/input.jsonl").exists())
        for name in ("scenario.jsonl", "scenario_analysis.json", "scenario_analysis_tracking_v2.json"):
            self.assertTrue((unpacked / "evidence/eval" / name).is_file())
            self.assertIn(name, descriptor["evidence"][0]["files"])
        for name in ("scenario_other.jsonl", "unrelated.json", "private.key", ".env.secret"):
            self.assertFalse((unpacked / "evidence/eval" / name).exists())

    def test_missing_or_changed_source_snapshot_is_rejected(self):
        self.metadata["source_sha256"] = {"g1_teleop/training.py": "0" * 64}
        self.write_metadata()
        with self.assertRaisesRegex(ValueError, "source snapshot"):
            bundle.create_bundle(self.run, self.root / "missing.tar.gz", repo=self.repo)
        source = self.run / "source/g1_teleop/training.py"
        source.parent.mkdir(parents=True)
        source.write_text("changed")
        with self.assertRaisesRegex(ValueError, "source snapshot"):
            bundle.create_bundle(self.run, self.root / "changed.tar.gz", repo=self.repo)

    def test_evidence_other_checkpoint_and_symlink_rejection(self):
        other = self.root / "other.pt"
        other.write_bytes(b"other checkpoint")
        evidence = self.evidence(checkpoint=other)
        result = bundle.copy_evidence(evidence, self.root / "selected", bundle.sha256(self.run / "model_final.pt"))
        self.assertEqual(result["checkpoint_relation"], "other_checkpoint")
        (evidence / "trace.npz").unlink()
        (evidence / "trace.npz").symlink_to(other)
        with self.assertRaisesRegex(ValueError, "non-symlink"):
            bundle.copy_evidence(evidence, self.root / "bad", bundle.sha256(self.run / "model_final.pt"))

    def test_scenario_evidence_obeys_symlink_size_and_file_count_guards(self):
        evidence = self.evidence()
        checkpoint_hash = bundle.sha256(self.run / "model_final.pt")
        scenario = evidence / "scenario.jsonl"
        scenario.symlink_to(evidence / "result.json")
        with self.assertRaisesRegex(ValueError, "non-symlink"):
            bundle.copy_evidence(evidence, self.root / "symlink", checkpoint_hash)
        scenario.unlink()
        # Sparse file exercises the real 64 MiB preflight without allocating it.
        with scenario.open("wb") as stream:
            stream.truncate(bundle.MAX_EVIDENCE_BYTES + 1)
        with self.assertRaisesRegex(ValueError, "64 MiB"):
            bundle.copy_evidence(evidence, self.root / "oversized", checkpoint_hash)
        scenario.unlink()
        for index in range(199):
            (evidence / f"scenario_analysis_{index}.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "200 files"):
            bundle.copy_evidence(evidence, self.root / "too_many", checkpoint_hash)


if __name__ == "__main__":
    unittest.main()
