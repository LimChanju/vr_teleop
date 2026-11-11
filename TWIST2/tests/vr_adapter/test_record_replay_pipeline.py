from pathlib import Path
import time

from twist2_vr.calibration import NeutralCalibration
from twist2_vr.config import load_json
from twist2_vr.controller import DryRunSink, SafetyGate
from twist2_vr.filtering import PoseFilter
from twist2_vr.normalization import PoseNormalizer
from twist2_vr.pipeline import TeleopPipeline
from twist2_vr.providers.mock import MockVRProvider
from twist2_vr.providers.replay import JSONLReplayProvider, JSONLWriter
from twist2_vr.reconstruction import SparseKinematicReconstructor
from twist2_vr.retargeting import NeutralTestRetargeter


ROOT = Path(__file__).resolve().parents[2]


def components(provider, sink):
    return TeleopPipeline(
        provider,
        PoseNormalizer(load_json(ROOT / "config/coordinates.openvr.json")),
        NeutralCalibration(load_json(ROOT / "config/calibration.mock.json")),
        PoseFilter(load_json(ROOT / "config/filtering.json")),
        SparseKinematicReconstructor(load_json(ROOT / "config/reconstruction.sparse.json")),
        NeutralTestRetargeter(), sink, SafetyGate(),
    )


def test_mock_to_reference_and_jsonl_replay(tmp_path):
    mock = MockVRProvider()
    raw = mock.read()
    recording = tmp_path / "trackers.jsonl"
    with JSONLWriter(recording) as writer:
        writer.write(raw)
    sink = DryRunSink()
    pipeline = components(JSONLReplayProvider(recording), sink)
    result = pipeline.step()
    assert result.published and result.reference.mimic_obs.shape == (35,)
    assert len(sink.references) == 1
    pipeline.close()


def test_missing_tracking_stops_without_publishing():
    provider = MockVRProvider(target_poses={"Head": ([0, 0, 1.7], [0, 0, 0, 1])})
    sink = DryRunSink()
    pipeline = components(provider, sink)
    result = pipeline.step()
    assert not result.published
    assert sink.failures and not sink.references
    assert pipeline.stopped
