"""One-way provider → GMR reference pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import time

from .calibration import NeutralCalibration
from .controller import ReferenceSink, SafetyGate
from .filtering import PoseFilter
from .models import BodyPoseFrame, RobotReference, TrackerFrame
from .normalization import PoseNormalizer
from .providers.base import VRInputProvider
from .reconstruction import SparseKinematicReconstructor, StrictDirectReconstructor
from .retargeting import Retargeter


@dataclass(frozen=True)
class PipelineResult:
    raw_frame: TrackerFrame
    normalized_frame: TrackerFrame | None
    body_frame: BodyPoseFrame | None
    reference: RobotReference | None
    published: bool
    reason: str = ""


class TeleopPipeline:
    def __init__(
        self,
        provider: VRInputProvider,
        normalizer: PoseNormalizer,
        calibration: NeutralCalibration,
        pose_filter: PoseFilter,
        reconstructor: SparseKinematicReconstructor | StrictDirectReconstructor,
        retargeter: Retargeter,
        sink: ReferenceSink,
        safety: SafetyGate | None = None,
        clock=time.monotonic,
    ) -> None:
        self.provider = provider
        self.normalizer = normalizer
        self.calibration = calibration
        self.pose_filter = pose_filter
        self.reconstructor = reconstructor
        self.retargeter = retargeter
        self.sink = sink
        self.safety = safety or SafetyGate()
        self.clock = clock
        self.stopped = False

    def _reject(self, raw: TrackerFrame, normalized: TrackerFrame | None,
                body: BodyPoseFrame | None, reason: str) -> PipelineResult:
        self.sink.fail_safe(reason)
        self.stopped = True
        return PipelineResult(raw, normalized, body, None, False, reason)

    def step(self) -> PipelineResult:
        if self.stopped:
            raise RuntimeError("pipeline is stopped after a safety rejection")
        raw = self.provider.read()
        try:
            normalized = self.normalizer(raw)
            filtered = self.pose_filter(normalized, now_s=self.clock())
            calibrated = self.calibration.apply(filtered)
        except Exception as error:
            return self._reject(raw, None, None, f"pose processing failed: {error}")
        body = self.reconstructor(calibrated)
        if not body.valid:
            return self._reject(raw, calibrated, body, body.reason)
        try:
            reference = self.retargeter.retarget(body)
        except Exception as error:
            return self._reject(raw, calibrated, body, f"retargeting failed: {error}")
        valid, reason = self.safety.validate(reference, self.clock())
        if not valid:
            return self._reject(raw, calibrated, body, reason)
        self.sink.publish(reference)
        return PipelineResult(raw, calibrated, body, reference, True)

    def close(self) -> None:
        self.sink.close()
        self.provider.close()
