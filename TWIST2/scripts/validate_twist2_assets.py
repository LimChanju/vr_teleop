#!/usr/bin/env python3
"""Static and CPU inference checks for official TWIST2 deploy assets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import pickle
import xml.etree.ElementTree as ET

import numpy as np
import onnx
import onnxruntime as ort


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_checkpoint(path: Path) -> dict:
    model = onnx.load(path)
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    input_meta, output_meta = session.get_inputs()[0], session.get_outputs()[0]
    if input_meta.shape[1] != 1432 or output_meta.shape[1] != 29:
        raise ValueError(f"unexpected ONNX contract: {input_meta.shape} -> {output_meta.shape}")
    output = session.run(None, {input_meta.name: np.zeros((1, 1432), np.float32)})[0]
    if output.shape != (1, 29) or not np.all(np.isfinite(output)):
        raise ValueError("ONNX CPU smoke inference returned invalid output")
    return {"path": str(path.relative_to(ROOT)), "sha256": sha256(path),
            "input": input_meta.shape, "output": output_meta.shape, "cpu_inference_finite": True}


def validate_motion(path: Path) -> dict:
    # These are trusted files tracked by the official repository. Never use this
    # loader on an untrusted pickle.
    with path.open("rb") as stream:
        data = pickle.load(stream)
    required = {"fps", "root_pos", "root_rot", "dof_pos", "local_body_pos", "link_body_list"}
    if not required.issubset(data):
        raise ValueError(f"motion missing keys: {required - set(data)}")
    q = np.asarray(data["dof_pos"])
    if q.ndim != 2 or q.shape[1] != 29 or not np.all(np.isfinite(q)):
        raise ValueError(f"invalid motion q shape: {q.shape}")
    return {"path": str(path.relative_to(ROOT)), "sha256": sha256(path),
            "frames": q.shape[0], "dof": q.shape[1], "fps": float(data["fps"])}


def actuator_names(path: Path) -> list[str]:
    actuator = ET.parse(path).getroot().find("actuator")
    return [node.attrib["joint"] for node in actuator if "joint" in node.attrib]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    checkpoints = [validate_checkpoint(path) for path in sorted((ROOT / "assets/ckpts").glob("*.onnx"))]
    motions = [validate_motion(path) for path in sorted((ROOT / "assets/example_motions").glob("*.pkl"))]
    deploy_joints = actuator_names(ROOT / "assets/g1/g1_sim2sim_29dof.xml")
    gmr_joints = actuator_names(ROOT / "external/GMR/assets/unitree_g1/g1_mocap_29dof.xml")
    if deploy_joints != gmr_joints or len(deploy_joints) != 29:
        raise ValueError("GMR and TWIST2 joint ordering differ")
    report = {"checkpoints": checkpoints, "example_motions": motions,
              "joint_order": deploy_joints, "joint_order_match": True}
    rendered = json.dumps(report, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
