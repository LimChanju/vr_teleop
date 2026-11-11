from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]


EXPECTED = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint",
    "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def actuator_order(path):
    root = ET.parse(path).getroot()
    actuator = root.find("actuator")
    return [node.attrib["joint"] for node in actuator if "joint" in node.attrib]


def test_twist2_and_gmr_joint_orders_match():
    official = actuator_order(ROOT / "assets/g1/g1_sim2sim_29dof.xml")
    gmr = actuator_order(ROOT / "external/GMR/assets/unitree_g1/g1_mocap_29dof.xml")
    assert official == EXPECTED
    assert gmr == EXPECTED


def test_checkpoint_and_example_motion_present():
    checkpoint = ROOT / "assets/ckpts/twist2_1017_20k.onnx"
    assert checkpoint.stat().st_size == 3_184_908
    assert len(list((ROOT / "assets/example_motions").glob("*.pkl"))) == 10
