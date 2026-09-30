"""One G1 29-body-DoF Dex1 articulation shared by learning and deployment.

Configuration follows Unitree's G129_CFG_WITH_DEX1_WHOLEBODY (Apache-2.0),
with explicit gains and limits so installed upstream files need no edits.
Import only after Isaac Lab's AppLauncher has started Isaac Sim.
"""
from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

BODY_JOINT_NAMES = [
    *(f"{side}_{joint}_joint" for side in ("left", "right")
      for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")),
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    *(f"{side}_{joint}_joint" for side in ("left", "right")
      for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                    "wrist_roll", "wrist_pitch", "wrist_yaw")),
]
HAND_JOINT_NAMES = [f"{side}_hand_Joint{finger}_1" for side in ("left", "right") for finger in (1, 2)]
TRACKED_BODY_NAMES = ["head_link", "left_wrist_yaw_link", "right_wrist_yaw_link"]
FOOT_BODY_NAMES = ["left_ankle_roll_link", "right_ankle_roll_link"]


def resolve_usd_path() -> str:
    override = os.environ.get("G1_USD_PATH")
    repo = Path(__file__).resolve().parents[2]
    relative = Path("g1-29dof_wholebody_dex1/g1_29dof_with_dex1_rev_1_0.usd")
    choices = [Path(override).expanduser()] if override else [
        repo / "assets/robots" / relative,
        repo / "assets/robot" / relative,
        Path.home() / "isaac_vr_humanoid_teleop/unitree_sim_isaaclab/assets/robots" / relative,
    ]
    for path in choices:
        if path.is_file():
            return str(path.resolve())
    raise FileNotFoundError("G1 Dex1 free-base USD is missing. Set G1_USD_PATH to the downloaded "
                            "g1_29dof_with_dex1_rev_1_0.usd. Checked: " + ", ".join(map(str, choices)))


def make_robot_cfg() -> ArticulationCfg:
    return ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=resolve_usd_path(), activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False, retain_accelerations=True, linear_damping=0.0,
                angular_damping=0.0, max_linear_velocity=1000.0,
                max_angular_velocity=1000.0, max_depenetration_velocity=1.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=4,
                solver_velocity_iteration_count=1),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.80),
            joint_pos={".*_hip_pitch_joint": -0.20, ".*_knee_joint": 0.42,
                       ".*_ankle_pitch_joint": -0.23, ".*_elbow_joint": 0.87,
                       "left_shoulder_roll_joint": 0.18, "right_shoulder_roll_joint": -0.18,
                       ".*_shoulder_pitch_joint": 0.35,
                       **{name: 0.024 for name in HAND_JOINT_NAMES}},
            joint_vel={".*": 0.0}),
        soft_joint_pos_limit_factor=0.90,
        actuators={
            "legs_waist": ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_.*", ".*_knee_joint", "waist_.*"],
                effort_limit_sim={".*_hip_pitch_joint": 88., ".*_hip_roll_joint": 139.,
                    ".*_hip_yaw_joint": 88., ".*_knee_joint": 139., "waist_yaw_joint": 88.,
                    "waist_roll_joint": 35., "waist_pitch_joint": 35.},
                velocity_limit_sim={".*_hip_pitch_joint": 32., ".*_hip_roll_joint": 20.,
                    ".*_hip_yaw_joint": 32., ".*_knee_joint": 20., "waist_yaw_joint": 32.,
                    "waist_roll_joint": 30., "waist_pitch_joint": 30.},
                stiffness={".*_hip_pitch_joint": 200., ".*_hip_roll_joint": 150.,
                    ".*_hip_yaw_joint": 150., ".*_knee_joint": 200., "waist_.*": 200.},
                damping=5., armature=0.01),
            "ankles": ImplicitActuatorCfg(joint_names_expr=[".*_ankle_.*"],
                effort_limit_sim=35., velocity_limit_sim=30., stiffness=20., damping=2., armature=0.01),
            "shoulders": ImplicitActuatorCfg(joint_names_expr=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
                effort_limit_sim=25., velocity_limit_sim=37., stiffness=100., damping=2., armature=0.01),
            "arms": ImplicitActuatorCfg(joint_names_expr=[".*_shoulder_yaw_joint", ".*_elbow_joint"],
                effort_limit_sim=25., velocity_limit_sim=37., stiffness=50., damping=2., armature=0.01),
            "wrists": ImplicitActuatorCfg(joint_names_expr=[".*_wrist_.*"],
                effort_limit_sim={".*_wrist_roll_joint": 25., ".*_wrist_pitch_joint": 5., ".*_wrist_yaw_joint": 5.},
                velocity_limit_sim={".*_wrist_roll_joint": 37., ".*_wrist_pitch_joint": 22., ".*_wrist_yaw_joint": 22.},
                stiffness=40., damping=2., armature=0.01),
            "hands": ImplicitActuatorCfg(joint_names_expr=HAND_JOINT_NAMES,
                effort_limit_sim=20., velocity_limit_sim=1., stiffness=800., damping=3., friction=200.),
        },
    )
