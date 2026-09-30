"""Run real reset code with CPU state doubles; no simulator or GPU is started."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from scripts.g1.run import set_teleop_hold_targets
from g1_teleop.runtime import TeleopGate
try:
    import torch
except ImportError:
    torch = None

ROOT = Path(__file__).resolve().parents[2]


def reset_harness():
    path = ROOT / 'g1_teleop/sim/env.py'
    tree = ast.parse(path.read_text())
    original = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'G1WholeBodyEnv')
    method = next(node for node in original.body if isinstance(node, ast.FunctionDef) and node.name == '_reset_idx')
    class BaseReset:
        def _reset_idx(self, env_ids):
            self.base_reset_ids = env_ids.clone()
    cls = ast.ClassDef(name='ResetHarness', bases=[ast.Name(id='BaseReset', ctx=ast.Load())],
                       keywords=[], body=[method], decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    namespace = {'BaseReset': BaseReset, 'torch': torch}
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace['ResetHarness']


def reset_fixture(*, external=False, reference=False):
    env = reset_harness()()
    env.cfg = SimpleNamespace(reference_state_initialization=True, randomize_reset=False,
                             action_scale=.5, action_clip=4., episode_length_s=20.)
    env.num_envs = 2
    env.device = 'cpu'
    env.body_joint_ids = list(range(29))
    state = torch.zeros(2, 13)
    state[:, 2] = .80  # The legacy USD default that must not select reset height.
    state[:, 3] = 1.
    data = SimpleNamespace(default_joint_pos=torch.zeros(2, 33), default_root_state=state)
    env.robot = SimpleNamespace(data=data, _ALL_INDICES=torch.arange(2), reset=Mock(),
        write_root_pose_to_sim=Mock(), write_root_velocity_to_sim=Mock(),
        write_joint_state_to_sim=Mock(), set_joint_position_target=Mock())
    env.scene = SimpleNamespace(env_origins=torch.tensor([[0.,0.,0.], [3.,0.,2.]]))
    env._sample_reference = Mock()
    env._update_reference = Mock()
    env.reset_target_history = Mock()
    env.external_mode = external
    env._motion = {} if reference else None
    env.nominal_root_height = .76792282
    env.nominal_q = torch.zeros(29)
    env.reference_q = torch.full((2,29), .2)
    env.reference_height = torch.tensor([.68,.72])
    env.nominal_keypoints = torch.tensor([[0.,0.,.45],[0.,.214,-.0568],[0.,-.214,-.0568]])
    env.target_positions = torch.full((2,3,3), .5)
    env.command_velocity = torch.ones(2,3)
    env.joint_limits = torch.tensor([[-1.,1.]]).expand(29,2)
    env.actions = torch.zeros(2,29)
    env.previous_actions = torch.zeros_like(env.actions)
    env.joint_targets = torch.zeros(2,33)
    env.extras = {}
    env._episode_sums = {}
    return env


@unittest.skipIf(torch is None, 'CPU PyTorch is required')
class GroundedResetTests(unittest.TestCase):
    def test_without_reference_uses_fk_nominal_plus_same_clearance(self):
        env=reset_fixture()
        env._reset_idx(None)
        pose=env.robot.write_root_pose_to_sim.call_args.args[0]
        expected=env.scene.env_origins[:,2] + .76792282 + .015
        torch.testing.assert_close(pose[:,2],expected)
        self.assertNotAlmostEqual(float(pose[0,2]),.80,places=5)

    def test_reference_initialization_still_uses_reference_height_and_pose(self):
        env=reset_fixture(reference=True)
        env._reset_idx(torch.arange(2))
        pose=env.robot.write_root_pose_to_sim.call_args.args[0]
        torch.testing.assert_close(pose[:,2],torch.tensor([.695,2.735]))
        q=env.robot.write_joint_state_to_sim.call_args.args[0]
        torch.testing.assert_close(q[:,:29],env.reference_q)

    def test_external_reset_clears_only_selected_targets_and_velocity(self):
        env=reset_fixture(external=True,reference=True)
        env._reset_idx(torch.tensor([1]))
        torch.testing.assert_close(env.target_positions[1],env.nominal_keypoints)
        torch.testing.assert_close(env.target_positions[0],torch.full((3,3),.5))
        torch.testing.assert_close(env.command_velocity,torch.tensor([[1.,1.,1.],[0.,0.,0.]]))
        pose=env.robot.write_root_pose_to_sim.call_args.args[0]
        self.assertAlmostEqual(float(pose[0,2]),2.+.76792282+.015,places=6)
        q=env.robot.write_joint_state_to_sim.call_args.args[0]
        torch.testing.assert_close(q,torch.zeros(1,33))
        env.reset_target_history.assert_called_once()

    def test_startup_manual_and_automatic_reset_hold_grounded_markers(self):
        env=reset_fixture(external=True)
        env.current_keypoints=Mock(return_value=env.nominal_keypoints.expand(2,-1,-1)+torch.tensor([0.,0.,.03207718]))
        env.set_external_targets=Mock()
        for _event in ('startup','manual_reset','automatic_reset'):
            set_teleop_hold_targets(env,after_reset=True)
            positions,commands=env.set_external_targets.call_args.args
            torch.testing.assert_close(positions,env.nominal_keypoints.expand(2,-1,-1))
            torch.testing.assert_close(commands,torch.zeros(2,3))
            self.assertTrue(env.set_external_targets.call_args.kwargs['reset_velocity'])
        env.current_keypoints.assert_not_called()

    def test_ordinary_disarm_preserves_actual_reachable_pose(self):
        env=reset_fixture(external=True)
        reached=env.nominal_keypoints.expand(2,-1,-1)+.012
        env.current_keypoints=Mock(return_value=reached)
        env.set_external_targets=Mock()
        set_teleop_hold_targets(env)
        positions,commands=env.set_external_targets.call_args.args
        torch.testing.assert_close(positions,reached)
        torch.testing.assert_close(commands,torch.zeros(2,3))
        self.assertTrue(env.set_external_targets.call_args.kwargs['reset_velocity'])

    def test_automatic_reset_gate_and_runner_active_flag_stay_disarmed(self):
        # Execute the actual post-step reset block: the next iteration must not
        # treat the reset as an ordinary disarm and re-freeze airborne markers.
        tree=ast.parse((ROOT/'scripts/g1/run.py').read_text())
        node=next(node for node in ast.walk(tree) if isinstance(node,ast.If)
                  and ast.unparse(node.test)=='receiver and done_mask.any()')
        env=reset_fixture(external=True)
        env.set_external_targets=Mock()
        env.current_keypoints=Mock(side_effect=AssertionError('airborne markers must not be sampled'))
        gate=TeleopGate();gate.active=True
        ns={'receiver':True,'done_mask':torch.tensor([True,False]),'fallen':torch.tensor([True,False]),
            'gate':gate,'raw':env,'active':True,'set_teleop_hold_targets':set_teleop_hold_targets,'print':lambda *a,**k:None}
        exec(compile(ast.Module(body=[node],type_ignores=[]),'post_step_reset','exec'),ns)
        self.assertFalse(ns['active'])
        self.assertFalse(gate.active)
        self.assertTrue(gate.latched)
        self.assertEqual(gate.reason,'fall')
        env.current_keypoints.assert_not_called()


if __name__=='__main__':
    unittest.main()
