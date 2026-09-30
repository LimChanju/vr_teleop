"""Native XR anchor mathematics without starting Kit, SteamVR, or a headset.

Requires the real USD ``pxr`` bindings (available in the Isaac Sim environment).
The compositor and physics state are mocked; these are NOT hardware tests.
"""
import math
from types import SimpleNamespace
import unittest

import numpy as np

try:
    from pxr import Gf
except ImportError:
    Gf = None

from g1_teleop.sim.xr_view import G1XRView


class ArrayTensor:
    """Only the torch data access interface used by the view helper."""
    def __init__(self, values):
        self.values = np.asarray(values, dtype=float)

    def __getitem__(self, key):
        return ArrayTensor(self.values[key])

    def __float__(self):
        return float(self.values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values.copy()

    def tolist(self):
        return self.values.tolist()


class TransformOp:
    def __init__(self):
        self.value = Gf.Matrix4d(1.)
        self.write_count = 0

    def Set(self, matrix):
        self.value = Gf.Matrix4d(matrix)
        self.write_count += 1

    def Get(self):
        return Gf.Matrix4d(self.value)


class SimulatedHead:
    """Physical Y-up tracker plus a mock compositor's calibrated anchor space."""
    ROOM_TO_STAGE = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])

    def __init__(self, anchor):
        self.anchor = anchor
        self.physical = np.array([0., 1.65, 0.])
        self.calibration_physical = self.physical.copy()
        self.calibration_local = Gf.Vec3d(0., 0., 0.)

    def get_pose(self):
        return Gf.Matrix4d(1.).SetTranslateOnly(Gf.Vec3d(*self.physical))

    def get_virtual_world_pose(self):
        delta = self.ROOM_TO_STAGE @ (self.physical - self.calibration_physical)
        local = self.calibration_local + Gf.Vec3d(*delta)
        world = self.anchor.Get().Transform(local)
        return Gf.Matrix4d(1.).SetTranslateOnly(world)


class MockXRCore:
    def __init__(self, anchor):
        self.anchor = anchor
        self.head = SimulatedHead(anchor)
        self.teleports = []
        self.detach_count = 0
        self.disable_count = 0
        self.fail_detach = False
        self.fail_disable = False

    def get_input_device(self, name):
        if name != "/user/head":
            raise AssertionError("The renderer must not read controller command coordinates")
        return self.head

    def schedule_teleport_to_view(self, path, view):
        self.teleports.append((path, Gf.Matrix4d(view)))
        # Model execution of the scheduled operation at the next frame boundary.
        self.head.calibration_physical = self.head.physical.copy()
        self.head.calibration_local = self.anchor.Get().GetInverse().Transform(view.ExtractTranslation())

    def detach_stage_anchor(self):
        self.detach_count += 1
        if self.fail_detach:
            raise RuntimeError("mock detach failure")

    def request_disable_profile(self):
        self.disable_count += 1
        if self.fail_disable:
            raise RuntimeError("mock profile failure")


@unittest.skipUnless(Gf is not None, "Native XR math tests need Isaac Sim's pxr bindings")
class XRViewTests(unittest.TestCase):
    def setUp(self):
        self.root = np.zeros((1, 13))
        self.root[0, 2:4] = [0.80, 1.0]
        self.points = np.array([[[0., 0., 0.45], [0., .21, -.05], [0., -.21, -.05]]])
        self.env = SimpleNamespace(
            num_envs=1,
            robot=SimpleNamespace(data=SimpleNamespace(root_state_w=ArrayTensor(self.root))),
            scene=SimpleNamespace(env_origins=ArrayTensor([[0., 0., 0.]])),
            nominal_root_height=0.76792282,
            current_keypoints=lambda: ArrayTensor(self.points),
        )
        # Bypass extension startup: this test must not launch SteamVR.
        self.view = G1XRView.__new__(G1XRView)
        self.view.env = self.env
        self.view.anchor_path = "/World/G1TeleopXRAnchor"
        self.view.attached = False
        self.view._closed = False
        self.view.status = "waiting_for_headset"
        self.view._transform = TransformOp()
        self.view.core = MockXRCore(self.view._transform)

    def eye(self):
        return np.asarray(self.view.core.head.get_virtual_world_pose().ExtractTranslation())

    def test_first_attach_schedules_robot_head_view(self):
        self.assertTrue(self.view.update())
        self.assertEqual(len(self.view.core.teleports), 1)
        np.testing.assert_allclose(self.eye(), [0.10, 0., 1.21792282], atol=1e-9)
        self.assertIn("not_verified", self.view.status)

    def test_crouch_moves_camera_once_and_does_not_oscillate(self):
        self.view.update()
        initial = self.eye()
        # Operator crouches 0.20m; learned G1 also crouches 0.20m.
        self.view.core.head.physical[1] -= .20
        self.root[0, 2] -= .20
        self.points[0, 0, 2] -= .20
        for _ in range(10):
            self.assertTrue(self.view.update())
            np.testing.assert_allclose(self.eye(), initial + [0., 0., -.20], atol=1e-9)
        self.assertEqual(len(self.view.core.teleports), 1)

    def test_head_translation_without_robot_motion_is_cancelled(self):
        self.view.update()
        initial = self.eye()
        self.view.core.head.physical += [.17, -.09, .23]
        self.view.update()
        np.testing.assert_allclose(self.eye(), initial, atol=1e-9)

    def test_root_translation_and_yaw_move_camera_once(self):
        self.view.update()
        self.root[0, :2] = [1.2, -.4]
        self.root[0, 3:7] = [math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)]
        self.view.core.head.physical += [.12, -.05, -.09]
        for _ in range(3):
            self.view.update()
            np.testing.assert_allclose(self.eye(), [1.2, -.3, 1.21792282], atol=1e-9)
        direction = self.view._transform.Get().TransformDir(Gf.Vec3d(1., 0., 0.))
        np.testing.assert_allclose(direction, [0., 1., 0.], atol=1e-9)

    def test_recenter_reschedules_without_restarting_profile(self):
        self.view.update()
        self.view.recenter()
        self.view.update()
        self.assertEqual(len(self.view.core.teleports), 2)
        self.assertEqual(self.view.core.disable_count, 0)

    def test_missing_or_invalid_tracking_does_not_write_anchor(self):
        self.view.core.head = None
        self.assertFalse(self.view.update())
        self.assertEqual(self.view._transform.write_count, 0)
        self.view.core.head = SimulatedHead(self.view._transform)
        for value in ([0., 0., 0.], [float("nan"), 1., 0.]):
            self.view.core.head.physical = np.array(value)
            self.assertFalse(self.view.update())
        self.assertEqual(self.view._transform.write_count, 0)

    def test_cleanup_is_idempotent(self):
        self.view.update()
        self.view.close()
        self.view.close()
        self.assertEqual(self.view.core.detach_count, 1)
        self.assertEqual(self.view.core.disable_count, 1)
        self.assertFalse(self.view.update())
        self.assertEqual(self.view.status, "closed")

    def test_cleanup_continues_if_compositor_is_failing(self):
        self.view.core.fail_detach = self.view.core.fail_disable = True
        self.view.close()
        self.assertEqual(self.view.core.disable_count, 1)
        self.assertEqual(self.view.status, "closed_with_cleanup_errors")
        self.assertFalse(self.view.update())


if __name__ == "__main__":
    unittest.main()
