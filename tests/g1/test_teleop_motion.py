import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

from g1_teleop.motion.kinematics import KinematicModel, DEFAULT_Q
from g1_teleop.motion.library import MotionLibrary
from g1_teleop.motion.teleop_curriculum import random_targets, optimize_targets, generate_teleop, soft_limits


class TeleopMotionTests(unittest.TestCase):
    def test_independent_bounded_reproducible_trajectories(self):
        a, knots = random_targets(71)
        b, _ = random_targets(71)
        c, _ = random_targets(97)
        np.testing.assert_array_equal(a, b)
        self.assertFalse(np.array_equal(a, c))
        self.assertEqual(a.shape, (480, 3, 3))
        np.testing.assert_allclose(a[[0, -1]], 0, atol=1e-14)
        self.assertTrue(np.all(np.abs(a[:, 1:]) <= .08))
        self.assertTrue(np.all(np.abs(a[:, 0, :2]) <= .03))
        self.assertTrue(np.all(a[:, 0, 2] >= -.05))
        self.assertTrue(np.all(a[:, 0, 2] <= .005))
        self.assertEqual(len({tuple(k['times']) for k in knots}), 9)
        for seed in range(30):
            _, sampled_knots = random_targets(seed)
            for k in sampled_knots:
                intervals=np.diff(k['times'])
                self.assertTrue(np.all(intervals>=1.5-1e-12))
                self.assertTrue(np.all(intervals<=3.5+1e-12))
        # Every hand axis actually moves; copying one arm or dropping Z fails.
        self.assertTrue(np.all(np.ptp(a[:, 1:], axis=0) > .035))

    def test_ik_satisfies_cartesian_feet_and_soft_limits(self):
        try:
            import torch
            import scipy
        except ImportError:
            self.skipTest('IK requires optional torch/scipy')
        model=KinematicModel()
        offsets,_=random_targets(42,seconds=4)
        q,heights,reports=optimize_targets(model,offsets[None],iterations=1000)
        lo,hi=soft_limits(model)
        self.assertTrue(np.all(q>=lo-1e-7));self.assertTrue(np.all(q<=hi+1e-7))
        self.assertLess(reports[0]['stance_foot_error_m']['max'], .0003)
        self.assertLess(reports[0]['stance_foot_tilt_deg']['max'], .05)
        self.assertLess(reports[0]['point_error_m']['head']['max'], .003)
        for name in ('left_wrist','right_wrist'):
            self.assertLess(reports[0]['point_error_m'][name]['max'], .005)
        # Recompute with the independent NumPy FK, including the Z-frame shift.
        actual=model.keypoints(q[0]);actual[...,2]+=heights[0,:,None]-float(model.nominal_root_height())
        desired=model.keypoints(DEFAULT_Q)+offsets
        self.assertGreater(np.ptp(actual[:,1,2]), .07)
        np.testing.assert_allclose(actual[:,1:],desired[:,1:],atol=.005)

    def test_portable_dataset_split_and_no_overwrite(self):
        try:
            import torch
            import scipy
        except ImportError:
            self.skipTest('IK requires optional torch/scipy')
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'synthetic.npz'
            # Schema/split check only; the full 1000-step IK is tested above.
            generate_teleop(out,clips=5,seconds=4,iterations=1,batch_size=5,progress=None)
            library=MotionLibrary(out)
            self.assertEqual(library.data['q'].shape,(600,29))
            np.testing.assert_array_equal(library.data['split'],[0,0,0,0,1])
            np.testing.assert_allclose(library.data['keypoints'],KinematicModel().keypoints(library.data['q']),atol=3e-8)
            metadata=json.loads(out.with_suffix('.json').read_text())
            self.assertNotEqual(metadata['train_seed'],metadata['test_seed'])
            self.assertFalse(metadata['locomotion'])
            before=out.read_bytes()
            with self.assertRaises(FileExistsError):
                generate_teleop(out,clips=5,seconds=4,iterations=1,progress=None)
            self.assertEqual(out.read_bytes(),before)


if __name__=='__main__':unittest.main()
