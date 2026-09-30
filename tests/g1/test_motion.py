import tempfile
import unittest
from pathlib import Path
import numpy as np
from g1_teleop.motion import KinematicModel, MotionLibrary, DEFAULT_Q, JOINT_NAMES
from g1_teleop.motion.procedural import generate_standing, generate_reaching, velocity_curriculum
from g1_teleop.motion.library import merge_libraries


class MotionTests(unittest.TestCase):
    def test_exact_usd_zero_chain_geometry(self):
        m=KinematicModel();p,r=m.forward(np.zeros(29))
        np.testing.assert_allclose(p['left_hip_pitch_link'],[0,.064452,-.1027],atol=1e-7)
        np.testing.assert_allclose(p['head_link'],[0,0,0],atol=1e-7)
        np.testing.assert_allclose(m.keypoints(np.zeros(29))[0],[0,0,.45],atol=1e-7)
        for rot in r.values(): np.testing.assert_allclose(rot@rot.T,np.eye(3),atol=2e-6)
        self.assertEqual(tuple(m.joint_names),JOINT_NAMES)

    def test_numpy_torch_fk_and_gradient(self):
        try: import torch
        except ImportError: self.skipTest('Optional retargeting dependency torch absent')
        m=KinematicModel();q=DEFAULT_Q+np.random.default_rng(4).normal(0,.07,(3,29))
        t=torch.tensor(q,dtype=torch.float64,requires_grad=True)
        p,r=m.forward(q);pt,rt=m.forward_torch(t)
        for key in p:np.testing.assert_allclose(pt[key].detach(),p[key],atol=1e-10)
        pt['left_wrist_yaw_link'][:,0].sum().backward()
        plus=q.copy();minus=q.copy();plus[:,15]+=1e-5;minus[:,15]-=1e-5
        numeric=(m.forward(plus)[0]['left_wrist_yaw_link'][:,0]-m.forward(minus)[0]['left_wrist_yaw_link'][:,0])/2e-5
        np.testing.assert_allclose(t.grad.numpy()[:,15],numeric,rtol=1e-5,atol=1e-8)

    def test_dataset_split_sampling_limits_and_fk(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'motion.npz';generate_standing(path,seconds=2,clip_count=5)
            lib=MotionLibrary(path);m=KinematicModel()
            for split in ('train','test'):
                ids,t,ref=lib.sample_batch(50,split,np.random.default_rng(1))
                self.assertTrue(np.all(lib.data['split'][ids]==(split=='test')))
                self.assertTrue(np.all(ref['q']>=m.lower-1e-6));self.assertTrue(np.all(ref['q']<=m.upper+1e-6))
                # Linear interpolation approximates FK closely at 30 Hz.
                np.testing.assert_allclose(m.keypoints(ref['q']),ref['keypoints'],atol=1e-4)
            self.assertEqual(set(np.unique(lib.data['clip_id'])),set(range(5)))
            a=lib.sample([0,1],[1e6,1e6]);self.assertTrue(a['finished'].all())
            np.testing.assert_allclose(a['q'][0],lib.data['q'][lib.data['clip_length'][0]-1])
            with self.assertRaises(ValueError):lib.sample([-1],[0])
            with self.assertRaises(ValueError):lib.sample([0],[np.nan])

    def test_broader_reaching_and_merge_preserve_heldouts(self):
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'stand.npz';b=Path(folder)/'reach.npz';out=Path(folder)/'mixed.npz'
            generate_standing(a,seconds=1,clip_count=5);generate_reaching(b,seconds=1,clip_count=5)
            merge_libraries([a,b],out);lib=MotionLibrary(out)
            self.assertEqual(len(lib.clip_names),10)
            np.testing.assert_array_equal(lib.data['split'],np.r_[MotionLibrary(a).data['split'],MotionLibrary(b).data['split']])
            m=KinematicModel();center=(m.lower+m.upper)/2
            rq=MotionLibrary(b).data['q'];self.assertTrue(np.all(rq>=center+(m.lower-center)*.9-1e-6))
            self.assertTrue(np.all(rq<=center+(m.upper-center)*.9+1e-6))

    def test_standing_foot_height_and_separate_velocity_curriculum(self):
        m=KinematicModel();p,_=m.forward(DEFAULT_Q);height=m.nominal_root_height()
        for side in ('left','right'):
            self.assertAlmostEqual(p[side+'_ankle_roll_link'][2]+height,.025,places=5)
        np.testing.assert_array_equal(velocity_curriculum(0.,np.arange(50)),np.zeros((50,3)))
        v=velocity_curriculum(1.,np.arange(50))
        self.assertTrue(np.all(np.abs(v)<=np.array([.25,.12,.35])+1e-8))

if __name__=='__main__':unittest.main()
