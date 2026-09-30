"""Portable exact G1 body kinematics, exported from the deployment USD.

Coordinates: pelvis body origin; +X forward, +Y left, +Z up; radians/metres.
The head tracking point has a documented local offset: the USD head *origin*
is close to the pelvis and must not be mistaken for the headset position.
"""
from pathlib import Path
import json
import numpy as np

JOINT_NAMES = tuple(
    [f'{side}_{part}_joint' for side in ('left', 'right')
     for part in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')]
    + ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
    + [f'{side}_{part}_joint' for side in ('left', 'right')
       for part in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                    'wrist_roll', 'wrist_pitch', 'wrist_yaw')])
SPARSE_LINKS = ('head_link', 'left_wrist_yaw_link', 'right_wrist_yaw_link')
SPARSE_OFFSETS = np.array([[0., 0., .45], [0., 0., 0.], [0., 0., 0.]])
DEFAULT_Q = np.array([-.2, 0, 0, .42, -.23, 0] * 2 + [0] * 3
                     + [.35, .18, 0, .87, 0, 0, 0] + [.35, -.18, 0, .87, 0, 0, 0], dtype=float)


def quaternion_matrix(wxyz):
    w, x, y, z = np.asarray(wxyz, dtype=float) / np.linalg.norm(wxyz)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class KinematicModel:
    def __init__(self, path=None, config=None):
        if config is None:
            path = Path(path or Path(__file__).with_name('g1_29dof_kinematics.json'))
            config = json.loads(path.read_text())
        self.config = config
        self.joint_names = tuple(config['joint_names'])
        self.joints = config['joints']
        self.links = tuple([config['root']] + [j['child'] for j in self.joints])
        self.root = config['root']
        self.lower = np.array(config['lower'])
        self.upper = np.array(config['upper'])
        self._index = {name: i for i, name in enumerate(self.joint_names)}
        self._torch_constants = {}

    def forward(self, q):
        """Return (position_dict, rotation_dict) for q[...,joint_count]."""
        q = np.asarray(q, dtype=float)
        if q.shape[-1] != len(self.joint_names) or not np.isfinite(q).all():
            raise ValueError('Expected finite q[..., %d]' % len(self.joint_names))
        shape = q.shape[:-1]
        p = {self.root: np.zeros(shape + (3,))}
        r = {self.root: np.broadcast_to(np.eye(3), shape + (3, 3))}
        for j in self.joints:
            axis = np.array(j['axis'])
            qj = q[..., self._index[j['name']]] if j['name'] in self._index else np.zeros(shape)
            k = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
            motion = np.eye(3) + np.sin(qj)[..., None, None]*k + (1-np.cos(qj))[..., None, None]*(k@k)
            r0, r1 = np.array(j['r0']), np.array(j['r1'])
            local_r = r0 @ motion @ r1.T
            local_p = np.array(j['p0']) - np.einsum('...ij,j->...i', local_r, j['p1'])
            p[j['child']] = p[j['parent']] + np.einsum('...ij,...j->...i', r[j['parent']], local_p)
            r[j['child']] = r[j['parent']] @ local_r
        return p, r

    def forward_torch(self, q):
        """Differentiable FK; torch is an optional retargeting dependency."""
        import torch
        key = (str(q.device), q.dtype)
        if key not in self._torch_constants:
            constants = []
            for j in self.joints:
                axis = np.array(j['axis'])
                k = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
                constants.append(tuple(torch.as_tensor(x, dtype=q.dtype, device=q.device)
                                       for x in (j['r0'], j['r1'], j['p0'], j['p1'], k, k@k)))
            self._torch_constants[key] = constants
        eye = torch.eye(3, dtype=q.dtype, device=q.device)
        p = {self.root: torch.zeros(q.shape[:-1]+(3,), dtype=q.dtype, device=q.device)}
        r = {self.root: eye.expand(q.shape[:-1]+(3, 3))}
        for j, (r0, r1, p0, p1, k, k2) in zip(self.joints, self._torch_constants[key]):
            qj = q[..., self._index[j['name']]] if j['name'] in self._index else q[..., 0]*0
            motion = eye + qj.sin()[..., None, None]*k + (1-qj.cos())[..., None, None]*k2
            local_r = r0 @ motion @ r1.T
            local_p = p0 - (local_r @ p1[..., None])[..., 0]
            p[j['child']] = p[j['parent']] + (r[j['parent']] @ local_p[..., None])[..., 0]
            r[j['child']] = r[j['parent']] @ local_r
        return p, r

    def keypoints(self, q):
        p, r = self.forward(q)
        return np.stack([p[n]+np.einsum('...ij,j->...i', r[n], o)
                         for n, o in zip(SPARSE_LINKS, SPARSE_OFFSETS)], axis=-2)

    def nominal_root_height(self, q=DEFAULT_Q, sole_thickness=.025):
        p, _ = self.forward(q)
        return -np.minimum(p['left_ankle_roll_link'][..., 2], p['right_ankle_roll_link'][..., 2]) + sole_thickness
