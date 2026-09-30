#!/usr/bin/env python3
"""Export numeric G1 body geometry from the exact deployment USD; needs pxr."""
import argparse, hashlib, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from g1_teleop.motion.kinematics import JOINT_NAMES, SPARSE_LINKS, SPARSE_OFFSETS, quaternion_matrix


def export(usd, output):
    from pxr import Usd, UsdPhysics
    import numpy as np
    stage = Usd.Stage.Open(str(usd))
    joints = []
    for p in stage.Traverse():
        if not p.IsA(UsdPhysics.Joint) or p.GetName() not in (*JOINT_NAMES, 'head_joint'):
            continue
        j = UsdPhysics.Joint(p)
        def quat(attr):
            v = attr.Get(); return quaternion_matrix([v.GetReal(), *v.GetImaginary()]).tolist()
        axis = [0., 0., 0.]
        if p.GetName() in JOINT_NAMES:
            axis['XYZ'.index(str(UsdPhysics.RevoluteJoint(p).GetAxisAttr().Get()))] = 1.
        joints.append(dict(name=p.GetName(), parent=j.GetBody0Rel().GetTargets()[0].name,
                           child=j.GetBody1Rel().GetTargets()[0].name, axis=axis,
                           p0=list(j.GetLocalPos0Attr().Get()), p1=list(j.GetLocalPos1Attr().Get()),
                           r0=quat(j.GetLocalRot0Attr()), r1=quat(j.GetLocalRot1Attr())))
    ordered = []; seen = {'pelvis'}
    while joints:
        ready = [j for j in joints if j['parent'] in seen]
        if not ready: raise ValueError('Incomplete or cyclic USD body chain')
        for j in ready:
            ordered.append(j); seen.add(j['child']); joints.remove(j)
    ranges = {}
    for p in stage.Traverse():
        if p.GetName() in JOINT_NAMES and p.IsA(UsdPhysics.RevoluteJoint):
            j = UsdPhysics.RevoluteJoint(p)
            ranges[p.GetName()] = np.deg2rad([j.GetLowerLimitAttr().Get(), j.GetUpperLimitAttr().Get()]).tolist()
    layers = [l for l in stage.GetUsedLayers() if Path(l.realPath).is_file()]
    cfg = dict(schema=1, source='Unitree unitree_sim_isaaclab G1 29DoF wholebody Dex1 USD',
               usd_name=Path(usd).name, source_layers={Path(l.realPath).name: hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest() for l in layers},
               root='pelvis', joint_names=list(JOINT_NAMES), joints=ordered,
               lower=[ranges[n][0] for n in JOINT_NAMES], upper=[ranges[n][1] for n in JOINT_NAMES],
               sparse_links=list(SPARSE_LINKS), sparse_offsets=SPARSE_OFFSETS.tolist())
    Path(output).write_text(json.dumps(cfg, indent=2)+'\n')
    print(output)

if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--usd',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();export(a.usd,a.output)
