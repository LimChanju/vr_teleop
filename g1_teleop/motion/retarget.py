"""Cartesian H1-to-G1 retargeting with joint limits and stationary-foot IK.

H1 angles are used only by H1 forward kinematics. G1 angles are independently
optimized against Cartesian segment directions using its exact USD geometry.
"""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import numpy as np
from .kinematics import KinematicModel, DEFAULT_Q, quaternion_matrix
from .library import save_clips


def h1_model(xml_path):
    root=ET.parse(xml_path).getroot().find('worldbody/body')
    joints=[]; names=[]; lower=[]; upper=[]
    def visit(node, parent=None):
        name=node.attrib['name']
        if parent is not None:
            joint=node.find('joint'); axis=[0.,0.,0.]; jname='fixed_'+name
            if joint is not None:
                if np.linalg.norm(np.fromstring(joint.attrib.get('pos','0 0 0'),sep=' '))>1e-7:
                    raise ValueError('Non-origin H1 hinge requires an explicit child joint frame')
                jname=joint.attrib['name'];axis=np.fromstring(joint.attrib.get('axis','0 0 1'),sep=' ').tolist()
                names.append(jname);limits=np.fromstring(joint.attrib.get('range','-3.14 3.14'),sep=' ')
                lower.append(limits[0]);upper.append(limits[1])
            joints.append(dict(name=jname,parent=parent,child=name,axis=axis,
                               p0=np.fromstring(node.attrib.get('pos','0 0 0'),sep=' ').tolist(),p1=[0.,0.,0.],
                               r0=quaternion_matrix(np.fromstring(node.attrib.get('quat','1 0 0 0'),sep=' ')).tolist(),r1=np.eye(3).tolist()))
        for child in node.findall('body'):visit(child,name)
    visit(root)
    return KinematicModel(config=dict(root=root.attrib['name'],joint_names=names,joints=joints,lower=lower,upper=upper))


def direction(v):
    return v/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),1e-8)


def cartesian_targets(h1, g1, hq):
    hp,hr=h1.forward(hq); gp,gr=g1.forward(DEFAULT_Q)
    n=len(hq); target={}; weights={}; root_height=float(g1.nominal_root_height())
    for side in ('left','right'):
        shoulder=side+'_shoulder_roll_link'; elbow=side+'_elbow_link'; wrist=side+'_wrist_yaw_link'
        # Preserve source upper-arm/forearm directions while using G1 lengths.
        anchor=np.einsum('nij,j->ni',hr['torso_link'],gp[shoulder])
        arm_length=np.linalg.norm(gp[elbow]-gp[shoulder])
        source_elbow=hp[elbow]
        upper_direction=direction(source_elbow-hp[shoulder])
        forearm_direction=np.einsum('nij,j->ni',hr[elbow],np.array([1.,0,0]))
        forearm_length=np.linalg.norm(gp[wrist]-gp[elbow])
        target[elbow]=anchor+arm_length*upper_direction; weights[elbow]=2.
        target[wrist]=target[elbow]+forearm_length*forearm_direction;weights[wrist]=6.
        knee=side+'_knee_link';hip=side+'_hip_yaw_link';foot=side+'_ankle_roll_link'
        thigh_length=np.linalg.norm(gp[knee]-gp[hip])
        source_thigh=direction(hp[knee]-hp[side+'_hip_pitch_link'])
        target[knee]=gp[hip]+thigh_length*source_thigh;weights[knee]=1.0
        # Stable-punch source has zero root translation; both feet are stance feet.
        # World stance constraints are applied separately and strongly weighted.
        target[foot]=np.broadcast_to(gp[foot],(n,3)).copy();weights[foot]=0.
    return target,weights,hr['torso_link'],root_height


def optimize_clip(g1,h1,clip,iterations=300,device='cpu'):
    import torch
    from scipy.ndimage import gaussian_filter1d
    # Get H1 joint angles from axis-angle data, validating against source dof.
    aa=np.asarray(clip['pose_aa'])[:,1:1+len(h1.joint_names)]
    axes=np.asarray([j['axis'] for j in h1.joints if j['name'] in h1.joint_names])
    hq=np.sum(aa*axes[None],axis=-1)
    if not np.allclose(aa, hq[...,None]*axes[None], atol=2e-5):
        raise ValueError('H1 pose axis angles do not align with skeleton hinge axes')
    source_dof_discrepancy = float(np.max(np.abs(hq-np.asarray(clip['dof']))))
    # stable_punch intentionally edits leg pose_aa without updating its legacy dof.
    # Match official MotionLibH1, which recomputes joint positions from pose_aa.
    targets,weights,torso_target,height0=cartesian_targets(h1,g1,hq)
    n=len(hq);dtype=torch.float32
    tensor=lambda x:torch.as_tensor(x,dtype=dtype,device=device)
    q=torch.nn.Parameter(tensor(np.tile(DEFAULT_Q,(n,1))))
    height=torch.nn.Parameter(torch.full((n,),height0,dtype=dtype,device=device))
    target={k:tensor(v) for k,v in targets.items()};target_rotation=tensor(torso_target)
    lower=tensor(g1.lower);upper=tensor(g1.upper);nominal=tensor(DEFAULT_Q)
    optimizer=torch.optim.Adam([q,height],lr=.035)
    pnom,_=g1.forward(DEFAULT_Q)
    foot_world={s:tensor(pnom[s+'_ankle_roll_link']+np.array([0,0,height0])) for s in ('left','right')}
    # No actuation orientation in sparse targets: wrists are regularized to zero.
    wrist_indices=[19,20,21,26,27,28]
    last_loss=None
    for i in range(iterations):
        optimizer.zero_grad();p,r=g1.forward_torch(q)
        loss=q.sum()*0
        for name in target:
            if weights[name]: loss=loss+weights[name]*((p[name]-target[name])**2).mean()
        for side in ('left','right'):
            name=side+'_ankle_roll_link'
            world=p[name]+torch.stack([height*0,height*0,height],dim=-1)
            loss=loss+20*((world-foot_world[side])**2).mean()
            # Flat sole: penalize roll/pitch without penalizing toe yaw.
            loss=loss+0.6*((r[name][...,:,2]-tensor([0,0,1]))**2).mean()
        loss=loss+.06*((r['torso_link']-target_rotation)**2).mean()
        loss=loss+.01*((q-nominal)**2).mean()+.025*(q[:,wrist_indices]**2).mean()
        if n>2:
            loss=loss+.06*((q[1:]-q[:-1])**2).mean()+.15*((q[2:]-2*q[1:-1]+q[:-2])**2).mean()
            loss=loss+2*((height[1:]-height[:-1])**2).mean()
        loss.backward();optimizer.step()
        with torch.no_grad():
            q.copy_(torch.maximum(torch.minimum(q,upper),lower));height.clamp_(.54,.84)
        last_loss=float(loss.detach())
    result=gaussian_filter1d(q.detach().cpu().numpy(),sigma=.7,axis=0,mode='nearest')
    result=np.clip(result,g1.lower,g1.upper)
    heights=gaussian_filter1d(height.detach().cpu().numpy(),sigma=.7,axis=0,mode='nearest')
    p,r=g1.forward(result)
    point_errors={name:np.linalg.norm(p[name]-target_np,axis=-1) for name,target_np in targets.items() if weights[name]}
    feet=np.stack([p[s+'_ankle_roll_link']+np.stack([heights*0,heights*0,heights],axis=-1) for s in ('left','right')],axis=1)
    nominal_world=np.stack([pnom[s+'_ankle_roll_link']+[0,0,height0] for s in ('left','right')])
    foot_error=np.linalg.norm(feet-nominal_world,axis=-1)
    report=dict(final_loss=last_loss,source_dof_vs_pose_max_difference_rad=source_dof_discrepancy,point_mean_error_m={k:float(v.mean()) for k,v in point_errors.items()},
                point_p95_error_m={k:float(np.percentile(v,95)) for k,v in point_errors.items()},
                stance_foot_mean_error_m=float(foot_error.mean()),stance_foot_max_error_m=float(foot_error.max()),
                ankle_min_world_height_m=float(feet[...,2].min()),
                max_joint_speed_rad_s=float((np.abs(np.diff(result,axis=0))*clip['fps']).max()))
    return result,heights,report


def prepare_h1(source_path,h1_xml,output,iterations=300,device='cpu',max_clips=None):
    import joblib, torch
    if device=='cpu':torch.set_num_threads(min(4,torch.get_num_threads()))
    # Only load the explicitly supplied trusted upstream artifact; runtime NPZ is safe.
    data=joblib.load(source_path);g1=KinematicModel();h1=h1_model(h1_xml)
    names=sorted(data);names=names[:max_clips] if max_clips else names
    clips=[];reports={}
    for cid,name in enumerate(names):
        q,height,report=optimize_clip(g1,h1,data[name],iterations,device)
        split=int(cid%5==4)  # Whole clips held out, deterministic lexicographic ordering.
        clips.append(dict(name=name,q=q,root_height=height,split=split))
        reports[name]=report
        print('%d/%d %s wrists %.3f/%.3fm feet %.3fm'%(cid+1,len(names),name,
              report['point_mean_error_m']['left_wrist_yaw_link'],report['point_mean_error_m']['right_wrist_yaw_link'],report['stance_foot_mean_error_m']),flush=True)
    if len(clips)>1 and not any(c['split'] for c in clips):clips[-1]['split']=1
    metadata=dict(source='LeCAR-Lab human2humanoid stable_punch.pkl',
                  source_url='https://github.com/LeCAR-Lab/human2humanoid',source_sha256=hashlib.sha256(Path(source_path).read_bytes()).hexdigest(),
                  source_license='CC BY-NC 4.0; inherited dataset rights also apply. Research/noncommercial derivative.',
                  h1_skeleton_sha256=hashlib.sha256(Path(h1_xml).read_bytes()).hexdigest(),
                  method='H1 FK Cartesian directions -> G1 exact USD FK joint-limit constrained differentiable IK; foot stance and temporal smoothness',
                  iterations=iterations,motion_kind='stationary punch/reach; original root XY constant',locomotion=False,
                  root_orientation='identity; global H1 root transform removed for root-body task coordinates',
                  constraints='Both feet constrained to nominal world stance, torso rotation fitted, wrist angles regularized',
                  limitations='No collision/self-collision or dynamic feasibility solver; validate with simulator policy. Kinematic residuals are not learned policy accuracy.',
                  reports=reports)
    return save_clips(output,clips,metadata,fps=30)
