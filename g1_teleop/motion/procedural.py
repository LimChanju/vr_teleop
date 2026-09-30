"""Reachable stationary curriculum and independent velocity-command schedule.

Synthetic joint trajectories are not mocap or dynamically validated locomotion.
"""
import numpy as np
from .kinematics import KinematicModel, DEFAULT_Q
from .library import save_clips


def velocity_curriculum(progress, time, max_forward=.25, max_lateral=.12, max_yaw=.35):
    """Bounded command targets for a locomotion reward, not a reference gait.

    Keep progress=0 during stationary motion pretraining. A nonzero command
    requires explicit locomotion reward/training; it does not create walking.
    """
    a=np.clip((np.asarray(progress)-.5)*2,0,1)
    t=np.asarray(time)
    return a[...,None]*np.stack([max_forward*np.sin(t*.25),max_lateral*np.sin(t*.19),max_yaw*np.sin(t*.17)],axis=-1)


def generate_standing(path, fps=30, seconds=12, clip_count=12):
    model=KinematicModel(); t=np.arange(round(seconds*fps))/fps; clips=[]
    for i in range(clip_count):
        q=np.tile(DEFAULT_Q,(len(t),1))
        strength=0.0 if i==0 else .12 + .16*((i-1)%4)/3
        phase=i*.83
        # Small, exactly reachable arm sweeps and knee flexion around standing.
        for start,sign in [(15,1),(22,-1)]:
            q[:,start] += strength*np.sin(.65*t+phase+sign*.5)
            q[:,start+1] += sign*strength*.45*np.sin(.43*t+phase)
            q[:,start+2] += strength*.45*np.sin(.39*t+phase)
            q[:,start+3] += strength*.8*np.sin(.56*t+phase+1)
        bend=.025*(1-np.cos(.4*t+phase)) if i else np.zeros_like(t)
        q[:,[0,6]]-=bend[:,None];q[:,[3,9]]+=2*bend[:,None];q[:,[4,10]]-=bend[:,None]
        q[:,12]=strength*.25*np.sin(.3*t+phase)
        q=np.clip(q,model.lower,model.upper)
        clips.append(dict(name='synthetic_stationary_%02d'%i,q=q,root_height=model.nominal_root_height(q),split=int(i>=clip_count-2)))
    return save_clips(path,clips,dict(source='Locally generated reachable G1 joint trajectories',license='Original project implementation',
                    motion_kind='synthetic stationary reaching and shallow squat',locomotion=False,
                    validation='Kinematic joint limits and matching USD FK; dynamic balance requires policy evaluation.'),fps)


def generate_reaching(path, fps=30, seconds=16, clip_count=20):
    """Broader reachable head/hand workspace for a second curriculum stage.

    Quasistatic arm sweeps, pelvis-height changes, and modest waist lean.
    No locomotion and no assertion of dynamic feasibility before RL evaluation.
    """
    model=KinematicModel();t=np.arange(round(seconds*fps))/fps;clips=[]
    center=(model.lower+model.upper)/2
    soft_low=center+(model.lower-center)*.9;soft_high=center+(model.upper-center)*.9
    for i in range(clip_count):
        rng=np.random.default_rng(6400+i);q=np.tile(DEFAULT_Q,(len(t),1))
        phase=rng.uniform(0,2*np.pi,8)
        for offset,start in enumerate((15,22)):
            sign=1 if offset==0 else -1
            q[:,start]=-.2 + .6*np.sin(.28*t+phase[offset])
            q[:,start+1]=sign*(.25+.3*np.sin(.24*t+phase[2+offset]))
            q[:,start+2]=.35*np.sin(.21*t+phase[4+offset])
            q[:,start+3]=.45+.6*np.sin(.32*t+phase[6+offset])
        bend=.08*(1+np.sin(.2*t+phase[0]))
        q[:,[0,6]]-=bend[:,None];q[:,[3,9]]+=2*bend[:,None];q[:,[4,10]]-=bend[:,None]
        q[:,12]=.25*np.sin(.2*t+phase[1])
        q[:,13]=.07*np.sin(.17*t+phase[2])
        q[:,14]=.09*np.sin(.23*t+phase[3])
        q=np.clip(q,soft_low,soft_high)
        clips.append(dict(name='synthetic_reaching_%02d'%i,q=q,root_height=model.nominal_root_height(q),split=int(i%5==4)))
    return save_clips(path,clips,dict(source='Locally generated broad reachable G1 upper-body and crouch trajectories',
                    license='Original project implementation',motion_kind='synthetic stationary reaching, waist lean, and crouch',locomotion=False,
                    curriculum_stage=2,validation='Exact USD FK and 90% soft joint limits; dynamic balance/self-collision not established.'),fps)
