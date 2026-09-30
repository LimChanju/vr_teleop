#!/usr/bin/env python3
"""Independent, reset-aware summaries of G1 evaluation result.json and trace.npz.

Population metrics are retained verbatim from pre-reset simulator snapshots.
Trace statistics concern environment zero only and exclude reset boundaries.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import re
import numpy as np

POINTS=('head','left_wrist','right_wrist')


def reset_valid_mask(done, guard=1):
    """Exclude auto-reset rows and guard adjacent rows; never erase fall counts."""
    if guard<0:raise ValueError('reset guard must be nonnegative')
    done=np.asarray(done,dtype=bool)
    if done.ndim!=1:raise ValueError('done must be [T]')
    valid=np.ones(len(done),dtype=bool)
    for index in np.flatnonzero(done):
        valid[max(0,index-guard):min(len(done),index+guard+1)]=False
    return valid


def stats(values):
    a=np.asarray(values,dtype=float).reshape(-1)
    a=a[np.isfinite(a)]
    if not len(a):return dict(count=0,mean=None,p50=None,p95=None,max=None)
    return dict(count=len(a),mean=float(a.mean()),p50=float(np.percentile(a,50)),
                p95=float(np.percentile(a,95)),max=float(a.max()))


def response_metrics(target, actual, valid, dt, done, max_lag_seconds=.5):
    """Component response diagnostics; correlation does not establish control quality."""
    result={}
    for point,name in enumerate(POINTS):
        result[name]={}
        for axis,label in enumerate('xyz'):
            x=target[:,point,axis]; y=actual[:,point,axis]
            xv=x[valid];yv=y[valid]
            item={'target_std_m':float(xv.std()) if len(xv) else None,
                  'actual_std_m':float(yv.std()) if len(yv) else None,
                  'target_range_m':float(np.ptp(xv)) if len(xv) else None,
                  'actual_range_m':float(np.ptp(yv)) if len(yv) else None,
                  'correlation':None,'least_squares_gain':None,'best_positive_lag_seconds':None}
            if len(xv)>=30 and xv.std()>.005 and yv.std()>.001:
                item['correlation']=float(np.corrcoef(xv,yv)[0,1])
                item['least_squares_gain']=float(np.mean((xv-xv.mean())*(yv-yv.mean()))/xv.var())
                # Restrict every pair to one uninterrupted episode segment.
                segment=np.cumsum(done)
                best=(-np.inf,0)
                for lag in range(max(1,int(max_lag_seconds/dt)+1)):
                    if lag==0:a=x;b=y;mask=valid
                    else:
                        a=x[:-lag];b=y[lag:]
                        mask=valid[:-lag]&valid[lag:]&(segment[:-lag]==segment[lag:])
                    if mask.sum()<30 or a[mask].std()<=.005 or b[mask].std()<=.001:continue
                    corr=float(np.corrcoef(a[mask],b[mask])[0,1])
                    if np.isfinite(corr) and corr>best[0]:best=(corr,lag)
                if np.isfinite(best[0]):
                    item['best_positive_lag_seconds']=float(best[1]*dt)
                    item['best_positive_lag_correlation']=best[0]
            result[name][label]=item
    return result


def locomotion_diagnostics(trace, valid, done, dt):
    """Optional env0 root/foot evidence; never bridge resets or invalid rows."""
    shapes={'root_position_w':(3,), 'root_quaternion_wxyz':(4,), 'root_linear_velocity_b':(3,),
            'foot_contact':(2,), 'foot_linear_velocity_w':(2,3)}
    present={key:np.asarray(trace[key]) for key in shapes if key in trace}
    if not present:return None
    n=len(valid); usable=valid.copy();invalid={};nonfinite={}
    for key,value in present.items():
        if value.shape!=(n,)+shapes[key]:raise ValueError(f'{key} must be [T,{shapes[key]}]')
        finite=np.isfinite(value).reshape(n,-1).all(-1)
        accepted=finite.copy()
        if key=='foot_contact':accepted &= np.isin(value,(0,1)).all(-1)
        if key=='root_quaternion_wxyz':
            norm=np.linalg.norm(value,axis=-1)
            accepted &= np.abs(norm-1.)<=1e-3
        nonfinite[key]=int((~finite).sum());invalid[key]=int((~accepted).sum());usable &= accepted
    pair=usable[:-1]&usable[1:]&~done[:-1]&~done[1:]
    result=dict(available_fields=list(present),missing_fields=[key for key in shapes if key not in present],
                valid_rows=int(usable.sum()),valid_adjacent_pairs=int(pair.sum()),
                data_valid=bool(n and all(count==0 for count in invalid.values())),
                invalid_rows=invalid,nonfinite_rows=nonfinite,
                note='Environment zero only. Reset guards and invalid rows split segments. No gait success threshold is implied.')
    velocity=present.get('root_linear_velocity_b')
    if velocity is not None:
        result['root_body_xy_speed_mps']=stats(np.linalg.norm(velocity[usable,:2],axis=-1))
        result['root_speed_mps']=stats(np.linalg.norm(velocity[usable],axis=-1))
    position=present.get('root_position_w');quaternion=present.get('root_quaternion_wxyz')
    if position is not None or quaternion is not None:
        yaw=None
        if quaternion is not None:
            normalized=quaternion.astype(float)/np.maximum(np.linalg.norm(quaternion,axis=-1,keepdims=True),1e-12)
            w,x,y,z=normalized.T
            yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        indices=np.flatnonzero(usable)
        segments=[]
        for ids in np.split(indices,np.flatnonzero(np.diff(indices)>1)+1) if len(indices) else []:
            item=dict(start_frame=int(ids[0]),end_frame=int(ids[-1]),samples=len(ids),
                      duration_seconds=float((len(ids)-1)*dt))
            if velocity is not None:
                item['root_body_xy_speed_mps']=stats(np.linalg.norm(velocity[ids,:2],axis=-1))
                item['root_speed_mps']=stats(np.linalg.norm(velocity[ids],axis=-1))
            if position is not None:
                xy=position[ids,:2];displacement=xy[-1]-xy[0]
                item.update(planar_displacement_xy_m=displacement.tolist(),
                            planar_net_displacement_m=float(np.linalg.norm(displacement)),
                            planar_path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=-1).sum()))
            if yaw is not None:
                angles=np.unwrap(yaw[ids])
                item['yaw_drift_rad']=float(angles[-1]-angles[0])
                item['yaw_path_rad']=float(np.abs(np.diff(angles)).sum())
            segments.append(item)
        result['root_motion_segments']=segments
        if position is not None:
            result['root_planar_path_length_m']=float(sum(item['planar_path_length_m'] for item in segments))
            result['root_planar_speed_from_position_mps']=stats(np.linalg.norm(np.diff(position[:,:2],axis=0)[pair],axis=-1)/dt)
    contact=present.get('foot_contact');foot_velocity=present.get('foot_linear_velocity_w')
    if contact is not None:
        # Invalid values are masked before counting; never turn NaN into True.
        contact=contact==1
        feet={}
        for side,index in [('left',0),('right',1)]:
            state=contact[:,index]
            item=dict(valid_samples=int(usable.sum()),contact_samples=int((usable&state).sum()),
                      contact_fraction=float(state[usable].mean()) if usable.any() else None,
                      liftoff_count=int((pair&state[:-1]&~state[1:]).sum()),
                      touchdown_count=int((pair&~state[:-1]&state[1:]).sum()))
            if foot_velocity is not None:
                item['foot_link_horizontal_velocity_during_contact_mps']=stats(
                    np.linalg.norm(foot_velocity[usable&state,index,:2],axis=-1))
            feet[side]=item
        result['feet']=feet
        result['foot_velocity_interpretation']='Horizontal world velocity of the tracked foot-link origin during contact; not exact contact-point slip. Link rotation can move this origin while a contact point remains fixed.'
    return result


def analyze_trace(trace, reset_guard=1):
    target=np.asarray(trace['target'],dtype=float);actual=np.asarray(trace['actual'],dtype=float)
    q=np.asarray(trace['q'],dtype=float);action=np.asarray(trace['action'],dtype=float)
    done=np.asarray(trace['done'],dtype=bool);dt=float(trace['dt'])
    if 'manual_reset' in trace:
        manual_reset=np.asarray(trace['manual_reset'],dtype=bool)
        if manual_reset.shape != done.shape:raise ValueError('manual_reset must match done shape')
        done=done|manual_reset
    n=len(done)
    if target.shape!=(n,3,3) or actual.shape!=(n,3,3) or q.shape!=(n,29) or action.shape!=(n,29):
        raise ValueError('Trace shape mismatch: expected target/actual[T,3,3], q/action[T,29]')
    if not np.isfinite(dt) or dt<=0:raise ValueError('Invalid trace dt')
    finite={name:np.isfinite(value).reshape(n,-1).all(-1) for name,value in
            [('target',target),('actual',actual),('q',q),('action',action)]}
    valid=reset_valid_mask(done,reset_guard)
    finite_all=np.logical_and.reduce(list(finite.values()))
    valid &= finite_all
    pair=valid[1:]&valid[:-1]&~done[:-1]&~done[1:]
    errors=np.linalg.norm(actual-target,axis=-1)
    result=dict(scope='environment_zero_only',frames=n,dt_seconds=dt,
                elapsed_simulation_seconds=n*dt,reset_rows=int(done.sum()),reset_guard_frames=reset_guard,
                excluded_boundary_rows=int((~reset_valid_mask(done,reset_guard)).sum()),
                valid_rows=int(valid.sum()),finite={k:bool(v.all()) for k,v in finite.items()},
                nonfinite_rows={k:int((~v).sum()) for k,v in finite.items()},
                point_tracking_m={name:stats(errors[valid,index]) for index,name in enumerate(POINTS)},
                mean_point_tracking_m=stats(errors[valid].mean(-1)),
                hand_tracking_m=stats(errors[valid,1:].mean(-1)),
                joint_speed_rad_s=stats(np.abs(np.diff(q,axis=0)[pair])/dt),
                action_step_change=stats(np.abs(np.diff(action,axis=0)[pair])),
                raw_action_abs=stats(np.abs(action[valid])),
                target_speed_m_s=stats(np.linalg.norm(np.diff(target,axis=0)[pair],axis=-1)/dt),
                actual_speed_m_s=stats(np.linalg.norm(np.diff(actual,axis=0)[pair],axis=-1)/dt),
                response=response_metrics(target,actual,valid,dt,done),
                note='Boundary exclusion affects trace tracking/speeds only. Population fall count remains unchanged. Raw actions precede wrapper clipping.')
    if 'enabled' in trace:result['enabled_rows']=int(np.asarray(trace['enabled'],dtype=bool).sum())
    locomotion=locomotion_diagnostics(trace,valid,done,dt)
    if locomotion is not None:result['locomotion']=locomotion
    return result,valid


def analyze_run(path,reset_guard=1):
    path=Path(path);result=json.loads((path/'result.json').read_text())
    meta=json.loads((path/'run_config.json').read_text()) if (path/'run_config.json').exists() else {}
    population={key:result.get(key) for key in ('steps','simulated_seconds','completed_episodes','falls','timeouts',
               'fall_rate_completed_episodes','mean_completed_episode_seconds','incomplete_episode_seconds','mean_metrics')}
    incomplete=population.pop('incomplete_episode_seconds')
    population['incomplete_episode_seconds_summary']=stats(incomplete or [])
    seconds=population.get('simulated_seconds')
    population['falls_per_simulated_minute']=(result.get('falls',0)*60/seconds) if seconds else None
    summary=dict(run=str(path.resolve()),label=path.name,status=result.get('status'),mode=result.get('mode'),
                 zero_policy=result.get('zero_policy'),seed=result.get('seed',meta.get('seed')),
                 num_envs=meta.get('num_envs'),evaluation_horizon_steps=result.get('steps'),checkpoint=result.get('checkpoint'),
                 motion_file=meta.get('motion_file'),motion_sha256=meta.get('motion_sha256'),
                 motion_split=result.get('motion_split',meta.get('motion_split')),coordinate_version=meta.get('coordinate_version'),
                 kinematics_validation=meta.get('kinematics_validation'),population=population,
                 environment_sha256=meta.get('source_sha256',{}).get('g1_teleop/sim/env.py'),
                 control_dt=meta.get('control_dt'),action_scale=meta.get('action_scale'),action_clip=meta.get('action_clip'),
                 input_sources=result.get('input_sources',[]),accepted_input_steps=result.get('accepted_input_steps'),
                 enabled_steps=result.get('enabled_steps'),
                 physical_quest_verified=result.get('physical_quest_verified',False),warnings=[])
    if result.get('status') not in ('evaluated','teleop_stopped'):summary['warnings'].append('Run is not a completed evaluation/teleop recording')
    if result.get('mode')=='teleop':summary['warnings'].append('Teleop input differs from scripted motion evaluation; compare input sources and active duration')
    if population.get('completed_episodes',0)==0:summary['warnings'].append('No completed episodes: completed-episode fall rate is undefined, not zero')
    data=None;valid=None
    if (path/'trace.npz').exists():
        with np.load(path/'trace.npz',allow_pickle=False) as source:data={key:source[key] for key in source.files}
        summary['trace'],valid=analyze_trace(data,reset_guard)
        if not all(summary['trace']['finite'].values()):summary['warnings'].append('Nonfinite trace values detected; inspect nonfinite_rows')
        if 'locomotion' in summary['trace'] and not summary['trace']['locomotion']['data_valid']:
            summary['warnings'].append('Invalid root/foot diagnostic values detected; inspect locomotion.invalid_rows')
    else:summary['warnings'].append('No trace.npz; response and joint/action finiteness were not independently checked')
    return summary,data,valid


def plot_run(summary,data,valid,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=(np.arange(len(valid))+1)*float(data['dt']);actual=np.array(data['actual'],copy=True);target=np.array(data['target'],copy=True)
    actual[~valid]=np.nan;target[~valid]=np.nan
    fig,axs=plt.subplots(3,2,figsize=(12,8),sharex=True)
    for i,name in enumerate(POINTS):
        for axis,color in enumerate(('tab:red','tab:green','tab:blue')):
            axs[i,0].plot(t,target[:,i,axis],color=color,ls='--',alpha=.75,label='target '+'xyz'[axis])
            axs[i,0].plot(t,actual[:,i,axis],color=color,label='actual '+'xyz'[axis])
        axs[i,0].set_ylabel(name+' position [m]')
        err=np.linalg.norm(actual[:,i]-target[:,i],axis=-1)
        axs[i,1].plot(t,err,color='black');axs[i,1].set_ylabel(name+' error [m]')
        for a in axs[i]:a.grid(alpha=.25)
    axs[0,0].legend(ncol=3,fontsize=8);axs[-1,0].set_xlabel('Simulation time [s]');axs[-1,1].set_xlabel('Simulation time [s]')
    fig.suptitle(summary['label']+' — environment 0; reset boundaries excluded')
    fig.tight_layout();fig.savefig(output,dpi=160);plt.close(fig)


def plot_comparison(summaries,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(1,3,figsize=(13,4));labels=[s['label'] for s in summaries]
    values=[[(s['population'].get('mean_metrics') or {}).get('hand_error_m',np.nan) for s in summaries],
            [s['population'].get('falls_per_simulated_minute',np.nan) for s in summaries],
            [s['population'].get('mean_completed_episode_seconds',np.nan) for s in summaries]]
    for ax,v,title in zip(axs,values,('Population mean hand error [m]','Falls / simulated minute','Mean completed episode [s]')):
        ax.bar(np.arange(len(labels)),[np.nan if x is None else x for x in v]);ax.set_xticks(np.arange(len(labels)),labels,rotation=25,ha='right');ax.set_title(title);ax.grid(axis='y',alpha=.25)
    fig.suptitle('Check matched motion, seed and horizon before interpreting comparisons')
    fig.tight_layout();fig.savefig(output,dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--reset-guard',type=int,default=1);p.add_argument('--plot',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for path in a.runs:
        summary,data,valid=analyze_run(path,a.reset_guard);summaries.append(summary)
        if a.plot and data is not None:plot_run(summary,data,valid,a.output/(re.sub(r'[^A-Za-z0-9_.-]','_',path.name)+'_trace.png'))
        pop=summary['population'];print('%s: falls=%s, completed=%s, hand_error=%s, trace_valid=%s'%(summary['label'],pop.get('falls'),pop.get('completed_episodes'),(pop.get('mean_metrics') or {}).get('hand_error_m'),summary.get('trace',{}).get('valid_rows')))
    fields=('motion_sha256','seed','num_envs','evaluation_horizon_steps','motion_split','coordinate_version','environment_sha256','control_dt','action_scale','action_clip')
    mismatch={key:[s.get(key) for s in summaries] for key in fields if len({str(s.get(key)) for s in summaries})>1}
    report=dict(schema=1,runs=summaries,comparison_mismatches=mismatch,
                note='Population pre-reset metrics and env0 post-step traces are separate evidence. No pass/fail threshold is implied.')
    (a.output/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if a.plot and len(summaries)>1:plot_comparison(summaries,a.output/'comparison.png')

if __name__=='__main__':main()
