"""Safe NumPy motion dataset: numeric arrays, no executable pickle at runtime."""
from pathlib import Path
import json
import numpy as np
from .kinematics import JOINT_NAMES


class MotionLibrary:
    def __init__(self, path):
        self.path = Path(path)
        with np.load(self.path, allow_pickle=False) as z:
            self.data = {k: z[k].copy() for k in z.files}
        d = self.data
        if tuple(d['joint_names'].tolist()) != JOINT_NAMES:
            raise ValueError('Motion dataset canonical joint order mismatch')
        n = len(d['q'])
        for key, shape in [('q', (n, 29)), ('keypoints', (n, 3, 3)), ('root_height', (n,)), ('command_velocity', (n, 3))]:
            if d[key].shape != shape or not np.isfinite(d[key]).all():
                raise ValueError('Invalid %s array' % key)
        self.fps = float(d['fps'])
        if self.fps <= 0: raise ValueError('fps must be positive')
        self.clip_names = d['clip_names'].tolist()
        if np.any(d['clip_length'] < 2): raise ValueError('A clip needs at least two frames')
        if np.any(d['clip_start'] < 0) or np.any(d['clip_start']+d['clip_length'] > n):
            raise ValueError('Clip bounds outside motion data')
        self.metadata = json.loads(self.path.with_suffix('.json').read_text()) if self.path.with_suffix('.json').exists() else {}

    def sample(self, clip_ids, times, loop=False):
        """Interpolate at seconds within clips; never interpolate across clips.

        Returns q, keypoints, root_height, command_velocity, and finished.
        Default clamps the last frame; loop=True explicitly wraps clip time.
        """
        ids, times = np.broadcast_arrays(np.asarray(clip_ids, dtype=np.int64), np.asarray(times, dtype=float))
        if np.any(ids < 0) or np.any(ids >= len(self.clip_names)) or not np.isfinite(times).all():
            raise ValueError('Invalid clip ids or time')
        length = self.data['clip_length'][ids]
        duration = (length-1)/self.fps
        finished = times >= duration
        time = np.mod(times, duration) if loop else np.clip(times, 0., duration)
        frame = time*self.fps
        lo = np.floor(frame).astype(np.int64)
        hi = np.minimum(lo+1, length-1)
        w = frame-lo
        start = self.data['clip_start'][ids]
        result = {}
        for key in ('q', 'keypoints', 'root_height', 'command_velocity'):
            a = self.data[key][start+lo]; b = self.data[key][start+hi]
            weight = w.reshape(w.shape+(1,)*(a.ndim-w.ndim))
            result[key] = a*(1-weight)+b*weight
        result['finished'] = finished
        return result

    def sample_batch(self, batch_size, split='train', rng=None):
        rng = np.random.default_rng() if rng is None else rng
        if split not in ('train', 'test'): raise ValueError('split must be train or test')
        candidates = np.flatnonzero(self.data['split'] == (split == 'test'))
        if not len(candidates): raise ValueError('Requested split is empty')
        ids = rng.choice(candidates, batch_size)
        time = rng.random(batch_size)*(self.data['clip_length'][ids]-1)/self.fps
        return ids, time, self.sample(ids, time)


def save_clips(path, clips, metadata, fps=30):
    """clips: list of {name, q, root_height, split, [command_velocity]} dictionaries."""
    from .kinematics import KinematicModel
    model=KinematicModel(); arrays={k: [] for k in ('q','keypoints','root_height','command_velocity','clip_id')}
    starts=[]; lengths=[]; offset=0
    for cid, clip in enumerate(clips):
        q=np.asarray(clip['q'],dtype=np.float32); n=len(q)
        if np.any(q < model.lower-1e-5) or np.any(q > model.upper+1e-5):
            raise ValueError('Joint limit violation in %s' % clip['name'])
        starts.append(offset); lengths.append(n); offset += n
        arrays['q'].append(q); arrays['keypoints'].append(model.keypoints(q).astype(np.float32))
        arrays['root_height'].append(np.asarray(clip['root_height'],dtype=np.float32))
        arrays['command_velocity'].append(np.asarray(clip.get('command_velocity',np.zeros((n,3))),dtype=np.float32))
        arrays['clip_id'].append(np.full(n,cid,dtype=np.int32))
    output={k:np.concatenate(v) for k,v in arrays.items()}
    output.update(clip_start=np.asarray(starts,dtype=np.int64),clip_length=np.asarray(lengths,dtype=np.int64),
                  clip_names=np.array([c['name'] for c in clips]),split=np.array([c['split'] for c in clips],dtype=np.uint8),
                  joint_names=np.array(JOINT_NAMES),fps=np.array(float(fps),dtype=np.float32),schema=np.array(1))
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path,**output)
    metadata=dict(metadata,schema=1,joint_names=list(JOINT_NAMES),fps=fps,frames=offset,
                  clips=[dict(name=c['name'],split='test' if c['split'] else 'train',frames=len(c['q'])) for c in clips],
                  keypoint_order=['head','left_wrist','right_wrist'],frame='pelvis body frame; metres; X forward Y left Z up',
                  head_offset_in_head_link=[0,0,.45])
    path.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return output


def merge_libraries(paths, output):
    """Merge already generated datasets, preserving clip membership and splits."""
    import hashlib
    clips=[];sources=[];fps=None
    for path in paths:
        lib=MotionLibrary(path);d=lib.data
        if fps is not None and fps!=lib.fps:raise ValueError('Cannot merge different frame rates')
        fps=lib.fps
        sources.append(dict(filename=Path(path).name,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                            license=lib.metadata.get('source_license',lib.metadata.get('license','See source metadata'))))
        for i,name in enumerate(lib.clip_names):
            a=d['clip_start'][i];b=a+d['clip_length'][i]
            clips.append(dict(name=name,q=d['q'][a:b],root_height=d['root_height'][a:b],
                              command_velocity=d['command_velocity'][a:b],split=int(d['split'][i])))
    if len({c['name'] for c in clips})!=len(clips):raise ValueError('Duplicate clip names across datasets')
    return save_clips(output,clips,dict(source='Concatenated curricula; every source clip split retained',sources=sources,
                      source_license='Mixed: procedural own data; LeCAR CC BY-NC 4.0 if retargeted clips included',locomotion=False),fps)
