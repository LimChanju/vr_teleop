"""Independent Cartesian head/hand targets projected onto the actual G1 skeleton.

This is a kinematic curriculum, not a dynamic balance or collision certificate.
Random knot trajectories are separate from deterministic runtime test signals.
"""
from pathlib import Path
import hashlib
import json
import time
import numpy as np

from .kinematics import KinematicModel, DEFAULT_Q, SPARSE_LINKS, SPARSE_OFFSETS
from .library import save_clips


def soft_limits(model, factor=.9):
    midpoint = (model.lower + model.upper) * .5
    radius = (model.upper - model.lower) * (.5 * factor)
    return midpoint - radius, midpoint + radius


def random_targets(seed, seconds=16., fps=30):
    """Smooth, independently sampled bounded XYZ offsets in metres.

    Every dimension has different irregular knot times and random amplitudes.
    Quintic interpolation has zero first/second derivatives at each knot; no
    overshoot, repeated sinusoid, or data from runtime evaluation is used.
    """
    if seconds < 4 or fps <= 0:
        raise ValueError('Require seconds >= 4 and fps > 0')
    rng = np.random.default_rng(seed)
    times = np.arange(round(seconds * fps), dtype=float) / fps
    bounds = np.array([[[-.03, .03], [-.03, .03], [-.05, .005]],
                       [[-.08, .08]] * 3, [[-.08, .08]] * 3])
    output = np.zeros((len(times), 3, 3), dtype=float)
    knots = []
    for point in range(3):
        for axis in range(3):
            kt = [0.]
            while kt[-1] < times[-1] - 3.5:
                # Reserve >=1.5 s for neutral return too. An unconstrained last
                # random step can otherwise leave a sub-frame final interval.
                largest_step = min(3.5, times[-1]-kt[-1]-1.5)
                kt.append(kt[-1] + rng.uniform(1.5, largest_step))
            kt.append(times[-1])
            kt = np.asarray(kt)
            values = rng.uniform(*bounds[point, axis], size=len(kt))
            values[[0, -1]] = 0.
            index = np.minimum(np.searchsorted(kt, times, side='right') - 1, len(kt)-2)
            fraction = (times - kt[index]) / (kt[index+1] - kt[index])
            blend = fraction**3 * (10. - 15.*fraction + 6.*fraction**2)
            output[:, point, axis] = values[index] + blend * (values[index+1] - values[index])
            knots.append(dict(point=point, axis=axis, times=kt.tolist(), values=values.tolist()))
    return output, knots


def _metric(array):
    a = np.asarray(array)
    return dict(mean=float(a.mean()), p95=float(np.percentile(a, 95)), max=float(a.max()))


def optimize_targets(model, offsets, iterations=500, fps=30, progress=None):
    """CPU differentiable IK on [clips, frames, 3 points, XYZ] offsets."""
    import torch
    from scipy.ndimage import gaussian_filter1d
    if iterations < 1:
        raise ValueError('iterations must be positive')
    offsets = np.asarray(offsets, dtype=np.float32)
    if (offsets.ndim != 4 or offsets.shape[2:] != (3, 3) or offsets.shape[0] < 1
            or offsets.shape[1] < 3 or not np.isfinite(offsets).all()):
        raise ValueError('Expected finite offsets[clips,frames,3,3] with >=1 clip and >=3 frames')
    torch.set_num_threads(min(4, torch.get_num_threads()))
    tensor = lambda x: torch.as_tensor(x, dtype=torch.float32, device='cpu')
    count, frames = offsets.shape[:2]
    h0 = float(model.nominal_root_height())
    nominal = tensor(DEFAULT_Q)
    lower, upper = map(tensor, soft_limits(model))
    q = torch.nn.Parameter(nominal.expand(count, frames, 29).clone())
    height = torch.nn.Parameter(torch.full((count, frames), h0))
    # Better initial crouch avoids the nearly-straight leg IK singularity.
    with torch.no_grad():
        height.add_(tensor(offsets[..., 0, 2]))
        crouch = (-tensor(offsets[..., 0, 2])).clamp(min=0.)
        # Two-link sag estimate, refined by exact FK below. A linear guess near
        # straight knees otherwise converges unnecessarily slowly for head Z.
        bend = 2.*torch.acos((np.cos(.42/2.)-crouch/.7).clamp(-1.,1.))-.42
        q[..., [0, 6]] -= bend[..., None] * .5
        q[..., [3, 9]] += bend[..., None]
        q[..., [4, 10]] -= bend[..., None] * .5
    desired = tensor(model.keypoints(DEFAULT_Q)) + tensor(offsets)
    p0, _ = model.forward(DEFAULT_Q)
    foot_names = ('left_ankle_roll_link', 'right_ankle_roll_link')
    foot_target = tensor(np.stack([p0[n] + [0, 0, h0] for n in foot_names]))
    head_offset = tensor(SPARSE_OFFSETS[0])
    up = tensor([0., 0., 1.])
    wrist_indices = [19, 20, 21, 26, 27, 28]
    frame_time = torch.arange(frames, dtype=torch.float32)/fps
    endpoint_weight = torch.exp(-(torch.minimum(frame_time, frame_time[-1]-frame_time)/.35).square())
    optimizer = torch.optim.Adam([q, height], lr=.018)
    start_time = time.monotonic()
    for step in range(iterations):
        optimizer.zero_grad(set_to_none=True)
        p, r = model.forward_torch(q)
        points = torch.stack([p[SPARSE_LINKS[0]] + (r[SPARSE_LINKS[0]] @ head_offset[..., None])[..., 0],
                              p[SPARSE_LINKS[1]], p[SPARSE_LINKS[2]]], dim=-2)
        # Policy targets use root XY and a fixed nominal-height Z origin.
        zshift = torch.stack([height * 0., height * 0., height-h0], dim=-1)
        points = points + zshift[..., None, :]
        world_shift = torch.stack([height*0., height*0., height], dim=-1)
        feet = torch.stack([p[n] + world_shift for n in foot_names], dim=-2)
        feet_up = torch.stack([r[n][..., :, 2] for n in foot_names], dim=-2)
        point_weights = tensor([2., 1., 1.])[..., None]
        loss = 100.*((points-desired).square()*point_weights).mean()
        loss = loss + 3000.*(feet-foot_target).square().mean()
        loss = loss + 40.*(feet_up-up).square().mean()
        loss = loss + .002*(q-nominal).square().mean() + .02*q[..., wrist_indices].square().mean()
        loss = loss + (endpoint_weight[None, :, None]*(q-nominal).square()).mean()
        # Torso may tilt to move the head, but large lean/yaw is discouraged.
        loss = loss + .03*q[..., 12].square().mean()
        torso_up = r['torso_link'][..., :, 2]
        loss = loss + 10.*torch.relu(np.cos(np.deg2rad(10.))-torso_up[..., 2]).square().mean()
        loss = loss + .0003*((q[:, 1:]-q[:, :-1])*fps).square().mean()
        if frames > 2:
            loss = loss + .000002*((q[:, 2:]-2*q[:, 1:-1]+q[:, :-2])*fps**2).square().mean()
        loss = loss + .002*((height[:, 1:]-height[:, :-1])*fps).square().mean()
        loss.backward()
        optimizer.step()
        with torch.no_grad():
            q.copy_(torch.maximum(torch.minimum(q, upper), lower))
            height.clamp_(h0-.075, h0+.012)
        # Anneal late steps to remove Adam's constraint residual oscillation.
        if step == int(iterations*.60):
            for group in optimizer.param_groups: group['lr'] = .006
        if step == int(iterations*.85):
            for group in optimizer.param_groups: group['lr'] = .002
        if progress and (step % 100 == 0 or step == iterations-1):
            progress(dict(iteration=step+1, iterations=iterations, loss=float(loss.detach()),
                          elapsed_seconds=time.monotonic()-start_time))
    result = gaussian_filter1d(q.detach().numpy(), 1., axis=1, mode='nearest')
    heights = gaussian_filter1d(height.detach().numpy(), 1., axis=1, mode='nearest')
    lo, hi = soft_limits(model)
    result = np.clip(result, lo, hi).astype(np.float32)
    reports = []
    for i in range(count):
        reports.append(assess_clip(model, result[i], heights[i], offsets[i], fps))
    return result, heights, reports


def assess_clip(model, q, heights, offsets, fps=30):
    h0 = float(model.nominal_root_height())
    p0, _ = model.forward(DEFAULT_Q)
    p, r = model.forward(q)
    actual = model.keypoints(q)
    actual[..., 2] += heights[:, None]-h0
    desired = model.keypoints(DEFAULT_Q) + offsets
    residual = np.linalg.norm(actual-desired, axis=-1)
    feet = np.stack([p[n] + np.stack([heights*0, heights*0, heights], axis=-1)
                     for n in ('left_ankle_roll_link', 'right_ankle_roll_link')], axis=1)
    foot_nominal = np.stack([p0[n]+[0,0,h0] for n in ('left_ankle_roll_link', 'right_ankle_roll_link')])
    foot_error = np.linalg.norm(feet-foot_nominal, axis=-1)
    foot_up = np.stack([r[n][..., :, 2] for n in ('left_ankle_roll_link', 'right_ankle_roll_link')], axis=1)
    foot_tilt = np.rad2deg(np.arccos(np.clip(foot_up[..., 2], -1., 1.)))
    lo, hi = soft_limits(model)
    return dict(point_error_m={name: _metric(residual[:, i]) for i, name in enumerate(('head','left_wrist','right_wrist'))},
                desired_axis_range_m=np.ptp(desired, axis=0).tolist(), actual_axis_range_m=np.ptp(actual, axis=0).tolist(),
                stance_foot_error_m=_metric(foot_error), stance_foot_tilt_deg=_metric(foot_tilt),
                torso_tilt_deg=_metric(np.rad2deg(np.arccos(np.clip(r['torso_link'][..., 2, 2],-1.,1.)))),
                soft_limit_violation_rad=float(max(0., np.max(lo-q), np.max(q-hi))),
                max_joint_speed_rad_s=float(np.max(np.abs(np.diff(q, axis=0))*fps)),
                root_height_range_m=[float(np.min(heights)),float(np.max(heights))])


def generate_teleop(output, clips=24, seconds=16., fps=30, iterations=1000, batch_size=4,
                   train_seed=710000, test_seed=970000, progress=print):
    """Write an original synthetic NPZ with a fixed 20% clip-level heldout split."""
    output = Path(output)
    if output.exists() or output.with_suffix('.json').exists():
        raise FileExistsError(f'Refusing to overwrite {output} or its metadata')
    if clips < 5 or batch_size < 1 or train_seed == test_seed:
        raise ValueError('Require >=5 clips, positive batch size, and distinct seed streams')
    model = KinematicModel()
    # SeedSequence has distinct spawn keys as well as different entropy inputs.
    ntest = max(1, round(clips*.2)); ntrain = clips-ntest
    specs = [(0, s) for s in np.random.SeedSequence(train_seed).spawn(ntrain)]
    specs += [(1, s) for s in np.random.SeedSequence(test_seed).spawn(ntest)]
    clip_data=[]; reports={}; knot_data={}
    for start in range(0, clips, batch_size):
        subset=specs[start:start+batch_size]
        targets=[]; names=[]
        for offset,(split, sequence) in enumerate(subset):
            name=f'teleop_cartesian_{"test" if split else "train"}_{start+offset:02d}'
            desired, knots = random_targets(sequence, seconds, fps)
            targets.append(desired); names.append(name); knot_data[name]=knots
        def notify(values):
            if progress: progress(json.dumps(dict(batch_start=start, batch_clips=len(subset), **values)), flush=True)
        q, heights, metrics = optimize_targets(model, np.stack(targets), iterations, fps, notify)
        for i, ((split, sequence), name) in enumerate(zip(subset, names)):
            clip_data.append(dict(name=name,q=q[i],root_height=heights[i],split=split))
            reports[name]=dict(**metrics[i], seed_entropy=sequence.entropy, spawn_key=list(sequence.spawn_key))
    config_path = Path(__file__).with_name('g1_29dof_kinematics.json')
    metadata=dict(source='Original independent Cartesian random-knot synthetic G1 curriculum',
                  source_license='Original project-generated synthetic data; robot kinematics retain Unitree asset provenance',
                  locomotion=False, command_velocity='All zero: fixed stance and upper-body/crouch targets only',
                  method='Exact deployed USD FK, CPU differentiable q29 and root-height IK, 90% soft joint limits, flat stationary feet, temporal regularization',
                  kinematics_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                  generator_revision=3,
                  generator_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  optimizer_settings=dict(points=100.,head_relative_weight=2.,feet_position=3000.,feet_up=40.,
                                          joint_velocity=.0003,joint_acceleration=.000002,
                                          neutral_endpoint_weight=1.,endpoint_gaussian_seconds=.35,
                                          final_gaussian_sigma_frames=1.,knot_interval_seconds=[1.5,3.5]),
                  iterations=iterations, batch_size=batch_size, train_seed=train_seed, test_seed=test_seed,
                  soft_joint_limit_factor=.9, nominal_root_height_m=float(model.nominal_root_height()),
                  requested_offsets_m=dict(head=[[-.03,.03],[-.03,.03],[-.05,.005]],hands=[[-.08,.08]]*3),
                  desired_target_frame='Pelvis yaw/root XY, fixed nominal-root-height Z origin; root orientation identity during IK',
                  stored_keypoint_frame='Root-body FK. Loader must add root_height minus nominal_root_height to Z exactly once.',
                  limits='No mass-weighted COM, self-collision, contacts, or dynamics optimization. Flat fixed feet and small torso lean are kinematic heuristics, not balance validation.',
                  evaluation_independence='Random independent per-axis knots; no runtime_scenario or deterministic held-out tracking-sweep trajectories used.',
                  reports=reports, random_knots=knot_data)
    return save_clips(output, clip_data, metadata, fps)
