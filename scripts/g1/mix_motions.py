#!/usr/bin/env python3
"""Reproducibly repeat and concatenate existing G1 clips without retargeting."""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from g1_teleop.motion.library import MotionLibrary

FRAME_FIELDS = ('q', 'keypoints', 'root_height', 'command_velocity')
STATIC_FIELDS = ('joint_names', 'fps', 'schema')
CLIP_FIELDS = ('clip_start', 'clip_length', 'clip_names', 'split', 'clip_id')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def input_spec(text):
    try:
        path, weight = text.rsplit(':', 1)
        weight = int(weight)
        if not path or weight < 1:
            raise ValueError
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError('Expected PATH:POSITIVE_INTEGER_WEIGHT') from error
    return Path(path), weight


def validate_layout(data, path):
    """Reject layouts that would silently omit, overlap or relabel frames."""
    expected = set(FRAME_FIELDS + STATIC_FIELDS + CLIP_FIELDS)
    if set(data) != expected:
        raise ValueError(f'{path}: unsupported/missing schema fields: {set(data) ^ expected}')
    if data['schema'].shape != () or int(data['schema']) != 1:
        raise ValueError(f'{path}: expected schema 1')
    if data['fps'].shape != () or not np.isfinite(data['fps']) or float(data['fps']) <= 0:
        raise ValueError(f'{path}: invalid fps')
    count = len(data['clip_names'])
    if count == 0:
        raise ValueError(f'{path}: no clips')
    for key in ('clip_start', 'clip_length', 'split'):
        if data[key].shape != (count,) or data[key].dtype.kind not in 'iu':
            raise ValueError(f'{path}: invalid {key}')
    if data['clip_names'].dtype.kind != 'U' or data['clip_names'].shape != (count,):
        raise ValueError(f'{path}: clip_names must be a Unicode vector')
    if not np.isin(data['split'], [0, 1]).all():
        raise ValueError(f'{path}: split must be 0=train or 1=eval')
    starts = np.r_[0, np.cumsum(data['clip_length'][:-1])]
    n = len(data['q'])
    if (not np.array_equal(data['clip_start'], starts)
            or int(data['clip_length'].sum()) != n):
        raise ValueError(f'{path}: clips must partition the source frames contiguously')
    expected_ids = np.repeat(np.arange(count), data['clip_length'])
    if data['clip_id'].shape != (n,) or not np.array_equal(data['clip_id'], expected_ids):
        raise ValueError(f'{path}: clip_id disagrees with clip boundaries')


def write_deterministic_npz(stream, arrays):
    """Stable member order and ZIP timestamps; preserve each array's dtype/bytes."""
    with zipfile.ZipFile(stream, 'w') as archive:
        for name in sorted(arrays):
            payload = io.BytesIO()
            np.lib.format.write_array(payload, np.ascontiguousarray(arrays[name]) if arrays[name].ndim else arrays[name],
                                      allow_pickle=False)
            info = zipfile.ZipInfo(name + '.npy', date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o600 << 16
            archive.writestr(info, payload.getvalue(), compresslevel=6)


def mix_motions(inputs, output):
    """Integer weights repeat whole clips; source train/eval membership is fixed."""
    output = Path(output)
    sidecar = output.with_suffix('.json')
    if output.suffix != '.npz':
        raise ValueError('Output must have .npz suffix')
    if output.exists() or sidecar.exists():
        raise FileExistsError('Output or metadata already exists; choose a new filename')
    if not inputs:
        raise ValueError('At least one weighted input is required')
    arrays = {name: [] for name in FRAME_FIELDS}
    starts, lengths, names, splits, frame_ids, clips, sources = [], [], [], [], [], [], []
    offset = 0
    first = None
    for source_index, (path, weight) in enumerate(inputs):
        path = Path(path)
        if isinstance(weight, bool) or not isinstance(weight, int) or weight < 1:
            raise ValueError('Weights must be positive integers')
        digest = sha256(path)
        library = MotionLibrary(path)
        if sha256(path) != digest:
            raise ValueError(f'Source changed while reading: {path}')
        data = library.data
        validate_layout(data, path)
        if first is None:
            first = data
        else:
            for name in STATIC_FIELDS:
                if data[name].dtype != first[name].dtype or not np.array_equal(data[name], first[name]):
                    raise ValueError(f'{path}: incompatible {name}')
            for name in FRAME_FIELDS:
                if data[name].dtype != first[name].dtype:
                    raise ValueError(f'{path}: incompatible {name} dtype; refusing implicit conversion')
        source_meta = path.with_suffix('.json')
        sources.append({'index': source_index, 'filename': path.name, 'sha256': digest, 'weight': weight,
                        'metadata_sha256': sha256(source_meta) if source_meta.is_file() else None,
                        'source_metadata': library.metadata,
                        'source_clips': len(library.clip_names),
                        'source_split_counts': {'train': int((data['split'] == 0).sum()),
                                                'eval': int((data['split'] == 1).sum())}})
        for repetition in range(weight):
            for cid, name in enumerate(library.clip_names):
                start, length = int(data['clip_start'][cid]), int(data['clip_length'][cid])
                new_id = len(names)
                new_name = f's{source_index:02d}_{path.stem}_copy{repetition:02d}_clip{cid:03d}__{name}'
                starts.append(offset); lengths.append(length); names.append(new_name)
                split = int(data['split'][cid]); splits.append(split)
                frame_ids.append(np.full(length, new_id, dtype=np.int32))
                for field in FRAME_FIELDS:
                    arrays[field].append(data[field][start:start + length])
                clips.append({'name': new_name, 'frames': length, 'split': 'eval' if split else 'train',
                              'source_index': source_index, 'source_clip_index': cid,
                              'source_clip_name': name, 'copy_index': repetition})
                offset += length
    result = {name: np.concatenate(parts) for name, parts in arrays.items()}
    result.update({name: first[name].copy() for name in STATIC_FIELDS})
    result.update(clip_start=np.asarray(starts, dtype=np.int64), clip_length=np.asarray(lengths, dtype=np.int64),
                  clip_names=np.asarray(names), split=np.asarray(splits, dtype=np.uint8),
                  clip_id=np.concatenate(frame_ids))
    metadata = {'schema': 1, 'source': 'Weighted exact copies of existing motion clips',
                'method': 'Integer whole-clip repetition; no interpolation, FK recomputation, resampling or data splitting',
                'generator': 'scripts/g1/mix_motions.py', 'generator_sha256': sha256(__file__),
                'numpy_version': np.__version__, 'joint_names': result['joint_names'].tolist(),
                'fps': float(result['fps']), 'frames': offset, 'clip_count': len(names),
                'split_counts': {'train': splits.count(0), 'eval': splits.count(1)},
                'sources': sources, 'clips': clips,
                'keypoint_order': ['head', 'left_wrist', 'right_wrist'],
                'frame': 'pelvis body frame; metres; X forward Y left Z up',
                'head_offset_in_head_link': [0, 0, .45],
                'source_license': 'Source-specific licenses/provenance preserved in sources[].source_metadata',
                'sampling_note': 'Weights multiply clip counts, not duration-normalized probabilities. Repeated heldout clips are not independent new evaluation examples.',
                'validation': 'Array/schema/split integrity only; no dynamic policy performance claim.'}
    output.parent.mkdir(parents=True, exist_ok=True)
    created = False
    try:
        with output.open('xb') as stream:
            created = True
            write_deterministic_npz(stream, result)
        metadata['output_sha256'] = sha256(output)
        with sidecar.open('x') as stream:
            json.dump(metadata, stream, indent=2, ensure_ascii=False)
            stream.write('\n')
    except BaseException:
        if created:
            output.unlink(missing_ok=True)
        raise
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=input_spec, action='append', required=True,
                        help='Repeat PATH:INTEGER_WEIGHT, e.g. data/motions/g1_stand_v1.npz:4')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        metadata = mix_motions(args.input, args.output)
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f'Motion mixing failed: {error}\n')
    print(json.dumps({'output': str(args.output), 'sha256': metadata['output_sha256'],
                      'frames': metadata['frames'], 'clips': metadata['clip_count'],
                      'split_counts': metadata['split_counts']}, indent=2))


if __name__ == '__main__':
    main()
