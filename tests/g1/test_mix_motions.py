"""Mixing must preserve authored targets and heldout membership byte-for-byte."""
import argparse
import copy
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from g1_teleop.motion import JOINT_NAMES, MotionLibrary
from scripts.g1.mix_motions import FRAME_FIELDS, input_spec, mix_motions, sha256


def fixture(path, *, offset=0., fps=30.):
    q=(np.arange(145,dtype=np.float32).reshape(5,29)/1000)+offset
    q[1,1]=-0.0
    arrays={'q':q,'keypoints':np.arange(45,dtype=np.float32).reshape(5,3,3)/113+offset,
            'root_height':np.arange(5,dtype=np.float32)/100+.7,
            'command_velocity':np.full((5,3),offset,dtype=np.float32),
            'clip_start':np.array([0,3],dtype=np.int64),'clip_length':np.array([3,2],dtype=np.int64),
            'clip_names':np.array(['train_clip','heldout_clip']),'split':np.array([0,1],dtype=np.uint8),
            'clip_id':np.array([0,0,0,1,1],dtype=np.int32),'joint_names':np.array(JOINT_NAMES),
            'fps':np.array(fps,dtype=np.float32),'schema':np.array(1)}
    path.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path,**arrays)
    path.with_suffix('.json').write_text(json.dumps({'source':'test original targets','license':'test provenance'}))
    return arrays


class MixMotionsTests(unittest.TestCase):
    def test_integer_weight_preserves_every_source_value_and_split(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);a=root/'a'/'same.npz';b=root/'b'/'same.npz';out=root/'mix.npz'
            original=[fixture(a),fixture(b,offset=.03)]
            before=[sha256(a),sha256(b)]
            metadata=mix_motions([(a,3),(b,1)],out)
            mixed=MotionLibrary(out)
            self.assertEqual(metadata['split_counts'],{'train':4,'eval':4})
            self.assertEqual(len(mixed.clip_names),8)
            self.assertEqual(len(set(mixed.clip_names)),8)
            for new_id,clip in enumerate(metadata['clips']):
                source=original[clip['source_index']];cid=clip['source_clip_index']
                lo=int(source['clip_start'][cid]);length=int(source['clip_length'][cid])
                target_start=int(mixed.data['clip_start'][new_id])
                for name in FRAME_FIELDS:
                    expected=source[name][lo:lo+length];actual=mixed.data[name][target_start:target_start+length]
                    self.assertEqual(actual.dtype,expected.dtype)
                    self.assertEqual(actual.tobytes(),expected.tobytes(),(new_id,name))
                self.assertEqual(int(mixed.data['split'][new_id]),int(source['split'][cid]))
                np.testing.assert_array_equal(mixed.data['clip_id'][target_start:target_start+length],new_id)
            self.assertEqual([sha256(a),sha256(b)],before)
            self.assertEqual([s['weight'] for s in metadata['sources']],[3,1])
            self.assertEqual(metadata['sources'][0]['source_metadata']['license'],'test provenance')
            self.assertEqual(metadata['output_sha256'],sha256(out))
            ids,_,_=mixed.sample_batch(100,'test',np.random.default_rng(5))
            self.assertTrue((mixed.data['split'][ids]==1).all())

    def test_repeated_builds_have_identical_archive_and_metadata_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.npz';fixture(source)
            out1=root/'one.npz';out2=root/'two.npz'
            mix_motions([(source,2)],out1);mix_motions([(source,2)],out2)
            self.assertEqual(out1.read_bytes(),out2.read_bytes())
            self.assertEqual(out1.with_suffix('.json').read_bytes(),out2.with_suffix('.json').read_bytes())

    def test_existing_output_or_metadata_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.npz';fixture(source)
            for suffix in ('.npz','.json'):
                out=root/('existing_'+suffix[1:]+'.npz');protected=out.with_suffix(suffix)
                protected.write_bytes(b'keep')
                with self.assertRaises(FileExistsError):mix_motions([(source,1)],out)
                self.assertEqual(protected.read_bytes(),b'keep')

    def test_incompatible_fps_dtype_joints_and_invalid_layout_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);a=root/'a.npz';b=root/'b.npz';fixture(a)
            original=fixture(b)
            mutations=[('fps',np.array(60.,dtype=np.float32)),('q',original['q'].astype(np.float64)),
                       ('clip_start',np.array([0,2])),('clip_id',np.zeros(5,dtype=np.int32)),
                       ('split',np.array([0,2],dtype=np.uint8)),('fps',np.array(float('nan'),dtype=np.float32)),
                       ('joint_names',np.array(JOINT_NAMES[::-1]))]
            for i,(name,value) in enumerate(mutations):
                data=copy.deepcopy(original);data[name]=value;np.savez_compressed(b,**data)
                out=root/f'bad{i}.npz'
                with self.assertRaises(ValueError):mix_motions([(a,1),(b,1)],out)
                self.assertFalse(out.exists())
            data=copy.deepcopy(original);data['extra_targets']=np.zeros((5,3));np.savez_compressed(b,**data)
            with self.assertRaises(ValueError):mix_motions([(b,1)],root/'extra.npz')

    def test_weight_validation_and_colon_in_input_filename(self):
        self.assertEqual(input_spec('a:b.npz:4'),(Path('a:b.npz'),4))
        for value in ('a.npz','a.npz:0','a.npz:-1','a.npz:1.5',':2'):
            with self.assertRaises(argparse.ArgumentTypeError):input_spec(value)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'source.npz';fixture(source)
            for weight in (0,-1,1.5,True):
                with self.assertRaises(ValueError):mix_motions([(source,weight)],root/'invalid.npz')


if __name__=='__main__':unittest.main()
