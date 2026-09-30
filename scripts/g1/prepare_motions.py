#!/usr/bin/env python3
"""Build portable stationary curriculum and optional genuinely retargeted H1 motions."""
import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from g1_teleop.motion.procedural import generate_standing, generate_reaching


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[2]/'data/motions')
    p.add_argument('--h1-source',type=Path,help='Trusted official stable_punch.pkl only; pickle can execute code')
    p.add_argument('--h1-xml',type=Path)
    p.add_argument('--iterations',type=int,default=300)
    p.add_argument('--device',default='cpu',choices=['cpu','cuda'])
    p.add_argument('--max-clips',type=int)
    p.add_argument('--merge',action='store_true',help='Merge generated standing/reaching and an available punch NPZ')
    a=p.parse_args()
    a.output_dir.mkdir(parents=True,exist_ok=True)
    generate_standing(a.output_dir/'g1_stand_v1.npz')
    generate_reaching(a.output_dir/'g1_reach_v1.npz')
    print('Wrote standing and broader reaching procedural curricula',flush=True)
    if a.h1_source:
        if not a.h1_xml:p.error('--h1-xml is required with --h1-source')
        if a.iterations<1:p.error('--iterations must be positive')
        from g1_teleop.motion.retarget import prepare_h1
        prepare_h1(a.h1_source,a.h1_xml,a.output_dir/'g1_punch_v1.npz',a.iterations,a.device,a.max_clips)
    if a.merge:
        from g1_teleop.motion.library import merge_libraries
        paths=[a.output_dir/n for n in ('g1_stand_v1.npz','g1_punch_v1.npz','g1_reach_v1.npz') if (a.output_dir/n).is_file()]
        merge_libraries(paths,a.output_dir/'g1_mixed_v1.npz')
        print('Merged curricula with original heldout splits preserved')

if __name__=='__main__':main()
