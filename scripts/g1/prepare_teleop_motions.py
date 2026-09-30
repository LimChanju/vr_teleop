#!/usr/bin/env python3
"""Prepare independent Cartesian G1 teleop IK motions, without replacing old data."""
import argparse
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from g1_teleop.motion.teleop_curriculum import generate_teleop
from g1_teleop.motion.library import merge_libraries


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('data/motions/g1_teleop_v3.npz'))
    parser.add_argument('--clips',type=int,default=24)
    parser.add_argument('--seconds',type=float,default=16.)
    parser.add_argument('--fps',type=int,default=30)
    parser.add_argument('--iterations',type=int,default=1000)
    parser.add_argument('--batch-size',type=int,default=4)
    parser.add_argument('--train-seed',type=int,default=710000)
    parser.add_argument('--test-seed',type=int,default=970000)
    parser.add_argument('--merge-with',type=Path)
    parser.add_argument('--merged-output',type=Path,default=Path('data/motions/g1_teleop_mixed_v3.npz'))
    args=parser.parse_args()
    if args.merge_with:
        if not args.merge_with.is_file(): parser.error('--merge-with does not exist')
        if args.output.resolve() == args.merged_output.resolve():
            parser.error('--output and --merged-output must be different')
        if args.merged_output.exists() or args.merged_output.with_suffix('.json').exists():
            parser.error('Merged output already exists; choose a new path')
    generate_teleop(args.output,args.clips,args.seconds,args.fps,args.iterations,args.batch_size,
                   args.train_seed,args.test_seed)
    print(f'Wrote {args.output}',flush=True)
    if args.merge_with:
        merge_libraries([args.merge_with,args.output],args.merged_output)
        print(f'Wrote {args.merged_output}; original clip splits preserved',flush=True)


if __name__=='__main__':main()
