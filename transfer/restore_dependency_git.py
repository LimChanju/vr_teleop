#!/usr/bin/env python3
"""Check or restore dependency Git metadata after unpacking the low-storage set.

Default is read-only. --restore-git fetches public pinned commits and fills
missing tracked files only. Existing working files are never checked out over.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess


def run(path,*args,check=True):
    return subprocess.run(['git','-C',str(path),*args],text=True,capture_output=True,check=check,
                          env={**os.environ,'GIT_LFS_SKIP_SMUDGE':'1','GIT_TERMINAL_PROMPT':'0'})


def restore(workspace,lock,write):
    problems=[]
    for row in sorted(lock,key=lambda r:(len(Path(r['path']).parts),r['path'])):
        rel=Path(row['path'])
        if rel.is_absolute() or '..' in rel.parts:raise ValueError('Unsafe dependency path')
        path=workspace/rel
        if not path.is_dir():
            problems.append(str(rel)+': working tree missing; unpack data archive first');continue
        if not (path/'.git').exists():
            if not write:problems.append(str(rel)+': Git metadata omitted (expected for lightweight archive)');continue
            run(path,'init','-q')
        head=run(path,'rev-parse','--verify','HEAD',check=False)
        if head.returncode==0 and head.stdout.strip()!=row['commit']:
            problems.append(str(rel)+': existing HEAD differs; preserved without changes');continue
        if write:
            remote=run(path,'remote','get-url','origin',check=False)
            if remote.returncode==0 and remote.stdout.strip()!=row['url']:
                problems.append(str(rel)+': existing origin differs; preserved without changes');continue
            if remote.returncode:run(path,'remote','add','origin',row['url'])
            if head.returncode:
                print('Fetching pinned commit:',rel,flush=True)
                run(path,'fetch','--depth','1','origin',row['commit'])
                run(path,'reset','--mixed',row['commit'])
            missing=run(path,'ls-files','--deleted','-z').stdout.split('\0')
            missing=[p for p in missing if p and not (path/p).exists()]
            for start in range(0,len(missing),100):
                run(path,'checkout','--',*missing[start:start+100])
            print('Ready:',rel,flush=True)
        elif head.returncode:
            problems.append(str(rel)+': HEAD missing')
        else:print('Pinned HEAD matches:',rel,flush=True)
    return problems


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--lock',type=Path,default=Path(__file__).with_name('low-storage-dependencies.json'))
    p.add_argument('--restore-git',action='store_true')
    a=p.parse_args();workspace=a.workspace.resolve()
    if not (workspace/'alvr/alvr_probe.py').is_file() or not (workspace/'TWIST2/twist2_vr').is_dir():
        p.error('--workspace must be the restored vr_teleop repository root')
    problems=restore(workspace,json.loads(a.lock.read_text()),a.restore_git)
    for problem in problems:print(problem)
    raise SystemExit(1 if problems else 0)


if __name__=='__main__':main()
