"""Execute paired Webots development/final trials with identical cases and limits.

This runner executes each seed with all three scenario families for BOTH
planners. Seed 0-2 are development cases; use separate held-out seeds for final
analysis after a common parameter/code freeze.
"""
import argparse
import subprocess
import sys
from pathlib import Path
from export_paired_summary import convert_run_dirs, write_csv

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed-start',type=int,default=5000)
    parser.add_argument('--trials',type=int,default=20)
    parser.add_argument('--webots',help='Full path to executable if not on PATH')
    parser.add_argument('--show',action='store_true')
    parser.add_argument('--output',default=str(ROOT/'results'/'webots_paired_summary.csv'))
    args=parser.parse_args()
    if args.trials < 1:
        parser.error('--trials must be >= 1')
    run_dirs=[]
    for seed in range(args.seed_start, args.seed_start+args.trials):
        for mode in ('reactive','predictive'):
            before=set((ROOT/'results').glob('*'))
            cmd=[sys.executable,str(ROOT/'run.py'),'--planner',mode,
                 '--scenario','crossing','--scenario','oncoming',
                 '--scenario','stop_turn','--seed',str(seed)]
            if args.webots: cmd += ['--webots',args.webots]
            if args.show: cmd += ['--show']
            print('RUN', ' '.join(cmd),flush=True)
            subprocess.run(cmd,cwd=ROOT,check=True)
            after=set((ROOT/'results').glob('*'))
            candidates=[p for p in after-before if p.is_dir() and (p/'run_config.json').exists()]
            if len(candidates)!=1:
                raise RuntimeError(f'Expected one new Webots batch folder, found {candidates}')
            run_dirs.append(candidates[0])
    rows,_ = convert_run_dirs(run_dirs)
    dest = write_csv(rows,args.output)
    print('Saved real Webots paired summary:',dest, flush=True)

if __name__=='__main__':
    main()
