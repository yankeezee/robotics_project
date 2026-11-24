#!/usr/bin/env python3
"""Analyze Anastasiia's trial summary without altering her logs."""
import argparse
from pathlib import Path
import sys
# Support exact command: python scripts/sophia_analyze.py from repository root.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pednav.sophia.analysis import load_paired, calculate_summary, export_analysis


def main():
    p=argparse.ArgumentParser(description='Strict paired analysis with backend integrity checks')
    p.add_argument('--input',required=True,help='path to summary.csv')
    p.add_argument('--output',required=True,help='destination directory for plots, CSVs and JSON')
    p.add_argument('--backend',choices=['webots','kinematic'],required=True)
    p.add_argument('--expected-pairs',type=int,default=20,help='per family (use 1 for smoke testing)')
    args=p.parse_args()
    backend,pairs,sha=load_paired(args.input, expected_pairs_per_family=args.expected_pairs, expected_backend=args.backend)
    result=calculate_summary(backend,pairs,sha)
    export_analysis(result,args.output)
    print(f'Analyzed {result["n_pairs"]} pairs / {result["n_runs"]} runs ({backend})')
    for mode,s in result['overall'].items():
        print(f'{mode}: success={s["success"]}/{s["n"]}, collision={s["collision"]}/{s["n"]}, '
              f'timeout={s["timeout"]}/{s["n"]}, below clearance margin={s["near_margin"]}/{s["n"]}')
    print('Wrote paired_analysis.json, paired_by_family.csv, joint_success_only.csv and three figures.')


if __name__=='__main__':
    main()
