"""Prospective paired Webots experiment: isolated trials, checkpoints, strict audits."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys
import threading
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
sys.path.insert(0,str(ROOT))
from navigation.core import Parameters
from navigation.scenarios import make_scenario
from experiment_integrity import audit_pair, audit_trial, read_json, source_files, source_manifest
from export_paired_summary import convert_run_dirs, write_csv

FAMILIES = ('crossing','oncoming','stop_turn')


def write_json(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(temporary,path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed-start',type=int,default=6000)
    parser.add_argument('--trials',type=int,default=20)
    parser.add_argument('--workers',type=int,default=1,choices=(1,2))
    parser.add_argument('--output-root',type=Path)
    parser.add_argument('--output',type=Path,help='Optional additional summary CSV destination.')
    parser.add_argument('--parameters',type=Path)
    parser.add_argument('--webots')
    parser.add_argument('--show',action='store_true')
    parser.add_argument('--snapshot',action='store_true')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--development',action='store_true',help='Mark smoke cases as development, permitting used seeds and dirty source.')
    args = parser.parse_args()
    if args.trials < 1 or (not args.development and args.seed_start < 100):
        parser.error('Use positive trial counts and reserved seeds >=100 for evaluation.')
    now = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    out = (args.output_root or REPO/'experiments'/('webots_'+now)).resolve()
    values = read_json(args.parameters) if args.parameters else asdict(Parameters())
    parameters = asdict(Parameters(**values))
    seeds = list(range(args.seed_start,args.seed_start+args.trials))
    manifest = source_manifest(REPO)
    if args.resume:
        protocol = read_json(out/'protocol.json')
        if protocol['seeds'] != seeds or protocol['parameters'] != parameters or protocol['source_manifest'] != manifest:
            raise ValueError('Resume refused: frozen seeds, parameters or source differ')
    else:
        dirty = subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()
        if dirty and not args.development:
            raise ValueError('Commit source/protocol tools before evaluation; development runs can use --development.')
        out.mkdir(parents=True,exist_ok=False)
        protocol = dict(created_utc=now, phase='development' if args.development else 'prospective_evaluation',
                        backend='webots',webots_version='R2025b',control_step_s=.064,
                        parameters=parameters,seeds=seeds,families=list(FAMILIES),
                        n_pairs=3*args.trials,n_runs=6*args.trials,pedestrians_per_case=1,
                        process_policy='Fresh Webots process and fresh planner instance for every individual trial.',
                        order_policy='Family order fixed; planner order alternates with seed parity.',
                        git_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
                        uncommitted_status_at_freeze=dirty,source_manifest=manifest,
                        python_version=platform.python_version(),
                        packages={name:importlib.metadata.version(name) for name in ('numpy','matplotlib','Pillow','pytest')},
                        predictor=dict(window=4,speed_cap_m_s=.8),
                        collision_definition='Conservative robot-circle overlap with pedestrian/static geometry or wall; not exact physical contact.',
                        travel_time_population='Pairs where both modes succeed; all outcome counts remain visible.',
                        clearance_threshold_m=.20,waiting_speed_m_s=.01,
                        future_script_access='Scene manager only; planners receive current observations.')
        write_json(out/'protocol.json',protocol)
        write_json(out/'parameters.json',parameters)
        write_json(out/'cases.json',[make_scenario(f,s).to_dict() for s in seeds for f in FAMILIES])
        with zipfile.ZipFile(out/'frozen_source.zip','w',zipfile.ZIP_DEFLATED) as archive:
            for path in source_files(REPO):
                archive.write(path,path.relative_to(REPO))
    jobs = []
    for seed in seeds:
        for family in FAMILIES:
            for mode in (('reactive','predictive') if seed % 2 == 0 else ('predictive','reactive')):
                jobs.append((family,seed,mode))
    completed, trial_audits, pair_audits = {}, {}, {}
    ports, lock = set(), threading.Lock()

    def execute(job):
        family, seed, mode = job
        folder = out/'raw'/family/f'seed_{seed:06d}'/mode
        if folder.exists():
            if not args.resume:
                raise FileExistsError(folder)
            audit = audit_trial(folder)
            if read_json(folder/'source_manifest.json') != manifest:
                raise ValueError('Existing trial source differs from frozen manifest')
            return folder,audit,read_json(folder/'summary.json')[0],0.0
        if source_manifest(REPO) != manifest:
            raise ValueError('Runtime source changed after protocol freeze')
        with lock:
            while True:
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1',0))
                    port = sock.getsockname()[1]
                if port not in ports:
                    ports.add(port)
                    break
        cmd = [sys.executable,str(ROOT/'run.py'),'--planner',mode,'--scenario',family,
               '--seed',str(seed),'--parameters',str(out/'parameters.json'),
               '--output-dir',str(folder),'--port',str(port)]
        if args.webots: cmd += ['--webots',args.webots]
        if args.show: cmd += ['--show']
        if args.snapshot: cmd += ['--snapshot']
        started = time.monotonic()
        process = subprocess.run(cmd,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                 text=True,encoding='utf-8',errors='replace')
        if folder.exists():
            (folder/'launcher.log').write_text(process.stdout,encoding='utf-8')
        if process.returncode:
            raise RuntimeError(f'Infrastructure failure in {job}:\n{process.stdout[-5000:]}')
        if read_json(folder/'source_manifest.json') != manifest:
            raise ValueError('Actual simulator source does not match prospective freeze')
        return folder,audit_trial(folder),read_json(folder/'summary.json')[0],time.monotonic()-started

    pool = ThreadPoolExecutor(max_workers=args.workers)
    pending = {pool.submit(execute,job):job for job in jobs}
    try:
        for future in as_completed(pending):
            job = pending[future]
            folder,audit,result,wall = future.result()
            completed[job], trial_audits[job] = folder,audit
            family,seed,mode = job
            key = family,seed
            if all((family,seed,m) in completed for m in ('reactive','predictive')):
                pair_audits[key] = audit_pair(completed[(family,seed,'reactive')],completed[(family,seed,'predictive')])
                paired_folders = [completed[(f,s,m)] for f,s in sorted(pair_audits) for m in ('reactive','predictive')]
                rows,signature = convert_run_dirs(paired_folders)
                write_csv(rows,out/'summary.csv')
            write_json(out/'integrity_audit.json',dict(trials=list(trial_audits.values()),pairs=list(pair_audits.values())))
            write_json(out/'progress.json',dict(completed_runs=len(completed),expected_runs=len(jobs),
                                                completed_pairs=len(pair_audits),last_job=job,last_result=result,
                                                status='complete' if len(completed)==len(jobs) else 'running'))
            print(f'{len(completed)}/{len(jobs)} {family} seed={seed} {mode}: {result["outcome"]}, '
                  f'{result["elapsed_s"]:.3f}s, clearance={result["min_pedestrian_clearance_m"]:.3f}m '
                  f'(wall {wall:.1f}s)',flush=True)
    except BaseException as error:
        for future in pending:
            future.cancel()
        write_json(out/'infrastructure_error.json',dict(error=str(error),completed_runs=len(completed)))
        raise
    finally:
        pool.shutdown(wait=True,cancel_futures=True)
    if len(pair_audits) != 3*args.trials:
        raise ValueError('Incomplete experiment')
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_bytes((out/'summary.csv').read_bytes())
    print(f'COMPLETE {len(pair_audits)} pairs / {len(completed)} real Webots runs -> {out}',flush=True)


if __name__ == '__main__':
    main()
