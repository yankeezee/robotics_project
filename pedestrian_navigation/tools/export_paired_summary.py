"""Convert actual Webots run folders into Sophia's strict paired CSV schema.

Only accepts source-stamped Webots batches. Refuses incomplete pairs, mixed
parameters, missing run logs, and duplicate trials. Never synthesizes results.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

FAMILIES = ('crossing', 'oncoming', 'stop_turn')
MODES = ('reactive', 'predictive')
FIELDS = ('backend', 'family', 'seed', 'planner', 'success', 'collision', 'timeout',
          'time_s', 'path_m', 'min_clearance_m', 'near_margin', 'wait_s')


def convert_run_dirs(folders):
    seen = {}
    protocol = None
    signature = None
    for folder in map(Path, folders):
        config_file = folder / 'run_config.json'
        summary_file = folder / 'summary.json'
        manifest_file = folder / 'source_manifest.json'
        log_file = folder / 'webots.log'
        for required in (config_file, summary_file, manifest_file, log_file):
            if not required.is_file():
                raise ValueError(f'Not a complete Webots run folder, missing {required}')
        settings = json.loads(config_file.read_text(encoding='utf-8'))
        snapshots = json.loads(manifest_file.read_text(encoding='utf-8'))
        # The same parameters and source-code snapshot are essential for the
        # paired comparison. This also catches a one-sided tuning change.
        if protocol is None:
            protocol = settings['parameters']
            signature = snapshots
        elif protocol != settings['parameters'] or signature != snapshots:
            raise ValueError(f'Parameters or code snapshot differ at {folder}')
        mode = settings['planner']
        if mode not in MODES:
            raise ValueError(f'Invalid planner in {config_file}: {mode}')
        results = json.loads(summary_file.read_text(encoding='utf-8'))
        if not isinstance(results, list) or len(results) != len(settings['scenarios']):
            raise ValueError(f'Incomplete trial summary: {summary_file}')
        for result in results:
            family = result['scenario']
            seed = int(result['seed'])
            if family not in FAMILIES:
                # 'empty' and 'static' are development diagnostics, not paired
                # pedestrian experiment families.
                continue
            if result['planner'] != mode or family not in settings['scenarios'] or seed != int(settings['seed']):
                raise ValueError(f'Summary/config mismatch in {folder}: {result}')
            outcome = result['outcome']
            if outcome not in ('success', 'collision', 'timeout'):
                raise ValueError(f'Incomplete/unknown outcome {outcome!r} in {folder}')
            minimum = result['min_pedestrian_clearance_m']
            if minimum is None or not math.isfinite(float(minimum)):
                raise ValueError(f'Missing pedestrian clearance for {family}/{seed}/{mode}')
            t, path, wait = (float(result[k]) for k in ('elapsed_s', 'path_length_m', 'waiting_s'))
            if (not all(math.isfinite(x) for x in (t,path,wait)) or
                    t < 0 or path < 0 or not 0 <= wait <= t + 1e-5):
                raise ValueError(f'Invalid metrics for {family}/{seed}/{mode}')
            row = dict(backend='webots',family=family, seed=seed, planner=mode,
                       success=outcome=='success',collision=outcome=='collision',
                       timeout=outcome=='timeout',time_s=t,path_m=path,
                       min_clearance_m=minimum,
                       near_margin=bool(result['clearance_violation']),wait_s=wait)
            key = (family, seed, mode)
            if key in seen:
                raise ValueError(f'Duplicate trial {key}; run folders: {folder}')
            seen[key] = row
    if not seen:
        raise ValueError('No pedestrian trials found in input folders')
    pair_ids = {(f,s) for f,s,mode in seen}
    for f,s in pair_ids:
        for mode in MODES:
            if (f,s,mode) not in seen:
                raise ValueError(f'Missing {mode} run in pair {(f,s)}')
    return [seen[k] for k in sorted(seen)], hashlib.sha256(json.dumps(signature,sort_keys=True).encode()).hexdigest()


def write_csv(rows, target):
    target = Path(target)
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', action='append', required=True,
                        help='A complete Webots output folder; repeat for each run.')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rows, source_sha = convert_run_dirs(args.run_dir)
    dest = write_csv(rows, args.output)
    print(f'{len(rows)//2} complete paired cases ({len(rows)} runs) -> {dest}')
    print(f'source_manifest_signature={source_sha}')
    print('Next: python scripts/sophia_analyze.py --input', dest,
          '--output analysis_webots --backend webots --expected-pairs 20')

if __name__=='__main__':
    main()
