"""Strict, reproducible paired-trial analysis for kinematic or Webots CSVs.

A trial pair is (scenario family, seed), with exactly one reactive and one
predictive result. Missing/duplicate members and mixed backends are errors,
not silently dropped. Comparative travel-time/path statistics use only pairs
where BOTH planners reached the goal collision-free.
"""
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path
import math
import numpy as np

FAMILIES = ('crossing', 'oncoming', 'stop_turn')
MODES = ('reactive', 'predictive')
REQUIRED = ('backend','family','seed','planner','success','collision','timeout',
            'time_s','path_m','min_clearance_m','near_margin','wait_s')


def _bool(raw):
    v = str(raw).strip().lower()
    if v in ('true', '1'): return True
    if v in ('false', '0'): return False
    raise ValueError(f'boolean must be true/false or 1/0, got {raw!r}')


def _finite(name, raw):
    value = float(raw)
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite, got {raw!r}')
    return value


def load_paired(csv_path, expected_pairs_per_family=None, expected_backend=None):
    """Return backend, dict of keyed paired records, file SHA256.

    expected_pairs_per_family=20 enforces the agreed held-out Webots protocol;
    set to 1 for integration smoke testing. None allows exploratory subsets.
    """
    source = Path(csv_path)
    data = source.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    with source.open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        missing = set(REQUIRED) - set(reader.fieldnames or [])
        if missing: raise ValueError('Missing fields: ' + ', '.join(sorted(missing)))
        raw = list(reader)
    if not raw: raise ValueError('Summary CSV is empty')
    seen = {}
    backends = set()
    for row in raw:
        family = row['family'].strip()
        mode = row['planner'].strip()
        backend = row['backend'].strip()
        if family not in FAMILIES or mode not in MODES:
            raise ValueError(f'unknown family/mode: {family}/{mode}')
        if backend not in ('kinematic', 'webots'):
            raise ValueError(f'unknown backend: {backend}')
        backends.add(backend)
        seed = int(row['seed'])
        key = (family, seed, mode)
        if key in seen: raise ValueError(f'duplicate trial {key}')
        b = {k: _bool(row[k]) for k in ('success','collision','timeout','near_margin')}
        if sum(int(b[k]) for k in ('success','collision','timeout')) != 1:
            raise ValueError(f'exactly one terminal outcome is required for {key}')
        nums = {k:_finite(k,row[k]) for k in ('time_s','path_m','min_clearance_m','wait_s')}
        if nums['time_s'] < 0 or nums['path_m'] < 0 or nums['wait_s'] < 0:
            raise ValueError(f'negative elapsed time/path/wait for {key}')
        if nums['wait_s'] > nums['time_s'] + 1e-7:
            raise ValueError(f'wait time exceeds elapsed time for {key}')
        seen[key] = dict(backend=backend, family=family, seed=seed, planner=mode, **b, **nums)
    if len(backends) != 1: raise ValueError('mixed backends not allowed in one analysis')
    backend = next(iter(backends))
    if expected_backend and backend != expected_backend:
        raise ValueError(f'expected {expected_backend}, found {backend}')
    pair_keys = {(family, seed) for family, seed, mode in seen}
    paired = {}
    for fam, seed in sorted(pair_keys):
        if (fam, seed, 'reactive') not in seen or (fam, seed, 'predictive') not in seen:
            raise ValueError(f'incomplete trial pair {fam}/{seed}')
        paired[(fam, seed)] = {m: seen[(fam, seed, m)] for m in MODES}
    if expected_pairs_per_family is not None:
        for fam in FAMILIES:
            n = sum(f == fam for f, seed in paired)
            if n != expected_pairs_per_family:
                raise ValueError(f'{fam}: expected {expected_pairs_per_family} pairs, got {n}')
    return backend, paired, sha


def calculate_summary(backend, paired, source_sha256):
    """Pure descriptive statistics; no significance or safety certification."""
    by_family = []
    joint_success = []
    for fam in FAMILIES:
        family_pairs = [p for (f,_), p in paired.items() if f == fam]
        if not family_pairs: continue
        both = [p for p in family_pairs if p['reactive']['success'] and p['predictive']['success']]
        # Discordant paired outcomes (useful to understand conditional differences).
        improved = sum(p['predictive']['success'] and not p['reactive']['success'] for p in family_pairs)
        worsened = sum(p['reactive']['success'] and not p['predictive']['success'] for p in family_pairs)
        diffs = {}
        for metric in ('time_s','path_m','wait_s'):
            diffs[metric] = [p['predictive'][metric]-p['reactive'][metric] for p in both]
        for mode in MODES:
            records = [p[mode] for p in family_pairs]
            by_family.append({
                'family':fam, 'planner':mode, 'n':len(records),
                'success_n':sum(r['success'] for r in records),
                'collision_n':sum(r['collision'] for r in records),
                'timeout_n':sum(r['timeout'] for r in records),
                'near_margin_n':sum(r['near_margin'] for r in records),
                'median_wait_all_s':float(np.median([r['wait_s'] for r in records])),
                'both_success_pairs':len(both),
                'predictive_only_success_pairs':improved,
                'reactive_only_success_pairs':worsened,
                **{f'median_paired_pred_minus_reactive_{metric}':
                   float(np.median(diffs[metric])) if diffs[metric] else None
                   for metric in diffs}
            })
        for p in both:
            joint_success.append({'family':fam, 'seed':p['reactive']['seed'],
                                  'reactive_time_s':p['reactive']['time_s'],
                                  'predictive_time_s':p['predictive']['time_s'],
                                  'reactive_path_m':p['reactive']['path_m'],
                                  'predictive_path_m':p['predictive']['path_m'],
                                  'reactive_wait_s':p['reactive']['wait_s'],
                                  'predictive_wait_s':p['predictive']['wait_s']})
    totals = {}
    for mode in MODES:
        all_records = [p[mode] for p in paired.values()]
        totals[mode] = {field:sum(r[field] for r in all_records)
                        for field in ('success','collision','timeout','near_margin')}
        totals[mode]['n'] = len(all_records)
    return {'backend':backend, 'input_sha256':source_sha256,
            'n_pairs':len(paired), 'n_runs':2*len(paired),
            'per_family':by_family, 'overall':totals,
            'both_success_data':joint_success,
            'interpretation': 'Descriptive, backend-specific analysis; no real-world safety claim.'}


def export_analysis(result, output_dir):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output/'paired_analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False) + '\n', encoding='utf-8')
    family_rows = result['per_family']
    with (output/'paired_by_family.csv').open('w', newline='') as f:
        w = csv.DictWriter(f,fieldnames=list(family_rows[0].keys()))
        w.writeheader(); w.writerows(family_rows)
    with (output/'joint_success_only.csv').open('w', newline='') as f:
        fields = ('family','seed','reactive_time_s','predictive_time_s',
                  'reactive_path_m','predictive_path_m','reactive_wait_s','predictive_wait_s')
        w = csv.DictWriter(f,fieldnames=fields)
        w.writeheader(); w.writerows(result['both_success_data'])
    _plot(result, output)


def _plot(result, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    family_labels = {'crossing':'Crossing','oncoming':'Oncoming','stop_turn':'Stop/turn'}
    families = [fam for fam in FAMILIES if any(r['family']==fam for r in result['per_family'])]
    fig, axes = plt.subplots(1,2,figsize=(11,4.3))
    for ax, metric, title in zip(axes, ('success_n','collision_n'),('Goal reached','Geometric collisions')):
        x = np.arange(len(families)); width=.36
        for j, mode in enumerate(MODES):
            vals = [next(r[metric] for r in result['per_family'] if r['family']==fam and r['planner']==mode) for fam in families]
            ax.bar(x + (j-.5)*width, vals,width,label=mode.capitalize())
        ax.set_xticks(x,[family_labels[f] for f in families])
        ax.set_ylabel('Number of runs'); ax.grid(axis='y',alpha=.18)
        ax.set_ylim(0,max(r['n'] for r in result['per_family'])+1)
        ax.set_title(title)
    axes[0].legend()
    fig.suptitle(f"{result['backend'].capitalize()} simulator | {result['n_pairs']} paired cases | not a safety certification")
    fig.tight_layout()
    fig.savefig(out/'sophia_outcomes.png',dpi=170)
    plt.close(fig)

    fig,ax=plt.subplots(figsize=(7.6,5))
    for fam,marker in zip(families,('o','s','^')):
        arr=[p for p in result['both_success_data'] if p['family']==fam]
        if arr:
            ax.scatter([p['reactive_time_s'] for p in arr],
                       [p['predictive_time_s'] for p in arr],
                       label=f'{family_labels[fam]} (n={len(arr)})',marker=marker,alpha=.8)
    vals=[v for p in result['both_success_data'] for v in (p['reactive_time_s'],p['predictive_time_s'])]
    mx=max(vals) if vals else 1
    ax.plot([0,mx*1.04],[0,mx*1.04],linestyle='--',color='grey')
    ax.set(xlim=(0,mx*1.05),ylim=(0,mx*1.05),
           xlabel='Reactive travel time (s)',ylabel='Predictive travel time (s)',
           title=f"{result['backend'].capitalize()}: paired travel times, both successful only")
    ax.legend(); ax.grid(alpha=.15)
    fig.tight_layout(); fig.savefig(out/'sophia_paired_times.png',dpi=170)
    plt.close(fig)

    fig,ax=plt.subplots(figsize=(7.6,4.3))
    x=np.arange(len(families)); width=.36
    for j,mode in enumerate(MODES):
        counts=[next(r['near_margin_n'] for r in result['per_family'] if r['family']==fam and r['planner']==mode) for fam in families]
        ax.bar(x+(j-.5)*width,counts,width,label=mode.capitalize())
    ax.set_xticks(x,[family_labels[f] for f in families]); ax.set_ylabel('Number of runs')
    ax.set_title(f"{result['backend'].capitalize()}: minimum pedestrian boundary gap < 0.20 m")
    ax.legend(); ax.grid(axis='y',alpha=.16)
    fig.tight_layout(); fig.savefig(out/'sophia_clearance.png',dpi=170)
    plt.close(fig)
