"""Strict Webots analysis, six-metric plots, and checksummed reproducibility bundle."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
sys.path.insert(0,str(REPO))
sys.path.insert(0,str(ROOT))
from pednav.sophia.analysis import load_paired, calculate_summary, export_analysis
from tools.experiment_integrity import read_json, audit_pair, audit_trial
from tools.export_paired_summary import convert_run_dirs, write_csv

FAMILIES = ('crossing','oncoming','stop_turn')
MODES = ('reactive','predictive')
COLORS = {'reactive':'#266c9f','predictive':'#d8762c'}
LABELS = {'crossing':'Crossing','oncoming':'Oncoming','stop_turn':'Stop/turn'}


def save_csv(path, rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def median(values):
    return float(np.median(values)) if values else None


def summary_tables(pairs):
    rows, effects, per_pair = [], [], []
    for family in FAMILIES:
        group=[p for (f,s),p in pairs.items() if f==family]
        both=[p for p in group if all(p[m]['success'] for m in MODES)]
        for mode in MODES:
            records=[p[mode] for p in group]
            rows.append(dict(family=family,planner=mode,n=len(records),
                             success_n=sum(r['success'] for r in records),
                             success_rate=sum(r['success'] for r in records)/len(records),
                             collision_n=sum(r['collision'] for r in records),
                             timeout_n=sum(r['timeout'] for r in records),
                             clearance_violation_n=sum(r['near_margin'] for r in records),
                             both_success_pairs=len(both),
                             median_time_both_success_s=median([p[mode]['time_s'] for p in both]),
                             median_path_both_success_m=median([p[mode]['path_m'] for p in both]),
                             median_min_clearance_all_m=median([r['min_clearance_m'] for r in records]),
                             worst_min_clearance_all_m=min(r['min_clearance_m'] for r in records),
                             median_wait_all_s=median([r['wait_s'] for r in records]),
                             median_wait_both_success_s=median([p[mode]['wait_s'] for p in both])))
        effects.append(dict(family=family,n_pairs=len(group),both_success_pairs=len(both),
                            predictive_only_success_n=sum(p['predictive']['success'] and not p['reactive']['success'] for p in group),
                            reactive_only_success_n=sum(p['reactive']['success'] and not p['predictive']['success'] for p in group),
                            median_pred_minus_reactive_time_s=median([p['predictive']['time_s']-p['reactive']['time_s'] for p in both]),
                            median_pred_minus_reactive_path_m=median([p['predictive']['path_m']-p['reactive']['path_m'] for p in both]),
                            median_pred_minus_reactive_wait_both_success_s=median([p['predictive']['wait_s']-p['reactive']['wait_s'] for p in both]),
                            median_pred_minus_reactive_clearance_all_m=median([p['predictive']['min_clearance_m']-p['reactive']['min_clearance_m'] for p in group])))
        for pair in group:
            r,p=pair['reactive'],pair['predictive']
            per_pair.append(dict(family=family,seed=r['seed'],reactive_success=r['success'],predictive_success=p['success'],
                                 both_success=r['success'] and p['success'],
                                 time_difference_s=p['time_s']-r['time_s'] if r['success'] and p['success'] else None,
                                 path_difference_m=p['path_m']-r['path_m'] if r['success'] and p['success'] else None,
                                 waiting_difference_all_s=p['wait_s']-r['wait_s'],
                                 clearance_difference_all_m=p['min_clearance_m']-r['min_clearance_m']))
    return rows,effects,per_pair


def plot_dashboard(pairs, output, phase):
    fig,axes=plt.subplots(2,3,figsize=(15,8.5),constrained_layout=True)
    for ax, metric,title in ((axes[0,0],'success','Success rate'),(axes[0,1],'collision','Collisions')):
        for offset,mode in ((-.18,'reactive'),(.18,'predictive')):
            groups=[[p[mode] for (f,s),p in pairs.items() if f==fam] for fam in FAMILIES]
            values=[sum(r[metric] for r in g) for g in groups]
            if metric=='success': values=[100*v/len(g) for v,g in zip(values,groups)]
            bars=ax.bar(np.arange(3)+offset,values,.34,color=COLORS[mode])
            ax.bar_label(bars,labels=[f'{v:.0f}%' if metric=='success' else str(v) for v in values],padding=3,fontsize=9)
        ax.set(title=title,ylabel='Percent of all runs' if metric=='success' else 'Number of runs')
        ax.set_ylim(0,110 if metric=='success' else max(len(g) for g in groups)+2)
    def paired_points(ax,metric,title,ylabel,both_only=False):
        for index,family in enumerate(FAMILIES):
            group=[p for (f,s),p in pairs.items() if f==family and
                   (not both_only or all(p[m]['success'] for m in MODES))]
            jitter=np.linspace(-.07,.07,len(group)) if group else []
            for shift,pair in zip(jitter,group):
                ax.plot([index-.17+shift,index+.17+shift],
                        [pair['reactive'][metric],pair['predictive'][metric]],color='#c3c8cc',lw=.6,zorder=1)
            for offset,mode in ((-.17,'reactive'),(.17,'predictive')):
                values=[p[mode][metric] for p in group]
                ax.scatter(index+offset+np.asarray(jitter),values,color=COLORS[mode],s=18,alpha=.8,zorder=2)
                if values:
                    ax.plot([index+offset-.11,index+offset+.11],[np.median(values)]*2,color=COLORS[mode],lw=3,zorder=3)
            if both_only:
                ax.text(index,.98,f'n={len(group)} pairs',ha='center',va='top',fontsize=9,transform=ax.get_xaxis_transform())
        ax.set(title=title,ylabel=ylabel)
    paired_points(axes[0,2],'time_s','Travel time: both successful','Seconds',True)
    paired_points(axes[1,0],'path_m','Path length: both successful','Metres',True)
    paired_points(axes[1,1],'min_clearance_m','Minimum clearance: all outcomes','Boundary gap (m)')
    axes[1,1].axhline(.20,color='#9e7b20',ls='--',lw=1.5)
    axes[1,1].axhline(0,color='#777',lw=.7)
    paired_points(axes[1,2],'wait_s','Waiting: all outcomes','Seconds')
    for ax in axes.flat:
        ax.set_xticks(range(3),[LABELS[f] for f in FAMILIES])
        ax.grid(axis='y',alpha=.18)
    handles=[Line2D([0],[0],color=COLORS[m],lw=4,label=m.capitalize()) for m in MODES]
    fig.legend(handles=handles,loc='outside lower center',ncol=2,frameon=False)
    fig.suptitle(f'Webots R2025b | {len(pairs)} identical-seed pairs | {phase}\n'
                 'Horizontal marks: medians. Grey lines: paired cases. Clearance threshold: 0.20 m.',fontsize=13)
    for suffix in ('png','svg'):
        fig.savefig(output/f'comparison_dashboard.{suffix}',dpi=180)
    plt.close(fig)

    fig,axes=plt.subplots(1,3,figsize=(14,4.5),constrained_layout=True)
    for ax,metric,title in zip(axes,('time_s','path_m','wait_s'),('Travel time (s)','Path length (m)','Waiting (s)')):
        for index,family in enumerate(FAMILIES):
            group=[p for (f,s),p in pairs.items() if f==family and all(p[m]['success'] for m in MODES)]
            values=[p['predictive'][metric]-p['reactive'][metric] for p in group]
            jitter=np.linspace(-.12,.12,len(values)) if values else []
            ax.scatter(values,index+np.asarray(jitter),s=25,color='#416b82',alpha=.75)
            if values:
                ax.scatter([np.median(values)],[index],marker='D',s=65,color='#bb5b22',zorder=3)
        ax.axvline(0,color='#555',ls='--',lw=1)
        ax.set_yticks(range(3),[LABELS[f] for f in FAMILIES])
        ax.set(xlabel='Predictive minus reactive',title=title)
        ax.grid(alpha=.15)
    fig.suptitle('Paired differences where both planners succeed | Negative = less time, distance or waiting\n'
                 'Each dot is a seed; diamonds are medians. Descriptive results, no significance claim.',fontsize=12)
    for suffix in ('png','svg'):
        fig.savefig(output/f'paired_differences.{suffix}',dpi=180)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--experiment',required=True,type=Path)
    args=parser.parse_args()
    root=args.experiment.resolve()
    protocol=read_json(root/'protocol.json')
    expected=protocol['n_pairs']//3
    folders=[root/'raw'/f/f'seed_{s:06d}'/m for s in protocol['seeds'] for f in FAMILIES for m in MODES]
    audits=[audit_trial(p) for p in folders]
    pair_audits=[audit_pair(root/'raw'/f/f'seed_{s:06d}'/'reactive',root/'raw'/f/f'seed_{s:06d}'/'predictive')
                 for s in protocol['seeds'] for f in FAMILIES]
    rows,source_signature=convert_run_dirs(folders)
    write_csv(rows,root/'summary.csv')
    backend,pairs,sha=load_paired(root/'summary.csv',expected_pairs_per_family=expected,expected_backend='webots')
    if len(pairs) != protocol['n_pairs']:
        raise ValueError('Pair count disagrees with prospective protocol')
    analysis=root/'analysis'
    analysis.mkdir(exist_ok=True)
    result=calculate_summary(backend,pairs,sha)
    export_analysis(result,analysis)
    metrics,effects,paired=summary_tables(pairs)
    save_csv(analysis/'metrics_by_family.csv',metrics)
    save_csv(analysis/'paired_effects.csv',effects)
    save_csv(analysis/'per_pair_differences.csv',paired)
    plot_dashboard(pairs,analysis,protocol['phase'].replace('_',' '))
    integrity=dict(trials=audits,pairs=pair_audits,source_manifest_signature=source_signature,
                   max_initial_state_difference=max(p['max_initial_state_difference'] for p in pair_audits),
                   all_raw_metrics_verified=True)
    (root/'integrity_audit.json').write_text(json.dumps(integrity,indent=2)+'\n',encoding='utf-8')
    def fmt(x): return 'NA' if x is None else f'{x:.3f}'
    lines=['# Paired Webots experiment results','',
           f"Phase: **{protocol['phase']}**. {len(pairs)} pairs / {2*len(pairs)} real simulator runs; "
           f"seeds {protocol['seeds'][0]}–{protocol['seeds'][-1]}; {expected} cases per family.", '',
           '## Outcomes and measurements','',
           '| Family | Mode | Success | Collisions | Timeouts | Below 0.20 m | Median time, both succeed (s) | Median path, both succeed (m) | Median min gap, all (m) | Worst gap (m) | Median waiting, all (s) |',
           '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in metrics:
        lines.append(f"| {r['family']} | {r['planner']} | {r['success_n']}/{r['n']} | {r['collision_n']} | {r['timeout_n']} | "
                     f"{r['clearance_violation_n']}/{r['n']} | {fmt(r['median_time_both_success_s'])} | {fmt(r['median_path_both_success_m'])} | "
                     f"{fmt(r['median_min_clearance_all_m'])} | {fmt(r['worst_min_clearance_all_m'])} | {fmt(r['median_wait_all_s'])} |")
    lines += ['', '## Paired differences','',
              'Predictive minus reactive. Travel time, path length and this waiting comparison include only cases where **both succeed**.', '',
              '| Family | Both-success pairs | Median time difference (s) | Median path difference (m) | Median waiting difference (s) | Median min-gap difference, all (m) |',
              '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in effects:
        lines.append(f"| {r['family']} | {r['both_success_pairs']} | {fmt(r['median_pred_minus_reactive_time_s'])} | "
                     f"{fmt(r['median_pred_minus_reactive_path_m'])} | {fmt(r['median_pred_minus_reactive_wait_both_success_s'])} | "
                     f"{fmt(r['median_pred_minus_reactive_clearance_all_m'])} |")
    lines += ['', '## Verification and interpretation limits','',
              '- Both variants use the same robot, parameters, scripts, control limits, guidance, scoring and braking code. Only the pedestrian motion model differs.',
              '- Every trial uses a fresh Webots process. Scenario/source/environment equality and initial physical states were audited within each pair.',
              '- The raw observations, trajectories and diagnostics were checked for timestamp alignment, scripted pedestrian positions, command limits and consistency with summary metrics.',
              '- Failures and clearance violations are retained. Collision means conservative circle/wall overlap, not exact measured physical contact.',
              '- Travel time and path comparisons are conditional on joint success. Minimum clearance and waiting summaries include all outcomes; early termination limits their interpretation.',
              '- The current integrated scenarios contain **one pedestrian** each. This does not cover the proposal\'s full one-to-five-person scope.',
              '- Constant-velocity prediction can be wrong after stops/turns. Shared conservative proximity guidance may limit its efficiency benefit.',
              '- The predictor uses four observations and a 0.8 m/s speed cap; it does not receive future script positions.',
              '- These are descriptive simulation results, not evidence of real-world safety or statistical significance.', '',
              '## Artifacts','',
              '- `protocol.json`, `parameters.json`, `cases.json`: prospective settings and exact seeded scripts.',
              '- `frozen_source.zip`: executable source snapshot; `summary.csv`: every measured run.',
              '- `raw_logs.zip`: all per-trial logs, configurations, source snapshots and environments.',
              '- `integrity_audit.json`: per-trial and per-pair checks; `artifact_manifest.json`: file hashes.',
              '- `analysis/metrics_by_family.csv`, `analysis/paired_effects.csv`, `analysis/per_pair_differences.csv`.',
              '- `analysis/paired_analysis.json` and Sophia\'s three plots are also retained.', '',
              '![Six requested metrics](analysis/comparison_dashboard.png)','',
              '![Paired effects](analysis/paired_differences.png)','']
    (root/'RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    archive_path=root/'raw_logs.zip'
    with zipfile.ZipFile(archive_path,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for path in sorted((root/'raw').rglob('*')):
            if path.is_file(): archive.write(path,path.relative_to(root))
    file_hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in sorted(root.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json'}
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),files=file_hashes,
                  raw_archive_bytes=archive_path.stat().st_size,raw_file_count=sum(k.startswith('raw/') for k in file_hashes))
    (root/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result['overall'],indent=2))
    print(root/'RESULTS.md')


if __name__=='__main__':
    main()
