"""Descriptive paired analysis: no hypothesis significance inflation on small N."""
import csv
from collections import defaultdict
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(csvfile, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with open(csvfile, newline='') as f:
        rows = list(csv.DictReader(f))
    grouped = defaultdict(dict)
    for row in rows:
        grouped[(row['family'], int(row['seed']))][row['planner']] = row
    families = ('crossing', 'oncoming', 'stop_turn')
    summary = []
    for family in families:
        pairs = [pair for (fam, _), pair in grouped.items() if fam == family and set(pair) == {'reactive','predictive'}]
        for mode in ('reactive', 'predictive'):
            rec = [p[mode] for p in pairs]
            both = [p for p in pairs if p['reactive']['success'] == 'True' and p['predictive']['success'] == 'True']
            time_diff = [float(p['predictive']['time_s']) - float(p['reactive']['time_s']) for p in both]
            summary.append({'family': family, 'planner': mode, 'n': len(rec),
                'success_n': sum(r['success'] == 'True' for r in rec),
                'collision_n': sum(r['collision'] == 'True' for r in rec),
                'timeout_n': sum(r['timeout'] == 'True' for r in rec),
                'near_margin_n': sum(r['near_margin'] == 'True' for r in rec),
                'median_wait_s': float(np.median([float(r['wait_s']) for r in rec])),
                'both_success_pairs': len(both),
                'median_paired_predictive_minus_reactive_time_s': float(np.median(time_diff)) if time_diff else float('nan')})
    with (output / 'by_family.csv').open('w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=summary[0].keys()); wr.writeheader(); wr.writerows(summary)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.1))
    x = np.arange(len(families)); width = .35
    for k, mode in enumerate(('reactive', 'predictive')):
        rec = [r for r in summary if r['planner'] == mode]
        axes[0].bar(x + (k - .5) * width, [r['success_n'] for r in rec], width, label=mode)
        axes[1].bar(x + (k - .5) * width, [r['near_margin_n'] for r in rec], width, label=mode)
    axes[0].set_title('Goal reached without collision')
    axes[1].set_title('Runs below 0.20 m clearance')
    for ax in axes:
        ax.set_xticks(x, families); ax.set_ylabel('Count / 20'); ax.set_ylim(0, 20.5); ax.grid(axis='y', alpha=.2)
    axes[0].legend()
    fig.suptitle('Kinematic simulator only — NOT Webots results', fontsize=12)
    fig.tight_layout()
    fig.savefig(output / 'outcomes.png', dpi=170)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    xlim = [0, 120]
    for family, mark in zip(families, ('o', 's', '^')):
        pairs = [p for (fam, _), p in grouped.items() if fam == family]
        xx = [float(p['reactive']['time_s']) for p in pairs if p.get('reactive', {}).get('success') == 'True' and p.get('predictive', {}).get('success') == 'True']
        yy = [float(p['predictive']['time_s']) for p in pairs if p.get('reactive', {}).get('success') == 'True' and p.get('predictive', {}).get('success') == 'True']
        ax.scatter(xx, yy, marker=mark, label=f'{family} (n={len(xx)})', alpha=.75)
    ax.plot(xlim, xlim, '--', color='grey', linewidth=1)
    ax.set(xlim=xlim, ylim=xlim, xlabel='Reactive travel time (s)', ylabel='Predictive travel time (s)',
           title='Travel time: only paired cases where both succeed')
    ax.legend(); ax.grid(alpha=.18)
    fig.tight_layout(); fig.savefig(output / 'paired_times.png', dpi=170); plt.close(fig)
    return summary
