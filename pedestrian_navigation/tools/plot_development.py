"""Plot two Webots development revisions. Requires optional matplotlib."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(root, family):
    trial = next(Path(root).glob(f'*_{family}_seed0'))
    with (trial/'trajectory.csv').open(encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    return rows, json.loads((trial/'summary.json').read_text()), json.loads((trial/'scenario.json').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', required=True, type=Path)
    parser.add_argument('--after', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 3, figsize=(14, 7), constrained_layout=True)
    for column, family in enumerate(('crossing', 'oncoming', 'stop_turn')):
        for root, label, color in ((args.before, 'Original baseline', '#bf4a42'),
                                    (args.after, 'Revised reactive', '#226e99')):
            rows, summary, scenario = load(root, family)
            x, y = [float(r['x_m']) for r in rows], [float(r['y_m']) for r in rows]
            axes[0, column].plot(x, y, color=color, label=label, lw=2)
            axes[0, column].scatter(x[-1], y[-1], color=color,
                                    marker='x' if summary['outcome'] == 'collision' else 'o', s=40)
            axes[1, column].plot([float(r['time_s']) for r in rows],
                                    [float(r['pedestrian_clearance_m']) for r in rows],
                                    color=color, label=label, lw=1.6)
        for pedestrian in scenario['pedestrians']:
            points = pedestrian['waypoints']
            axes[0, column].plot([v[1] for v in points], [v[2] for v in points],
                                 '--', color='#555555', alpha=0.8, label='Scripted pedestrian path')
        axes[0, column].scatter(*scenario['goal'], color='#39804c', marker='*', s=90, label='Goal')
        for wall in (-scenario['half_width'], scenario['half_width']):
            axes[0, column].axhline(wall, color='#666666', lw=1)
        axes[0, column].set(title=family.replace('_', '/'), xlabel='x (m)', ylabel='y (m)',
                             xlim=(-3.2, 3.2), ylim=(-1.5, 1.5))
        axes[0, column].set_aspect('equal', adjustable='box')
        axes[1, column].axhline(0.20, color='#b18d2b', ls='--', label='0.20 m evaluation margin')
        axes[1, column].axhline(0, color='#555555', lw=1)
        axes[1, column].set(xlabel='Trial time (s)', ylabel='Pedestrian clearance (m)')
        axes[1, column].grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8, loc='upper left')
    axes[1, 0].legend(fontsize=8, loc='upper right')
    fig.suptitle('Webots development comparison: original vs revised reactive baseline, seed 0\n'
                 'Single cases; no predictive planner and no held-out evaluation', fontsize=13)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
