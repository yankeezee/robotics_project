"""Retrospective illustration of a selected paired case, separate from aggregate evidence."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--family',default='oncoming',choices=('crossing','oncoming','stop_turn'))
    parser.add_argument('--seed',type=int,default=6008)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    fig,axes=plt.subplots(2,2,figsize=(12,7.4),constrained_layout=True)
    closest=[]
    for mode,color in (('reactive','#266c9f'),('predictive','#d8762c')):
        trial=next((root/'raw'/args.family/f'seed_{args.seed:06d}'/mode).glob('00_*'))
        with (trial/'trajectory.csv').open(newline='') as f:
            rows=list(csv.DictReader(f))
        observations=[json.loads(line) for line in (trial/'observations.jsonl').read_text().splitlines()]
        values={k:[float(r[k]) for r in rows] for k in
                ('time_s','x_m','y_m','pedestrian_clearance_m','speed_m_s','command_v_m_s')}
        minimum=min(range(len(rows)),key=lambda i:values['pedestrian_clearance_m'][i])
        closest.append(values['time_s'][minimum])
        label=f'{mode.capitalize()} (min {values["pedestrian_clearance_m"][minimum]:.3f} m)'
        axes[0,0].plot(values['x_m'],values['y_m'],color=color,lw=2,label=mode.capitalize())
        axes[0,0].scatter(values['x_m'][minimum],values['y_m'][minimum],marker='x',s=60,color=color)
        axes[0,1].plot(values['time_s'],values['pedestrian_clearance_m'],color=color,lw=2,label=label)
        axes[1,0].plot(values['time_s'],values['speed_m_s'],color=color,lw=1.8,label=mode.capitalize()+' actual')
        axes[1,0].plot(values['time_s'],values['command_v_m_s'],color=color,ls='--',lw=1,alpha=.7)
        axes[1,1].plot(values['time_s'],values['y_m'],color=color,lw=2,label=mode.capitalize()+' robot y')
    pedestrian_x=[o['pedestrians'][0]['x'] for o in observations]
    pedestrian_y=[o['pedestrians'][0]['y'] for o in observations]
    axes[0,0].plot(pedestrian_x,pedestrian_y,'--',color='#666',label='Observed pedestrian path')
    axes[1,1].plot(values['time_s'],pedestrian_y,'--',color='#666',label='Pedestrian y (same script)')
    axes[0,0].set(xlabel='x (m)',ylabel='y (m)',title='Full measured paths; crosses mark closest approach')
    axes[0,0].set_aspect('equal',adjustable='datalim')
    axes[0,1].axhline(.20,color='#9e7b20',ls='--',label='0.20 m threshold')
    axes[0,1].set(xlabel='Time (s)',ylabel='Boundary gap (m)',title='Clearance near the encounter',ylim=(0,.8))
    axes[1,0].set(xlabel='Time (s)',ylabel='Speed (m/s)',title='Actual speed (solid), command (dashed)')
    axes[1,1].set(xlabel='Time (s)',ylabel='y (m)',title='Lateral displacement near the encounter')
    for ax in (axes[0,1],axes[1,0],axes[1,1]):
        ax.set_xlim(min(closest)-3,max(closest)+3)
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle(f'Illustrative {args.family} case, seed {args.seed}: reactive vs predictive\n'
                 'Selected for close-proximity review; not representative aggregate evidence.',fontsize=12)
    output=root/'analysis'/f'illustrative_{args.family}_seed{args.seed}.png'
    output.parent.mkdir(exist_ok=True)
    fig.savefig(output,dpi=170)
    plt.close(fig)
    print(output)


if __name__=='__main__':
    main()
