"""Tests that unsafe/incomplete outcome data cannot quietly enter the report."""
import csv
from pathlib import Path
import pytest
from pednav.sophia.analysis import load_paired, calculate_summary

FIELDS = ('backend','family','seed','planner','success','collision','timeout',
          'time_s','path_m','min_clearance_m','near_margin','wait_s')


def make_csv(tmp_path, rows):
    f=tmp_path/'summary.csv'
    with f.open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=FIELDS)
        writer.writeheader(); writer.writerows(rows)
    return f


def base_row(mode='reactive', **kwargs):
    row=dict(backend='kinematic',family='crossing',seed=5,planner=mode,success='True',
             collision='False',timeout='False',time_s='22.0',path_m='3.5',
             min_clearance_m='0.01',near_margin='True',wait_s='0.0')
    row.update(kwargs)
    return row


def test_paired_summary_valid(tmp_path):
    f=make_csv(tmp_path,[base_row(),base_row('predictive',time_s='21.0')])
    backend,pairs,sha=load_paired(f)
    assert backend=='kinematic' and len(pairs)==1
    x=calculate_summary(backend,pairs,sha)
    assert x['n_runs']==2
    assert x['overall']['predictive']['success']==1
    assert x['per_family'][0]['median_paired_pred_minus_reactive_time_s']==-1.0


def test_missing_pair_rejected(tmp_path):
    f=make_csv(tmp_path,[base_row()])
    with pytest.raises(ValueError,match='incomplete'):
        load_paired(f)


def test_duplicates_rejected(tmp_path):
    f=make_csv(tmp_path,[base_row(),base_row()])
    with pytest.raises(ValueError,match='duplicate'):
        load_paired(f)


def test_mixed_backend_rejected(tmp_path):
    f=make_csv(tmp_path,[base_row(),base_row('predictive',backend='webots')])
    with pytest.raises(ValueError,match='mixed backends'):
        load_paired(f)


def test_conflicting_outcome_rejected(tmp_path):
    f=make_csv(tmp_path,[base_row(collision='True'),base_row('predictive')])
    with pytest.raises(ValueError,match='terminal outcome'):
        load_paired(f)


def test_wrong_count_rejected(tmp_path):
    f=make_csv(tmp_path,[base_row(),base_row('predictive')])
    with pytest.raises(ValueError,match='expected 20 pairs'):
        load_paired(f,expected_pairs_per_family=20)
