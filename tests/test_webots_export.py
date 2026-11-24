"""Tests use SYNTHETIC fixture JSON, never reported as actual Webots runs."""
import json
from pathlib import Path
import pytest
from pedestrian_navigation.tools.export_paired_summary import convert_run_dirs,write_csv
from pednav.sophia.analysis import load_paired


def make_fake_run(root, mode, outcome='success', change_source=False, seed=3):
    folder = root / f'{mode}_{seed}'
    folder.mkdir()
    settings = {'planner': mode, 'scenarios':['crossing'], 'seed': seed, 'parameters':{'horizon':2.0}}
    result = {'scenario':'crossing','seed':seed,'planner':mode,'outcome':outcome,
              'elapsed_s':12.0,'path_length_m':2.1,'waiting_s':0.5,
              'min_pedestrian_clearance_m':0.24,'clearance_violation':False}
    (folder/'run_config.json').write_text(json.dumps(settings))
    (folder/'summary.json').write_text(json.dumps([result]))
    (folder/'source_manifest.json').write_text(json.dumps({'controller.py':'different' if change_source else 'abc'}))
    (folder/'webots.log').write_text('synthetic fixture; not a Webots run')
    return folder


def test_convert_paired_runs_to_sophia_schema(tmp_path):
    react = make_fake_run(tmp_path,'reactive')
    pred = make_fake_run(tmp_path,'predictive')
    rows,_ = convert_run_dirs([react,pred])
    path=write_csv(rows,tmp_path/'paired.csv')
    backend,pairs,_=load_paired(path,expected_backend='webots')
    assert backend=='webots' and len(pairs)==1
    assert pairs[('crossing',3)]['reactive']['success']
    assert pairs[('crossing',3)]['predictive']['success']


def test_missing_pair_rejected(tmp_path):
    react=make_fake_run(tmp_path,'reactive')
    with pytest.raises(ValueError, match='Missing predictive'):
        convert_run_dirs([react])


def test_version_mismatch_rejected(tmp_path):
    react=make_fake_run(tmp_path,'reactive')
    pred=make_fake_run(tmp_path,'predictive',change_source=True)
    with pytest.raises(ValueError,match='code snapshot'):
        convert_run_dirs([react,pred])


def test_incomplete_trial_rejected(tmp_path):
    react=make_fake_run(tmp_path,'reactive')
    pred=make_fake_run(tmp_path,'predictive',outcome='interrupted')
    with pytest.raises(ValueError,match='Incomplete/unknown outcome'):
        convert_run_dirs([react,pred])


def test_duplicate_trial_rejected(tmp_path):
    a=make_fake_run(tmp_path,'reactive')
    b=make_fake_run(tmp_path,'predictive')
    with pytest.raises(ValueError,match='Duplicate trial'):
        convert_run_dirs([a,b,a])
