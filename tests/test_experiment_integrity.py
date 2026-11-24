"""Synthetic metadata fixtures test rejection of unfair pairs; not experiment data."""
import json
import pytest
from pedestrian_navigation.tools.experiment_integrity import audit_pair


def pair_fixture(tmp_path):
    folders=[]
    for mode in ('reactive','predictive'):
        folder=tmp_path/mode
        trial=folder/'00_crossing_seed3'
        trial.mkdir(parents=True)
        config=dict(planner=mode,seed=3,scenarios=['crossing'],parameters={'horizon':2})
        for name,value in [('run_config.json',config),('source_manifest.json',{'core.py':'abc'}),
                           ('environment.json',{'backend':'webots'})]:
            (folder/name).write_text(json.dumps(value))
        for name,value in [('scenario.json',{'seed':3,'pedestrians':['same']}),
                           ('static_route.json',[[3,0]]),
                           ('initial_state.json',{'position':[-3,0,0],'wheel_positions':[0,0]})]:
            (trial/name).write_text(json.dumps(value))
        folders.append(folder)
    return folders


def test_pair_audit_accepts_identical_physical_inputs(tmp_path):
    result=audit_pair(*pair_fixture(tmp_path))
    assert result['max_initial_state_difference']==0


def test_pair_audit_rejects_changed_initial_state(tmp_path):
    folders=pair_fixture(tmp_path)
    path=folders[1]/'00_crossing_seed3/initial_state.json'
    state=json.loads(path.read_text())
    state['wheel_positions'][0]=.01
    path.write_text(json.dumps(state))
    with pytest.raises(ValueError,match='starting states'):
        audit_pair(*folders)


def test_pair_audit_rejects_changed_pedestrian_script(tmp_path):
    folders=pair_fixture(tmp_path)
    (folders[1]/'00_crossing_seed3/scenario.json').write_text(json.dumps({'seed':3,'pedestrians':['changed']}))
    with pytest.raises(ValueError,match='scripts differ'):
        audit_pair(*folders)
