"""Outcome failures must not be rewarded as shorter successful trips."""
from pedestrian_navigation.tools.analyze_experiment import summary_tables


def record(mode,success,time,path,clearance,wait,seed):
    return dict(planner=mode,seed=seed,success=success,collision=not success,timeout=False,
                near_margin=clearance<.2,time_s=time,path_m=path,min_clearance_m=clearance,wait_s=wait)


def test_failed_pairs_excluded_only_from_success_conditioned_metrics():
    pairs={}
    for family in ('crossing','oncoming','stop_turn'):
        pairs[(family,1)]={'reactive':record('reactive',True,20,4,.25,2,1),
                           'predictive':record('predictive',True,18,3.8,.3,1,1)}
        pairs[(family,2)]={'reactive':record('reactive',True,30,5,.21,4,2),
                           'predictive':record('predictive',False,2,.1,-.01,0,2)}
    rows,effects,details=summary_tables(pairs)
    predictive=next(r for r in rows if r['family']=='crossing' and r['planner']=='predictive')
    assert predictive['n']==2 and predictive['success_n']==1 and predictive['collision_n']==1
    assert predictive['median_time_both_success_s']==18
    assert predictive['worst_min_clearance_all_m']==-.01
    assert effects[0]['median_pred_minus_reactive_time_s']==-2
    assert effects[0]['reactive_only_success_n']==1
    failed=next(p for p in details if p['family']=='crossing' and p['seed']==2)
    assert failed['time_difference_s'] is None
