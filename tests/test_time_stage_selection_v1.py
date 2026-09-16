from copy import deepcopy

import pytest

from aic_transfuser_lite.evaluation.time_stage_selection_v1 import StageSelectionPolicy,select_stage_candidate


def report(name='initial',factor=1.):
    return dict(candidate_id=name,population_sha256='fixed',xy={stage:dict(
        anchors=20,runs=2,ade_m=.02*factor,endpoint_3s_m=.05*factor) for stage in ['nominal','recovery']},
        launch=[dict(case_id=f'r{i}:{age}',run_id=f'r{i}',age_s=age,teacher_accepted=True,
                     accepted=True,steer_rad=.25) for i in (1,2) for age in (0.,.5)])


def choose(*rows):
    return select_stage_candidate(rows,baseline_id='initial',policy=StageSelectionPolicy())


def test_epoch_with_better_coordinate_error_but_launch_failure_is_not_promoted():
    initial=report();bad=report('epoch2',.8)
    bad['launch'][0].update(accepted=False,steer_rad=None)
    result=choose(initial,bad)
    assert result['selected_candidate_id']=='initial'
    assert result['runtime_test_allowed'] is False
    assert result['decisions'][1]['launch_rejected_case_ids']==['r1:0.0']


def test_initial_checkpoint_participates_in_ranking_and_wins_ties():
    assert choose(report(),report('new',1.01))['selected_candidate_id']=='initial'
    assert choose(report(),report('new'))['selected_candidate_id']=='initial'
    assert choose(report(),report('new',.9))['runtime_test_allowed'] is True


def test_recovery_gain_cannot_hide_nominal_regression_or_margin_loss():
    bad=report('new',.5);bad['xy']['nominal']['ade_m']=.025
    assert choose(report(),bad)['selected_candidate_id']=='initial'
    bad=report('new',.5);bad['launch'][0]['steer_rad']=.299
    assert choose(report(),bad)['selected_candidate_id']=='initial'


@pytest.mark.parametrize('change',['drop','teacher','age','support','population'])
def test_changing_validation_denominator_is_rejected(change):
    bad=report('new',.5)
    if change=='drop':bad['launch'].pop()
    elif change=='teacher':bad['launch'][0]['teacher_accepted']=False
    elif change=='age':bad['launch'][0]['age_s']=.1
    elif change=='support':bad['xy']['nominal']['anchors']=19
    else:bad['population_sha256']='different'
    with pytest.raises(ValueError):choose(report(),bad)


def test_no_qualified_checkpoint_keeps_baseline_without_runtime_authorization():
    initial=report();initial['launch'][0]['accepted']=False
    other=deepcopy(initial);other['candidate_id']='new'
    result=choose(initial,other)
    assert result['selected_candidate_id']=='initial'
    assert result['selected_meets_gate'] is False and result['runtime_test_allowed'] is False


def test_candidate_can_fix_an_initial_launch_failure():
    initial=report();initial['launch'][0]['accepted']=False
    result=choose(initial,report('new',.99))
    assert result['selected_candidate_id']=='new' and result['runtime_test_allowed'] is True


@pytest.mark.parametrize('value',[None,float('nan'),float('inf')])
def test_nonfinite_prediction_metric_fails_candidate_without_dropping_support(value):
    bad=report('new');bad['xy']['nominal']['ade_m']=value
    result=choose(report(),bad)
    assert result['decisions'][1]['eligible'] is False


def test_accepted_angle_above_physical_limit_is_not_admitted():
    bad=report('new',.8);bad['launch'][0]['steer_rad']=.3000001
    assert not choose(report(),bad)['runtime_test_allowed']
