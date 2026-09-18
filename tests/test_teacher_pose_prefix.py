from dataclasses import replace
import json
import math

import numpy as np
import pytest

from aic_transfuser_lite.data.teacher_pose_prefix import (
    HeadingPrefixConfig, heading_prefix, mask_teacher_arrays, prefix_anchor_mask,
)


CFG = HeadingPrefixConfig(warmup_ns=100_000_000,baseline_ns=400_000_000,
    smoothing_ns=200_000_000,persistence_ns=300_000_000,backoff_ns=100_000_000)


def stream(error=lambda t: 0.):
    return [(i*100_000_000,math.atan2(math.sin(math.pi+error(i/10)),math.cos(math.pi+error(i/10))))
            for i in range(61)]


def evaluate(samples, **kwargs):
    return heading_prefix(samples,drive_start_ns=0,observed_end_ns=6_000_000_000,config=CFG,**kwargs)


def test_constant_mount_offset_and_angle_wrap_are_not_heading_drift():
    result = evaluate(stream(lambda t: math.radians(.3*math.sin(4*t))))
    assert result['reason'] == 'NO_DRIFT_DETECTED'
    assert result['invalid_from_ns'] is None
    assert result['valid_until_ns'] == 6_000_000_000
    assert abs(abs(result['baseline_offset_rad'])-math.pi) < .01


def test_sustained_drift_backdates_onset_and_never_readmits_recovered_tail():
    result = evaluate(stream(lambda t: math.radians(8.) if 2 <= t <= 3 else 0.))
    assert result['reason'] == 'HEADING_DRIFT'
    assert result['confirmed_at_ns']-result['first_violation_ns'] >= CFG.persistence_ns
    assert result['valid_until_ns'] == result['first_violation_ns']-CFG.smoothing_ns-CFG.backoff_ns
    assert result['valid_until_ns'] < 2_000_000_000
    assert result['valid_until_ns'] == result['invalid_from_ns']


def test_single_sensor_spike_does_not_truncate_a_good_prefix():
    result = evaluate(stream(lambda t: math.radians(30) if t == 3. else 0.))
    assert result['invalid_from_ns'] is None


def test_missing_heading_evidence_cannot_be_bridged_by_later_good_data():
    samples = [(t,a) for t,a in stream() if not 2_000_000_000 <= t <= 2_300_000_000]
    result = evaluate(samples)
    assert result['reason'] == 'HEADING_EVIDENCE_GAP'
    assert result['valid_until_ns'] == 1_800_000_000


def test_unverified_baseline_keeps_no_candidate_and_nonfinite_later_cuts_prefix():
    samples = stream(); samples[3] = (samples[3][0],float('nan'))
    assert evaluate(samples)['reason'] == 'BASELINE_UNVERIFIED'
    samples = stream(); samples[30] = (samples[30][0],float('nan'))
    assert evaluate(samples)['valid_until_ns'] == 2_900_000_000


def test_tail_without_enough_persistence_is_not_certified_as_good():
    result = evaluate(stream(lambda t: math.radians(8) if t >= 5.8 else 0.))
    assert result['reason'] == 'UNRESOLVED_TAIL_DRIFT'
    assert result['valid_until_ns'] < 5_800_000_000


@pytest.mark.parametrize('changes',[
    {'max_error_rad':float('nan')},{'max_error_rad':0.},{'max_error_rad':math.pi},
    {'persistence_ns':-1},{'max_gap_ns':True},{'smoothing_ns':.2},
])
def test_invalid_units_and_thresholds_are_rejected(changes):
    with pytest.raises(ValueError): replace(CFG,**changes)


def test_unordered_clock_or_duplicate_capture_is_rejected():
    for samples in ([(1,0.),(1,0.)],[(2,0.),(1,0.)],[(.1,0.)]):
        with pytest.raises(ValueError): evaluate(samples)


def test_repeated_ekf_update_matches_last_receipt_without_hiding_large_conflict():
    from tools.filter_teacher_pose_prefix import append_heading_capture
    samples = [(100,1.)]
    change = math.radians(.0170207)  # Same-stamp update measured in PC10 a09.
    assert append_heading_capture(samples,100,1.+change,math.radians(5)) == pytest.approx(change)
    assert samples == [(100,1.+change)]
    append_heading_capture(samples,100,1.5,math.radians(5))
    assert math.isnan(samples[0][1])
    append_heading_capture(samples,100,1.,math.radians(5))
    assert math.isnan(samples[0][1])
    with pytest.raises(ValueError,match='reversal'):
        append_heading_capture(samples,99,1.,math.radians(5))


def test_full_horizon_and_right_interpolation_endpoint_must_be_strictly_before_cutoff():
    times = np.array([0,6_949_999_999,6_950_000_000,6_990_000_000,7_000_000_000,10_000_000_000],np.int64)
    keep = prefix_anchor_mask(times,valid_from_ns=1,valid_until_ns=10_000_000_000)
    assert keep.tolist() == [False,True,False,False,False,False]
    # Target at 9.99 s is pre-cut, but possible support at 10.04 s is not.
    assert not keep[3]


def test_prefix_timestamp_arithmetic_does_not_overflow_int64():
    end = np.iinfo(np.int64).max
    times = np.array([end-4_000_000_000,end-1],np.int64)
    assert prefix_anchor_mask(times,valid_from_ns=0,valid_until_ns=int(end)).tolist() == [True,False]
    with pytest.raises(ValueError): prefix_anchor_mask(times.astype(float),valid_from_ns=0,valid_until_ns=int(end))


def arrays(n=4):
    return dict(observation_ns=np.arange(n,dtype=np.int64),xy_m=np.ones((n,30,2),np.float32),
        xy_mask=np.ones((n,30),bool),velocity_mps=np.ones((n,30),np.float32),
        velocity_mask=np.ones((n,30),bool),forward_avoidance_eligible=np.ones(n,bool))


def test_exclusion_masks_all_heads_without_modifying_source_or_promoting_old_rejection():
    original = arrays(); original['forward_avoidance_eligible'][0] = False
    result = mask_teacher_arrays(original,np.array([True,True,False,False]))
    assert result['forward_avoidance_eligible'].tolist() == [False,True,False,False]
    assert result['xy_mask'][:2].all() and not result['xy_mask'][2:].any()
    assert result['velocity_mask'][:2].all() and not result['velocity_mask'][2:].any()
    assert np.isnan(result['xy_m'][2:]).all() and np.isnan(result['velocity_mps'][2:]).all()
    assert np.isfinite(original['xy_m']).all() and original['xy_mask'].all()


def test_shape_and_invalid_valid_label_are_not_silently_accepted():
    wrong = arrays(); wrong['xy_m'] = np.ones((4,2,30),np.float32)
    with pytest.raises(ValueError,match='shape'): mask_teacher_arrays(wrong,np.ones(4,bool))
    wrong = arrays(); wrong['xy_m'][0,0,0] = float('nan')
    with pytest.raises(ValueError,match='finite'): mask_teacher_arrays(wrong,np.ones(4,bool))


def test_artifact_filter_retains_source_identity_and_existing_clearance_rejection(tmp_path,monkeypatch):
    import tools.filter_teacher_pose_prefix as module
    audit = tmp_path/'audit'; audit.mkdir()
    source_arrays = arrays()
    source_arrays['observation_ns'] = np.array([1_000_000_000,6_900_000_000,6_950_000_000,9_000_000_000],np.int64)
    source_arrays['forward_avoidance_eligible'][0] = False
    np.savez_compressed(audit/'observed_teachers.npz',**source_arrays)
    rows = [dict(run_id='r',epoch='e',label_index=i,observation_ns=int(t),epoch_bounds_ns=[0,12_000_000_000],
                 rejection_reasons=['RUN_NOT_CLEAN'] if i == 0 else [],teacher_reasons=['OK'],
                 usable_full=True,usable_partial=True,forward_avoidance_eligible=i != 0,
                 stop_probability=None,stop_reason='UNKNOWN') for i,t in enumerate(source_arrays['observation_ns'])]
    (audit/'anchors.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (audit/'audit.json').write_text(json.dumps(dict(run_id='r',driving_start_ns=0,
        label_contract=dict(dt_s=.1,shape=[4,30,2],config=dict(teacher_tolerance_ms=50.)))))
    original = {p.name:p.read_bytes() for p in audit.iterdir()}
    monkeypatch.setattr(module,'verified_bag',lambda *_: (tmp_path/'bag',{'raw/r/bag':'sha'}))
    monkeypatch.setattr(module,'paired_heading_samples',lambda *_: ([],{}))
    monkeypatch.setattr(module,'heading_prefix',lambda *_,**kw: dict(valid_from_ns=0,
        valid_until_ns=10_000_000_000,invalid_from_ns=10_000_000_000,reason='HEADING_DRIFT',trace=[]))
    out = tmp_path/'filtered'
    summary = module.filter_audit(tmp_path/'collected',audit,out)
    assert summary['prefix_candidate_anchors'] == 2
    assert summary['strict_forward_eligible_anchors'] == 1
    assert not summary['automatic_training_admission'] and not summary['training_split_assigned']
    with np.load(out/'prefix_candidates.npz') as bundle:
        assert bundle['source_label_index'].tolist() == [0,1]
        assert bundle['forward_avoidance_eligible'].tolist() == [False,True]
    actual = [json.loads(r) for r in (out/'anchors.jsonl').read_text().splitlines()]
    assert actual[0]['rejection_reasons'] == ['RUN_NOT_CLEAN']
    assert actual[2]['usable_full'] is False and actual[2]['stop_reason'] == 'UNKNOWN'
    assert all((audit/name).read_bytes() == value for name,value in original.items())
    with pytest.raises(FileExistsError): module.filter_audit(tmp_path/'collected',audit,out)
