"""Synthetic P1 regressions for causal time dataset (run in WSL)."""
from __future__ import annotations
import numpy as np
from aic_transfuser_lite.data.mcap_converter_v2 import TimedCommand, TimedImage, TimedLidar, TimedPose, TimedSteering, TimedVelocity
from aic_transfuser_lite.data.time_dataset_v1 import TimeDataset, TimeDatasetConfig, assemble_time_inputs, assemble_time_sample
from aic_transfuser_lite.data.time_history_v1 import TimeEvent

def ev(role, t, payload, *, avail=None, seq=0, clock="sim", epoch="ep"):
    return TimeEvent(role, "run", epoch, clock, "receipt", t, t if avail is None else avail, seq, payload, "bag_receipt_proxy")

def fixture(*, camera_late=False, steering=False):
    rows = []
    for i in range(56):
        t = i * 100_000_000
        rows += [ev("pose", t, TimedPose(t, i*.2, 0., 0., "map", "base_link")), ev("velocity", t, TimedVelocity(t, 2., 0., 0.)), ev("lidar", t, TimedLidar(t, np.ones(4), 0., 1., .1, 25.)), ev("final_command", t, TimedCommand(t, 2., 0., 0.))]
        rows.append(ev("camera", t, TimedImage(t, np.zeros((2, 3, 3), dtype=np.uint8)), avail=t + (100_000_000 if camera_late and i == 20 else 1)))
        if steering:
            rows.append(ev("actual_steering", t, TimedSteering(t, .1)))
    return rows, next(x for x in rows if x.role == "camera" and x.capture_ns == 2_000_000_000)

def cfg():
    return TimeDatasetConfig(image_shape=(3, 2, 3), lidar_shape=(2, 4), camera_history_length=2, lidar_history_length=2, ego_history_length=2, command_history_length=2, available_clock="receipt")

def kwargs():
    return dict(config=cfg(), epoch_start_ns=0, epoch_end_ns=5_500_000_000, freeze_ns=2_000_000_001)

def test_full_future_teacher_and_causal_input_are_separate():
    rows, anchor = fixture()
    sample = assemble_time_sample(rows, anchor, **kwargs())
    assert sample.teacher is not None and sample.teacher.xy_mask.all()
    assert sample.inputs is not None and sample.inputs.image.shape == (1, 2, 3, 2, 3)
    assert sample.stop_probability is None

def test_clock_offset_and_delayed_duplicate_are_cut_before_dedup():
    rows, anchor = fixture()
    rows.append(ev("camera", 1_900_000_000, TimedImage(1_900_000_000, np.full((2, 3, 3), 9, dtype=np.uint8)), avail=2_100_000_000, seq=99))
    batch, refs = assemble_time_inputs(rows, anchor, **kwargs())
    assert batch.image_mask[0, -1]
    assert any(r["role"] == "camera" for r in refs)

def test_missing_current_camera_keeps_full_teacher():
    rows, anchor = fixture(camera_late=True)
    sample = assemble_time_sample(rows, anchor, **kwargs())
    assert sample.teacher is not None and sample.teacher.xy_mask.all()
    assert sample.inputs is None and sample.input_invalid_reason == "CURRENT_SENSOR_MISSING"


def test_independent_available_clock_offset_and_future_pose_changes_do_not_change_inputs():
    from dataclasses import replace
    import torch
    rows,anchor=fixture()
    shifted=[replace(e,available_ns=e.available_ns+100_000_000_000) for e in rows]
    shifted_anchor=next(e for e in shifted if e.role=='camera' and e.capture_ns==anchor.capture_ns)
    args={**kwargs(),'freeze_ns':102_000_000_001}
    a=assemble_time_sample(shifted,shifted_anchor,**args)
    changed=[replace(e,payload=replace(e.payload,x_world_m=e.payload.x_world_m+9))
             if e.role=='pose' and e.capture_ns>anchor.capture_ns else e for e in shifted]
    b=assemble_time_sample(changed,shifted_anchor,**args)
    assert a.teacher.xy_mask.all() and b.teacher.xy_mask.all()
    assert not np.array_equal(a.teacher.xy_m,b.teacher.xy_m)
    for name in ('image','lidar','ego','command_history'):
        torch.testing.assert_close(getattr(a.inputs,name),getattr(b.inputs,name),rtol=0,atol=0)
    assert all(s['available_ns']<=args['freeze_ns'] for r in a.provenance for s in r['sources'])


def test_interpolation_requires_both_available_endpoints_and_records_them():
    from dataclasses import replace
    rows,anchor=fixture()
    t=2_000_000_000
    rows=[e for e in rows if not(e.role=='pose' and e.capture_ns==t)]
    for stamp,x in ((t-25_000_000,3.95),(t+25_000_000,4.05)):
        rows.append(ev('pose',stamp,TimedPose(stamp,x,0.,0.,'map','base_link'),avail=stamp))
    early=assemble_time_sample(rows,anchor,**kwargs())
    assert early.teacher is None and early.inputs is not None
    later=assemble_time_sample(rows,anchor,**{**kwargs(),'freeze_ns':t+30_000_000})
    assert later.teacher.xy_mask.all()
    assert len(later.provenance[0]['sources'])==2
    other=[replace(e,epoch='other') if e.role=='pose' and e.capture_ns==t+25_000_000 else e for e in rows]
    assert assemble_time_sample(other,anchor,**{**kwargs(),'freeze_ns':t+30_000_000}).teacher is None


def test_lidar_invalid_beams_are_masked_before_resampling():
    from dataclasses import replace
    rows,anchor=fixture()
    scan=TimedLidar(anchor.capture_ns,np.array([np.nan,np.inf,-1.,1.]),-np.pi,np.pi/2,.1,25.)
    rows=[replace(e,payload=scan) if e.role=='lidar' and e.capture_ns==anchor.capture_ns else e for e in rows]
    batch,_=assemble_time_inputs(rows,anchor,**kwargs())
    np.testing.assert_array_equal(batch.lidar[0,-1,1].numpy(),[0.,0.,0.,1.])
    assert np.isfinite(batch.lidar.numpy()).all()

def test_optional_steering_is_masked_and_speed_is_mandatory():
    rows, anchor = fixture()
    sample = assemble_time_sample(rows, anchor, **kwargs())
    assert sample.inputs is not None and not sample.inputs.ego_feature_mask[0, -1, 3]
    missing = assemble_time_sample([x for x in rows if x.role != "velocity"], anchor, **kwargs())
    assert missing.inputs is None and missing.input_invalid_reason == "CURRENT_LONGITUDINAL_SPEED_MISSING"

def test_stop_evidence_is_explicit_and_unknown_has_no_probability():
    rows, anchor = fixture()
    assert assemble_time_sample(rows, anchor, **kwargs()).stop_reason == "UNKNOWN"
    assert assemble_time_sample(rows, anchor, **kwargs(), intervention_ns=2_500_000_000).stop_reason == "COLLECTION_INTERVENTION"
    assert assemble_time_sample(rows, anchor, **kwargs(), safety_brake=True).stop_reason == "SAFETY_BRAKE"
    assert assemble_time_sample(rows, anchor, **kwargs(), environment_stop_intent=True).stop_reason == "ENVIRONMENT_STOP_INTENT"
    assert assemble_time_sample(rows, anchor, **kwargs(), observed_stationary=True).stop_reason == "OBSERVED_STATIONARY"

def test_dataset_uses_run_epoch_bounds_and_per_anchor_freeze():
    rows, anchor = fixture()
    ds = TimeDataset([(rows, anchor, 2_000_000_001)], config=cfg(), epoch_bounds={("run", "ep"): (0, 5_500_000_000)})
    assert ds[0].teacher is not None
    assert TimeDataset([(rows, anchor, 2_000_000_001)], config=cfg(), epoch_bounds={})[0].input_invalid_reason == "EPOCH_BOUNDS_MISSING"
