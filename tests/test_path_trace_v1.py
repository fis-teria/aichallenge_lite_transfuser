import pytest

from aic_transfuser_lite.runtime.path_trace_v1 import ObservedPathTrace


def test_display_decimation_retains_original_coordinates_and_resets_on_new_epoch():
    trace = ObservedPathTrace(spacing_m=.1, max_points=3)
    for stamp, point in [(10,(1.,2.)), (11,(1.01,2.)), (12,(1.12,2.)), (13,(1.24,2.)), (14,(1.36,2.))]:
        trace.add(stamp, point)
    assert list(trace.points) == [(1.12,2.), (1.24,2.), (1.36,2.)]
    assert trace.last_stamp_ns == 14
    trace.add(14, (9.,9.))
    assert trace.points[-1] == (1.36,2.)
    trace.add(0, (3.,4.))
    assert list(trace.points) == [(3.,4.)]


def test_display_pose_and_configuration_contracts():
    trace = ObservedPathTrace()
    for stamp, point in [(-1,(0.,0.)), (0,(0.,)), (0,(0.,float('nan'))), (.5,(0.,0.))]:
        with pytest.raises(ValueError, match='DISPLAY_POSE_CONTRACT'):
            trace.add(stamp, point)
    for options in ({'spacing_m':0.}, {'spacing_m':float('nan')}, {'max_points':1}):
        with pytest.raises(ValueError, match='DISPLAY_TRACE_CONFIG'):
            ObservedPathTrace(**options)
