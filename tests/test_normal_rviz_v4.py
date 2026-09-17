import numpy as np
import pytest
from aic_transfuser_lite.runtime.slam_shadow_geometry_v4 import lidar_display_points
import importlib.util
from pathlib import Path
spec=importlib.util.spec_from_file_location('normal_rviz',Path(__file__).parents[1]/'tools/integrate_normal_rviz_v4.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
integrate,DISPLAY=module.integrate,module.DISPLAY


def test_preserve_normal_rviz_and_refuse_duplicate():
    original='Panels: []\nVisualization Manager:\n  Class: ""\n  Displays:\n    - Name: Existing\n  Global Options:\n    Fixed Frame: map\n'
    result=integrate(original)
    assert result.replace(DISPLAY,'')==original
    with pytest.raises(ValueError):integrate(result)
    with pytest.raises(ValueError):integrate('unknown')


def test_time_path_is_enabled_without_duplicates_or_changes_to_other_displays():
    original='Panels: []\nVisualization Manager:\n  Class: ""\n  Displays:\n    - Class: Existing\n      Enabled: false\n  Global Options:\n    Fixed Frame: map\n'
    configured = module.ensure_time_path(original)
    assert configured.count('/visualization/time_path/raw_path') == 1
    assert module.ensure_time_path(configured) == configured
    disabled = configured.replace('Name: Time model raw prediction\n      Enabled: true', 'Name: Time model raw prediction\n      Enabled: false')
    assert module.ensure_time_path(disabled) == configured
    assert 'Class: Existing\n      Enabled: false' in configured


def test_stock_follow_view_changes_only_current_view_and_is_idempotent():
    current = '  Views:\n    Current:\n      Class: rviz_default_plugins/TopDownOrtho\n      Scale: 13\n      Target Frame: viewer\n      X: 1\n      Y: 2\n'
    saved = '    Saved:\n      - Target Frame: viewer\n        Scale: 13\n'
    result = module.follow_ego_view(current+saved)
    assert result.endswith(saved) and 'Target Frame: base_link' in result and 'Scale: 60' in result
    assert module.follow_ego_view(result) == result
    with pytest.raises(ValueError):
        module.follow_ego_view('unknown')


def test_display_mount_is_rigid_and_keeps_full_raw():
    raw=np.c_[np.arange(46)*.2,np.sin(np.arange(46))]
    record=dict(event='PLAN',source='LONG_V4_20M_EPOCH12_UNCORRECTED',raw_xy_m=raw.tolist(),observation_pose_xyyaw=[0.,0.,0.])
    xyz=np.array(lidar_display_points(record))
    assert xyz.shape==(46,3)
    assert np.allclose(np.diff(xyz[:,:2],axis=0),np.diff(raw,axis=0))
    assert np.allclose(xyz[:,0]+1.1649999618530273,raw[:,0])
    assert np.array_equal(record['raw_xy_m'],raw)


def test_lidar_comparison_is_wide_and_preserves_map_and_raw_scan():
    import yaml
    original = ('Visualization Manager:\n  Class: ""\n  Displays:\n'
                '    - Class: rviz_default_plugins/MarkerArray\n      Name: Existing map\n'
                '  Views:\n    Current:\n      Class: rviz_default_plugins/TopDownOrtho\n'
                '      Scale: 13\n      Target Frame: viewer\n      X: 1\n      Y: 2\n'
                '    Saved: []\n')
    result = module.enable_lidar_map_comparison(original)
    manager = yaml.safe_load(result)['Visualization Manager']
    scans = [d for d in manager['Displays'] if d['Class'].endswith('/LaserScan')]
    assert {d['Topic']['Value'] for d in scans} == {
        '/sensing/lidar/scan', '/time_path/localization/scan'}
    assert len({d['Color'] for d in scans}) == 2
    assert all(d['Topic']['Reliability Policy'] == 'Best Effort' for d in scans)
    assert manager['Displays'][-1]['Name'] == 'Existing map'
    assert manager['Views']['Current']['Scale'] == 10
    assert manager['Views']['Current']['Target Frame'] == 'base_link'
    with pytest.raises(ValueError, match='ALREADY_PRESENT'):
        module.enable_lidar_map_comparison(result)
