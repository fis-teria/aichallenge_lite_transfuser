import pytest

from tools.time_trial_video import select_video_windows


def test_recording_selects_owned_application_windows_and_ignores_other_apps() -> None:
    tree = '''
    0x1 "autoware.rviz - RViz": ("rviz2" "rviz2") 1850x1136+0+0
    0x2 "AWSIM": ("AWSIM" "Unity") 1280x720+10+10
    0x3 "Unrelated desktop app": ("browser" "Browser") 1920x1080+0+0
    0x4 "AWSIM helper": ("AWSIM" "Unity") 1x1+0+0
    '''
    assert select_video_windows(tree, '0x1') == {'awsim': '0x2', 'rviz': '0x1'}
    with pytest.raises(ValueError, match='WINDOWS_NOT_UNIQUE'):
        select_video_windows(tree, '0x9')
    with pytest.raises(ValueError, match='WINDOWS_NOT_UNIQUE'):
        select_video_windows(tree+'\n 0x5 "AWSIM": ("AWSIM" "Unity") 1280x720+0+0', '0x1')


def test_recording_requires_visible_simulator_and_never_falls_back_to_desktop() -> None:
    with pytest.raises(ValueError, match='WINDOWS_NOT_UNIQUE'):
        select_video_windows('0x1 "autoware.rviz - RViz": ("rviz2" "rviz2") 1850x1136+0+0', '0x1')
