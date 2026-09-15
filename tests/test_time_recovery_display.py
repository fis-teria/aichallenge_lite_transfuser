from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from run_time_recovery_awsim import collection_display_environment


def test_explicit_xwayland_display_and_auth_are_preserved(tmp_path):
    auth = tmp_path / '.mutter-Xwaylandauth.example'
    auth.write_text('fixture only')
    env = dict(DISPLAY=':0', XAUTHORITY=str(auth))
    assert collection_display_environment(env) == env
    assert collection_display_environment({**env, 'DISPLAY':':0.1'})['DISPLAY'] == ':0.1'


def test_missing_auth_file_never_falls_back_to_another_session(tmp_path):
    with pytest.raises(RuntimeError, match='DISPLAY_AUTH_MISSING'):
        collection_display_environment(dict(DISPLAY=':0', XAUTHORITY=str(tmp_path / 'missing')))


@pytest.mark.parametrize('display', ['remote:0', '', ':0;command', '0'])
def test_nonlocal_or_invalid_display_is_rejected(display, tmp_path):
    auth = tmp_path / 'auth'; auth.write_text('fixture only')
    with pytest.raises(ValueError, match='LOCAL_DISPLAY_REQUIRED'):
        collection_display_environment(dict(DISPLAY=display, XAUTHORITY=str(auth)))
