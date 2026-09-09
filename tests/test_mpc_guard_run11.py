"""Run11 static contract checks, no ROS or driving."""
import ast
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]/'integrations/mpc_guard_run11'


def test_run11_scope_and_syntax():
    for path in ROOT.glob('*.py'):
        ast.parse(path.read_text())
    run=(ROOT/'run.py').read_text()
    assert "project='codex-mpc-guard-lap-11'" in run
    assert "mpc_speed20_lap_10/budget.json" in run
    assert "V4_SHADOW_ENABLED='false'" in run
    assert 'wall_s=120.,forward=0,mpc=60000' in run
    assert 'powered_s=90.' in run
    guard=(ROOT/'guard.py').read_text()
    assert "limits['gain'],final=False" in guard
    assert "limits['gain'],final=True" in guard
    assert 'create_publisher' not in guard


def test_config_matches_unvalidated_corner_candidate():
    config=yaml.safe_load((ROOT/'config.yaml').read_text())
    baseline=yaml.safe_load((ROOT.parent/'mpc_speed20/config.corner_candidate.yaml').read_text())
    assert config==baseline
