"""Refuse NPC experiments with missing, fallback or contradictory spawn evidence."""
import pytest

from aic_transfuser_lite.runtime.awsim_trial_session import npc_startup_evidence, validate_npc_count


SPAWN = 'NpcRuntimeManager: spawned 2 NPC kart(s) named C1..C2 in racing-line mode.\n'
SETTINGS = 'Applied race settings: boosts=2, collisions=True, wallRecovery=True\n'


def test_two_npcs_and_collision_flag_are_verified_from_actual_log():
    result = npc_startup_evidence(SPAWN + SETTINGS, 2)
    assert result['spawned_count'] == 2
    assert result['mode'] == 'racing-line'
    assert result['vehicle_collisions_enabled'] is True


@pytest.mark.parametrize('log', ['', SETTINGS, SPAWN.replace('spawned 2', 'spawned 1') + SETTINGS,
    SPAWN.replace('C2', 'C1') + SETTINGS, SPAWN.replace('racing-line', 'formation-follow') + SETTINGS,
    SPAWN + SPAWN + SETTINGS])
def test_missing_wrong_count_fallback_and_repeated_spawn_are_rejected(log):
    with pytest.raises(ValueError, match='NPC_SPAWN'):
        npc_startup_evidence(log, 2)


@pytest.mark.parametrize('settings', ['', SETTINGS.replace('True', 'False'),
    SETTINGS + SETTINGS.replace('collisions=True', 'collisions=False')])
def test_missing_or_disabled_vehicle_collision_setting_is_rejected(settings):
    with pytest.raises(ValueError, match='COLLISIONS_NOT_ENABLED'):
        npc_startup_evidence(SPAWN + settings, 2)


def test_no_npc_trial_rejects_unexpected_spawn():
    assert npc_startup_evidence(SETTINGS, 0)['spawned_count'] == 0
    with pytest.raises(ValueError, match='UNEXPECTED_NPC'):
        npc_startup_evidence(SPAWN + SETTINGS, 0)


@pytest.mark.parametrize('count', [-1, 4, 2.0, True, '2'])
def test_invalid_npc_count_is_rejected(count):
    with pytest.raises(ValueError, match='NPC_COUNT'):
        validate_npc_count(count)
