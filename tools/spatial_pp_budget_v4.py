"""User-approved four short PP attempts, preserving all earlier charges."""
from __future__ import annotations
import math
from pathlib import Path
import time
from spatial_dev_host_v4 import AttemptBudget, atomic_json

PROFILE = 'V4_LIVE_PP_FOUR_20260907'
LIMITS = dict(wall_s=3600., forward=7100, mpc=6000, snapshots=16,
              powered=11, powered_s=320., log_bytes=536870912)


class PPBudget(AttemptBudget):
    def __init__(self, path: Path):
        super().__init__(path)
        self.limits = dict(LIMITS)

    def renew_map_test_deadline(self) -> dict:
        """One explicit renewal after 10:57 JST; preserve all prior charges."""
        auth=self.authorize();key='V4_MAP_TEST_RENEWAL_20260907_1057'
        if self.value.get('pp_map_renewal'):
            if self.value['pp_map_renewal']['id']!=key: raise ValueError('RENEWAL_CONFLICT')
            return self.value['pp_map_renewal']
        now=time.time()
        record=dict(id=key,old_cutoff_unix_s=auth['driving_cutoff_unix_s'],
            new_cutoff_unix_s=now+1800.,applied_unix_s=now,used_at_change=dict(self.value['used']),
            limits_unchanged=dict(self.limits),authority_source='USER approval after explicit deadline proposal')
        self.value['pp_authorization']=dict(auth,driving_cutoff_unix_s=now+1800.)
        self.value['pp_map_renewal']=record
        self.value.setdefault('authorization_changes',[]).append(record)
        atomic_json(self.path,self.value)
        return record

    def authorize(self) -> dict:
        if self.value.get('active'):
            raise ValueError('UNRESOLVED_PRIOR_RESERVATION')
        for key in (*self.limits, 'tiny_forward'):
            v = self.value['used'].get(key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
                raise ValueError('INVALID_USED:'+key)
        prior = self.value.get('pp_authorization')
        if prior:
            if prior['profile'] != PROFILE or prior['new_limits'] != self.limits:
                raise ValueError('PP_AUTH_CONFLICT')
            return prior
        if self.value.get('tiny_authorized_limits') != dict(self.limits, powered=7):
            raise ValueError('EXPECTED_PRIOR_LIMITS_MISSING')
        now = time.time()
        record = dict(profile=PROFILE, new_limits=self.limits, old_limits=dict(self.limits,powered=7),
            used_at_change=dict(self.value['used']), applied_unix_s=now, driving_cutoff_unix_s=now+1800.,
            user_approval='4 additional powered trials; <=10sim s each including braking; old deadline replaced; proceed',
            authority_source='CURRENT_USER_MESSAGES', applied_retroactively=False)
        self.value['pp_authorization'] = record
        self.value.setdefault('authorization_changes', []).append(record)
        atomic_json(self.path, self.value)
        return record

    def reserve(self, attempt: str, reservation: dict) -> None:
        auth = self.value['pp_authorization']
        if time.time()+reservation['wall_s'] >= auth['driving_cutoff_unix_s']:
            raise ValueError('PP_CUTOFF_RESERVATION')
        runs = [a for a in self.value.get('attempts', []) if
                a.get('pp_profile') == PROFILE and a['reserved']['powered']]
        if reservation['powered'] and len(runs) >= 4:
            raise ValueError('FOUR_PP_RUN_ATTEMPTS_EXHAUSTED')
        if (reservation['forward'] > 240 or reservation['powered_s'] > 10 or
                reservation['mpc'] != 0 or reservation['snapshots'] != 0):
            raise ValueError('PP_ATTEMPT_CAP')
        if self.value['used']['forward']+self.value['used']['tiny_forward']+reservation['forward'] > self.limits['forward']:
            raise ValueError('COMMON_FORWARD_LIMIT_INCLUDING_TINY')
        super().reserve(attempt, reservation)
        self.value['active']['pp_profile'] = PROFILE
        atomic_json(self.path, self.value)
