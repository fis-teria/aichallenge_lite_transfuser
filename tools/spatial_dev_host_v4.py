"""Task-only host watchdog/cleanup and cumulative reservation, no ROS imports."""
from __future__ import annotations
import json
from pathlib import Path
import time


def atomic_json(path: Path, value: dict) -> None:
    pending = path.with_suffix('.pending')
    pending.write_text(json.dumps(value, indent=2), encoding='utf-8')
    pending.replace(path)


class HostWatch:
    """Armed before commands. Old powered=false does not disable supervision."""
    def __init__(self, token: str):
        self.token = token
        self.armed = False

    def check(self, heartbeat: dict | None, now_ns: int) -> str | None:
        if heartbeat is None:
            return 'HEARTBEAT_MISSING' if self.armed else None
        if heartbeat.get('token') != self.token:
            return 'HEARTBEAT_IDENTITY'
        if not 0 <= now_ns-heartbeat['monotonic_ns'] <= 750_000_000:
            return 'HEARTBEAT_STALE' if self.armed else None
        if heartbeat.get('logger_ok') is not True:
            return 'LOGGER_FAILED'
        self.armed = True
        return None


def cleanup_owned(run, sim_id: str | None, runtime_id: str | None, *, paused_kill_verified: bool) -> dict:
    """Every resource attempted independently; NEVER thaw a paused simulator.

    Docker KILL on a frozen task helper was verified on the selected host.
    If unsupported here, leave the owned sim frozen; never resume to clean up.
    """
    errors = []
    outcomes = {}
    def attempt(label, argv):
        try:
            result = run(argv, timeout=15)
            outcomes[label] = result.stdout
            return result
        except BaseException as exc:
            errors.append(dict(stage=label, error=type(exc).__name__+': '+str(exc)))
            return None
    if sim_id:
        state = attempt('sim_inspect', ['docker','inspect',sim_id,'--format','{{json .State}}'])
        try: parsed = json.loads(state.stdout) if state is not None else None
        except (ValueError,TypeError) as exc:
            errors.append(dict(stage='sim_inspect_parse',error=str(exc))); parsed=None
        if parsed is None or parsed['Running']:
            # Freeze first even on normal cleanup. Natural stop is separately
            # established by observed velocity, never by this process action.
            if parsed is None or not parsed['Paused']:
                attempt('sim_pause', ['docker','pause',sim_id])
            if paused_kill_verified:
                attempt('sim_kill_frozen', ['docker','kill','--signal','KILL',sim_id])
            else:
                errors.append(dict(stage='sim_kill_frozen', error='UNVERIFIED_KEEP_PAUSED'))
    if runtime_id:
        attempt('runtime_stop', ['docker','stop','-t','8',runtime_id])
        attempt('runtime_logs', ['docker','logs','--tail','300',runtime_id])
    for name, cid in [('sim_final',sim_id),('runtime_final',runtime_id)]:
        if cid: attempt(name,['docker','inspect',cid,'--format','{{json .State}}'])
    return dict(errors=errors, outcomes=outcomes, unpause_called=False)


class AttemptBudget:
    """Small task JSON with one outstanding reservation; no reset on retry."""
    limits = dict(wall_s=3600., forward=3000, mpc=6000, snapshots=16, powered=3, powered_s=180.,log_bytes=512*1024**2)
    def __init__(self, path: Path):
        import fcntl
        self.path = path
        self.lock = path.with_suffix('.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.value = json.loads(path.read_text())

    def reserve(self, attempt: str, reservation: dict) -> None:
        if self.value.get('active'):
            raise ValueError('UNRESOLVED_PRIOR_RESERVATION')
        for key, limit in self.limits.items():
            if self.value['used'].get(key) is None or self.value['used'][key]+reservation[key] > limit:
                raise ValueError('BUDGET_UNKNOWN_OR_EXCEEDED:'+key)
        self.value['active'] = dict(id=attempt, reserved=reservation, started_wall_ns=time.time_ns())
        atomic_json(self.path,self.value)

    def finish(self, consumption: dict, *, exact: bool) -> None:
        active = self.value['active']
        # Missing final counters consume their upper reservation, not invented 0.
        charged = {k:consumption.get(k,active['reserved'][k]) for k in self.limits}
        for key,value in charged.items(): self.value['used'][key] += value
        self.value.setdefault('attempts',[]).append(dict(**active, charged=charged, exact=exact))
        self.value['active'] = None
        atomic_json(self.path,self.value)

    def close(self) -> None:
        self.lock.close()
