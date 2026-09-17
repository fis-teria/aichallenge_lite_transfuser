"""ROS-free contracts for existing PP cars in separate AWSIM vehicle domains."""
from __future__ import annotations

import math
from typing import Any
import xml.etree.ElementTree as ET


def traffic_domains(count: int, npcs: int = 0) -> tuple[int, ...]:
    """Domain 1 is ego; normal background vehicles occupy domains 2..4."""
    if type(count) is not int or not 0 <= count <= 3:
        raise ValueError('PP_VEHICLE_COUNT_0_TO_3')
    if count and npcs:
        raise ValueError('PP_AND_BUILTIN_NPCS_ARE_EXCLUSIVE')
    return tuple(range(1, count + 2))


def background_launch(source_xml: str, speed_cap_mps: float) -> str:
    """Forward a lower reference cap through a task-local copy of system launch.

    No controller, sensor, initialization or race-arm guard is removed.
    """
    if not math.isfinite(speed_cap_mps) or not 0 < speed_cap_mps <= 20 / 3.6:
        raise ValueError('BACKGROUND_SPEED_CAP_MPS')
    root = ET.fromstring(source_xml)
    includes = [e for e in root.findall('include')
                if e.get('file') == '$(find-pkg-share aichallenge_submit_launch)/launch/aichallenge_submit.launch.xml']
    if len(includes) != 1 or any(e.get('name') == 'reference_execution_speed_cap_mps' for e in includes[0]):
        raise ValueError('BACKGROUND_SYSTEM_LAUNCH_CONTRACT')
    ET.SubElement(includes[0], 'arg', name='reference_execution_speed_cap_mps', value=str(speed_cap_mps))
    return ET.tostring(root, encoding='unicode') + '\n'


class DomainLapJudge:
    """Ego-only LapCount.UpdateSimulatorStatus, never shared Unity log lines.

    Float32MultiArray shape (7,): remaining seconds, current lap (0 before
    starting), current lap seconds, section+1, time scale, boosts, boosting.
    A first lap is 0 -> 1 -> 2 with observed ordered section transitions.
    Lap time is an interval from adjacent 10 Hz observations, not an exact time.
    """

    def __init__(self, run_id: str = 'SYNTHETIC') -> None:
        self.run_id = run_id
        self.section_events: list[dict[str, Any]] = []
        self.laps: list[dict[str, Any]] = []
        self.invalid = False
        self.last: dict[str, Any] | None = None
        self.last_monotonic_ns = -1

    def feed(self, record: dict[str, Any]) -> None:
        try:
            self._feed(record)
        except (ValueError, KeyError, TypeError, OverflowError):
            self.invalid = True
            raise ValueError('EGO_STATUS_JUDGE_CONTRACT') from None

    def _feed(self, record: dict[str, Any]) -> None:
        values = record['data']
        stamp = record['monotonic_ns']
        if (record['domain_id'] != 1 or record['publisher_count'] != 1
                or type(stamp) is not int or stamp <= self.last_monotonic_ns
                or not isinstance(values, list) or len(values) != 7
                or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            raise ValueError()
        remaining, lap, elapsed, section, scale, boosts, boosting = values
        if (lap != int(lap) or section != int(section) or min(lap, section, elapsed, scale) < 0
                or boosts < 0 or boosting not in (0, 1)):
            raise ValueError()
        current = dict(lap=int(lap), section=int(section), elapsed=elapsed, remaining=remaining)
        previous = self.last
        if previous is None:
            if lap != 0 or section != 0:
                raise ValueError()
        else:
            # A reset or interleaved publishers must not look like another lap.
            if remaining > previous['remaining'] + .01:
                raise ValueError()
            transition = (int(lap), int(section)) != (previous['lap'], previous['section'])
            if transition:
                if previous['lap'] == 0:
                    valid = lap == 1 and section == 1
                elif lap == previous['lap']:
                    valid = section == previous['section'] + 1
                else:
                    valid = lap == previous['lap'] + 1 and section == 1 and previous['section'] >= 3
                if not valid:
                    raise ValueError()
                self.section_events.append(dict(next=int(section)-1, lap=int(lap), domain_id=1,
                                                monotonic_ns=stamp, run_id=self.run_id))
                if lap >= 2 and lap == previous['lap'] + 1:
                    gap = previous['remaining'] - remaining
                    self.laps.append(dict(laps=int(lap)-1, domain_id=1,
                        lap_seconds_lower_bound=previous['elapsed'],
                        lap_seconds_upper_bound=previous['elapsed']+gap,
                        ordered_section_evidence=True, source='/awsim/status', run_id=self.run_id))
            if lap == previous['lap'] and elapsed < previous['elapsed'] - .01:
                raise ValueError()
        self.last = current
        self.last_monotonic_ns = stamp

    @property
    def completed(self) -> bool:
        return bool(self.laps) and not self.invalid
