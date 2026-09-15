"""Display-only records of published steering disturbances in the map frame.

Positions [m] are the measured pose at command publication, not the planned
site or proof of actuator response. Positive steering is left; angles are rad.
This module has no ROS dependencies and never supplies control/model inputs.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

from aic_transfuser_lite.data.time_random_steering_pulse_v1 import PulseSite, SCHEMA

MARKER_TOPIC = '/recovery_teacher/disturbance_markers'
RVIZ_MARKER_DISPLAY = f'''    - Class: rviz_default_plugins/MarkerArray
      Name: Recovery disturbance locations
      Enabled: true
      Namespaces:
        recovery_disturbance: true
      Topic:
        Value: {MARKER_TOPIC}
        Reliability Policy: Reliable
        Durability Policy: Transient Local
        History Policy: Keep Last
        Depth: 1
'''


@dataclass(frozen=True)
class DisturbanceLocation:
    event_id: int
    site_id: str
    sign: int
    planned_s_m: float
    actual_s_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    pose_stamp_ns: int
    publication_sim_ns: int
    publication_sequence: int
    effective_rad: float

    @property
    def label(self) -> str:
        return self.site_id + (' LEFT' if self.sign == 1 else ' RIGHT')


class DisturbanceLocations:
    """Retain the first nonzero published disturbance per event, at most three."""

    def __init__(self) -> None:
        self.events: dict[int, DisturbanceLocation] = {}

    def add(self, row: Mapping[str, Any]) -> bool:
        """Consume a CONTROL_AND_PHASE row; return whether a location was added."""
        if row.get('annotation_schema') != SCHEMA or row.get('phase') != 'hold':
            return False
        pulse = row.get('pulse') or {}
        publication = row.get('publication')
        if pulse.get('applied') is not True or not publication:
            return False
        random = row.get('random_pulse') or {}
        sites = random.get('config', {}).get('sites', [])
        if not sites:  # Earlier unnamed collections are not relabelled as sites.
            return False
        state = random['state']
        event_id, index = state['event_id'], state['active_site_index']
        if (type(event_id) is not int or not 1 <= event_id <= 3
                or type(index) is not int or not 0 <= index < len(sites)):
            raise ValueError('DISTURBANCE_EVENT_OR_SITE_INDEX')
        site = PulseSite(**sites[index])
        requested, effective = pulse['requested_rad'], pulse['effective_rad']
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (requested, effective)):
            raise ValueError('DISTURBANCE_ANGLE_FINITE')
        if requested * site.sign <= 1e-6 or effective * site.sign <= 1e-6:
            return False
        if event_id in self.events:
            if self.events[event_id].site_id != site.site_id or self.events[event_id].sign != site.sign:
                raise ValueError('DISTURBANCE_EVENT_ID_REUSED')
            return False
        pose = row['current_pose']
        values = [pose['x_m'], pose['y_m'], pose['yaw_rad'], row['projection']['s_m']]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError('DISTURBANCE_MAP_POSE_FINITE')
        stamps = [pose['stamp_ns'], publication['sim_ns'], publication['sequence']]
        if not all(type(v) is int and v > 0 for v in stamps):
            raise ValueError('DISTURBANCE_PUBLICATION_STAMPS')
        self.events[event_id] = DisturbanceLocation(
            event_id, site.site_id, site.sign, float(site.start_s_m), float(values[3]),
            *map(float, values[:3]), *stamps, float(effective))
        return True

    def report(self) -> dict[str, Any]:
        return dict(frame_id='map', location_basis='pose_at_first_nonzero_command_publication',
                    topic=MARKER_TOPIC, events=[dict(asdict(event), label=event.label)
                    for event in self.events.values()])
