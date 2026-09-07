"""Receive external controller requests, never shadow controls or vehicle reports.

ROS-free Ackermann duck-type decoder. Binding evidence is supplied by the caller;
this module does not discover/attest a live graph or start a controller.
"""
from dataclasses import dataclass
import math

from .spatial_input_v4 import PassiveCommand, Stamp


@dataclass(frozen=True)
class ControllerCommandBinding:
    topic: str
    producer_id: str  # Selected external publisher identity, not a topic alias.
    source: str  # nominal before actuation OR verified final_fallback
    contract_evidence: str
    semantics: str  # explicit tire-angle rad / target speed m/s / accel m/s^2

    def validate(self) -> None:
        if (not self.topic.startswith('/') or self.topic.startswith('/shadow/') or
                not self.producer_id or not self.contract_evidence or
                self.source not in ('nominal', 'final_fallback') or
                self.semantics != 'TIRE_RAD_TARGET_MPS_ACCEL_MPS2'):
            raise ValueError('EXTERNAL_COMMAND_BINDING_UNVERIFIED')

    def decode(self, message: object, stamp: Stamp, *, producer_id: str,
               external_controller: bool) -> PassiveCommand:
        """stamp.header_ns is message.stamp; receive/available clocks stay separate.

        Availability, 50 ms history support and strict pastness are checked by
        SpatialInputV4 at input finalization, NOT by restamping on receipt.
        external_controller must come from source admission, not message data.
        """
        self.validate()
        if producer_id != self.producer_id or not external_controller:
            raise ValueError('EXTERNAL_COMMAND_PRODUCER_MISMATCH')
        try:
            header_ns = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
            values = (float(message.lateral.steering_tire_angle),
                      float(message.longitudinal.speed),
                      float(message.longitudinal.acceleration))
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError('ACKERMANN_COMMAND_FIELDS_REQUIRED') from exc
        if (not 0 <= message.stamp.nanosec < 1_000_000_000 or
                header_ns != stamp.header_ns or
                stamp.received_ns > stamp.available_ns):
            raise ValueError('COMMAND_TIMESTAMP_MISMATCH')
        if not all(math.isfinite(v) for v in values):
            raise ValueError('COMMAND_NONFINITE')
        return PassiveCommand(stamp, *values, source=self.source)
