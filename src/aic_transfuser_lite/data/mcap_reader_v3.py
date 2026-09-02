from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .mcap_converter import message_image_to_rgb
from .mcap_converter_v2 import (
    RunStreams,
    TimedCommand,
    TimedGear,
    TimedImage,
    TimedLidar,
    TimedPose,
    TimedSteering,
    TimedVelocity,
    _deduplicate_sorted,
    _stamp_ns,
    _yaw_from_quaternion,
)
from .topic_profile_v3 import TopicProfileV3


_STREAM_ROLES = (
    "camera",
    "lidar",
    "pose",
    "velocity",
    "actual_steering",
    "gear",
    "nominal_command",
    "final_command",
)


def select_reader_topics_v3(
    profile: TopicProfileV3, observed_topics: Mapping[str, str]
) -> dict[str, str]:
    """Return observed MCAP topic to V3 stream-role mappings.

    Required-for-conversion roles are fail-closed and every observed configured
    topic must have the exact declared ROS message type. Optional command roles
    remain optional here so trajectory-only datasets and full-control datasets
    using either nominal or explicitly tagged final-fallback commands share the
    same canonical reader.
    """

    selected: dict[str, str] = {}
    missing: list[str] = []
    for role in _STREAM_ROLES:
        spec = profile.roles.get(role)
        if spec is None:
            continue
        observed_type = observed_topics.get(spec.name)
        if observed_type is None:
            if spec.required_for_conversion:
                missing.append(spec.name)
            continue
        if observed_type != spec.message_type:
            raise ValueError(
                f"topic type mismatch for {spec.name}: observed={observed_type!r}, "
                f"expected={spec.message_type!r}"
            )
        selected[spec.name] = role
    if missing:
        raise ValueError(f"missing required V3 conversion topics: {sorted(missing)}")
    return selected


def read_run_messages_v3(bag_dir: Path, *, profile: TopicProfileV3) -> RunStreams:
    """Deserialize canonical V3 Camera, LiDAR, ego, gear and command streams.

    Timestamps are nanoseconds, LiDAR ranges are metres, pose yaw and steering
    are radians, velocity is m/s, yaw rate is rad/s, and acceleration is m/s^2.
    The returned image arrays are contiguous RGB uint8 ``[H,W,3]`` values.
    """

    from rosbags.highlevel import AnyReader

    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        raise FileNotFoundError(f"rosbag2 metadata not found: {metadata}")
    buckets: dict[str, list[Any]] = {role: [] for role in _STREAM_ROLES}
    topic_types: dict[str, str] = {}
    fallback_counts: dict[str, int] = {role: 0 for role in _STREAM_ROLES}
    with AnyReader([bag_dir]) as reader:
        available = {connection.topic: connection.msgtype for connection in reader.connections}
        topic_roles = select_reader_topics_v3(profile, available)
        connections = [
            connection for connection in reader.connections if connection.topic in topic_roles
        ]
        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            role = topic_roles[connection.topic]
            topic_types[connection.topic] = connection.msgtype
            message = reader.deserialize(rawdata, connection.msgtype)
            timestamp_ns, timestamp_source = _stamp_ns(message, int(bag_timestamp_ns))
            if timestamp_source == "bag_timestamp_fallback":
                fallback_counts[role] += 1
            if role == "camera":
                item = TimedImage(
                    timestamp_ns,
                    message_image_to_rgb(message),
                    int(bag_timestamp_ns),
                    timestamp_source,
                )
            elif role == "lidar":
                item = TimedLidar(
                    timestamp_ns=timestamp_ns,
                    ranges_m=np.asarray(message.ranges, dtype=np.float32),
                    angle_min_rad=float(message.angle_min),
                    angle_increment_rad=float(message.angle_increment),
                    range_min_m=float(message.range_min),
                    range_max_m=float(message.range_max),
                    frame_id=str(message.header.frame_id),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif role == "pose":
                item = TimedPose(
                    timestamp_ns=timestamp_ns,
                    x_world_m=float(message.pose.pose.position.x),
                    y_world_m=float(message.pose.pose.position.y),
                    yaw_world_rad=_yaw_from_quaternion(message.pose.pose.orientation),
                    frame_id=str(message.header.frame_id),
                    child_frame_id=str(message.child_frame_id),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif role == "velocity":
                item = TimedVelocity(
                    timestamp_ns=timestamp_ns,
                    longitudinal_mps=float(message.longitudinal_velocity),
                    lateral_mps=float(message.lateral_velocity),
                    yaw_rate_rps=float(message.heading_rate),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif role == "actual_steering":
                item = TimedSteering(
                    timestamp_ns=timestamp_ns,
                    steering_rad=float(message.steering_tire_angle),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif role == "gear":
                item = TimedGear(
                    timestamp_ns=timestamp_ns,
                    gear=int(message.report),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            elif role in {"nominal_command", "final_command"}:
                item = TimedCommand(
                    timestamp_ns=timestamp_ns,
                    speed_mps=float(message.longitudinal.speed),
                    acceleration_mps2=float(message.longitudinal.acceleration),
                    steering_rad=float(message.lateral.steering_tire_angle),
                    bag_timestamp_ns=int(bag_timestamp_ns),
                    timestamp_source=timestamp_source,
                )
            else:  # pragma: no cover - selected roles are closed above.
                raise AssertionError(f"unsupported V3 MCAP stream role: {role}")
            buckets[role].append(item)
    return RunStreams(
        images=_deduplicate_sorted(buckets["camera"]),
        lidars=_deduplicate_sorted(buckets["lidar"]),
        poses=_deduplicate_sorted(buckets["pose"]),
        velocities=_deduplicate_sorted(buckets["velocity"]),
        actual_steering=_deduplicate_sorted(buckets["actual_steering"]),
        nominal_commands=_deduplicate_sorted(buckets["nominal_command"]),
        final_commands=_deduplicate_sorted(buckets["final_command"]),
        gears=_deduplicate_sorted(buckets["gear"]),
        topic_types=topic_types,
        timestamp_fallback_counts=fallback_counts,
    )
