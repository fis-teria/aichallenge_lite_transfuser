from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TopicContract:
    role: str
    name: str
    message_type: str
    required_for_conversion: bool = True


DATASET_V2_TOPICS = (
    TopicContract(
        "camera",
        "/sensing/camera/image_raw",
        "sensor_msgs/msg/Image",
    ),
    TopicContract(
        "lidar",
        "/sensing/lidar/scan",
        "sensor_msgs/msg/LaserScan",
    ),
    TopicContract(
        "pose",
        "/localization/kinematic_state",
        "nav_msgs/msg/Odometry",
    ),
    TopicContract(
        "velocity",
        "/vehicle/status/velocity_status",
        "autoware_auto_vehicle_msgs/msg/VelocityReport",
    ),
    TopicContract(
        "actual_steering",
        "/vehicle/status/steering_status",
        "autoware_auto_vehicle_msgs/msg/SteeringReport",
        required_for_conversion=False,
    ),
    TopicContract(
        "gear",
        "/vehicle/status/gear_status",
        "autoware_auto_vehicle_msgs/msg/GearReport",
    ),
    TopicContract(
        "final_command",
        "/control/command/control_cmd",
        "autoware_auto_control_msgs/msg/AckermannControlCommand",
    ),
    TopicContract(
        "nominal_command",
        "/nominal_control_cmd",
        "autoware_auto_control_msgs/msg/AckermannControlCommand",
    ),
)

CLOCK_RECORDING_TOPIC = TopicContract(
    "clock",
    "/clock",
    "rosgraph_msgs/msg/Clock",
    required_for_conversion=False,
)

RECORDING_TOPICS = DATASET_V2_TOPICS + (CLOCK_RECORDING_TOPIC,)

TOPIC_BY_ROLE = {contract.role: contract for contract in DATASET_V2_TOPICS}
TOPIC_BY_NAME = {contract.name: contract for contract in DATASET_V2_TOPICS}

if len(TOPIC_BY_ROLE) != len(DATASET_V2_TOPICS):
    raise AssertionError("Dataset v2 topic roles must be unique")
if len(TOPIC_BY_NAME) != len(DATASET_V2_TOPICS):
    raise AssertionError("Dataset v2 topic names must be unique")
