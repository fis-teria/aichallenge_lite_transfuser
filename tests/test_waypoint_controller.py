import numpy as np

from aic_transfuser_lite.control.waypoint_controller import control_from_waypoints


def test_left_waypoint_produces_positive_steering() -> None:
    waypoints = np.array([[1.0, 0.1], [2.0, 0.2], [3.0, 0.3]], dtype=np.float32)
    command = control_from_waypoints(waypoints, target_speed_mps=3.0, current_speed_mps=2.0)
    assert command.steering_rad > 0.0
    assert command.acceleration_mps2 > 0.0


def test_speed_error_can_brake() -> None:
    waypoints = np.array([[1.0, 0.0], [2.0, 0.0]], dtype=np.float32)
    command = control_from_waypoints(waypoints, target_speed_mps=1.0, current_speed_mps=3.0)
    assert command.acceleration_mps2 < 0.0
