"""Exercise the built reference node with race inputs, without starting AWSIM."""
import csv
import json
from pathlib import Path
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from autoware_auto_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray


class RaceReferenceRosTest(unittest.TestCase):
    def test_lap_priority_and_bidirectional_leader_switch(self):
        self.exercise_race_reference(simulation=True)

    def test_real_mode_has_no_awsim_subscription_and_counts_geometric_laps(self):
        self.exercise_race_reference(simulation=False)

    def exercise_race_reference(self, simulation):
        directory = Path(get_package_share_directory("multi_purpose_mpc_ros")) / "env/final_ver3"
        files = [directory / name for name in (
            "in_corce_line.csv", "mppi_cma_normal35_20260913.csv", "mppi_cma_leader75_20260913.csv")]
        rows = [list(csv.DictReader(path.read_text().splitlines())) for path in files]
        expected_first = [(float(data[0]["x_m"]), float(data[0]["y_m"])) for data in rows]
        node = rclpy.create_node("race_reference_test_probe")
        status_pub = node.create_publisher(Float32MultiArray, "/awsim/status", 10)
        odom_pub = node.create_publisher(Odometry, "input/kinematics", qos_profile_sensor_data)
        vehicles_pub = node.create_publisher(V2XVehiclePositionArray, "input/vehicle_positions", 10)
        received = {"status": None, "trajectory": None}
        node.create_subscription(String, "reference/status",
                                 lambda msg: received.update(status=json.loads(msg.data)), 10)
        node.create_subscription(Trajectory, "trajectory",
                                 lambda msg: received.update(trajectory=msg), qos_profile_sensor_data)
        executable = Path(get_package_prefix("simple_trajectory_generator")) / (
            "lib/simple_trajectory_generator/simple_trajectory_generator_node")
        command = [str(executable), "--ros-args", "-p", "race_reference.enabled:=true",
                   "-p", f"simulation:={str(simulation).lower()}",
                   "-r", "input/race_status:=/awsim/status",
                   "-p", "race_reference.own_vehicle_id:=d1",
                   "-p", f"dual_reference.lap1_csv_path:={files[0]}",
                   "-p", f"dual_reference.lap2plus_csv_path:={files[1]}",
                   "-p", f"race_reference.leader_csv_path:={files[2]}",
                   "-p", "execution_profile.max_speed_mps:=9.722222222222221"]
        process = subprocess.Popen(command)

        def publish(lap, ego_index, other_index):
            stamp = node.get_clock().now().to_msg()
            ego = rows[0][ego_index]
            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.position.x = float(ego["x_m"])
            odom.pose.pose.position.y = float(ego["y_m"])
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            status_pub.publish(Float32MultiArray(data=[240.0, float(lap), 0.0, 0.0, 1.0, 0.0, 0.0]))
            if other_index is not None:
                other = rows[0][other_index]
                vehicle = V2XVehiclePosition()
                vehicle.vehicle_id = "d2"
                vehicle.header.stamp = stamp
                vehicle.position.x = float(other["x_m"])
                vehicle.position.y = float(other["y_m"])
                vehicles_pub.publish(V2XVehiclePositionArray(vehicles=[vehicle]))

        names = ["IN_CORCE_LAP1", "CMA_NORMAL_LAP2PLUS", "CMA_LEADER_LAP2PLUS"]

        def expect(lap, ego, other, selected, rank, sent_lap=None):
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                self.assertIsNone(process.poll(), "reference process stopped")
                publish(lap if sent_lap is None else sent_lap, ego, other)
                rclpy.spin_once(node, timeout_sec=0.02)
                status = received["status"]
                trajectory = received["trajectory"]
                if not status or not trajectory or not trajectory.points:
                    continue
                first = trajectory.points[0].pose.position
                x, y = expected_first[selected]
                if (status["active_reference_id"] == names[selected] and
                    status["current_lap"] == lap and status["estimated_rank"] == rank and
                    abs(first.x - x) < 1e-6 and abs(first.y - y) < 1e-6):
                    self.assertGreaterEqual(len(trajectory.points), 350)
                    self.assertEqual(status["lap_source"],
                                     "awsim_status" if simulation else "odometry_progress")
                    self.assertGreater(trajectory.points[0].longitudinal_velocity_mps, 0.0)
                    self.assertLessEqual(trajectory.points[0].longitudinal_velocity_mps,
                                         35.0 / 3.6 + 1e-5)
                    return status["generation"]
            self.fail(f"No matching route: expected lap={lap}, rank={rank}, "
                      f"reference={names[selected]}, received={received['status']}")

        try:
            expect(1, 10, 5, 0, 1)
            subscriptions = dict(node.get_subscriber_names_and_types_by_node(
                "csv_to_trajectory_node", "/"))
            self.assertEqual("/awsim/status" in subscriptions, simulation)
            if not simulation:
                # An AWSIM lap message must have no effect in real mode.
                expect(1, 10, 15, 0, 2, sent_lap=3)
                for index in list(range(11, len(rows[0]))) + [0]:
                    publish(3, index, (index + 5) % len(rows[0]))
                    deadline = time.monotonic() + 0.025
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.005)
                expect(2, 0, 5, 1, 2, sent_lap=3)
                expect(2, 10, 5, 2, 1, sent_lap=3)
                return
            expect(1, 10, 15, 0, 2)
            expect(2, 10, 15, 1, 2)
            generation = expect(2, 20, 15, 2, 1)
            self.assertEqual(expect(2, 20, 15, 2, 1), generation)
            expect(2, 20, 25, 1, 2)
            expect(3, 30, 25, 2, 1)
            expect(3, 30, None, 1, None)
            expect(1, 10, 5, 0, 1)
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            node.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    try:
        unittest.main(verbosity=2)
    finally:
        rclpy.shutdown()
