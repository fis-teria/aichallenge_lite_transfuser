"""Synthetic ROS display-only smoke in a network-isolated official image."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from aic_transfuser_lite.runtime.recovery_disturbance_markers import MARKER_TOPIC

root = Path(tempfile.mkdtemp(prefix='marker-smoke-'))
ref = root/'reference.json'
points = [[float(i), 0.] for i in range(20)]
ref.write_text(json.dumps(dict(baseline_xy_m=points, reference_xy_m=points)))
rclpy.init(); node = rclpy.create_node('marker_synthetic_smoke')
odom_pub = node.create_publisher(Odometry, '/localization/kinematic_state', 10)
phase_pub = node.create_publisher(String, '/recovery_teacher/phase', 10)
odom = Odometry(); odom.header.frame_id = 'map'; odom.child_frame_id = 'base_link'
odom.header.stamp.sec = 1; odom.pose.pose.orientation.w = 1.
odom.pose.pose.position.x = 4.; odom.pose.pose.position.y = 5.
sample = dict(annotation_schema='measured_random_steering_pulse_v1', phase='hold',
    publication=dict(sim_ns=1_100_000_000, sequence=20),
    pulse=dict(applied=True, requested_rad=.1, effective_rad=.02),
    random_pulse=dict(config=dict(sites=[dict(site_id='S00', start_s_m=116., sign=1)]),
                      state=dict(event_id=1, active_site_index=0)),
    current_pose=dict(x_m=4., y_m=5., yaw_rad=.3, stamp_ns=1_000_000_000),
    projection=dict(s_m=116.2))
log = (root/'paths.log').open('w')
child = subprocess.Popen([sys.executable, '/capture/source/tools/time_recovery_paths_node.py',
    '--reference', str(ref), '--output', str(root)], stdout=log, stderr=subprocess.STDOUT)
received = []
try:
    deadline = time.monotonic()+25
    while time.monotonic()<deadline:
        assert child.poll() is None, (root/'paths.log').read_text()
        odom_pub.publish(odom); phase_pub.publish(String(data=json.dumps(sample)))
        rclpy.spin_once(node, timeout_sec=.1)
        heartbeat = root/'path_heartbeat.json'
        if heartbeat.exists() and json.loads(heartbeat.read_text())['disturbance_marker_count'] == 3:
            break
    else:
        raise RuntimeError('MARKER_SMOKE_PUBLISH_TIMEOUT')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MarkerArray, MARKER_TOPIC, lambda m: received.append(m), qos)
    deadline = time.monotonic()+10
    while not received and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    assert received and len(received[-1].markers) == 3
    markers = received[-1].markers
    assert {m.type for m in markers} == {Marker.CYLINDER, Marker.TEXT_VIEW_FACING, Marker.ARROW}
    assert all(m.header.frame_id == 'map' and m.pose.position.x == 4. and m.pose.position.y == 5. for m in markers)
    assert [m.text for m in markers if m.type == Marker.TEXT_VIEW_FACING] == ['S00 LEFT']
    assert all(m.lifetime.sec == 0 and m.color.a == 1. for m in markers)
    assert len(json.loads((root/'disturbance_markers.json').read_text())['events']) == 1
    info = node.get_publishers_info_by_topic(MARKER_TOPIC)
    assert len(info) == 1 and info[0].qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    print(json.dumps(dict(scope='SYNTHETIC_ROS_DISPLAY_ONLY_NO_VEHICLE_MOTION', passed=True,
        actual_map_position=True, late_subscriber_received=True, marker_count=3, persistent_lifetime=True)), flush=True)
finally:
    child.send_signal(signal.SIGINT); child.wait(timeout=10); log.close()
    if child.returncode != 0:
        print((root/'paths.log').read_text(), file=sys.stderr)
    assert child.returncode == 0
    node.destroy_node(); rclpy.shutdown()
