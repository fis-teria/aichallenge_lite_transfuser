"""Read-only ROS graph verification during this candidate run."""
from pathlib import Path
import json
import shlex
import subprocess, sys

kind = sys.argv[1]
assert kind in ('cone','box')
root = Path('/home/graneple/e2e_autonomous/time_native_retained_awsim_20260919')
code = '''import rclpy,json,time
rclpy.init()
n=rclpy.create_node("native_replay_graph_audit",enable_rosout=False)
until=time.monotonic()+3
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
print(json.dumps(dict(subscriptions=n.get_subscriber_names_and_types_by_node("time_path_controller","/"),
 command_publishers=[dict(name=e.node_name,namespace=e.node_namespace) for e in n.get_publishers_info_by_topic("/control/command/control_cmd")],
 path_subscribers=[e.node_name for e in n.get_subscriptions_info_by_topic("/visualization/time_path/raw_path")],
 nodes=n.get_node_names_and_namespaces())))
n.destroy_node();rclpy.shutdown()
'''
cmd = ['docker', 'exec', '-e', 'ROS_DOMAIN_ID=1', 'codex-time-retained-' + kind + '-01-nodes', 'bash', '-lc',
       'source /aichallenge/workspace/install/setup.bash && python3 -c ' + shlex.quote(code)]
p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=15)
result = dict(exit=p.returncode, stdout=p.stdout, stderr=p.stderr)
if p.returncode == 0:
    graph = json.loads(p.stdout.strip().splitlines()[-1])
    expected = {'/clock', '/sensing/lidar/scan', '/time_path/plan',
                '/vehicle/status/steering_status', '/vehicle/status/velocity_status'}
    result.update(graph=graph, subscriptions_match={t for t, _ in graph['subscriptions']} == expected,
        exclusive_command_publisher=graph['command_publishers'] == [dict(name='time_path_controller', namespace='/')],
        normal_rviz_path_subscribed='rviz2' in graph['path_subscribers'])
with (root / (kind + '_live_graph.json')).open('x') as f:
    json.dump(result, f, indent=2)
print(json.dumps({k: result.get(k) for k in ('exit', 'subscriptions_match', 'exclusive_command_publisher', 'normal_rviz_path_subscribed')}))
