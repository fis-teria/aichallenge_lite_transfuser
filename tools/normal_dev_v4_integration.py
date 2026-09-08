"""Minimal normal-make-dev hooks, preserving all unrelated racingkart bytes.

Pure text transformation for a reviewed patch; no deployment or execution.
No CONTROL_METHOD, Start, simulator, Safety or authority changes.
"""
from __future__ import annotations


def integrate(runtime: str, launch: str, compose: str) -> tuple[str, str, str]:
    if any('V4_SHADOW_ENABLED' in s for s in (runtime, launch, compose)):
        raise ValueError('ALREADY_INTEGRATED_OR_PARTIAL')
    if runtime.count('export ROS_DOMAIN_ID=$id') != 1 or launch.count('</launch>') != 1:
        raise ValueError('UNKNOWN_LAUNCH_LAYOUT')
    if compose.count('    - CONTROL_METHOD=${CONTROL_METHOD:-}') != 1:
        raise ValueError('UNKNOWN_COMPOSE_LAYOUT')
    # Source only when explicitly enabled, before the existing ros2 launch.
    # No new background process, separate runner or Start receipt producer.
    hook = '''# Optional V4 shadow package overlay; existing controller is unchanged.
case "${V4_SHADOW_ENABLED:-false}" in
true)
    [[ "${mode}" == "awsim" || "${mode}" == "awsim-no-viz" ]] || exit 2
    [[ "${V4_SHADOW_SETUP:-}" = /* && -f "${V4_SHADOW_SETUP}" ]] || exit 2
    [[ "${V4_SHADOW_CONFIG:-}" = /* && -f "${V4_SHADOW_CONFIG}" ]] || exit 2
    [[ "${V4_SHADOW_LAUNCH:-}" = /* && -f "${V4_SHADOW_LAUNCH}" ]] || exit 2
    source "${V4_SHADOW_SETUP}" || exit 2
    ;;
false) ;;
*) exit 2 ;;
esac

'''
    runtime = runtime.replace('export ROS_DOMAIN_ID=$id',hook+'export ROS_DOMAIN_ID=$id')
    launch = launch.replace('</launch>', '''  <!-- Shadow only; no control-method or Start authority replacement. -->
  <group if="$(env V4_SHADOW_ENABLED false)">
    <include file="$(env V4_SHADOW_LAUNCH '')">
      <arg name="config_file" value="$(env V4_SHADOW_CONFIG '')"/>
    </include>
  </group>
</launch>''')
    anchor='    - CONTROL_METHOD=${CONTROL_METHOD:-}'
    compose=compose.replace(anchor,anchor+'''
    - V4_SHADOW_ENABLED=${V4_SHADOW_ENABLED:-false}
    - V4_SHADOW_SETUP=${V4_SHADOW_SETUP:-}
    - V4_SHADOW_LAUNCH=${V4_SHADOW_LAUNCH:-}
    - V4_SHADOW_CONFIG=${V4_SHADOW_CONFIG:-}''')
    return runtime, launch, compose
