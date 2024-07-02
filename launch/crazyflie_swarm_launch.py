# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Launch Crazyflie Swarm platform node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def find_elem_in_dict(_dict, elem):
    """Find element in dictionary."""
    if elem in _dict:
        return _dict[elem]
    for _, value in _dict.items():
        if isinstance(value, dict):
            item = find_elem_in_dict(value, elem)
            if item is not None:
                return item
    return None


def get_drones_with_cam(swarm_config_path) -> list:
    """Get drones with camera."""
    swarm_config = {}
    drones_with_cam = []
    with open(swarm_config_path, 'r', encoding='utf8') as swarm_config_file:
        swarm_config = yaml.safe_load(swarm_config_file)
    for ns, value in swarm_config.items():
        if ns.startswith('/'):
            ns = ns[1:]
        if ns.find('**') != -1:
            continue
        # find cam key in the dict recursively
        cam = find_elem_in_dict(value, 'cam')
        if cam is not None and isinstance(cam, dict) and 'ip' in cam:
            drones_with_cam.append((ns, cam))
            # print(f'{ns} : Found cam: {cam}')

    print(f'Found {len(drones_with_cam)} drones with camera')
    return drones_with_cam


def get_camera_launch_description(context, *args, **kwargs):
    """Get the camera launch description."""
    swarm_config_file = context.launch_configurations['swarm_config_file']
    drones_with_cam = get_drones_with_cam(swarm_config_file)
    launch_description = []
    for ns, _ in drones_with_cam:
        launch_description.append(
            Node(
                package='as2_platform_crazyflie',
                executable='aideck_node.py',
                name='aideck_pub',
                namespace=ns,
                output='screen',
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('swarm_config_file')
                ],
            )
        )
    return launch_description


def generate_launch_description():
    """Entrypoint."""
    control_modes = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'control_modes.yaml'
    ])

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'platform_config_file.yaml'
    ])

    swarm_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_crazyflie'),
        'config', 'swarm_config_file.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        DeclareLaunchArgument('platform_config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),
        DeclareLaunchArgument('swarm_config_file',
                              default_value=swarm_config_file,
                              description='Platform swarm URI configuration file'),

        Node(
            package='as2_platform_crazyflie',
            executable='as2_platform_crazyflie_swarm_node',
            name='platform',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'control_modes_file': LaunchConfiguration('control_modes_file'),
                    'swarm_config_file': LaunchConfiguration('swarm_config_file')
                },
                LaunchConfiguration('platform_config_file'),
                LaunchConfiguration('swarm_config_file')
            ],
        ),
        OpaqueFunction(function=get_camera_launch_description)
    ])
