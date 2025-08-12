#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Universal Robot simulation nodes."""

import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController

PACKAGE_NAME = 'ec63_webots'


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    robot_description_path = os.path.join(package_dir, 'resource', 'ec63.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control_config.yaml')

    with open(robot_description_path, 'r') as urdf_file:
        robot_description = ""
        for line in urdf_file:
            robot_description += line

    # Define your URDF robots here
    # The name of an URDF robot has to match the name of the robot of the driver node
    # You can specify the URDF file to use with "urdf_path"
    # spawn_URDF_ur5e = URDFSpawner(
    #     name='UR5e',
    #     urdf_path=robot_description_path,
    #     translation='0 0 0.6',
    #     rotation='0 0 1 -1.5708',
    # )

    # Driver nodes
    # When having multiple robot it is mandatory to specify the robot name.
    universal_robot_driver = WebotsController(
        robot_name='Ec63',
        namespace='ec63',
        respawn=True,
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True},
            {'set_robot_state_publisher': False},
            ros2_control_params
        ],
    )

    # Other ROS 2 nodes
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ec_joint_trajectory_controller', '-c', 'ec63/controller_manager'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ec_joint_state_broadcaster', '-c', 'ec63/controller_manager'] + controller_manager_timeout,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        namespace='ec63',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }],
    )

    return LaunchDescription([
        # Request to spawn the URDF robot
        # spawn_URDF_ur5e,

        # Other ROS 2 nodes
        universal_robot_driver,
        robot_state_publisher,
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,

        # Launch the driver node once the URDF robot is spawned
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessIO(
        #         target_action=robot_state_publisher,
        #         on_stdout=lambda event: get_webots_driver_node(event, universal_robot_driver),
        #     )
        # ),

        # Kill all the nodes when the driver node is shut down
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=universal_robot_driver,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # ),
    ])
