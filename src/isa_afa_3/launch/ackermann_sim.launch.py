# Copyright 2024 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("isa_afa_3")
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")
    world_file = "streets_with_walls_world.sdf"

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        # performed_description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('isa_afa_3'),
                    'urdf',
                    f'ackermann_vehicle.xacro.urdf'
                ]),
            ]
        )
        robot_description = {'robot_description': robot_description_content}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        )
        return [node_robot_state_publisher]

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('isa_afa_3'),
            'config',
            'ackermann_drive_controller.yaml',
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ackermann', '-allow_renaming', 'true', '-z', '3.5'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /ackermann_steering_controller/tf_odometry:=/tf',
                   ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    ld = LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            "rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        ),
        bridge,
        robot_localization,
        rviz,
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 1 {world_file}'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        gz_spawn_entity,
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
