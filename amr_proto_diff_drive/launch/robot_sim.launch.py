# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # Robot Description declaration path and parse
    # This will parse the xacro file and generate the URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('amr_proto_diff_drive'),
                 'description', 'robot_main.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    #LAUNCH RSP
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Launch gazebo environment
    world_path = os.path.join(
        get_package_share_directory('amr_proto_diff_drive'),
        'worlds',
        'empty_world.world'
        )
    gz_args = f'-r -v 4 {world_path}'

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Spawn robot in gz 
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'
                   '-x', '12.0', '-y', '12.0', '-z', '0.0',],
    )
    # Robot Controllers declaration path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('amr_proto_diff_drive'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )


    # Spawn joint state broadcaster controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    # Spawn diff drive controller
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
		            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
		            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
	                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
	                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
	                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen'
    )
    joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource ([os.path.join(get_package_share_directory('amr_proto_diff_drive'),'launch','teleop_joy.launch.py')
                                        ]), launch_arguments={'use_sim_time': 'true'}.items()
            )   


    return LaunchDescription([
        # SetEnvironmentVariable(
        # name='IGN_GAZEBO_RESOURCE_PATH',
        # value=os.path.join(
        #     get_package_share_directory('amr_proto_diff_drive'),
        #     'meshes'  
        #     )
        # ),
        gz,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        # rviz_node,
        node_robot_state_publisher,
        bridge,
        joystick,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])