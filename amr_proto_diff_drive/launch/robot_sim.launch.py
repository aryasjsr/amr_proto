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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
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

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('amr_proto_diff_drive'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('amr_proto_diff_drive'),'launch','teleop_joy.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    

    
    gazebo_params_file = os.path.join(get_package_share_directory('amr_proto_diff_drive'),'config','ros2_control_sim.yaml')


    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
            )

 

    gz_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description','-entity', 'my_robot',],
                    output='screen')

    # Robot Controllers declaration path
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('amr_proto_diff_drive'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    # # Spawn joint state broadcaster
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    #     output='screen',
    # )
    # # Spawn diff drive controller
    # diff_drive_base_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['diff_drive_base_controller'],
    #     output='screen',
    # )
    joint_state_broadcaster_spawner  = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_base_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='If true, use simulated clock'),
        
        rsp,
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
        joystick,

    ])