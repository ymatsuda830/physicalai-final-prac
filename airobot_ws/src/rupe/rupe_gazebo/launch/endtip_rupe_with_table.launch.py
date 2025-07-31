# Copyright 2022 RT Corporation
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
from rupe_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('rupe_description'))}
    world_file = os.path.join(
        get_package_share_directory('rupe_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('rupe_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'rupe',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'rupe_control'
    description_loader.gz_control_config_file_path = 'config/rupe_controllers.yaml'
    description_loader.endtip_offset_xyz = '0 0 0.09'
    description_loader.endtip_offset_rpy = '0 -1.57 0'
    description = description_loader.load()

    rviz_config_file = get_package_share_directory(
            'rupe_moveit_config') + '/launch/endtip_run_move_group.rviz'
    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('rupe_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={
                'loaded_description': description,
                'rviz_config_file': rviz_config_file
            }.items()
        )

    spawn_joint_state_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner rupe_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner rupe_gripper_controller'],
                shell=True,
                output='screen',
            )

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ign_gazebo,
        gazebo_spawn_entity,
        move_group,
        spawn_joint_state_controller,
        spawn_arm_controller,
        spawn_gripper_controller,
        bridge
    ])
