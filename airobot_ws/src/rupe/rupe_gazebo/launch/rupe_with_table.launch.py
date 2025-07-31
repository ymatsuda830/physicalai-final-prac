# rupe_with_table.launch.py (Gazebo Classic + URDF + Transmission 版)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_rupe_description = get_package_share_directory('rupe_description')
    pkg_rupe_gazebo = get_package_share_directory('rupe_gazebo')
    
    # crane_plusのワールドファイルをコピーして使う
    world_file = os.path.join(pkg_rupe_gazebo, 'worlds', 'table.world')

    # Gazebo Classicを起動
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
    )

    # URDFを準備
    urdf_xacro_file = os.path.join(pkg_rupe_description, 'urdf', 'rupe.urdf.xacro')
    # use_gazebo:=true を渡して、gazebo_ros2_control が有効になるようにする
    robot_description_config = xacro.process_file(urdf_xacro_file, mappings={'use_gazebo': 'true'})
    robot_description_param = {'robot_description': robot_description_config.toxml()}
    
    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': True}]
    )

    # GazeboにURDFモデルをスポーン
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'rupe',
                   '-z', '1.015'],
        output='screen'
    )
    
    # Spawners
    spawners = [
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner', arguments=['rupe_arm_controller', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner', arguments=['rupe_gripper_controller', '--controller-manager-timeout', '30']),
    ]

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ] + spawners)
