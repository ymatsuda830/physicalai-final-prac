# spawn_rupe.launch.py (xacro引数修正版)
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_rupe_description_share = get_package_share_directory('rupe_description')
    pkg_rupe_gazebo_share = get_package_share_directory('rupe_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # --- 環境変数設定 ---
    model_path = os.path.dirname(pkg_rupe_description_share)
    install_dir = os.path.dirname(os.path.dirname(model_path))
    plugin_path = os.path.join(install_dir, 'lib')
    env = {}
    if 'GAZEBO_MODEL_PATH' in os.environ:
        env['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + os.pathsep + model_path
    else:
        env['GAZEBO_MODEL_PATH'] = model_path
    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in os.environ:
        env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] + os.pathsep + plugin_path
    else:
        env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = plugin_path
    # --- ここまで ---

    world_file = os.path.join(pkg_rupe_gazebo_share, 'worlds', 'table.sdf')
    gui_config = os.path.join(pkg_rupe_gazebo_share, 'gui', 'gui.config')
    ign_gazebo = ExecuteProcess(
        cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
        output='screen',
        additional_env=env,
        shell=True
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui_config_file': gui_config,
            'extra_gazebo_args': '-v 4',  # 詳細ログを出力してデバッグ
            'gdb': 'false', # デバッガを使わない場合はfalse
            'verbose': 'true', # gazebo.launch.py自体のログも出す
        }.items()
    )

    # --- xacroに渡す共通の引数を定義 ---
    mesh_path = pkg_rupe_description_share
    mappings = {'mesh_path': mesh_path}

    # --- ROS側で使うURDFの準備 ---
    urdf_xacro_file = os.path.join(pkg_rupe_description_share, 'urdf', 'rupe.urdf.xacro')
    # xacroに引数を渡す
    urdf_config = xacro.process_file(urdf_xacro_file, mappings=mappings)
    robot_description_param = {'robot_description': urdf_config.toxml()}
    
    # robot_state_publisher には URDF を渡す
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': True}]
    )

    # --- Gazebo側で使うSDFの準備 ---
    sdf_xacro_file = os.path.join(pkg_rupe_description_share, 'urdf', 'rupe.sdf.xacro')
    # xacroに引数を渡す
    sdf_config = xacro.process_file(sdf_xacro_file, mappings=mappings)
    sdf_string = sdf_config.toxml()

    # Gazeboには SDF を渡す
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', sdf_string,
                   '-name', 'rupe',
                   '-z', '1.015'],
        output='screen'
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    # Spawnerノード
    spawners = [
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner', arguments=['rupe_arm_controller', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner', arguments=['rupe_gripper_controller', '--controller-manager-timeout', '30']),
    ]

    return LaunchDescription([
        ign_gazebo,
#        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=spawners,
            )
        ),
    ])