import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aerial_manipulator_description')
    world_path = os.path.join(
        pkg_share,
        'worlds',
        'my_simple_room.world',
    )
    models_dir = os.path.join(pkg_share, 'models')
    model_path = os.path.join(
        models_dir, 'roarm_quad', 'model.sdf'
    )

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_dir
        + (
            os.pathsep + existing_model_path
            if existing_model_path
            else ''
        ),
    )

    # gazebo_ros_init initializes ROS 2 for ModelPlugins.
    # gazebo_ros_factory provides /spawn_entity.
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
        output='screen',
    )

    spawn_roarm_quad = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'roarm_quad',
                    '-file', model_path,
                    '-x', '0',
                    '-y', '0',
                    '-z', '0',
                ],
                output='screen',
            )
        ],
    )

    hover_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hover_control',
                executable='hover_node',
                parameters=[{
                    'use_sim_time': True,
                    'mass_kg': 4.495,
                    'gravity': 9.8,
                    'target_z': 1.0,
                    'control_rate_hz': 50.0,
                }],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        tf_imu,
        spawn_roarm_quad,
        hover_node,
    ])
