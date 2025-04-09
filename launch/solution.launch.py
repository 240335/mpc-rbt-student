import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')
    sim_dir = get_package_share_directory('mpc_rbt_simulator')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path]
        )

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_dir, 'launch', 'simulation.launch.py'))
    )

    local_node = Node(
            package='mpc_rbt_student',
            executable='localization',
            name='localization',
            output = 'screen'
        )

    
    return LaunchDescription([
        rviz_node,
        TimerAction(
            period = 3.0,
            actions = [simulator_launch]
        ),
        TimerAction(
            period = 15.0,
            actions = [local_node]
        )
    ])
