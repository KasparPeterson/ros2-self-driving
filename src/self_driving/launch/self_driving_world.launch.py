import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_file_name = 'car_world.world'
    world = os.path.join(get_package_share_directory('self_driving'), world_file_name)

    print("\nWorld:")
    print(world)
    print("\n")

    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', f'gz_args:={world}'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
    ])
