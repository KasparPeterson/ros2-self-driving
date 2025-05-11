import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
    cmd_vel_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    parameter_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )
    """robot_description = {"robot_description": robot_description_content}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )"""

    print("SHARE DIRECTORY:", get_package_share_directory('self_driving'))

    urdf_file = os.path.join(
        get_package_share_directory('self_driving'),
        'models',
        'robot',
        'model.sdf'
    )

    print("URDF FILE:", urdf_file)
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        gazebo,
        cmd_vel_bridge,
        parameter_bridge,
        robot_state_publisher,
        joint_state_publisher,
    ])
