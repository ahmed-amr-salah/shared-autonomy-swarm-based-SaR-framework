import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Set TURTLEBOT3_MODEL environment variable to 'burger'
    set_turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Paths to the TurtleBot3 Gazebo package and model files
    sdf_file_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'

    # Path to PX4 drone
    drone_sdf_file_path = os.path.join(
        os.getenv('HOME'), 'PX4-Autopilot', 'Tools', 'simulation', 'gazebo-classic', 
        'sitl_gazebo-classic', 'models', 'iris', 'iris.sdf'
    )

    # # Path to PX4 launch file
    # px4_pkg_path = get_package_share_directory('px4')
    # px4_launch_file = os.path.join(px4_pkg_path, 'launch', 'sitl.launch')
    
    # Include the standard empty world launch file to launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'empty_world.launch.py')
        )
    )

    # Spawn the first TurtleBot3 with a unique namespace and robot namespace
    robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='turtlebot1_ns',
        arguments=['-entity', 'turtlebot1', '-file', sdf_file_path, '-robot_namespace', 'turtlebot1_ns', '-x', '1', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Spawn the second TurtleBot3 with a unique namespace and robot namespace
    robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='turtlebot2_ns',
        arguments=['-entity', 'turtlebot2', '-file', sdf_file_path, '-robot_namespace', 'turtlebot2_ns', '-x', '2', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Spawn the drone
    drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='drone_ns',
        arguments=['-entity', 'iris', '-file', drone_sdf_file_path, '-x', '1', '-y', '0', '-z', '0.05'],
        output='screen'
    )

    return LaunchDescription([
        set_turtlebot_model,  # Set environment variable
        gazebo,
        robot1,
        robot2,
        drone
    ])
