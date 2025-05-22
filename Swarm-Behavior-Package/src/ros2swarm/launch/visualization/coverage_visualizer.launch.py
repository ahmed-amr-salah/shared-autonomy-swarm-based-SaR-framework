from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2swarm',
            executable='coverage_visualizer',
            name='coverage_visualizer',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ]) 