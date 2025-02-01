from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # lane_filter_visualizer 실행
        Node(
            package='lane_filter_visualizer',
            executable='lane_filter_visualizer',
            name='lane_filter_visualizer',
            output='screen',
        ),
    ])
