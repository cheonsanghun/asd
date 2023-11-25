from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_term_project',
            executable='car_control_node',
            name='car_control_node',
            output='screen',
        ),
    ])