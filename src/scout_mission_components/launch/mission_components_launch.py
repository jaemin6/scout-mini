from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scout_mission_components',
            executable='qr_detector',
            name='qr_detector',
            output='screen'
        ),
        Node(
            package='scout_mission_components',
            executable='robot_rotator_node',
            name='robot_rotator',
            output='screen'
        ),
        Node(
            package='scout_mission_components',
            executable='amcl_reset_node',
            name='amcl_resetter',
            output='screen'
        ),
        Node(
            package='scout_mission_components',
            executable='nav2_commander',
            name='room_navigator',
            output='screen'
        ),
    ])

