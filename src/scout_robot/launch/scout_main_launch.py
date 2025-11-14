from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 1. Action Client 노드 (제어의 시작점)
    room_client_node = Node(
        package='scout_robot',
        executable='room_client',  # setup.py와 일치
        name='room_command_client',
        output='screen'
    )

    # 2. Action Server 노드 1 (이동 명령 수행)
    nav2_commander_node = Node(
        package='scout_robot',
        executable='nav2_commander', # setup.py와 일치
        name='nav2_commander_server',
        output='screen'
    )

    # 3. Action Server 노드 2 (회전 명령 수행)
    robot_rotator_node = Node(
        package='scout_robot',
        executable='robot_rotator', # setup.py와 일치
        name='robot_rotator_server',
        output='screen'
    )

    # 4. QR 코드 인식 노드
    qr_detector_node = Node(
        package='scout_robot',
        executable='qr_detector', # setup.py와 일치
        name='qr_detector_node',
        output='screen'
    )
    
    # 5. Topic Publisher 노드 (위치 리셋, setup.py에 먼저 추가해야 함)
    amcl_reset_node = Node(
        package='scout_robot',
        executable='amcl_reset', 
        name='amcl_reset_publisher',
        output='screen'
    )

    return LaunchDescription([
        room_client_node,
        nav2_commander_node,
        robot_rotator_node,
        qr_detector_node,
        amcl_reset_node # setup.py 추가
    ])