from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process

# 1. URDF 파일 경로 설정 (scout_ros2 패키지의 urdf 폴더를 가정)
robot_description_path = os.path.join(
    get_package_share_directory('scout_description'), 'urdf', 'scout_v2.xacro')

def generate_launch_description():
    # 2. URDF 파일 내용 읽기
    xacro_args = {
        'base_x_size': '0.90',
        'base_y_size': '0.58',
        'base_z_size': '0.25',
        'wheel_vertical_offset': '0.08'
        # scout_v2.xacro에 필요한 모든 변수를 여기에 추가해야 합니다.
    }

    # 2. Xacro 파일 파싱 (읽는 방식 변경)
    try:
        # ⚠️ xacro.process() 함수를 사용하고 mappings 인수를 전달합니다.
        robot_desc = process(robot_description_path, mappings=xacro_args)
    except FileNotFoundError:
        print(f"ERROR: Xacro file not found at {robot_description_path}")
        return LaunchDescription([])
    
    # 3. RPLiDAR 노드 (라이다 드라이버 실행)
    rplidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200, 
            'frame_id': 'lidar_link', # <--- ⚠️ URDF의 link name과 일치하도록 수정
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    # 4. Robot State Publisher 노드 (TF 변환 게시)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }]
    )

    return LaunchDescription([
        rplidar_node,
        robot_state_publisher_node,
    ])
