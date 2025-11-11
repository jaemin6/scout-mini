from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 🏭 컴포넌트를 담을 컨테이너 노드 정의 (mission_component_container)
    # rclcpp_components 패키지의 component_container_mt를 사용해 멀티스레드 환경을 구축합니다.
    container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name='mission_component_container',
        output='screen',
        emulate_tty=True,
    )

    # 2. 컴포넌트 로드 정의
    
    # 로딩하려는 각 컴포넌트는 'Node' 액션을 사용하며, 
    # 'executable'은 setup.py의 'rclpy_components'에 등록된 이름(왼쪽 값)을 사용합니다.
    
    # 2-1. AmclResetter 컴포넌트 로드
    amcl_reset_node = Node(
        package='scout_mission_components',
        executable='amcl_reset', # setup.py에 등록된 이름
        name='amcl_resetter',
        output='screen',
        emulate_tty=True,
        container='mission_component_container', # 컨테이너 이름 지정
    )

    # 2-2. RoomNavigator (Nav2 Commander) 컴포넌트 로드
    nav2_commander_node = Node(
        package='scout_mission_components',
        executable='nav2_commander', # setup.py에 등록된 이름
        name='room_navigator',
        output='screen',
        emulate_tty=True,
        container='mission_component_container',
    )
    
    # 2-3. QrDetector 컴포넌트 로드
    qr_detector_node = Node(
        package='scout_mission_components',
        executable='qr_detector', # setup.py에 등록된 이름
        name='qr_detector',
        output='screen',
        emulate_tty=True,
        container='mission_component_container',
    )
    
    # 2-4. RobotRotator 컴포넌트 로드
    robot_rotator_node = Node(
        package='scout_mission_components',
        executable='robot_rotator', # setup.py에 등록된 이름
        name='robot_rotator',
        output='screen',
        emulate_tty=True,
        container='mission_component_container',
    )

    return LaunchDescription([
        container,
        amcl_reset_node,
        nav2_commander_node,
        qr_detector_node,
        robot_rotator_node,
    ])
