# scout-mini
<details>
  
# 🚀 Scout Mini ROS2 네비게이션 (Nav2) + 파이썬 제어 완전 가이드

<summary> > 본 문서는 **Scout Mini (ROS2 Humble)** 기준으로  
> SLAM → 맵핑 완료 이후, Nav2(네비게이션) 구성 및  
> **파이썬(`rclpy`) 기반 목표 전송**을 구현하는 과정을 정리한 자료입니다. </summary>

---

## 🧭 1. 전체 아키텍처 요약

- SLAM으로 얻은 `map` (map_server)
- 로컬라이제이션: `amcl`
- TF 체계: `map → odom → base_link`
- 센서: `scan`(LiDAR), `odom`, IMU
- Nav2 구성 노드:
  - `map_server`
  - `amcl`
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - `behavior_server`
  - `recoveries`
  - `lifecycle_manager`
- 파이썬(`rclpy`)로 `NavigateToPose` 액션을 사용

---

## ⚙️ 2. Nav2 띄우기 (기본 명령)

bash
ros2 launch nav2_bringup nav2_bringup_launch.py \
    map:=/home/ubuntu/maps/my_map.yaml \
    use_sim_time:=false \
    params_file:=/home/ubuntu/nav2_params/nav2_params.yaml```

설명
map: SLAM 결과 YAML 파일 경로
params_file: Nav2 파라미터 파일 (예시 아래 참조)

✅ 체크리스트

TF 정상 여부 (ros2 topic echo /tf)

odom, scan 토픽 데이터 정상 유입

amcl_pose가 출력되는지 확인 (ros2 topic echo /amcl_pose)```


## ⚙️ 3. Nav2 파라미터 예시 (nav2_params.yaml)

amcl:
  ros__parameters:
    use_sim_time: False
    min_particles: 500
    max_particles: 2000
    odom_frame_id: odom
    base_frame_id: base_link
    global_frame_id: map
    scan_topic: /scan

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      acc_lim_x: 0.5
      dec_lim_x: 0.5
      max_vel_x: 0.6
      min_vel_x: -0.2
      max_rotational_vel: 1.0
      min_rotational_vel: -1.0

bt_navigator:
  ros__parameters:
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    global_frame: odom

## ⚙️4. Costmap 예시 (추가 구성)
global_costmap:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 1.0
    publish_frequency: 1.0
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: true
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "laser_scan_sensor"
      laser_scan_sensor:
        topic: /scan
        expected_update_rate: 10.0
        data_type: "LaserScan"
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      inflation_radius: 0.55

## 🗂️ 5. 파이썬 기반 네비게이션 액션 구조

### Nav2 액션 타입:

### nav2_msgs/action/NavigateToPose

### 파이썬(rclpy)으로 액션 클라이언트 작성

## 📁 5-1. ROS2 파이썬 패키지 디렉토리 구조
```
scout_nav/
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/
│  └─ scout_nav
├─ scout_nav/
│  ├─ __init__.py
│  ├─ send_goal.py
│  └─ joystick_bridge.py  # (선택)
├─ launch/
│  └─ nav_with_py.launch.py
├─ params/
│  └─ my_nav_params.yaml
└─ README.md
```

## 📁 5-2. setup.py 예시
```
from setuptools import setup
import os
from glob import glob

package_name = 'scout_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Scout navigation helper',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'send_goal = scout_nav.send_goal:main',
        ],
    },
)
```

## 🧠 6. send_goal.py (전체 파이썬 코드)
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from tf_transformations import quaternion_from_euler

class NavClient(Node):
    def __init__(self):
        super().__init__('nav2_py_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav client started')

    def send_goal(self, pose: PoseStamped, timeout_sec: int = 30):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=timeout_sec)
        result = get_result_future.result().result
        status = get_result_future.result().status
        self.get_logger().info(f'Goal status: {status}, result: {result}')
        return result

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: distance_remaining={fb.distance_remaining:.2f}')

def make_pose(x, y, yaw=0.0, frame='map'):
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = rclpy.time.Time().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps

def main(args=None):
    rclpy.init(args=args)
    node = NavClient()
    try:
        pose = make_pose(1.2, 0.3, yaw=0.0)
        result = node.send_goal(pose, timeout_sec=120)
        node.get_logger().info(f'Navigation finished: {result}')
    except KeyboardInterrupt:
        node.get_logger().info('User interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
 ```   


## 🧩 7. 동작 흐름 요약
```
Nav2가 navigate_to_pose 액션 서버로 동작 중

파이썬 노드가 액션 클라이언트 생성

PoseStamped 생성 → NavigateToPose.Goal()에 삽입

feedback_callback()으로 진행 상황 수신

get_result_async()로 결과 확인

필요 시 goal_handle.cancel_goal_async()로 취소 가능
```

## 🚫 8. 목표 취소 예시
cancel_future = goal_handle.cancel_goal_async()
rclpy.spin_until_future_complete(node, cancel_future)


## 🌳 9. Behavior Tree / Recovery 동작
```
Nav2는 BT(Behavior Tree) 로 이동, 재계획, 복구를 제어합니다.

기본 BT 파일: navigate_w_replanning_and_recovery.xml

실패 시 rotate/backup 리커버리 수행
```

## 🧰 10. 트러블슈팅 팁
```
ros2 topic echo /amcl_pose 확인

TF 체계 검증: ros2 run tf2_tools view_frames.py

scan 토픽 이름과 YAML 일치 여부 확인

ROS_DOMAIN_ID 충돌 확인

Nav2 로그는 bt_navigator에서 출력 확인
```

## 🚀 11. 예제 Launch 파일 (nav_with_py.launch.py)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/ubuntu/maps/my_map.yaml'),
        DeclareLaunchArgument('params', default_value='/home/ubuntu/nav2_params/nav2_params.yaml'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_yaml]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),
    ])


## ✅ 12. 실행 전 최종 점검 리스트

 nav2_bringup이 정상적으로 실행 중

 /amcl_pose, /odom, /scan 토픽 확인

 /navigate_to_pose 액션 존재 (ros2 action list)

 colcon build 완료 후 ros2 run scout_nav send_goal 실행

 지도 상 좌표로 목표 전송 후 로봇 이동 확인

 ## 참고
 ```
 📘 참고

ROS2 Humble Nav2 공식 문서: https://navigation.ros.org/

tf_transformations 설치:

sudo apt install ros-humble-tf-transformations


nav2_bringup 패키지 설치:

sudo apt install ros-humble-nav2-bringup


💾 추천 저장 이름:
scout_mini_nav2_setup.md

📁 경로 예시:
~/ros2_ws/docs/scout_mini_nav2_setup.md
```


</details>
