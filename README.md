# scout-mini
<details>
  
<summary> 
  
# 🚀 Scout Mini ROS2 네비게이션 (Nav2) + 파이썬 제어 완전 가이드

> 본 문서는 **Scout Mini (ROS2 Humble)** 기준으로  
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

 </details>

 <details>
<summary>
  
 ## 참고 </summary>

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

<details>
<summary>

## 관련 링크 </summary>
로봇 팔, slam, nav2 : https://wiki.hiwonder.com/projects/PuppyPi/en/latest/docs/31.ROS2_SLAM_Mapping_Course.html#slam-mapping-principle
ros2 : https://github.com/roasinc/scout_mini_ros2
매뉴얼 : https://docs.roas.co.kr/scout_mini.html
scout mini : https://github.com/mattiadutto/scout_navigation
scout mini : https://github.com/agilexrobotics/scout_ros2

 </details>

<details>
<summary> 
  
# scout mini 실행 방법, 패키지 </summary> 
 
  
## 🤖 Scout Mini 내비게이션 패키지 사용법 (scout_navigation)

이 문서는 **ROS 2** 환경에서 **AgileX Scout Mini 로봇**의 내비게이션 기능을 설정하고 실행하는 방법을 설명
</summary>

### 1. 다운로드 (Download)

| 구분 | 명령어 | 설명 |
| :--- | :--- | :--- |
| **워크스페이스 준비** | `mkdir -p <ros2_workspace>/src` <br> `cd <ros2_workspace>/src` | ROS 2 워크스페이스 내에 `src` 폴더 생성 및 이동 |
| **기본 패키지 (필수)** | `git clone https://github.com/mattiadutto/scout_navigation.git` | 내비게이션 핵심 패키지 |
| **시뮬레이션 추가 패키지** | `git clone https://github.com/agilexrobotics/ugv_sdk.git` <br> `git clone https://github.com/ROSETEA-lab/ugv_gazebo_sim` <br> `git clone -b humble https://github.com/ROSETEA-lab/scout_ros2` | 시뮬레이션 환경 구축을 위한 패키지 |
| **실제 로봇 추가 패키지** | `git clone https://github.com/agilexrobotics/ugv_sdk.git` <br> `git clone -b humble https://github.com/ROSETEA-lab/scout_ros2` | 실제 로봇 제어를 위한 패키지 |

---

### 2. 빌드 (Build)

```bash
cd ..
colcon build
source install/setup.bash
```

###  3. 탐색 (Navigation)
**파일명:** `nav2.launch.py`  
**기능:** ROS 2 Navigation 2 (Nav2) 스택을 사용하여 로봇의 자율 내비게이션을 실행합니다.

---

#### 🔹 사용 예시 (Launch Command)

| 구분 | 명령어 예시 |
|:------|:-------------|
| **시뮬레이션 환경** | `ros2 launch scout_navigation nav2.launch.py namespace:=scout_mini map_name:=workshop_big_empty_slam.yaml rviz_params_file:=scout_mini_navigation.rviz` |
| **실제 로봇 환경** | `ros2 launch scout_navigation nav2.launch.py use_sim_time:=False map_name:=velodyne_andata_5_destra.yaml nav2_params_file:=nav2_params_scout_mini.yaml rviz_params_file:=scout_mini_robot.rviz` |

---

#### 🔹 매개변수 (Parameters)

| 매개변수 | 기본값 | 설명 |
|:-----------|:-----------|:-----------|
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 여부 |
| `use_rviz` | `true` | RViz2 사용 여부 (원격 측정 작업용) |
| `map_name` | `slam_farm.yaml` | Nav2 스택에 로드될 맵 이름 (맵 폴더에 위치) |
| `namespace` | *(비어 있음)* | 로봇의 네임스페이스 |
| `ekf_params_file` | `ekf_localization_with_gps.yaml` | 확장 칼만 필터(EKF) 구성 파일 (`config` 폴더) |
| `nav2_params_file` | `nav2_params.yaml` | Navigation 2 스택 구성 파일 (`config` 폴더) |
| `rviz_params_file` | `scout_mini_navigation.yaml` | RViz2 구성 파일 (`config` 폴더) |

---



### 4. 내비게이션: gps.launch.py
기능: GPS/IMU/로봇 주행거리계 간의 데이터 융합을 테스트하기 위한 실행 파일

📝 TODO: GPS/IMU/로봇 주행거리계 간의 데이터 융합을 광범위하게 테스트


### 5. 매핑 (Mapping)
**파일명:** `slam_offline.launch.py`  
**기능:** 미리 기록된 데이터를 기반으로 환경 맵을 생성하여 탐색(Navigation) 스택에 활용합니다.

---

#### 🔹 사용 예시 (Launch Command)

| 구분 | 명령어 예시 |
|:------|:-------------|
| **실제 로봇** | `ros2 launch scout_navigation slam_offline.launch.py` |

---

#### 🔹 매개변수 (Parameters)

| 매개변수 | 기본값 | 설명 |
|:-----------|:-----------|:-----------|
| `namespace` | *(비어 있음)* | 로봇의 네임스페이스 |
| `slam_params_file` | `mapper_params_offline.yaml` | SLAM Toolbox 구성 파일 (`config` 폴더) |
| `autostart` | `true` | SLAM Toolbox를 자동으로 시작 (`use_lifecycle_manager`가 `true`이면 무시됨) |
| `use_lifecycle_manager` | `false` | 노드 활성화 중 본드 연결 활성화 여부 |

---

### 6. 매핑 (Mapping)
**파일명:** `slam_online_async.launch.py`  
**기능:** 실시간 센서 데이터를 기반으로 환경 맵을 생성하여 탐색(Navigation) 스택에 활용합니다.

---

#### 🔹 사용 예시 (Launch Command)

| 구분 | 명령어 예시 |
|:------|:-------------|
| **실제 로봇** | `ros2 launch scout_navigation slam_online_async.launch.py use_sim_time:=False` |

---

#### 🔹 매개변수 (Parameters)

| 매개변수 | 기본값 | 설명 |
|:-----------|:-----------|:-----------|
| `namespace` | *(비어 있음)* | 로봇의 네임스페이스 |
| `use_sim_time` | `true` | 시뮬레이션 시간 사용 여부 |
| `slam_params_file` | `mapper_params_online_async.yaml` | SLAM Toolbox 구성 파일 (`config` 폴더) |
| `scan` | `scan` | 로봇의 레이저 스캔 리매핑 토픽 |
| `tf` | `tf` | 로봇의 TF 리매핑 토픽 |
| `tf_static` | `tf_static` | 로봇의 정적 TF 리매핑 토픽 |

---
</details>



<details>
<summary>
  
## scout mini 실행 방법 </summary>


```
bring up 실행 시 
source ~/scout_ws/install/setup.bash
ros2 launch scout_base scout_base.launch.py

정상 동작시 /cmd_vel → /odom, /imu, /battery_state 등 토픽이 반환, 활성화 됨
라즈베리파이와 스카우트 본체가 CAN으로 통신 시작
```

```
| 단계 | 내용             | 명령어                                                          |
| -- | -------------- | ------------------------------------------------------------ |
| 1  | ROS2 워크스페이스 생성 | `mkdir -p ~/scout_ws/src`                                    |
| 2  | Git 클론         | `git clone https://github.com/agilexrobotics/scout_ros2.git` |
| 3  | 의존성 설치         | `rosdep install --from-paths src --ignore-src -r -y`         |
| 4  | 빌드             | `colcon build --symlink-install`                             |
| 5  | 실행             | `ros2 launch scout_base scout_base.launch.py`                |
```


### can-utils 설치
sudo apt install can-utils -y


### ② CAN 인터페이스 설정
```
USB를 꽂으면 /dev/ttyUSB0 로 보이지만, ROS2는 can0 인터페이스를 사용
아래 명령으로 CAN을 활성화 시켜야 함
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

활성화 확인
ifconfig can0
반환 값이 can0: flags=193<UP,RUNNING,NOARP> 이런 식으로 나오면 성공
```

### ③ 다시 scout_base 실행
```
이제는 포트를 can0으로 지정
source ~/ros2_ws/install/setup.bash
ros2 run scout_base scout_base_node --ros-args -p port_name:=can0 -p is_scout_mini:=true
✅ 정상 동작 시 출력 예시
Loading parameters:
- port name: can0
- is scout mini: true
----------------------------
Robot base: Scout Mini
Start listening to port: can0
Received feedback from MCU ...
```

아래 명령으로 확인
ros2 topic list
반환값이 → /odom, /battery_state, /imu/data, /cmd_vel 등이 보이면 성공

</details>

