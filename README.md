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
### 디렉토리 구조 분석
```
scout_nav/	패키지 루트	ROS 2 패키지의 최상위 폴더.
├─ package.xml	패키지 정의	패키지 이름, 버전, 작성자, 빌드 및 실행 의존성 등 메타데이터 정의.
├─ setup.py	Python 빌드 스크립트	Python 소스 코드(노드)를 빌드하고 설치하는 방법을 정의 (ROS 2 Python 패키지에서 필수).
├─ setup.cfg	설정 파일 (선택적)	setuptools나 다른 도구에 대한 설정을 포함할 수 있음.
├─ resource/	리소스/마커 파일	ROS 2가 패키지를 인식하는 데 사용하는 마커 파일 포함.
│ └─ scout_nav	마커 파일 (내용은 패키지 이름과 동일).	
├─ scout_nav/	Python 모듈 폴더	실제 Python 코드가 포함된 폴더. setup.py에 의해 Python 모듈로 인식됨.
│ ├─ __init__.py	Python 모듈 초기화 파일.	
│ ├─ send_goal.py	Nav2의 목표 지점(Goal)을 전송하는 등의 기능을 하는 Python 노드/스크립트.	
│ └─ joystick_bridge.py	(선택) 조이스틱 입력을 로봇 제어 명령이나 Nav2 관련 명령으로 변환하는 노드.	
├─ launch/	실행 파일 (Launch files)	Nav2 스택과 로봇 노드들을 한 번에 실행하기 위한 .launch.py 파일을 포함.
│ └─ nav_with_py.launch.py	Nav2와 Python 노드들을 실행하는 주요 런치 파일.	
├─ params/	매개변수 파일 (Parameters)	Nav2 스택(AMCL, Planner, Controller, Costmap 등)의 상세 설정을 위한 YAML 파일 포함.
│ └─ my_nav_params.yaml	Nav2 관련 매개변수를 담고 있는 설정 파일.	
└─ README.md	문서	패키지 사용 방법 및 설명을 담은 문서.
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
```
dofbot: https://www.yahboom.net/study/Dofbot-Jetson_nano
로봇 팔, slam, nav2 : https://wiki.hiwonder.com/projects/PuppyPi/en/latest/docs/31.ROS2_SLAM_Mapping_Course.html#slam-mapping-principle
ros2 : https://github.com/roasinc/scout_mini_ros2
매뉴얼 : https://docs.roas.co.kr/scout_mini.html
scout mini : https://github.com/mattiadutto/scout_navigation
scout mini : https://github.com/agilexrobotics/scout_ros2
```
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

정상 동작시 /cmd_vel → /odom, /imu, /battery_state, /tf, /scout_base/feedback, /scan or /lidar/points 등 토픽이 반환, 활성화 됨
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



<details>
  
<summary>  
  
# 🛠️ scout_base 빌드 오류 해결 가이드 (ROS 2 Humble 대응) 

# 🧩 오류 해결: scout_base 빌드 실패 (tf2_geometry_msgs 관련)

## 🔍 문제 원인
`scout_base` 패키지를 `colcon build`로 컴파일할 때  
`tf2_geometry_msgs` 관련 헤더나 라이브러리를 찾지 못하는 오류가 발생할 수 있습니다.  
이는 `package.xml` 또는 `CMakeLists.txt`에 해당 패키지 의존성이 선언되어 있지 않기 때문입니다. </summary>



## ✅ 1. package.xml 확인

```
📂 경로: `/home/eddy/ros2_ws/src/scout_ros2/scout_base/package.xml`

`tf2_geometry_msgs`가 의존성으로 선언되어 있는지 확인합니다.  
다음 두 줄 중 하나가 `<depend>`, `<build_depend>`, `<exec_depend>` 태그 내에 반드시 포함되어야 합니다.

xml
<depend>tf2_geometry_msgs</depend>
만약 없다면, <build_depend>와 <exec_depend> 섹션에 다음을 추가하세요
<build_depend>tf2_geometry_msgs</build_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
```

### 2. CMakeLists.txt 확인
📂 경로: /home/eddy/ros2_ws/src/scout_ros2/scout_base/CMakeLists.txt

tf2_geometry_msgs를 컴파일러에 인식시키려면 다음 세 부분이 모두 존재해야 합니다.
```
find_package(tf2_geometry_msgs REQUIRED)

target_include_directories(scout_base_node PRIVATE
  ...
  ${tf2_geometry_msgs_INCLUDE_DIRS}  # ✅ 반드시 포함
)

target_link_libraries(scout_base_node
  ...
  ${tf2_geometry_msgs_LIBRARIES}     # ✅ 반드시 포함
)
```


### 수정이 끝 난 후
cd ~/ros2_ws
colcon build --packages-select scout_base
source install/setup.bash


## 요약
| 파일               | 확인 항목        | 내용                                         |
| ---------------- | ------------ | ------------------------------------------ |
| `package.xml`    | 의존성 선언       | `<depend>tf2_geometry_msgs</depend>`       |
| `CMakeLists.txt` | find_package | `find_package(tf2_geometry_msgs REQUIRED)` |
| `CMakeLists.txt` | include 디렉토리 | `${tf2_geometry_msgs_INCLUDE_DIRS}`        |
| `CMakeLists.txt` | 라이브러리 링크     | `${tf2_geometry_msgs_LIBRARIES}`           |
</details>


<details>

<summary> 

## 🚨 문제 요약

ROS 2 Humble 버전에서 `declare_parameter()`를 기본값 없이 사용하면 다음과 같은 **CMake 오류** 또는 **rclcpp 파라미터 오류**가 발생
CMake Error at CMakeLists.txt: ...
rclcpp::ParameterTypeException: parameter 'port_name' has not been declared

이 문제는 ROS 2 Foxy 이하 버전에서는 허용되던 코드가 Humble 이상에서는 **기본값을 반드시 지정해야 하는 방식으로 변경**되었기 때문입니다.</summary>

---
```
## 📍 원인

`scout_base_ros.cpp` 파일 내 `declare_parameter()` 호출부에 기본값이 누락되어 있습니다.

### 🔴 기존 코드 (오류 발생 예시)

cpp
this->declare_parameter("port_name");
this->declare_parameter("odom_frame");
this->declare_parameter("base_frame");
this->declare_parameter("odom_topic_name");
this->declare_parameter("is_scout_mini");
this->declare_parameter("is_omni_wheel");
this->declare_parameter("simulated_robot");
this->declare_parameter("control_rate");
```

## 해결 방법

1. 파일 열기
nano ~/ros2_ws/src/scout_ros2/scout_base/src/scout_base_ros.cpp

2. 코드 수정
```
| 줄 번호 | 원래 코드                                         |   타입   | 수정된 코드                                                |
| :--: | :-------------------------------------------- | :----: | :---------------------------------------------------- |
|  18  | `this->declare_parameter("port_name");`       | String | `this->declare_parameter("port_name", "can0");`       |
|  20  | `this->declare_parameter("odom_frame");`      | String | `this->declare_parameter("odom_frame", "odom");`      |
|  21  | `this->declare_parameter("base_frame");`      | String | `this->declare_parameter("base_frame", "base_link");` |
|  22  | `this->declare_parameter("odom_topic_name");` | String | `this->declare_parameter("odom_topic_name", "odom");` |
|  24  | `this->declare_parameter("is_scout_mini");`   |  Bool  | `this->declare_parameter("is_scout_mini", false);`    |
|  25  | `this->declare_parameter("is_omni_wheel")     |  Bool  | `this->declare_parameter("is_omni_wheel", false);`    |
|  27  | `this->declare_parameter("simulated_robot")   |  Bool  | `this->declare_parameter("simulated_robot", false);`  |
|  28  | `this->declare_parameter("control_rate");`    | Double | `this->declare_parameter("control_rate", 50.0);`      |
```

## 실행 테스트
ros2 run scout_base scout_base_node --ros-args -p port_name:=can0 -p is_scout_mini:=true
실행 시 오류가 발생하지 않으면 정상적으로 패치가 완료


### 요약
| 항목    | 내용                                                                               |
| ----- | ---------------------------------------------------------------------------------    |
| 오류 원인 | declare_parameter() 기본값 누락                                                   |
| 발생 버전 | ROS 2 Humble 이상                                                                 |
| 해결 방법 | 각 파라미터에 타입에 맞는 기본값 추가                                               |
| 빌드 명령 | `colcon build --packages-select scout_base --symlink-install --cmake-clean-cache` |
| 결과    | scout_base 노드 실행 성공                                                            |


### 요약 2

// 문자열
this->declare_parameter("param_name", "기본값");

// 숫자
this->declare_parameter("rate", 10.0);

// 불리언
this->declare_parameter("enabled", false);
</details>



<details>
  
<summary>  

# 🛰️ RPLidar Frame ID 불일치 오류 해결 가이드 (SLAM 데이터 미표시 문제)

---

## 🚨 문제 요약

RPLidar를 실행했을 때 `rviz2` 또는 `slam_toolbox`에서 **LaserScan 토픽이 보이지 않거나**,  
보이더라도 로봇 본체(`/base_link`) 기준으로 좌표가 맞지 않는 문제가 발생함

이 문제는 **RPLidar의 프레임 ID(`frame_id`)가 로봇 본체(`base_link`)와 일치하지 않아서**  
SLAM 알고리즘이 `/scan` 데이터를 좌표 변환(`tf`) 트리에 연결하지 못하기 때문  </summary>



## ⚙️ 원인 분석

- RPLidar 드라이버(`rplidar_ros`)는 기본적으로 `frame_id: laser_frame`으로 데이터를 발행합니다.
- 반면, `scout_base` 노드(로봇 베이스)는 `base_link`를 본체 프레임으로 사용합니다.
- 두 프레임이 연결되지 않으면 `/tf` 트리 상에서 **laser → base_link** 변환이 없어 SLAM이 라이다 데이터를 무시합니다.



## ✅ 해결 방법

### 1️⃣ RPLidar 런치 파일 수정

`rplidar_a1_launch.py` (또는 사용하는 모델에 해당하는 런치 파일)을 열고  
`frame_id` 값을 명시적으로 `"base_link"`로 수정합니다.

#### 🔧 파일 열기

bash
nano /home/eddy/ros2_ws/install/rplidar_ros/share/rplidar_ros/launch/rplidar_a1_launch.py

### 해결 방법
| 구분         | 코드 내용                                                                                                                                                                                                                                                                                                                                                                      |
| ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **기존 코드**  | `python<br>Node(<br>    package='rplidar_ros',<br>    executable='rplidar_node',<br>    name='rplidar_node',<br>    output='screen',<br>    parameters=[{<br>        'serial_port': '/dev/ttyUSB0',<br>        'serial_baudrate': 115200,<br>        'inverted': False,<br>        'angle_compensate': True,<br>        # frame_id 누락<br>    }]<br>),`                     |
| **수정된 코드** | `python<br>Node(<br>    package='rplidar_ros',<br>    executable='rplidar_node',<br>    name='rplidar_node',<br>    output='screen',<br>    parameters=[{<br>        'serial_port': '/dev/ttyUSB0',<br>        'serial_baudrate': 115200,<br>        'frame_id': 'base_link',  # ✅ 추가됨<br>        'inverted': False,<br>        'angle_compensate': True,<br>    }]<br>),` |

### 모든 노드 재 시작

# 1️⃣ 라이다 노드 실행
ros2 launch rplidar_ros rplidar_a1_launch.py

# 2️⃣ Scout Mini 본체 구동
ros2 run scout_base scout_base_node --ros-args -p port_name:=can0 -p is_scout_mini:=true

# 3️⃣ SLAM 실행 (예: slam_toolbox)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False

# 4️⃣ RViz2 시각화
ros2 run rviz2 rviz2
</details>


<details>
  
<summary> 
  
# 🚀 SLAM 실행을 위한 브링업 절차 가이드

## 🧭 목적
Scout Mini + RPLidar + SLAM Toolbox 환경에서  
자율주행용 맵핑(SLAM)을 수행하기 위한 기본 런치 순서를 정리 </summary> 

```
⚙️ 1️⃣ 워크스페이스 환경 설정
bash
cd ~/ros2_ws/
source install/setup.bash
```

## 실행 순서 요약

| 단계 | 명령어                                                    | 주요 역할                   |
| -- | ------------------------------------------------------ | ----------------------- |
| 1  | `ros2 launch scout_base scout_base.launch.py`          | 로봇 본체 (Odometry, TF 발행) |
| 2  | `ros2 launch rplidar_ros rplidar_a1_launch.py`         | 라이다 데이터 발행              |
| 3  | `ros2 launch slam_toolbox online_async_launch.py`      | SLAM 실행 (맵 + 위치 추정)     |
| 4  | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` | 로봇 이동 제어                |
| 5  | `ros2 run rviz2 rviz2`                                 | 데이터 시각화                 |

</details>


<details>
  
<summary> 

## ROS2에서 토픽이 공유되는 조건
### 로컬 pc와 라즈베리 환경 변수 통일 </summary> 

| 항목                     | 설명                                   | 두 환경에서 같아야 함 |
| ---------------------- | ------------------------------------ | ------------ |
| **ROS_DOMAIN_ID**      | 같은 도메인 ID여야 DDS 통신 가능                | ✅ 같아야 함      |
| **ROS_LOCALHOST_ONLY** | 로컬 통신 제한 여부 (`0`이면 네트워크 허용)          | ✅ 둘 다 `0`    |
| **RMW_IMPLEMENTATION** | DDS 미들웨어 종류 (기본: FastRTPS)           | ✅ 같아야 함      |
| **네트워크 대역**            | 같은 네트워크 (예: 192.168.0.x)             | ✅ 같아야 함      |
| **방화벽 / NAT**          | UDP 브로드캐스트 차단되면 안 됨                  | ✅ 오픈되어야 함    |
| **패키지명 / 토픽명**         | 상관없음 (단, 퍼블리셔/서브스크라이버 토픽명이 일치해야 통신됨) | ⚙️ 코드에 따라 다름 |


### 🟢 라즈베리파이
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_HOSTNAME=192.168.x.xxx
```

### 🟢 로컬 PC (VM)
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_HOSTNAME=192.168.x.xxx
```

</details>


<details>
  
<summary> 

# 🧭 SLAM 시 RViz2에서 선택해야 할 주요 토픽 가이드 (Scout Mini 기준)  </summary> 


---

## ✅ 1️⃣ Fixed Frame
- **반드시 `map`으로 설정**
  - RViz 상단 메뉴 → `Fixed Frame` → `map` 선택  
  - 이 프레임이 RViz 전체의 기준 좌표계가 됩니다.  
  - `odom`, `base_link`, `laser` 등은 `TF`를 통해 `map`과 연결되어야 합니다.

---

## ✅ 2️⃣ TF (Transform)
- **추가 방법:** `Add` → `TF`
- **기능:** 좌표계(`map`, `odom`, `base_link`, `laser`, `camera_link` 등) 간 관계를 시각화  
- **확인 포인트:**
  - `map → odom → base_link → laser` 연결이 반드시 이어져야 함  
  - 연결이 끊기면 로봇 위치가 튀거나 맵이 갱신되지 않음

---

## ✅ 3️⃣ Map
- **추가 방법:** `Add` → `By topic` → `/map`
- **기능:** SLAM 노드가 생성한 지도(OccupancyGrid 형태) 시각화  
- **Tip:** Color Scheme을 `map` 또는 `costmap`으로 두면 가시성이 좋음

---

## ✅ 4️⃣ Odometry
- **추가 방법:** `Add` → `Odometry` → Topic: `/odom`
- **기능:** 로봇 이동 궤적 시각화  
- **옵션:** `Shape: Arrow` 또는 `Line` 설정 가능

---

## ✅ 5️⃣ LaserScan (Lidar)
- **추가 방법:** `Add` → `LaserScan` → Topic: `/scan` (또는 `/rplidar/scan`, `/lidar/scan`)
- **기능:** 라이다 센서가 인식한 점 구름(빨간 점 등) 표시  
- **확인 포인트:**  
  - 벽, 장애물 등이 제대로 찍히는지 확인  
  - 점이 뜨지 않으면 라이다 연결 문제

---

## ✅ 6️⃣ RobotModel
- **추가 방법:** `Add` → `RobotModel`
- **기능:** `TF`를 기반으로 URDF 로봇 모델을 표시  
- **확인 포인트:**  
  - `base_link` 기준으로 로봇 형태가 표시되어야 함

---

## ✅ 7️⃣ Path (선택)
- **추가 방법:** `Add` → `Path` → Topic: `/path` 또는 `/slam_toolbox/trajectory`
- **기능:** 로봇이 지나온 경로를 선으로 표시

---

## ✅ 8️⃣ Pose 관련 (버튼)
- **2D Pose Estimate:**  
  - 로봇 초기 위치를 수동으로 지정 (Localization 때 필요)
- **2D Nav Goal:**  
  - Nav2 실행 시 목표 위치 지정용 (SLAM 단계에서는 비활성화해도 됨)

---

## ⚙️ 필수 확인 포인트
- `TF` 체인: `map → odom → base_link → laser` 가 모두 연결되어야 함  
- `/scan` 점들이 표시되지 않으면 라이다 문제  
- `/map`이 갱신되지 않으면 SLAM 노드가 맵을 발행하지 못한 상태

---

## 💡 추가 팁
- `slam_toolbox` 또는 `hector_slam`, `gmapping`에 따라 일부 토픽 이름이 다를 수 있습니다.  
- 원하신다면 RViz2 구성을 자동으로 불러올 수 있는 **`.rviz2 설정 파일`** 도 만들어드릴 수 있습니다.  
  → 사용 중인 SLAM 패키지 이름(`slam_toolbox`, `gmapping`, 등)을 알려주세요.  
  → 바로 불러서 `File → Open Config` 로 한 번에 세팅 가능합니다.

---

## 정리 표
| 구분    | RViz 항목          | 토픽 이름 예시             | 역할           |
| ----- | ---------------- | -------------------- | ------------ |
| 좌표계   | Fixed Frame      | map                  | RViz의 기준 좌표  |
| 좌표 연결 | TF               | /tf, /tf_static      | 좌표 변환 관계 확인  |
| 지도    | Map              | /map                 | SLAM이 생성한 지도 |
| 위치 추정 | Odometry         | /odom                | 로봇 이동 경로     |
| 센서    | LaserScan        | /scan                | 라이다 거리 데이터   |
| 로봇 모델 | RobotModel       | (TF 기반)              | 로봇 구조 표시     |
| 경로    | Path             | /trajectory 또는 /path | 이동 궤적 표시     |
| 위치 지정 | 2D Pose Estimate | (버튼)                 | 초기 위치 지정용    |
| 목표 지정 | 2D Nav Goal      | (버튼)                 | Nav2 때 사용    |


</details>


<details>
  
<summary> 

# 📦 ROS2 패키지 설치 명령어 목적 및 Nav2 관련성 정리 (Humble 기준)  </summary>

| 명령어 | 주요 기능 / 목적 | Nav2와의 관련성 | 없어도 되는지 여부 |
|---------|------------------|------------------|--------------------|
| **sudo apt install ros-humble-urdf-tutorial** | URDF(로봇 구조 설명 파일) 예제 패키지.<br>로봇 모델(`RobotModel`)을 RViz에서 시각화할 때 참고용.<br>로봇의 링크·조인트 구조를 학습하거나 테스트할 때 사용. | 🔹 **간접적 관련**<br>URDF는 TF 구조 형성의 기반이므로 Nav2가 올바르게 동작하려면 로봇 URDF가 필요하지만,<br>이 패키지는 단순 예제용이라 **필수는 아님**. | ✅ 없어도 됨 (단, URDF 예제 학습 시 유용) |
| **sudo apt install ros-humble-nav2-simple-commander** | Nav2를 **파이썬 코드에서 제어**할 수 있게 해주는 라이브러리.<br>`BasicNavigator` 클래스 등 포함.<br>Python으로 `goToPose()` 등 호출 가능. | 🟢 **강력히 관련 있음 (Nav2 파이썬 제어 핵심)**<br>자율주행, 서빙 로봇 등에서 Python 스크립트로 Nav2를 제어할 때 반드시 필요. | ❌ **필수** (파이썬으로 Nav2 제어 시 반드시 설치) |
| **sudo apt install ros-humble-tf-transformations** | TF 좌표 변환 관련 유틸리티 제공.<br>예: Euler ↔ Quaternion 변환 함수 등 (`euler_from_quaternion`, `quaternion_from_euler`) | 🔹 **보조적 관련**<br>Nav2 자체는 사용하지 않지만, Pose 계산/변환 시 유용함.<br>특히 파이썬 코드에서 yaw, roll, pitch 계산 시 자주 사용. | ✅ 없어도 됨 (필요 시만 설치) |
| **sudo apt install python3-transforms3d** | 3D 변환 수학 라이브러리 (순수 Python).<br>TF가 아닌 **독립적인 행렬·쿼터니언 변환**을 지원.<br>ROS 외부 수학 계산용. | 🔹 **보조적 관련**<br>Nav2 자체에는 필요 없음.<br>파이썬 코드에서 쿼터니언·행렬 연산이 많을 때 편리. | ✅ 없어도 됨 (수학 계산이 필요할 때만 설치) |
| **sudo apt install ros-humble-v4l2-camera** | ROS2용 **카메라 드라이버 패키지**.<br>UVC 웹캠 등 `/dev/video0` 장치를 ROS 노드(`/image_raw`)로 변환하여 발행.<br>RViz나 ArUco 마커 인식, 객체 추적 등 비전 기반 기능에서 사용. | 🟢 **간접적 관련 있음**<br>Nav2 자체에는 필요 없지만,<br>**Aruco 마커 인식 기반 위치 보정** 등 비전 기반 기능을 쓸 때 반드시 필요. | ✅ 없어도 됨 (카메라 비전 기능을 쓰지 않을 경우) |
| **sudo apt install ros-humble-aruco-ros** | **ArUco 마커 인식용 ROS2 노드** 제공.<br>카메라 입력으로부터 마커 ID, 포즈(TF) 등을 검출 및 발행.<br>마커 기반 위치 인식·로봇 정렬 등에 활용 가능. | 🟢 **간접적 관련 있음**<br>Nav2 내 위치 보정이나 Docking, Visual Localization을 구현할 때 매우 유용.<br>특히 ArUco 마커 기반 정밀 위치 인식에 필수. | ✅ 없어도 됨 (마커 인식 기능을 사용하지 않을 경우) |


```
 정리 요약
- **Nav2 파이썬 제어용 필수:** `ros-humble-nav2-simple-commander`  
- **보조적·학습용:** `urdf-tutorial`, `tf-transformations`, `transforms3d`  
- **결론:**  
  → Nav2를 단순히 실행하거나 RViz에서 제어하는 데는 불필요  
  → Python 코드로 Nav2를 제어하려면 `nav2-simple-commander` **꼭 필요**
  - **추천:** `v4l2-camera` (카메라 기반 ArUco 등 비전 기능 사용 시)
```
</details>

<details>
  
<summary> 

# 🤖 Realsense + RPLidar + Scout SLAM 통합 실행 정리 </summary> 


## 📦 전체 구성 요약
| 구분 | 실행 환경 | 목적 | 주요 노드 / 센서 |
|------|-------------|--------|------------------|
| **1. Base Bringup** | SSH (라즈베리) | 로봇 구동부 활성화 (odom, tf, cmd_vel 등) | `scout_base` |
| **2. RPLidar** | SSH (라즈베리) | 주변 거리 데이터 수집 | `rplidar_ros` |
| **3. Realsense 카메라** | SSH (라즈베리) | 영상 + 깊이(Depth) 데이터 수집 | `realsense2_camera` |
| **4. SLAM Toolbox** | SSH (라즈베리) | 라이다 + odom을 이용해 지도 작성 | `slam_toolbox` |
| **5. Robot Model** | 로컬 (노트북) | RViz에서 URDF 모델 로딩 | `your_robot_description` |
| **6. RViz2 시각화** | 로컬 (노트북) | 맵, 라이다, 카메라 등 시각화 | `rviz2` |
| **7. Teleop Keyboard** | 로컬 (노트북) | 키보드로 로봇 제어 (`cmd_vel` 발행) | `teleop_twist_keyboard` |

---

## ⚙️ 실행 명령어 정리

### 🧩 [라즈베리 SSH 환경]
> 센서 및 SLAM 노드는 실제 하드웨어가 연결된 라즈베리에서 실행해야 합니다.

| 실행 순서 | 기능 | 명령어 |
|------------|--------|---------|
| ① | **로봇 브링업 (Scout Base)** | `ros2 launch scout_base scout_base.launch.py` |
| ② | **라이다 (RPLidar)** | `ros2 launch rplidar_ros rplidar_a1_launch.py` |
| ③ | **Realsense 카메라** | `ros2 launch realsense2_camera rs_launch.py` |        ## 생략
| ④ | **SLAM Toolbox** | `ros2 launch slam_toolbox online_async_launch.py` |

---

### 💻 [로컬 PC 환경]
> 시각화 및 제어용 노드들은 로컬 PC에서 실행합니다.  
> 로컬과 라즈베리가 같은 **ROS_DOMAIN_ID**로 연결되어 있어야 합니다.

| 실행 순서 | 기능 | 명령어 |
|------------|--------|---------|
| ⑤ | **로봇 모델 표시 (URDF)** | `ros2 launch scout_description scout_base_description.launch.py` |
| ⑥ | **RViz2 시각화** | `ros2 run rviz2 rviz2` |
| ⑦ | **텔레옵 키보드** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

---

## 🧠 RViz2에서 추가해야 할 주요 Topic

| RViz2 Display 항목 | 구독할 토픽 이름 | 설명 |
|---------------------|------------------|------|
| **LaserScan** | `/scan` | RPLidar 거리 스캔 데이터 |
| **Map** | `/map` | SLAM Toolbox에서 생성된 지도 |
| **TF** | `/tf`, `/tf_static` | 좌표 변환 (map, odom, base_link 등) |
| **Odometry** | `/odom` | 로봇의 위치 및 이동 정보 |
| **Image (RGB)** | `/camera/color/image_raw` | RealSense 컬러 영상 |
| **Depth Image** | `/camera/depth/image_rect_raw` | 깊이 영상 |
| **RobotModel** | - | URDF 모델 표시 |
| **Path (선택)** | `/slam_toolbox/trajectory` | 로봇의 이동 경로 시각화 |
| **CmdVel (선택)** | `/cmd_vel` | 키보드 조작 속도 명령 확인용 |

---

## 🚀 실행 순서 예시 (권장 흐름)
```bash
# [라즈베리 터미널들]
ros2 launch scout_base scout_base.launch.py
ros2 launch rplidar_ros rplidar_a1_launch.py
ros2 launch realsense2_camera rs_launch.py       ## 생략
ros2 launch slam_toolbox online_async_launch.py

# [로컬 노트북 터미널들]
ros2 launch scout_description scout_base_description.launch.py
ros2 run rviz2 rviz2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
</details>

<details>
  
<summary> 


# 📡 라이다(LiDAR)의 위치와 방향 설정이 중요한 이유 </summary> 

## 🧩 문제 상황
- SLAM 실행 시, RViz2에서 **지도(Map)** 가 겹쳐서 보이거나,  
  **빔(빨간 선)** 이 엉뚱한 방향으로 퍼지는 현상이 발생.
- 아래 그림처럼 LiDAR 데이터가 왜곡되어 맵이 뒤틀리거나 중첩됨.

<img width="549" height="567" alt="slam 겹침" src="https://github.com/user-attachments/assets/12c3ca5e-645f-473a-95f3-c2bc78ebdc6b" />


---

## ⚙️ 원인 분석
1. **라이다의 실제 설치 위치** (로봇 중심에서 얼마나 떨어져 있는가)
2. **라이다의 방향(전방 기준 회전 각도)**  
   이 두 가지가 URDF(로봇 모델)이나 TF(좌표 변환)에서 정확히 반영되지 않았기 때문입니다.

예를 들어,
- 라이다가 실제로는 로봇의 앞쪽에 있고,
- 앞을 바라보도록 설치되어 있는데,  
  URDF 상에서는 **로봇 중심(0,0,0)** 에 위치하고 **뒤쪽을 향하고 있다면**  
  → SLAM은 “센서가 뒤를 보고 있다”고 인식하게 됩니다.

그 결과,
- 맵이 뒤집히거나 겹쳐서 표시됨  
- odom → base_link → laser 프레임 변환이 잘못되어 위치 누적 오차 발생  
- scan 데이터가 실제 환경과 맞지 않게 해석됨  

---

## ✅ 해결 방법
URDF(또는 Xacro) 파일에 **라이다의 위치와 방향을 명시적으로 설정**해야 합니다.

   xml
<!-- LiDAR 위치 및 방향 설정 예시 -->
<link name="lidar_link">
  <visual>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.05" radius="0.03"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <!-- 라이다가 로봇 중심에서 앞쪽으로 0.1m, 위로 0.2m -->
  <!-- 회전이 필요하다면 rpy 값으로 조정 (예: rpy="0 0 3.1415" → 180도 회전) -->
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>

## 핵심 포인트 요약

| 구분           | 설명                                  | 잘못된 경우         | 올바른 경우                |
| ------------ | ----------------------------------- | -------------- | --------------------- |
| **위치 (xyz)** | 라이다의 실제 물리적 설치 위치                   | 맵이 이동 중 겹침     | 맵이 안정적으로 그려짐          |
| **방향 (rpy)** | 라이다가 바라보는 방향 (보통 앞을 향해야 함)          | 빔이 뒤/옆 방향으로 뻗음 | 빔이 전방으로 고르게 분포        |
| **TF 일관성**   | odom → base_link → laser 간 좌표 관계 유지 | scan이 뒤집혀 표현됨  | 올바른 좌표 변환으로 정확한 지도 생성 |



</details>


<details>
  
<summary> 
  
# 🛠️ VMware 스냅샷 생성 방법과 중요성
스냅샷은 현재 가상 머신(VM)의 **모든 상태 (디스크, 메모리, 설정 등)**를 파일로 저장하는 기능입니다.  
나중에 문제가 생기면 이 상태로 몇 초 만에 복구할 수 있습니다.  </summary>

---

## 1. VMware 화면 예시

### 🛠️ VMware 부팅 실패 해결책

<img width="1561" height="944" alt="local날림" src="https://github.com/user-attachments/assets/5a45e95b-6f83-4901-89ec-4f5f5527366a" />

- ## 위 이미지는가상 머신의 손상
- ROS 문제가 아니라 로컬 PC의 우분투 환경 자체가 부팅되지 않고 있는 상태이므로 스냅샷이 없다면 다시 로컬 환경 재구성
- ubuntu 자체를 다시 받아야 함
- 스냅샷이 있다면 VMware Workstation에서 VM 상태 확인 및 스냅샷 기능 접근 방법을 보여줍니다.
- 이미지처럼 VM 이름과 상태를 확인 후 스냅샷 메뉴를 이용합니다.

---

## 2. 왜 스냅샷이 중요한가?

1. **빠른 복구**  
   - 시스템이 꼬이거나 설정이 잘못되어도 스냅샷으로 몇 초 만에 이전 안정 상태로 돌아갈 수 있습니다.

2. **실험/개발 안전 확보**  
   - ROS2 빌드, SLAM, Nav2, 드라이버 설치 등 중요한 실습 전에 스냅샷을 찍으면 실패해도 안전하게 복구 가능.

3. **환경 공유 및 백업**  
   - 팀원이나 다른 기기에서 동일 환경을 재현할 수 있고, VM 전체 백업 역할을 합니다.

---

## 3. 스냅샷 생성 방법

### 3-1. 가상 머신 정지 또는 일시 중지 (권장)

- **권장**: VM 내부에서 **시스템 종료(Shutdown)**  
- **빠른 방법**: VM 일시 중지(Suspend)  
- 🚨 스냅샷 중에는 VM을 사용하지 않음

### 3-2. 스냅샷 메뉴 접근

| VMware 버전 | 메뉴 경로 |
|------------|-----------|
| Workstation | VM 메뉴 → Snapshot → Take Snapshot |
| Player     | Player 메뉴 → Manage → Take Snapshot |

### 3-3. 스냅샷 정보 입력

- **Name (이름)**: ROS2_Humble_Scout_Build_Complete 등 상태를 명확히 나타내는 이름  
- **Description (설명)**: scout_base 빌드, Nav2/SLAM 설치 완료, Git 설정 완료 등

**Take Snapshot** 버튼 클릭 → 저장 완료

---

## 4. 스냅샷 확인 및 복구

| 기능 | 설명 |
|------|------|
| Snapshot Manager (관리) | VM 메뉴 → Snapshot → Snapshot Manager에서 확인/관리 |
| Go to Snapshot (복구) | 원하는 스냅샷 선택 후 **Go to** 클릭 → 즉시 선택 시점 복구 |

</details>

<details>
  
<summary> 

# 🤖 ROS2 SLAM → Nav2 자율주행 통합 실행 가이드 (ScoutMini + RPLidar) </summary> 

---

## 🗺️ 1️⃣ SLAM (지도 생성 및 저장)

### ⚙️ 단계별 설명

SLAM은 라이다 센서 데이터를 이용해 로봇이 주행하는 공간의 지도를 실시간으로 작성하는 과정입니다.  
ScoutMini + RPLidar를 사용하는 경우 다음 순서로 진행합니다.

---

### 🧩 (로컬 PC) 로봇 모델 및 시각화 실행
```bash
ros2 launch scout_description scout_base_description.launch.py
- RViz에서 로봇의 모델이 나타나는지 확인합니다. (TF, base_link 등 확인) -
```
### 🌐 (SSH - 라즈베리파이) RPLidar 실행
```
ros2 launch rplidar_ros rplidar.launch.py
LiDAR 센서가 /scan 토픽을 정상적으로 발행하는지 확인합니다.
확인은 다음 명령어로 가능
ros2 topic echo /scan
```
### 🧭 (SSH - 라즈베리파이) SLAM Toolbox 실행
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False
이제 로봇을 주행시켜 주변 환경을 스캔, RViz에서 실시간으로 지도(Map)가 생성되는지 확인
```
### 💾 (SSH - 라즈베리파이) 맵 저장
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: \"/home/eddy/ros2_ws/maps/slamdunk\"}"
정상적으로 저장되면 /home/eddy/ros2_ws/maps/ 경로에 다음 파일이 생성
slamdunk_map.yaml
slamdunk_map.pgm
```
image: slamdunk.pgm
mode: trinary
resolution: 0.05
origin: [-6.06, -2.34, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
</details>

<details>
  
<summary> 
  
# 🚗 2️⃣ Nav2 자율주행 실행 </summary> 
## 이제 생성한 맵(slamdunk_map.yaml, slamdunk_map.pgm)을 사용하여 자율주행을 수행합니다.

### 🌐 (SSH - 라즈베리파이) 라이다 재실행
```
ros2 launch rplidar_ros rplidar.launch.py
```
### 🔧 (SSH - 라즈베리파이) 로봇 Bringup 실행
```
ros2 launch scout_bringup base_bringup.launch.py
만약 scout_bringup 패키지가 없으면, 로봇 기본 TF와 odom을 담당하는 launch 파일을 대신 실행해야 
```
### 🧭 (SSH - 라즈베리파이) Nav2 실행
```
패키지가 있을 경우
ros2 launch scout_navigation2 navigation2.launch.py map:=maps/slamdunk_map.yaml
패키지가 없을 경우, 기본 Nav2 Bringup을 실행
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False map:=/home/eddy/ros2_ws/maps/slamdunk_map.yaml
```
### 💡 (선택) 초기 시작 좌표 설정
Nav2 실행 시 로봇의 시작 좌표를 지정하려면 다음 인자를 추가, RViz에서 수동으로 2D Pose Estimate 버튼을 눌러 시작 위치를 지정할 수도 있음
```
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=False \
  map:=/home/eddy/ros2_ws/maps/slamdunk_map.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_a:=0.0
```
### 🧭 (로컬 PC) 로봇 모델 및 RViz 실행
```
ros2 launch scout_description scout_base_description.launch.py
ros2 run rviz2 rviz2
```

## 📋 실행 전체 요약 순서
| 순서 | 실행 위치 | 명령어                                                                   | 설명            |
| -- | ----- | --------------------------------------------------------------------- | ------------- |
| 1  | Local | `ros2 launch scout_description scout_base_description.launch.py`      | 로봇 모델 표시      |
| 2  | SSH   | `ros2 launch rplidar_ros rplidar.launch.py`                           | 라이다 실행        |
| 3  | SSH   | `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False` | SLAM 수행       |
| 4  | SSH   | `ros2 service call /slam_toolbox/save_map ...`                        | 맵 저장          |
| 5  | SSH   | `ros2 launch scout_bringup base_bringup.launch.py`                    | 로봇 bringup 실행 |
| 6  | SSH   | `ros2 launch nav2_bringup navigation_launch.py ...`                   | Nav2 실행       |
| 7  | Local | `ros2 run rviz2 rviz2`                                                | 자율주행 시각화      |


### 📦 맵 파일 예시 (slamdunk.yaml)
```
image: slamdunk.pgm
mode: trinary
resolution: 0.05
origin: [-1.0, -2.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

</details>












