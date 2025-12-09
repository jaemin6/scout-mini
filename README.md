# scout-mini
<details>
  
<summary> 
  

# 🤖 Scout Robot Navigation System   </summary> 
ROS 2 / Nav2 기반 자율주행 + QR 코드 기반 도착 검증 시스템

---

## 📌 1. 프로젝트 개요

이 프로젝트는 **ROS 2 (Humble/Iron)** 및 **Nav2 스택**을 기반으로,  
자율 이동 로봇 **Scout**가 지정된 목표 지점(방 또는 홈)에 도착했는지  
**QR 코드 인식을 통해 최종 검증**하는 시스템입니다.

| 항목 | 설명 |
|-----|------|
| **목표** | Nav2 자율 주행 후, 목표 지점의 QR 코드 인식으로 도착 확정 |
| **주요 기능** | `/room_command` 토픽으로 목표 설정 → QR 코드 인식으로 완료 확인 |
| **ROS 버전** | ROS 2 Humble / Iron (환경에 맞게 기재) |

---

## 🛠️ 2. 빌드 매뉴얼 (Build Manual)

본 패키지는 **ROS 2 워크스페이스(ros2_ws)** 내에서 **colcon**으로 빌드됩니다.

---

### 2.1. 코드 클론 및 워크스페이스 이동

```bash
# 1. src 디렉토리로 이동
cd ~/ros2_ws/src

# 2. 프로젝트 저장소 클론
git clone https://github.com/jaemin6/scout-mini.git

# 3. 워크스페이스 루트로 이동
cd ~/ros2_ws

# 4. 패키지 빌드 (symlink 사용)
colcon build --packages-select scout_robot --symlink-install

```
</details>
------------------------------------------------------------

 <details>
<summary>

# 🤖 자율주행 로봇 네비게이션 시스템 구조   </summary>

## Nav2 BasicNavigator + 토픽 기반 상태 제어 알고리즘 사용

### 1. 🗺️ 네비게이션 기본 원리 및 커스텀 구현
🔍 RViz 표준 동작 vs 본 시스템  
| 기능           | RViz 방식                           | 본 시스템 방식                                       | 비고       |
| ------------ | --------------------------------- | ---------------------------------------------- | -------- |
| **초기 위치 설정** | 2D Pose Estimate → `/initialpose` | `setInitialPose()` 또는 `/initialpose` 직접 발행     | AMCL 초기화 |
| **목표 위치 설정** | 2D Goal Pose 클릭                   | `rooms.yaml` 에서 좌표 읽기 → `navigator.goToPose()` | 완전 자동화   |
| **경로 계획/실행** | RViz에서 실행                         | BasicNavigator가 Nav2 액션 서버와 통신                 | 동일 기능    |

### ✨ 2. 좌표 관리 및 변환

시스템은 rooms.yaml 파일을 사용하여 목표 좌표를 관리합니다.
이는 RViz에서 2D Goal Pose를 찍는 행위를 완전 자동화한 것입니다.

A. 좌표 파일 생성 과정

AMCL 은 /amcl_pose 로 로봇 위치를 발행

좌표 정보:

위치: x, y

방향: 쿼터니언 z, w

쿼터니언 → Yaw(θ) 변환 후 YAML 저장

B. 쿼터니언 → Yaw 변환 공식

Nav2/ROS2에서 사용하는 표준 변환:

𝜃
=
atan2
(
2
(
𝑞
𝑤
𝑞
𝑧
+
𝑞
𝑥
𝑞
𝑦
)
,
 
1
−
2
(
𝑞
𝑦
2
+
𝑞
𝑧
2
)
)
θ=atan2(2(q
w
	​

q
z
	​

+q
x
	​

q
y
	​

),1−2(q
y
2
	​

+q
z
2
	​

))

2D 네비게이션에서
qx = 0, qy = 0 이므로 최종 식은:

𝜃
=
atan2
(
2
𝑞
𝑤
𝑞
𝑧
,
  
1
−
2
𝑞
𝑧
2
)
θ=atan2(2q
w
	​

q
z
	​

,1−2q
z
2
	​

)

### 🔄 3. 토픽 기반 상태 제어 알고리즘 (핵심 구조)

본 시스템은 다음과 같은 명확한 순차 상태 제어 흐름을 갖습니다:

구독 → 노드 동작 → 완료 토픽 발행 → 노드 대기 → 다음 노드 실행

이 방식은 Nav2 안정성을 크게 향상시키고,
QR 스캔 / 회전 / AMCL 초기화 등 복잡한 기능 간 충돌을 방지합니다.

✔️ 전체 노드 & 토픽 플로우 요약
| 노드(파일)                                     | 구독 토픽(작동 시작)            | 발행 토픽(임무 위임)                                     | 수행 임무                           |
| ------------------------------------------ | ----------------------- | ------------------------------------------------ | ------------------------------- |
| **RoomNavigator** (`nav2_commander.py`)    | `/room_command`         | `/qr_check_command`                              | Nav2 이동 수행 (BasicNavigator)     |
| **QrDetector** (`qr_detector_node.py`)     | `/qr_check_command`     | `/amcl_reset_command` 또는 `/robot_rotate_command` | 10초 QR 스캔 → 성공/실패 분기            |
| **AmclResetter** (`amcl_reset_node.py`)    | `/amcl_reset_command`   | `/room_command` *(home 일 때만)*                    | AMCL 위치 재설정 (`/initialpose` 발행) |
| **RobotRotator** (`robot_rotator_node.py`) | `/robot_rotate_command` | `/qr_check_command` 또는 `/room_command`           | 45도 회전 × 최대 8회 → 실패 시 go_home   |


### 🔁 4. 임무 순환의 특징

✔️ 1) 상태 분리

로봇 이동 중(RoomNavigator 동작)
→ QR 스캔, 회전 등 다른 노드 실행 없음

✔️ 2) 동기적 처리

Nav2 액션 완료까지
rclpy.spin_once() 로 안정적 대기

✔️ 3) 노드 자동 대기 상태

노드는 트리거 토픽을 받을 때까지만 활성

발행 후 즉시 대기(Idle) 상태 전환

</details>
