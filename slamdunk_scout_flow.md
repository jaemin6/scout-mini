<img width="4666" height="5167" alt="slamdunk_flow" src="https://github.com/user-attachments/assets/c653ace8-ccdb-49fc-ae75-f28f4e7fd939" />

# 🧭 ROS2 Node & Topic Flow — Scout Mini SLAM+Navigation System

rt](./slamdunk_flow.png)

> 본 다이어그램은 Scout Mini ROS2 기반 자율주행 시스템의 전체 토픽 흐름과 노드 간 상호작용을 나타냅니다.  
> 센서 입력부터 내비게이션, 제어, 서비스 모듈, 하드웨어까지의 모든 주요 구성요소를 포함합니다.

---

## 📡 상세 토픽 매핑 테이블

### 🔹 센서 → 내비게이션 매핑

| **소스 (Sensor)** | **토픽 (Topic)** | **목적지 (Destination)** | **용도 (Purpose)** |
|--------------------|------------------|---------------------------|---------------------|
| **LIDAR** | `/scan` | `AMCL`, `Costmap` | 위치 추정 및 장애물 회피 |
| **Camera (V4L2)** | `/image_raw` | `QR Detector` | QR 코드 인식 |
| **Odometry** | `/odom` | `AMCL`, `Controller` | 주행 거리 계산 및 자세 추정 |
| **Scout Base Node** | `/scout_status` | `Diagnostics` | 시스템 상태 모니터링 및 오류 감지 |

---

### 🔹 내비게이션 내부 구조 (Nav2 Core Components)

| **컴포넌트 (Component)** | **입력 토픽 (Input Topics)** | **출력 토픽 (Output Topics)** | **기능 (Function)** |
|--------------------------|------------------------------|-------------------------------|----------------------|
| **AMCL** | `/scan`, `/odom`, `/map` | `/amcl_pose`, `/particle_cloud` | 로봇 위치 추정 |
| **Global Planner** | `/goal_pose`, `/map` | `/global_plan` | 전역 경로 생성 |
| **Local Planner (Controller)** | `/local_costmap`, `/scan` | `/local_plan`, `/cmd_vel_nav` | 근거리 장애물 회피 및 속도 명령 |
| **Costmap** | `/scan`, `/map` | `/global_costmap/*`, `/local_costmap/*` | 장애물 맵 생성 및 갱신 |
| **BT Navigator (Behavior Tree)** | `/goal_pose`, `/amcl_pose` | `/cmd_vel_nav` | 목표까지 동작 시퀀스 관리 |

---

### 🔹 서비스 모듈 (Service Modules)

| **모듈 (Module)** | **트리거 (Trigger)** | **액션 (Action)** | **결과 (Result)** |
|--------------------|----------------------|--------------------|-------------------|
| **Nav2 Commander** | `/room_command` | `/goal_pose` 발행 | 목적지 설정 및 내비게이션 시작 |
| **QR Detector** | `/image_raw` | `/qr_check_command` 발행 | QR 코드 인식 및 명령 전달 |
| **Robot Rotator** | `/qr_check_command` | `/cmd_vel`로 45도 회전 | QR 재스캔 및 방향 보정 |
| **AMCL Reset** | QR 인식 성공 | `/amcl_reset_command` 발행 | 위치 재설정 (Localization Reset) |

---

### 🔹 하드웨어 제어 (Hardware Control Layer)

| **디바이스 (Device)** | **제어 토픽 (Control Topic)** | **센서 토픽 (Sensor Feedback)** | **용도 (Purpose)** |
|------------------------|-------------------------------|----------------------------------|---------------------|
| **모터 드라이버 (Motor Driver)** | `/cmd_vel` | `/odom` | 주행 제어 |
| **RFID-RC522** | - | `/rfid_card_uid` | 사용자 인증 / 도착지 확인 |
| **RGB LED** | `/light_control` | - | 상태 표시 (대기/이동/완료 등) |
| **Buzzer** | `/light_control` | - | 알림 및 이벤트 음향 |
| **초음파 센서 (Ultrasonic)** | - | `/ultrasonic_distance` | 장애물 / 배달 완료 감지 |

---

### 🔹 상위 노드 및 통신 인터페이스

| **모듈** | **설명** |
|-----------|-----------|
| **Scout Base Node** | CAN 통신 기반 하위 하드웨어 제어 |
| **V4L2 Camera Node** | 실시간 영상 입력 (QR, 객체 인식 등) |
| **LIDAR Node (RPLIDAR)** | 거리 데이터 스트리밍 `/scan` |
| **RFID Node (RC522)** | 카드 UID 송신 `/rfid_card_uid` |
| **Diagnostics Aggregator** | 전체 시스템 상태 모니터링 `/diagnostics` |
| **TF / TF2** | 좌표 변환 브로드캐스트 `/tf`, `/tf_static` |

---

## ⚙️ Summary

이 시스템은 다음과 같은 전체 흐름을 가집니다:

1. **센서 입력 단계**: LiDAR, Camera, RFID, Odometry 등에서 데이터 수집  
2. **내비게이션 엔진 (Nav2)**: AMCL, Planner, Controller를 통해 자율 주행 경로 계산  
3. **서비스 모듈**: QR 인식, 위치 초기화, 목적지 설정, 회전 등 이벤트 처리  
4. **하드웨어 제어**: `/cmd_vel`을 통해 실제 모터, LED, Buzzer를 제어  
5. **피드백 루프**: `/odom`, `/scan`, `/diagnostics`를 통해 상태 피드백 및 수정

---
