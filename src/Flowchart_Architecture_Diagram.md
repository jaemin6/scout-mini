```mermaid
---
config:
  layout: elk
---
flowchart TD
 subgraph B["하드웨어 인터페이스"]
        B1["Scout Base Node<br>/cmd_vel → 모터제어<br>/odom<br>/scout_status"]
        B2["LIDAR Node<br>/scan"]
        B3["V4L2 Camera<br>/image_raw<br>/camera_info"]
        B4["초음파 센서<br>배달 완료 감지"]
        B5["RFID-RC522<br>카드 인식"]
        B6["RGB LED + Buzzer<br>/light_control"]
  end
 subgraph C["내비게이션 스택"]
        C1["Nav2 Bringup<br>맵: slamdunk.yaml"]
        C2["AMCL<br>/amcl_pose<br>/particle_cloud"]
        C3["Costmap<br>/global_costmap/*<br>/local_costmap/*"]
        C4["Planner Server<br>/global_plan"]
        C5["Controller Server<br>/cmd_vel_nav<br>/local_plan"]
        C6["Behavior Tree<br>/behavior_tree_log"]
        C7["TF 변환<br>/tf<br>/tf_static"]
        C8["Map Server<br>/map"]
  end
 subgraph D["인지 및 서비스"]
        D1["QR Detector<br>/image_raw → /qr_check_command"]
        D2["Nav2 Commander<br>/room_command → /goal_pose"]
        D3["Robot Rotator<br>/robot_rotate_command → /cmd_vel"]
        D4["AMCL Reset<br>/amcl_reset_command"]
        D5["Velocity Smoother<br>/cmd_vel_nav → /cmd_vel"]
  end
 subgraph E["상태 모니터링"]
        E1["RViz2<br>시각화 인터페이스"]
        E2["Diagnostics<br>/diagnostics<br>/scout_status"]
        E3["Parameter Events<br>/parameter_events"]
  end
 subgraph F["음성 출력 시스템"]
        F1["음성 출력 노드<br>/speaker_command → 블루투스 스피커"]
        F2["블루투스 스피커<br>상태 안내 음성 출력"]
        F3["음성 명령 리스트<br>도착안내/배달시작/완료안내 등"]
  end
    A["Scout Mini 시스템"] --> B & C & D & E & F
    B2 -- /scan --> C2
    B3 -- /image_raw --> D1
    B1 -- /odom --> C2
    B1 -- /scout_status --> E2
    C1 --> C2 & C3 & C4 & C5 & C6 & C8
    C2 --> C4 & C5 & E1
    C3 --> C4 & C5 & E1
    C4 --> C6 & E1
    C5 --> C6 & E1
    C6 --> D5
    C8 --> C2 & C4 & E1
    D2 -- /room_command --> C6
    D1 -- /qr_check_command --> D3 & D4
    D3 -- /cmd_vel --> B1
    D4 --> C2
    D5 -- /cmd_vel --> B1
    B2 --> E1
    C7 --> E1
    B5 --> B6
    B4 -- 배달 완료 --> G["웹 서버"]
    D2 -- 목적지 설정 시 --> F1
    D1 -- QR 인식 시 --> F1
    B4 -- 배달 완료 시 --> F1
    C6 -- 주행 상태 변경 시 --> F1
    F1 -- 음성 출력 --> F2
    H["사용자 명령"] -- /room_command --> D2
    I["Teleop"] -- /cmd_vel_teleop --> D5
    J["시스템 초기화"] --> K["CAN 통신 설정"]
    K --> L["블루투스 연결"]
    L --> M["노드 실행"]
    style B fill:#e1f5fe
    style C fill:#f3e5f5
    style D fill:#e8f5e8
    style E fill:#fff3e0
    style F fill:#ffebee
