flowchart TD
    A[Scout Mini 시스템] --> B[하드웨어 인터페이스]
    A --> C[내비게이션 스택]
    A --> D[인지 및 서비스]
    A --> E[상태 모니터링]
    
    subgraph B [하드웨어 인터페이스]
        B1[Scout Base Node<br>/cmd_vel → 모터제어<br>/odom<br>/scout_status]
        B2[LIDAR Node<br>/scan]
        B3[V4L2 Camera Node<br>/image_raw<br>/camera_info<br>video_device: /dev/video0<br>image_size: 640x480<br>pixel_format: YUYV<br>frame_rate: 30<br>image_transport: compressed]
        B4[초음파 센서<br>배달 완료 감지]
        B5[RFID-RC522<br>카드 인식]
        B6[RGB LED + Buzzer<br>/light_control]
    end
    
    subgraph C [내비게이션 스택]
        C1[Nav2 Bringup<br>/map: /home/eddy/ros2_ws/maps/slamdunk.yaml]
        C2[AMCL<br>/amcl_pose<br>/particle_cloud<br>/amcl/transition_event]
        C3[Costmap<br>/global_costmap/*<br>/local_costmap/*]
        C4[Planner Server<br>/global_plan<br>/planner_server/transition_event]
        C5[Controller Server<br>/cmd_vel_nav<br>/local_plan<br>/controller_server/transition_event]
        C6[Behavior Tree<br>/behavior_tree_log<br>/bt_navigator/transition_event]
        C7[TF 변환<br>/tf<br>/tf_static<br>/joint_states]
        C8[Map Server<br>/map<br>/map_server/transition_event]
    end
    
    subgraph D [인지 및 서비스]
        D1[QR Detector<br>/image_raw → /qr_check_command]
        D2[Nav2 Commander<br>/room_command → /goal_pose]
        D3[Robot Rotator<br>/robot_rotate_command → /cmd_vel]
        D4[AMCL Reset<br>/amcl_reset_command → 초기위치 설정]
        D5[Velocity Smoother<br>/cmd_vel_nav → /cmd_vel]
    end
    
    subgraph E [상태 모니터링]
        E1[RViz2<br>/marker<br>/robot_description<br>/cost_cloud<br>/plan<br>/particle_cloud]
        E2[Diagnostics<br>/diagnostics<br>/scout_status]
        E3[Parameter Events<br>/parameter_events]
    end
    
    %% 센서 → 내비게이션 흐름
    B2 --> C2
    B3 --> D1
    B1 -->|/odom| C2
    B1 -->|/scout_status| E2
    
    %% 내비게이션 내부 흐름
    C1 --> C2
    C1 --> C3
    C1 --> C4
    C1 --> C5
    C1 --> C6
    C1 --> C8
    C2 --> C4
    C2 --> C5
    C3 --> C4
    C3 --> C5
    C4 --> C6
    C5 --> C6
    C6 --> D5
    C8 --> C2
    C8 --> C4
    
    %% 서비스 모듈 흐름
    D2 -->|/room_command| C6
    D1 -->|/qr_check_command| D3
    D1 -->|/qr_check_command| D4
    D3 -->|/cmd_vel| B1
    D4 --> C2
    D5 -->|/cmd_vel| B1
    
    %% 상태 모니터링 흐름
    B2 --> E1
    C2 --> E1
    C3 --> E1
    C4 --> E1
    C5 --> E1
    C7 --> E1
    C8 --> E1
    
    %% 하드웨어 제어 흐름
    B5 --> B6
    B4 -->|배달 완료| F[웹 서버]
    
    %% 사용자 인터페이스
    G[사용자 명령] -->|ros2 topic pub --once /room_command<br>std_msgs/String "data: 'go_room501'"<br>--qos-reliability reliable| D2
    H[Teleop] -->|/cmd_vel_teleop| D5
    
    %% V4L2 카메라 상세 설정
    I[V4L2 설정] --> B3
    J[USB Webcam<br>/dev/video0] --> B3
    K[이미지 설정<br>640x480 YUYV 30fps<br>compressed] --> B3
    
    %% 초기화 시퀀스
    L[시스템 초기화] --> M[CAN 통신 설정]
    M --> N[sudo ip link set can0 type can bitrate 500000<br>sudo ip link set up can0]
    N --> O[노드 실행]

    style B fill:#e1f5fe
    style C fill:#f3e5f5
    style D fill:#e8f5e8
    style E fill:#fff3e0