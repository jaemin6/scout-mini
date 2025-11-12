graph TD
    A[room_command] --> B[room_navigator]
    
    B --> C[robot_rotator_node]
    C --> D[qr_detector_node]
    
    D --> E{qr_detection_success}
    E -->|성공| F[PICO_MCU]
    E -->|실패| G[retry_room_command]
    G --> B
    
    F --> H{RFID_인식}
    H -->|성공| I[LED_성공_부저_성공]
    I --> J[rfid_detection_success]
    J --> K[speaker_command]
    
    H -->|실패| L[LED_실패_부저_실패]
    L --> M[retry_room_command]
    M --> B
    
    K --> N[amcl_reset_node]
    N --> O[basic_navigator]
    O --> P[go_home]