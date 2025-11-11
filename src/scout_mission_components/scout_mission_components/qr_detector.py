#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage 
from std_msgs.msg import String
import cv2
from pyzbar import pyzbar
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# --- 토픽 및 상수 정의 ---
QR_COMMAND_TOPIC = "/qr_check_command"
AMCL_RESET_COMMAND_TOPIC = "/amcl_reset_command"
ROBOT_ROTATE_COMMAND_TOPIC = "/robot_rotate_command"
QR_DETECTION_SUCCESS_TOPIC = "/qr_detection_success"

COMMAND_TO_QR_MAP = {
    "go_room501": "501",
    "go_home": "home",  
    "go_room502": "502",
    "go_room503": "503",
}
QR_DATA_TO_POSE = {}

# QR 코드 검사 시간 제한
QR_CHECK_TIMEOUT_SEC = 10.0


class QrDetector(Node):
    # 컴포넌트 클래스: __init__은 **kwargs를 통해 NodeOptions를 받습니다.
    def __init__(self, **kwargs):
        # super().__init__ 호출 시 **kwargs 전달 필수
        super().__init__('qr_detector', **kwargs)
        
        self.load_room_coordinates()
        
        self.expected_qr_data = None  
        self.is_qr_detected = False 
        self.qr_check_active = False # QR 검사 활성화 상태 플래그
        self.timeout_timer = None    # 타임아웃 타이머 객체
        self.last_command = None     # 마지막으로 받은 명령을 저장
        
        # 1. 카메라 구독
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  
            self.image_callback,
            10)
        
        # 2. 목표 명령 구독 (RoomNavigator -> QR Detector)
        self.command_subscription = self.create_subscription(
            String,
            QR_COMMAND_TOPIC, 
            self.command_callback,
            10
        )
        
        # 3. AMCL 리셋 명령 발행
        self.amcl_reset_pub = self.create_publisher(
            String,
            AMCL_RESET_COMMAND_TOPIC,
            10
        )
        
        # 4. 로봇 회전 명령 발행
        self.rotate_command_pub = self.create_publisher(
            String,
            ROBOT_ROTATE_COMMAND_TOPIC,
            10
        )
        # 5. QR 인식 성공 신호 발행 Publisher
        self.qr_success_pub = self.create_publisher(
            String,
            QR_DETECTION_SUCCESS_TOPIC,
            10
        )
        self.get_logger().info(f'QR Detector Component started. Publishing rotation commands on {ROBOT_ROTATE_COMMAND_TOPIC}...')

    def load_room_coordinates(self):
        """rooms.yaml 파일을 읽어 QR 코드에 해당하는 좌표를 로드합니다."""
        global QR_DATA_TO_POSE
        try:
            package_share = get_package_share_directory('scout_robot')
            yaml_path = os.path.join(package_share, 'rooms.yaml')
            
            with open(yaml_path, 'r') as f:
                rooms_data = yaml.safe_load(f)['rooms']
                
            for cmd, qr_data in COMMAND_TO_QR_MAP.items():
                room_name = cmd.replace("go_", "")
                if room_name in rooms_data:
                    QR_DATA_TO_POSE[qr_data] = rooms_data[room_name]
                    
            self.get_logger().info("✅ rooms.yaml에서 QR 목표 좌표 로드 완료.")
        
        except FileNotFoundError:
            self.get_logger().error(f"rooms.yaml 파일을 찾을 수 없습니다: {yaml_path}")
            
    # QR 검사 타임아웃 처리 함수 수정
    def check_qr_timeout(self):
        """10초 후 타이머에 의해 호출됩니다. QR 인식 성공 여부를 최종 확인합니다."""
        
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        
        self.qr_check_active = False # 검사 비활성화

        if self.is_qr_detected:
            # QR 코드 인식 성공은 이미 image_callback에서 처리되었으므로 추가 행동 불필요
            return
        else:
            # 10초 동안 QR 코드를 인식하지 못했을 경우 (QR 인식 실패)
            self.get_logger().error(f"❌❌❌ QR 코드 인식 실패! {QR_CHECK_TIMEOUT_SEC}초 동안 '{self.expected_qr_data}'를 찾지 못했습니다. ❌❌❌")
            
            # 1. Robot Rotator Node에게 회전 명령과 목표 정보를 함께 발행
            if self.last_command is not None:
                rotate_msg = String()
                # 회전 명령과 목표 명령을 'ROTATE_LEFT_45:go_room501' 형태로 보냄
                rotate_msg.data = f"ROTATE_LEFT_45:{self.last_command}"
                self.rotate_command_pub.publish(rotate_msg)
                self.get_logger().warn(f"🔄 Robot Rotator Node에게 45도 회전 명령을 발행했습니다. 목표: {self.last_command}")
            else:
                self.get_logger().error("⚠️ last_command 정보가 없어 회전 명령을 보낼 수 없습니다.")
                
            # 2. 다음 명령을 기다리기 위해 QR 감지 비활성화 (expected_qr_data는 유지할 필요가 없음, command_callback에서 재설정됨)
            self.expected_qr_data = None # 재검사 명령을 기다리는 대기 상태로 전환


    def command_callback(self, msg: String):
        """/qr_check_command 토픽을 구독하여 기대 QR 코드를 동적으로 설정"""
        command = msg.data.strip()
        
        if command in COMMAND_TO_QR_MAP:
            # 명령을 last_command에 저장
            self.last_command = command 
            self.expected_qr_data = COMMAND_TO_QR_MAP[command]
            self.is_qr_detected = False
            self.qr_check_active = True # 검사 활성화
            
            self.get_logger().info(f"✅ QR 검사 명령 수신: '{command}'. 기대 QR 코드가 '{self.expected_qr_data}'(으)로 설정되었습니다.")
            
            # 10초 타이머 시작
            if self.timeout_timer is not None:
                self.timeout_timer.cancel() # 이전 타이머 취소
            self.timeout_timer = self.create_timer(QR_CHECK_TIMEOUT_SEC, self.check_qr_timeout)
            self.get_logger().warn(f"⏳ QR 스캔 모드 활성화. {QR_CHECK_TIMEOUT_SEC}초 동안 스캔을 시작합니다.")
            
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 QR 명령 수신: {command}.")


    def image_callback(self, data: CompressedImage):
        """QR 코드를 감지하고 성공 시 AMCL 재설정 명령을 발행합니다."""
        
        # QR 인식 실패 후 expected_qr_data가 None이 되므로, 여기서 완전히 동작 중지 상태가 유지됩니다.
        if not self.qr_check_active or self.expected_qr_data is None or self.is_qr_detected:
            # ... (비활성화 상태 디버그 뷰 로직 유지) ...
            try:
                np_arr = np.frombuffer(data.data, dtype=np.uint8)
                current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if current_frame is not None:
                    status_text = f"Target: {self.expected_qr_data if self.expected_qr_data else 'None'}. Scanning {'ON' if self.qr_check_active else 'OFF'}"
                    cv2.putText(current_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.imshow(f"QR Detector View", current_frame)
                    cv2.waitKey(1)
            except Exception as e:
                # self.get_logger().error(f'Image data decoding/display failed: {e}') # 컴포넌트 환경에서 너무 많은 에러 메시지 방지
                pass
            return
            
        try:
            np_arr = np.frombuffer(data.data, dtype=np.uint8)
            current_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if current_frame is None: return

            decoded_objects = pyzbar.decode(current_frame)
            
            for obj in decoded_objects:
                decoded_data = obj.data.decode("utf-8")
                
                if decoded_data == self.expected_qr_data:
                    
                    if not self.is_qr_detected:
                        self.is_qr_detected = True # 감지 상태로 변경
                        self.qr_check_active = False # 검사 즉시 비활성화
                        
                        self.get_logger().warn(f"✅✅✅ QR 코드 '{self.expected_qr_data}' 인식 성공! AMCL 재설정을 요청합니다. ✅✅✅")
                        
                        # 타이머 즉시 취소
                        if self.timeout_timer is not None:
                            self.timeout_timer.cancel()
                            self.timeout_timer = None
                        
                        # AMCL Reset Node에게 명령 발행
                        reset_msg = String()
                        reset_msg.data = decoded_data  
                        self.amcl_reset_pub.publish(reset_msg)
                        
                        # 초음파 노드에게 인식 성공 신호 발행
                        success_msg = String()
                        success_msg.data = f"QR_SUCCESS:{decoded_data}" 
                        self.qr_success_pub.publish(success_msg)
                        self.get_logger().info(f"Published QR Success Signal: {success_msg.data}")
                    
                        # QR 코드 감지 성공 후 스캔 중지
                        self.expected_qr_data = None
                        
                # ... (기타 QR 코드 표시 로직 유지) ...
                (x, y, w, h) = obj.rect
                color = (0, 255, 0) if decoded_data == self.expected_qr_data else (0, 0, 255)
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(current_frame, decoded_data, (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            cv2.imshow(f"QR Detector View", current_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

# ----------------------------------------------------
# 🌟🌟🌟 컴포넌트 등록을 위한 필수 진입점 🌟🌟🌟
# ----------------------------------------------------
def create_node():
    """QrDetector 컴포넌트 인스턴스를 생성하고 반환합니다."""
    # QrDetector 클래스 인스턴스 생성 및 반환
    return QrDetector() 

# 기존 main 함수, rclpy.init(), rclpy.spin(), rclpy.shutdown() 및 if __name__ == '__main__': 구문은 제거합니다.
# 컴포넌트 컨테이너가 이 함수를 호출하여 노드를 로드합니다.
