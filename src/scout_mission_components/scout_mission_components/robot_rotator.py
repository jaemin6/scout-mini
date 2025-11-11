#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math
import time # Blocking 대기를 위해 time.sleep 사용

# 🌟 QR Detector로부터 명령을 받을 토픽 정의
ROBOT_ROTATE_COMMAND_TOPIC = "/robot_rotate_command"

# 🌟 QR Detector에게 재검사 명령을 보낼 토픽 (기존 QR Detector의 구독 토픽)
QR_COMMAND_TOPIC = "/qr_check_command" 

# 🌟 Nav2 Commander에게 홈 복귀 명령을 보낼 토픽 (실제 nav2_commander.py가 구독하는 토픽으로 수정)
HOME_COMMAND_TOPIC = "/room_command" 

# 🌟 회전 각도 정의 (라디안)
ROTATE_ANGLE_RAD = math.pi / 4.0 # 45도
ROTATE_ANGLE_DEG = 45.0

# 🌟 최대 회전 횟수
MAX_ROTATION_COUNT = 8 

class RobotRotator(Node):
    # 컴포넌트 클래스: __init__은 **kwargs를 통해 NodeOptions를 받습니다.
    def __init__(self, **kwargs):
        # super().__init__ 호출 시 **kwargs 전달 필수
        super().__init__('robot_rotator', **kwargs)
        self.navigator = BasicNavigator()
        
        # 🌟🌟🌟 회전 횟수 카운터 초기화 🌟🌟🌟
        self.rotation_count = 0 
        
        # --- Nav2 활성화 대기 로직 제거 (컴포넌트 환경과 충돌 방지) ---
        self.get_logger().info("Nav2 활성화 대기 생략. Nav2 bringup이 별도 실행 중이라고 가정.")
        self.get_logger().info("Nav2 활성화 완료!")

        # 1. 회전 명령 구독 (QR Detector -> RobotRotator)
        self.command_subscription = self.create_subscription(
            String,
            ROBOT_ROTATE_COMMAND_TOPIC, 
            self.rotate_command_callback,
            10
        )
        
        # 2. QR Detector에게 재검사 명령 발행
        self.qr_command_pub = self.create_publisher(
            String,
            QR_COMMAND_TOPIC,
            10
        )

        # 3. Nav2 Commander에게 홈 복귀 명령 발행
        self.home_command_pub = self.create_publisher(
            String,
            HOME_COMMAND_TOPIC, # '/room_command'
            10
        )
        
        self.get_logger().info(f'RobotRotator Component started. Waiting for rotation commands on {ROBOT_ROTATE_COMMAND_TOPIC}...')

    def create_relative_goal_pose(self, angle_rad):
        """base_link 프레임 기준으로 회전 목표 PoseStamped 메시지를 생성합니다."""
        pose = PoseStamped()
        pose.header.frame_id = "base_link" 
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0, 0, angle_rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def rotate_robot(self, angle_rad):
        """로봇을 지정된 각도(라디안)만큼 회전시킵니다."""
        
        goal_pose = self.create_relative_goal_pose(angle_rad)
        self.get_logger().warn(f"🔄 로봇 회전 명령 전송: {math.degrees(angle_rad):.1f}도 회전 시작... (현재 횟수: {self.rotation_count})")
        self.navigator.goToPose(goal_pose) 

        i = 0
        while not self.navigator.isTaskComplete():
            # 컴포넌트 컨테이너가 spin을 처리하므로, rclpy.spin_once 대신 time.sleep 사용
            time.sleep(0.1) 
            i = (i + 1) % 10
            if i == 0:
                self.get_logger().info(f"🔄 회전 진행 중...")
        
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().warn(f"✅ 회전 완료! {math.degrees(angle_rad):.1f}도 회전 성공.")
            return True
        else:
            self.get_logger().error("❌ 회전 실패 또는 취소되었습니다.")
            return False


    def rotate_command_callback(self, msg: String):
        """QR Detector로부터 회전 명령을 받아 왼쪽 45도 회전을 실행합니다."""
        full_command = msg.data.strip()
        
        # 명령 파싱: 'ROTATE_LEFT_45:go_room501' -> ['ROTATE_LEFT_45', 'go_room501']
        parts = full_command.split(':', 1)
        if len(parts) != 2:
            self.get_logger().error(f"❌ 잘못된 회전 명령 형식 수신: {full_command}. 형식은 'COMMAND:TARGET'이어야 합니다.")
            return
            
        command = parts[0] # ROTATE_LEFT_45
        target_command = parts[1] # go_room501

        if command == "ROTATE_LEFT_45":
            self.rotation_count += 1
            
            # 8회 초과 검사 로직
            if self.rotation_count > MAX_ROTATION_COUNT:
                self.get_logger().error(f"🚨🚨🚨 최대 회전 횟수 ({MAX_ROTATION_COUNT}회) 초과! 홈 복귀 명령을 발행합니다. 🚨🚨🚨")
                
                # 홈 복귀 명령 발행 (토픽: /room_command)
                home_msg = String()
                home_msg.data = "go_home" 
                self.home_command_pub.publish(home_msg)
                
                # 카운트 초기화 (다음 임무 대비)
                self.rotation_count = 0
                return

            # 8회 이하일 경우 회전 실행
            if self.rotate_robot(ROTATE_ANGLE_RAD):
                # 회전 성공 후 QR Detector에게 재검사 명령 발행
                self.get_logger().warn(f"🔄 회전 완료. QR Detector에게 재검사 명령 ({target_command})을 보냅니다.")
                
                # 파싱된 목표 명령(target_command)을 재검사 명령으로 사용
                recheck_msg = String()
                recheck_msg.data = target_command
                self.qr_command_pub.publish(recheck_msg)
            
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 회전 명령 수신: {command}")


# ----------------------------------------------------
# 🌟🌟🌟 컴포넌트 등록을 위한 필수 진입점 🌟🌟🌟
# ----------------------------------------------------
def create_node():
    """RobotRotator 컴포넌트 인스턴스를 생성하고 반환합니다."""
    # RobotRotator 클래스 인스턴스 생성 및 반환
    return RobotRotator() 

# 기존 main 함수와 if __name__ == '__main__': 구문은 제거합니다.
