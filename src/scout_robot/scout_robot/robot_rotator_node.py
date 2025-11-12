#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rclpy.action
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import math

# 🌟 액션 메시지 임포트 (사용자 정의 인터페이스 패키지에서)
from scout_robot_interfaces.action import RotateRobot 

# 🌟 QR Detector에게 재검사 명령을 보낼 토픽 (토픽 유지)
QR_COMMAND_TOPIC = "/qr_check_command"

# 🌟 Nav2 Commander에게 홈 복귀 명령을 보낼 토픽 (토픽 유지)
HOME_COMMAND_TOPIC = "/room_command"

# 🌟 회전 각도 정의 (라디안)
ROTATE_ANGLE_RAD = math.pi / 4.0 # 45도

# 🌟 최대 회전 횟수
MAX_ROTATION_COUNT = 8

class RobotRotator(Node):
    def __init__(self):
        super().__init__('robot_rotator_node')
        self.navigator = BasicNavigator()
        
        # 🌟🌟🌟 회전 횟수 카운터 초기화 🌟🌟🌟
        self.rotation_count = 0
        self._goal_handle = None # 현재 처리 중인 액션 목표 핸들러

        # --- Nav2 활성화까지 대기 ---
        self.get_logger().info("Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 활성화 완료!")

        # 1. 🌟🌟🌟 회전 명령을 액션 서버로 대체 🌟🌟🌟
        self._action_server = rclpy.action.ActionServer(
            self,
            RotateRobot,
            'rotate_robot',
            self.execute_callback, # 목표 실행 함수
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        
        # 2. QR Detector에게 재검사 명령 발행 (토픽 유지)
        self.qr_command_pub = self.create_publisher(
            String,
            QR_COMMAND_TOPIC,
            10
        )

        # 3. Nav2 Commander에게 홈 복귀 명령 발행 (토픽 유지)
        self.home_command_pub = self.create_publisher(
            String,
            HOME_COMMAND_TOPIC,
            10
        )
        
        self.get_logger().info(f'RobotRotator Node started. Action Server /rotate_robot active.')


    def create_relative_goal_pose(self, angle_rad):
        """base_link 기준으로 상대적인 회전 목표 PoseStamped를 생성합니다."""
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

    def rotate_robot(self, goal_handle, angle_rad):
        """
        로봇을 지정된 각도(라디안)만큼 회전시키고 액션 피드백을 처리합니다.
        Nav2의 goToPose를 이용한 상대 회전입니다.
        """
        goal_pose = self.create_relative_goal_pose(angle_rad)
        
        angle_deg = math.degrees(angle_rad)
        self.get_logger().warn(f"🔄 로봇 회전 명령 전송: {angle_deg:.1f}도 회전 시작... (현재 횟수: {self.rotation_count})")
        self.navigator.goToPose(goal_pose)

        # 이동 완료 대기 및 취소 요청 확인
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                return False # 취소됨

            # 🌟 피드백 발행 (회전 작업은 진행률 피드백이 어렵기에, count 정보만 보냄)
            feedback_msg = RotateRobot.Feedback()
            feedback_msg.current_count = self.rotation_count
            goal_handle.publish_feedback(feedback_msg)
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Nav2 결과 확인
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().warn(f"✅ 회전 완료! {angle_deg:.1f}도 회전 성공.")
            return True
        else:
            self.get_logger().error(f"❌ 회전 실패 또는 취소되었습니다. 결과: {result.name}")
            return False

    # --- 🌟 액션 서버 콜백 함수 🌟 ---
    
    def goal_callback(self, goal_request):
        """목표 요청 수락/거부 결정"""
        # 현재는 모든 유효한 회전 요청을 수락
        if goal_request.angle_deg in [-45.0, 45.0]: # 예를 들어, 45도 회전만 받도록 제한
             self.get_logger().info(f"회전 목표 수락: {goal_request.angle_deg}도")
             return rclpy.action.GoalResponse.ACCEPT
        else:
             self.get_logger().error(f"요청된 회전 각도가 유효하지 않음: {goal_request.angle_deg}도")
             return rclpy.action.GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle):
        """목표가 수락된 후 실행될 핸들러 등록"""
        # 이전 목표가 있다면 취소하고 새 목표 수락 (여기서는 한 번의 명령이 하나의 회전 단계를 의미하므로, 이전 카운터는 유지)
        self._goal_handle = goal_handle
        # 비동기적으로 실행 콜백 호출
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        """목표 실행 로직 (실제 회전 실행 및 후속 명령 발행)"""
        self.get_logger().info('회전 목표 실행 시작...')
        request = goal_handle.request
        
        # 1. 요청 처리 및 카운트 증가
        angle_deg = request.angle_deg
        angle_rad = math.radians(angle_deg)
        target_command = request.target_command
        
        # 액션이 실행되면 회전 횟수를 증가
        self.rotation_count += 1
        
        result_msg = RotateRobot.Result()

        # 2. 🌟🌟🌟 최대 횟수 초과 검사 🌟🌟🌟
        if self.rotation_count > MAX_ROTATION_COUNT:
            self.get_logger().error(f"🚨🚨🚨 최대 회전 횟수 ({MAX_ROTATION_COUNT}회) 초과! 홈 복귀 명령")