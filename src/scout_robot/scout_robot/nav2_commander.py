#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rclpy.action
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import os
import yaml
import time # sleep 대신 spin_once의 timeout을 위해 필요하지만, 예시에서는 rclpy.spin_once(self)로 충분합니다.

# 🌟 액션 메시지 임포트 (사용자 정의 인터페이스 패키지에서)
from scout_robot_interfaces.action import NavigateRoom 

# 🌟 QR Detector에게 보낼 명령 토픽 정의 (이것은 그대로 토픽으로 유지)
QR_COMMAND_TOPIC = "/qr_check_command"

class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        self.navigator = BasicNavigator()

        # --- rooms.yaml 경로 설정 및 좌표 로드 ---
        package_share = get_package_share_directory('scout_robot')
        yaml_path = os.path.join(package_share, 'rooms.yaml')

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"rooms.yaml 파일을 찾을 수 없습니다: {yaml_path}")
            raise FileNotFoundError(yaml_path)

        with open(yaml_path, 'r') as f:
            rooms = yaml.safe_load(f)['rooms']
            
        self.poses = {
            'go_start': [rooms['start']['x'], rooms['start']['y'], rooms['start']['theta']],
            'go_room501': [rooms['room501']['x'], rooms['room501']['y'], rooms['room501']['theta']],
            'go_room502': [rooms['room502']['x'], rooms['room502']['y'], rooms['room502']['theta']],
            'go_room503': [rooms['room503']['x'], rooms['room503']['y'], rooms['room503']['theta']],
            'go_home': [rooms['home']['x'], rooms['home']['y'], rooms['home']['theta']],
        }
        self.start_pose_coords = rooms['start']

        # --- 초기 위치 PoseStamped 생성 및 Nav2 설정 ---
        initial_pose = self.create_goal_pose(self.start_pose_coords['x'], self.start_pose_coords['y'], self.start_pose_coords['theta'], is_initial=True)
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info("Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 활성화 완료!")
        
        # 1) 🌟 명령 수신을 액션 서버로 대체 🌟
        self._action_server = rclpy.action.ActionServer(
            self,
            NavigateRoom,
            'navigate_to_room',
            self.execute_callback, # 목표 실행 함수
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        self._goal_handle = None

        # 2) QR 검사 명령 발행 (토픽으로 유지)
        self.qr_command_pub = self.create_publisher(
            String,
            QR_COMMAND_TOPIC,
            10
        )
        self.get_logger().info(f'RoomNavigator Node started. Action Server /navigate_to_room active.')


    def create_goal_pose(self, x, y, theta, frame_id="map", is_initial=False):
        """목표 좌표(x, y, theta)로부터 PoseStamped 메시지를 생성합니다."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if not is_initial:
            pose.header.stamp = self.get_clock().now().to_msg()
            
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_qr_command(self, command: str):
        """QR Detector에게 QR 검사를 요청하는 명령을 발행합니다."""
        msg = String()
        msg.data = command 
        self.qr_command_pub.publish(msg)
        self.get_logger().warn(f"➡️ '{command}' 도착 완료. QR Detector에게 검사 명령 발행 완료.")
    
    # --- 🌟 액션 서버 콜백 함수 🌟 ---

    def goal_callback(self, goal_request):
        """목표 요청 수락/거부 결정"""
        room_name = goal_request.room_name
        
        if room_name not in self.poses:
            self.get_logger().error(f"알 수 없는 목표 이름: {room_name}")
            return rclpy.action.GoalResponse.REJECT

        # 기존 목표가 있다면 취소하고 새 목표 수락 (선택 사항)
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().warn('이전 목표가 진행 중입니다. 새 목표 수락.')
            # Nav2 이동 취소
            self.navigator.cancelTask() 
            # 이전 목표 완료/취소 상태로 설정
            self._goal_handle.abort() 
            
        self.get_logger().info(f"목표 수락: '{room_name}'로 이동")
        return rclpy.action.GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """목표가 수락된 후 실행될 핸들러 등록"""
        self._goal_handle = goal_handle
        # 비동기적으로 실행 콜백 호출
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        """목표 실행 로직 (실제 이동 및 QR 명령 발행)"""
        self.get_logger().info('목표 실행 시작...')
        command = goal_handle.request.room_name # 예: 'go_room501'
        
        # 1. 목표 포즈 생성
        try:
            x, y, theta = self.poses[command]
            pose = self.create_goal_pose(x, y, theta)
        except KeyError:
            self.get_logger().error(f"설정되지 않은 목표 '{command}'. 목표 실행 실패.")
            result = NavigateRoom.Result()
            result.success = False
            result.message = f"알 수 없는 목표 이름: {command}"
            return result
        
        name = command.replace('go_', '') # 이름 (예: room501)
        check_qr = (command != "go_start") # 'go_start'인 경우만 QR 검사 안함

        self.get_logger().info(f"'{name}'(x:{x:.2f}, y:{y:.2f})로 이동 명령 전송. 출발합니다.")
        self.navigator.goToPose(pose)

        # 2. 이동 완료 대기 및 피드백 발행
        while not self.navigator.isTaskComplete():
            if goal_handle.is_cancel_requested:
                self.navigator.cancelTask()
                goal_handle.canceled()
                self.get_logger().warn(f"⚠️ '{name}' 이동이 액션 클라이언트에 의해 취소되었습니다.")
                result = NavigateRoom.Result()
                result.success = False
                result.message = f"'{command}' 이동 취소됨"
                return result

            # 🌟 피드백 발행 (Nav2에서 진행 상황을 가져와서 발행)
            i = 0
            while not self.navigator.isTaskComplete():
                i += 1
                if i % 10 == 0:
                    feedback = NavigateRoom.Feedback()
                    feedback.current_command = command
                    # Nav2의 진행률을 사용할 수 있지만, 간단하게 0에서 100까지 증가하는 예시로 대체
                    # 실제 Nav2 API를 통해 퍼센트 정보를 얻어야 합니다.
                    # 여기서는 간단히 0%로 고정하거나 타이머로 진행률을 높일 수 있습니다.
                    # 현재 Nav2 Simple Commander는 직접적인 진행률 피드백을 제공하지 않아 TaskState를 확인합니다.
                    feedback.progress_percentage = 0.0 
                    goal_handle.publish_feedback(feedback)
                
                rclpy.spin_once(self, timeout_sec=0.1) # 짧은 시간 동안 spin

        # 3. 결과 처리
        nav2_result = self.navigator.getResult()
        result = NavigateRoom.Result()

        if nav2_result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ '{name}' 도착 완료!")
            if check_qr:
                self.publish_qr_command(command) 
            
            result.success = True
            result.message = f"'{command}' 목표 지점에 성공적으로 도착했습니다."
            goal_handle.succeed()

        elif nav2_result == TaskResult.CANCELED:
            self.get_logger().warn(f"⚠️ '{name}' 이동이 Nav2 내부에서 취소되었습니다.")
            result.success = False
            result.message = f"'{command}' 이동이 Nav2에 의해 취소되었습니다."
            goal_handle.abort()
            
        else: # FAILED 등
            self.get_logger().error(f"❌ '{name}' 이동 실패. 결과: {nav2_result.name}")
            result.success = False
            result.message = f"'{command}' 이동 실패. 로봇 상태 확인 필요."
            goal_handle.abort()

        return result


def main():
    rclpy.init()
    node = RoomNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료 시 액션 서버와 Nav2 네비게이터도 정리
        if hasattr(node, '_action_server'):
            node._action_server.destroy()
        if hasattr(node, 'navigator'):
            node.navigator.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()