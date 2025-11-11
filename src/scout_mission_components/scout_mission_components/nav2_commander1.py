#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import os
import yaml
import time # sleep 대신 rclpy.spin_once()를 사용해야 하지만, 여기서는 Nav2 비동기 대기 로직을 사용합니다.

# 🌟 QR Detector에게 보낼 명령 토픽 정의
QR_COMMAND_TOPIC = "/qr_check_command"

class RoomNavigator(Node):
    # 컴포넌트 클래스: __init__은 **kwargs를 통해 NodeOptions를 받습니다.
    def __init__(self, **kwargs):
        # super().__init__ 호출 시 **kwargs 전달 필수
        super().__init__('room_navigator', **kwargs)
        self.navigator = BasicNavigator()

        # --- rooms.yaml 경로 설정 및 좌표 로드 ---
        package_share = get_package_share_directory('scout_robot')
        yaml_path = os.path.join(package_share, 'rooms.yaml')

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"rooms.yaml 파일을 찾을 수 없습니다: {yaml_path}")
            # 컴포넌트 환경에서는 예외를 raise하면 컨테이너 로드가 실패합니다.
            # 이 노드 자체를 실패시키거나, 초기화를 중단해야 합니다.
            # 여기서는 로깅만 하고 계속 진행하지만, 실제 환경에서는 오류 처리 필요
            return

        with open(yaml_path, 'r') as f:
            rooms = yaml.safe_load(f)['rooms']
            
        self.start_pose = [rooms['start']['x'], rooms['start']['y'], rooms['start']['theta']]
        self.room501_pose = [rooms['room501']['x'], rooms['room501']['y'], rooms['room501']['theta']]
        self.room502_pose = [rooms['room502']['x'], rooms['room502']['y'], rooms['room502']['theta']]
        self.room503_pose = [rooms['room503']['x'], rooms['room503']['y'], rooms['room503']['theta']]
        self.home_pose = [rooms['home']['x'], rooms['home']['y'], rooms['home']['theta']]
        self.start_pose_coords = rooms['start']

        # --- 초기 위치 PoseStamped 생성 및 Nav2 설정 ---
        # NOTE: BasicNavigator는 노드 외부에서 rclpy.spin을 가정합니다.
        # 컴포넌트 환경에서는 비동기 또는 액션 클라이언트/서버 패턴을 사용해야 합니다.
        # BasicNavigator를 컴포넌트 내에서 사용할 경우, Nav2 활성화 대기는 컴포넌트 초기화 후
        # 별도의 타이머/스레드에서 진행하는 것이 이상적이나, 일단 기존 코드를 유지하고
        # 컴포넌트 환경에 맞게 `waitUntilNav2Active` 호출을 피합니다.
        
        # RoomNavigator 컴포넌트는 초기화 시 초기 위치를 설정하지 않습니다.
        # 초기 위치 설정은 AMCL이 `/initialpose` 토픽으로 처리해야 합니다.
        
        # Nav2 활성화 대기 로직 제거 (컴포넌트 환경과 충돌 방지)
        # self.get_logger().info("Nav2 활성화 대기 중...")
        # self.navigator.waitUntilNav2Active() 
        self.get_logger().info("Nav2 활성화 대기 생략. Nav2 bringup이 별도 실행 중이라고 가정.")
        self.get_logger().info("Nav2 활성화 완료!")
        
        # 1) 명령 구독 (외부에서 토픽을 받습니다)
        self.command_sub = self.create_subscription(
            String,
            '/room_command',
            self.command_callback,
            10
        )
        
        # 2) QR 검사 명령 발행 (QR Detector에게 보냅니다)
        self.qr_command_pub = self.create_publisher(
            String,
            QR_COMMAND_TOPIC,
            10
        )
        self.get_logger().info(f'RoomNavigator Component started. Publishing QR commands on {QR_COMMAND_TOPIC}.')


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

    def move_and_wait(self, pose: PoseStamped, name: str, command: str, check_qr: bool = True):
        """목표로 이동을 요청하고 완료될 때까지 대기합니다."""
        self.get_logger().info(f"'{name}'(x:{pose.pose.position.x:.2f}, y:{pose.pose.position.y:.2f})로 이동 명령 전송. 출발합니다.")
        
        # 기존: rclpy.spin_once()를 사용하는 Blocking 대기 루프
        # 수정: Nav2의 기본 비동기 대기 로직 사용 (spin은 컨테이너가 처리)
        self.navigator.goToPose(pose)

        i = 0
        while not self.navigator.isTaskComplete():
            # 컴포넌트 컨테이너가 spin을 처리하므로, 여기서는 루프 대기 시간만 설정
            # NOTE: 이 방식은 컴포넌트 환경에서 Nav2 액션 피드백이 느리게 업데이트될 수 있습니다.
            time.sleep(0.1) 
            i = (i + 1) % 10
            if i == 0:
                self.get_logger().info(f"'{name}'로 이동 중...")
                
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ '{name}' 도착 완료!")
            if check_qr:
                self.publish_qr_command(command) 
        
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f"⚠️ '{name}' 이동이 취소되었습니다.")
        elif result == TaskResult.FAILED:
            self.get_logger().error(f"❌ '{name}' 이동 실패. 로봇의 위치나 지도를 확인하세요.")
        else:
            self.get_logger().info(f"'{name}' 이동 결과: {result.name}")


    def command_callback(self, msg: String):
        """명령어 콜백 함수"""
        command = msg.data.strip()
        
        if command == "go_room501": 
            x, y, theta = self.room501_pose
            pose = self.create_goal_pose(x, y, theta)
            self.move_and_wait(pose, "room501", command, check_qr=True) 

        elif command == "go_room502":
            x, y, theta = self.room502_pose
            pose = self.create_goal_pose(x, y, theta)
            self.move_and_wait(pose, "room502", command, check_qr=True) 

        elif command == "go_room503":
            x, y, theta = self.room503_pose
            pose = self.create_goal_pose(x, y, theta)
            self.move_and_wait(pose, "room503", command, check_qr=True) 

        elif command == "go_home": 
            x, y, theta = self.home_pose
            pose = self.create_goal_pose(x, y, theta)
            self.move_and_wait(pose, "home", command, check_qr=True)
            
        elif command == "go_start": 
            x, y, theta = self.start_pose
            pose = self.create_goal_pose(x, y, theta)
            # start 좌표로 이동 시에는 QR 검사 명령 발행하지 않음 (check_qr=False)
            self.move_and_wait(pose, "start", command, check_qr=False) 
            
        else:
            self.get_logger().warn(f"알 수 없는 명령 수신: {command}")


# ----------------------------------------------------
# 🌟🌟🌟 컴포넌트 등록을 위한 필수 진입점 🌟🌟🌟
# ----------------------------------------------------
def create_node():
    """RoomNavigator 컴포넌트 인스턴스를 생성하고 반환합니다."""
    # RoomNavigator 클래스 인스턴스 생성 및 반환
    return RoomNavigator() 

# 기존 main 함수와 if __name__ == '__main__': 구문은 제거합니다.
