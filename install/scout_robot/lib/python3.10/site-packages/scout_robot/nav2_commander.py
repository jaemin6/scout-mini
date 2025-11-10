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

# 🌟 QR Detector에게 보낼 명령 토픽 정의
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
            
        self.start_pose = [rooms['start']['x'], rooms['start']['y'], rooms['start']['theta']]
        self.room501_pose = [rooms['room501']['x'], rooms['room501']['y'], rooms['room501']['theta']]
        self.room502_pose = [rooms['room502']['x'], rooms['room502']['y'], rooms['room502']['theta']] # 🌟 추가
        self.room503_pose = [rooms['room503']['x'], rooms['room503']['y'], rooms['room503']['theta']] # 🌟 추가
        self.home_pose = [rooms['home']['x'], rooms['home']['y'], rooms['home']['theta']]
        self.start_pose_coords = rooms['start']

        # --- 초기 위치 PoseStamped 생성 및 Nav2 설정 생략 ---
        initial_pose = self.create_goal_pose(self.start_pose_coords['x'], self.start_pose_coords['y'], self.start_pose_coords['theta'], is_initial=True)
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info("Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 활성화 완료!")
        
        # 1) 명령 구독 (외부에서 토픽을 받습니다)
        self.command_sub = self.create_subscription(
            String,
            '/room_command',
            self.command_callback,
            10
        )
        
        # 2) 🌟 QR 검사 명령 발행 (QR Detector에게 보냅니다)
        self.qr_command_pub = self.create_publisher(
            String,
            QR_COMMAND_TOPIC,
            10
        )
        self.get_logger().info(f'RoomNavigator Node started. Publishing QR commands on {QR_COMMAND_TOPIC}.')


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
        # command 그대로 QR Detector에게 전달
        msg.data = command 
        self.qr_command_pub.publish(msg)
        self.get_logger().warn(f"➡️ '{command}' 도착 완료. QR Detector에게 검사 명령 발행 완료.")

    def move_and_wait(self, pose: PoseStamped, name: str, command: str, check_qr: bool = True): # 🌟 check_qr 인자 추가
        """목표로 이동을 요청하고 완료될 때까지 대기합니다. 완료 후 QR 검사 명령을 발행합니다."""
        self.get_logger().info(f"'{name}'(x:{pose.pose.position.x:.2f}, y:{pose.pose.position.y:.2f})로 이동 명령 전송. 출발합니다.")
        self.navigator.goToPose(pose)

        # 이동 완료 대기
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1) 
        
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ '{name}' 도착 완료!")
            # 🌟 check_qr이 True일 경우에만 QR 검사 명령 발행
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

        elif command == "go_room502": # 🌟 추가
            x, y, theta = self.room502_pose
            pose = self.create_goal_pose(x, y, theta)
            self.move_and_wait(pose, "room502", command, check_qr=True) 

        elif command == "go_room503": # 🌟 추가
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
            # 🌟🌟🌟 start 좌표로 이동 시에는 QR 검사 명령 발행하지 않음 (check_qr=False) 🌟🌟🌟
            self.move_and_wait(pose, "start", command, check_qr=False) 
            
        else:
            self.get_logger().warn(f"알 수 없는 명령 수신: {command}")

def main():
    rclpy.init()
    node = RoomNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
