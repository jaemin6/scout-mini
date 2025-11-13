# ros2_ws/src/scout_robot/scout_robot/room_command_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from scout_robot_interfaces.action import NavigateRoom
import re # 문자열 파싱을 위해 정규 표현식 모듈 사용

class RoomCommandClient(Node):
    def __init__(self):
        super().__init__('room_command_client')
        
        # 1. 액션 클라이언트 생성
        self._action_client = ActionClient(
            self,
            NavigateRoom,
            'navigate_to_room' # 액션 서버 이름은 노드에서 정의한 이름과 일치해야 함
        )

        # 2. /room_command 토픽 구독
        self.subscription = self.create_subscription(
            String,
            'room_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Room Command Client Node started and subscribing to /room_command.")

    def command_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"Received raw command: '{command}'")
        
        # 'go_roomXXX' 형태의 명령 파싱
        match = re.search(r'go_(room\w+)', command)
        
        if match:
            room_name = match.group(1)
            self.get_logger().info(f"Parsed room name: {room_name}")
            
            # 액션 서버 호출
            self.send_goal(room_name)
        else:
            self.get_logger().warn(f"Command format not recognized: {command}. Expected 'go_roomXXX'.")

    def send_goal(self, room_name):
        goal_msg = NavigateRoom.Goal()
        goal_msg.room_name = room_name

        self.get_logger().info(f'Waiting for action server: navigate_to_room...')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal to navigate to room: {room_name}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation Result: {"Success" if result.success else "Failed"}')
        self.get_logger().info(f'Message: {result.message}')
        # 클라이언트가 피드백을 처리하려면 여기에 로직을 추가해야 함

def main(args=None):
    rclpy.init(args=args)
    room_command_client = RoomCommandClient()
    rclpy.spin(room_command_client)
    room_command_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()