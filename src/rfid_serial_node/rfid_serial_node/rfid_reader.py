import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

# ⭐ 1단계에서 확인한 시리얼 포트 이름으로 변경하세요!
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 115200

class RFIDReaderNode(Node):
    def __init__(self):
        super().__init__('rfid_reader_node')
        self.publisher_ = self.create_publisher(String, 'rfid_room_complete', 10)
        self.get_logger().info(f'Attempting to open serial port: {SERIAL_PORT} @ {BAUDRATE}')

        try:
            # 시리얼 포트 열기
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            time.sleep(2) # Pico가 완전히 부팅될 때까지 대기
            self.get_logger().info('Serial port opened successfully.')
            
            # 0.1초마다 시리얼 데이터 확인하는 타이머 설정
            self.timer = self.create_timer(0.1, self.read_serial_data)

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {SERIAL_PORT}: {e}')
            # 오류 발생 시 노드 종료 준비
            self.timer = None
            self.ser = None

    def read_serial_data(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                # 한 줄 읽기 (Pico에서 printf("\n")으로 줄바꿈이 있을 경우)
                line = self.ser.readline().decode('utf-8').strip()
                
                # Pico 펌웨어의 성공 시그널 확인 (예: ROOM_COMPLETE:501)
                if line.startswith('ROOM_COMPLETE:'):
                    room_id = line.split(':')[1]
                    
                    msg = String()
                    msg.data = room_id
                    
                    # ROS 2 토픽 발행
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published RFID Signal for Room: {room_id}')
                
                # 디버깅을 위해 Pico가 출력하는 모든 메시지 출력
                # self.get_logger().info(f'Pico Output: {line}')

            except Exception as e:
                self.get_logger().error(f'Error processing serial data: {e}')

def main(args=None):
    rclpy.init(args=args)
    rfid_reader_node = RFIDReaderNode()
    
    # 노드 실행 시 권한 오류가 발생하면, 다음 명령을 사용해 권한을 부여해야 합니다:
    # sudo chmod 666 /dev/ttyACM0 
    
    try:
        rclpy.spin(rfid_reader_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rfid_reader_node.ser:
            rfid_reader_node.ser.close()
            rfid_reader_node.get_logger().info('Serial port closed.')
        rfid_reader_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()