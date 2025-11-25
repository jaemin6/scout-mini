import rclpy

from rclpy.node import Node

from std_msgs.msg import String 

import pigpio

import time



# 서보 제어 노드 클래스 정의

class ServoController(Node):

    def __init__(self):

        super().__init__('servo_controller_node')

        self.get_logger().info('✅ Servo Controller Node starting...')

        

        # --- 1. 서보모터 파라미터 선언 ---

        self.declare_parameter('servo_pin', 12)     # 라즈베리 파이 GPIO 핀 번호 (BCM)

        self.declare_parameter('open_angle', 0.0)    # RFID 성공 시 움직일 각도 (문 열림)

        self.declare_parameter('close_angle', 180.0)     # 초기/대기 각도 (문 닫힘)

        self.declare_parameter('min_pulse', 500)      # 0도에 해당하는 펄스 폭 (us)

        self.declare_parameter('max_pulse', 2500)     # 180도에 해당하는 펄스 폭 (us)



        # 파라미터 값 로드

        self.servo_pin = self.get_parameter('servo_pin').get_parameter_value().integer_value

        self.open_angle = self.get_parameter('open_angle').get_parameter_value().double_value

        self.close_angle = self.get_parameter('close_angle').get_parameter_value().double_value

        self.min_pulse = self.get_parameter('min_pulse').get_parameter_value().integer_value

        self.max_pulse = self.get_parameter('max_pulse').get_parameter_value().integer_value



        # --- 2. [추가] 서보 속도 제어를 위한 상태 변수 및 설정 ---

        self.current_angle = self.close_angle  # 서보의 현재 각도

        self.target_angle = self.close_angle   # 서보의 목표 각도

        self.speed_timer = None                # 속도 제어용 타이머

        

        # ⭐ 속도 제어 핵심 설정 (값을 작게/길게 설정하여 느리게 만듦) ⭐

        self.angle_step = 1.0                 # 0.05초당 움직일 각도 (작을수록 느림)

        self.TIMER_PERIOD = 0.02               # 타이머 주기 (길수록 느림, 0.05초 = 50ms)

        # ⭐





        # --- 3. pigpio 라이브러리 초기화 및 연결 ---

        try:

            self.pi = pigpio.pi() 

            if not self.pi.connected:

                self.get_logger().error("❌ Failed to connect to pigpio daemon. Is 'sudo pigpiod' running?")

                raise Exception("pigpio connection error")

            

            # PWM 설정 (서보 주파수 50Hz)

            self.pi.set_PWM_frequency(self.servo_pin, 50)

            self.get_logger().info(f"⚙️ pigpio connected. Servo on pin {self.servo_pin}")

            

            # 초기 상태: 문 닫힘 각도로 즉시 설정 (부팅 시 빠른 초기화)

            self.__set_servo_pulse_direct(self.close_angle)

            self.get_logger().info(f"🚪 Initial position set to: {self.close_angle} degrees.")

            

        except Exception as e:

            self.get_logger().fatal(f"🛑 Initialization Error: {e}")

            raise SystemExit



        # --- 4. 토픽 구독 설정 ---

        # A. UARTSender가 발행하는 /rfid_detection_success 토픽 구독 (열림 트리거)

        self.rfid_subscription = self.create_subscription(

            String,

            '/rfid_detection_success',

            self.rfid_success_callback,

            10)

        

        # B. 초음파 노드가 발행하는 /room_command 토픽 구독 (닫힘 트리거)

        self.command_subscription = self.create_subscription(

            String,

            '/room_command',

            self.room_command_callback,

            10)

        

        self.get_logger().info('✅ Subscribing to /rfid_detection_success (Open) and /room_command (Close) for control.')





    # --------------------------------------------------------------------------

    # 📌 헬퍼 함수: 각도를 펄스 폭으로 변환

    # --------------------------------------------------------------------------

    def angle_to_pulse_width(self, angle):

        """각도(0~180)를 PWM 펄스 폭(us)으로 변환합니다."""

        # 각도를 0 ~ 180 범위로 클램프 (기존 코드의 120도 클램프 유지)

        clamped_angle = max(0.0, min(120.0, angle))

        

        # 펄스 폭 = min_pulse + (max_pulse - min_pulse) * (angle / 180)

        pulse_width = int(self.min_pulse + (self.max_pulse - self.min_pulse) * (clamped_angle / 180.0))

        return pulse_width



    # --------------------------------------------------------------------------

    # 📌 [추가] 서보 PWM 신호를 직접 설정하는 내부 함수 (부드러운 움직임에 사용)

    # --------------------------------------------------------------------------

    def __set_servo_pulse_direct(self, angle):

        """서보를 해당 각도로 즉시 이동시키는 내부 함수 (로깅 제외)"""

        pulse_width = self.angle_to_pulse_width(angle)

        self.pi.set_servo_pulsewidth(self.servo_pin, pulse_width)



    # --------------------------------------------------------------------------

    # 📌 [수정] 서보모터 제어 함수 (속도 제어 로직 시작)

    # --------------------------------------------------------------------------

    def set_servo_angle(self, angle):

        """서보모터의 목표 각도를 설정하고 부드러운 이동을 시작합니다."""

        

        self.target_angle = angle

        

        # 이미 실행 중인 타이머가 있다면 취소 (새로운 명령이 들어오면 기존 움직임 중단)

        if self.speed_timer:

            self.speed_timer.cancel()

            

        # 새로운 속도 제어 타이머 시작

        self.speed_timer = self.create_timer(self.TIMER_PERIOD, self.move_servo_smoothly) 

        

        self.get_logger().info(f'➡️ Servo Target set to: {self.target_angle:.1f} deg. Starting smooth movement (Step: {self.angle_step}, Period: {self.TIMER_PERIOD}s).')





    # --------------------------------------------------------------------------

    # 📌 [추가] 타이머 콜백 함수: 서보를 부드럽게 움직이는 핵심 로직

    # --------------------------------------------------------------------------

    def move_servo_smoothly(self):

        """타이머에 의해 주기적으로 호출되어 서보를 목표까지 점진적으로 이동시킵니다."""

        

        diff = self.target_angle - self.current_angle

        

        if abs(diff) < self.angle_step:

            # 목표에 도달했거나 거의 근접했을 경우: 최종 위치로 설정하고 타이머 종료

            self.current_angle = self.target_angle

            

            self.__set_servo_pulse_direct(self.current_angle)

            

            if self.speed_timer:

                self.speed_timer.cancel()

                self.speed_timer = None

            

            self.get_logger().info(f'✅ Servo movement complete at {self.current_angle:.1f} deg.')

            

        else:

            # 목표를 향해 angle_step만큼 이동

            direction = 1.0 if diff > 0 else -1.0

            self.current_angle += direction * self.angle_step

            

            # 새로운 현재 각도로 PWM 설정

            self.__set_servo_pulse_direct(self.current_angle)

            # self.get_logger().info(f'Smooth Move: {self.current_angle:.1f} deg') # 디버깅용





    # --------------------------------------------------------------------------

    # 📌 콜백 함수: RFID 인식 성공 신호 수신 (문 열림)

    # --------------------------------------------------------------------------

    def rfid_success_callback(self, msg: String):

        """UARTSender로부터 RFID 인식 성공 신호를 받아 서보를 엽니다."""

        data = msg.data.strip() 

        self.get_logger().info(f"🟡 Received RFID Success Trigger: {data}")

        

        if data.startswith("RFID_SUCCESS:"):

            try:

                room_id = data.split(':')[1].strip()

            except IndexError:

                self.get_logger().warn(f'⚠️ Malformed RFID Success data received: {data}')

                return

            

            # 1. 서보를 '열림' 각도로 이동 (set_servo_angle이 속도 제어를 시작함)

            self.set_servo_angle(self.open_angle)

            self.get_logger().info(f'🎉 Success for Room {room_id}. Servo moving to OPEN position ({self.open_angle} degrees).')



    

    # --------------------------------------------------------------------------

    # 📌 콜백 함수: 명령 신호 수신 (문 닫힘)

    # --------------------------------------------------------------------------

    def room_command_callback(self, msg: String):

        """/room_command 토픽에서 'close_servo' 명령을 받아 서보를 닫습니다."""

        command = msg.data.strip()

        

        if command == "close_servo":

            self.get_logger().info("🚪 Received 'close_servo' command. Closing servo.")

            # 서보를 '닫힘' 각도로 이동 (set_servo_angle이 속도 제어를 시작함)

            self.set_servo_angle(self.close_angle)

            self.get_logger().info(f'✅ Servo moving back to CLOSE position ({self.close_angle} degrees).')





    # --------------------------------------------------------------------------

    # 📌 노드 종료 시 정리 작업

    # --------------------------------------------------------------------------

    def destroy_node(self):

        self.get_logger().info('🔌 Shutting down Servo Controller...')

        

        # 타이머 중지

        if self.speed_timer:

            self.speed_timer.cancel()



        # 펄스 폭을 0으로 설정하여 PWM 신호 중지

        if hasattr(self, 'pi') and self.pi.connected:

            self.pi.set_servo_pulsewidth(self.servo_pin, 0)

            # pigpio 연결 해제

            self.pi.stop()

            self.get_logger().info('👍 pigpio connection stopped.')

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    servo_controller = ServoController()

    

    try:

        rclpy.spin(servo_controller)

    except KeyboardInterrupt:

        pass

    except Exception as e:

        servo_controller.get_logger().error(f'Node failed with exception: {e}')

    finally:

        servo_controller.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()
