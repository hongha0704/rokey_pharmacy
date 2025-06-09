#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
import RPi.GPIO as gpio
import time
from collections import deque

# 핀 설정
TRIG_A, ECHO_A = 24, 23
TRIG_B, ECHO_B = 17, 27

# 거리 필터 및 감지 조건
WINDOW_SIZE = 10
MAX_DIST = 400
THRESHOLD = 50
MIN_DETECTION_DISTANCE = 5
MAX_DETECTION_DISTANCE = 40
DETECTION_DURATION = 1  # seconds

# # 거리 측정 함수 (칼만 필터 제거됨)
# def measure_distance(trig, echo, window, prev_dist):
#     gpio.output(trig, gpio.LOW)
#     time.sleep(0.1)
#     gpio.output(trig, gpio.HIGH)
#     time.sleep(0.00002)
#     gpio.output(trig, gpio.LOW)

#     start_time = time.time()
#     timeout = start_time + 0.02
#     while gpio.input(echo) == gpio.LOW and time.time() < timeout:
#         start_time = time.time()

#     timeout = time.time() + 0.02
#     end_time = start_time
#     while gpio.input(echo) == gpio.HIGH and time.time() < timeout:
#         end_time = time.time()

#     if end_time == start_time:
#         return None, prev_dist

#     period = end_time - start_time
#     dist = round(period * 1000000 / 58, 2)

#     if dist > MAX_DIST or abs(dist - prev_dist) > THRESHOLD:
#         return None, prev_dist

#     window.append(dist)
#     avg = round(sum(window) / len(window), 2)

#     return avg, avg

# 온도에 따른 초음파 속도 계산 함수 (℃ 입력, m/s 반환)
def get_speed_of_sound(temperature_celsius):
    return 331.3 + 0.606 * temperature_celsius  # 습도 무시, 정밀 보정 불필요 시

# 거리 측정 함수 수정
def measure_distance(trig, echo, window, prev_dist, temperature=22.0):  # 기본 온도 22도
    gpio.output(trig, gpio.LOW)
    time.sleep(0.1)
    gpio.output(trig, gpio.HIGH)
    time.sleep(0.00002)
    gpio.output(trig, gpio.LOW)

    start_time = time.time()
    timeout = start_time + 0.02
    while gpio.input(echo) == gpio.LOW and time.time() < timeout:
        start_time = time.time()

    timeout = time.time() + 0.02
    end_time = start_time
    while gpio.input(echo) == gpio.HIGH and time.time() < timeout:
        end_time = time.time()

    if end_time == start_time:
        window.clear()  # 측정 실패 시 필터 초기화
        return None, prev_dist

    # 거리 계산
    period = end_time - start_time
    speed = get_speed_of_sound(temperature)  # 단위: m/s
    distance_cm = round((period * speed * 100) / 2, 2)

    if distance_cm > MAX_DIST or abs(distance_cm - prev_dist) > THRESHOLD:
        window.clear()  # 갑작스러운 변화 또는 초과 거리 시 필터 초기화
        return None, prev_dist

    window.append(distance_cm)
    avg = round(sum(window) / len(window), 2)

    return avg, avg
    
class ProximityDetectorNode(Node):
    def __init__(self):
        super().__init__('proximity_detector_node')
        self.publisher = self.create_publisher(TaskState, '/task_state', 10)

        gpio.setmode(gpio.BCM)
        for trig, echo in [(TRIG_A, ECHO_A), (TRIG_B, ECHO_B)]:
            gpio.setup(trig, gpio.OUT)
            gpio.setup(echo, gpio.IN)

        # 센서 상태
        self.window_A = deque(maxlen=WINDOW_SIZE)
        self.window_B = deque(maxlen=WINDOW_SIZE)
        self.prev_A = 0
        self.prev_B = 0
        self.start_time_A = None
        self.start_time_B = None
        self.detected = False

        self.timer = self.create_timer(0.05, self.check_sensors)

    def check_sensors(self):
        temperature = 26.0  # 현재는 고정 온도, 나중에 센서로 받아도 됨

        dist_A, self.prev_A = measure_distance(TRIG_A, ECHO_A, self.window_A, self.prev_A, temperature)
        time.sleep(0.2)
        dist_B, self.prev_B = measure_distance(TRIG_B, ECHO_B, self.window_B, self.prev_B, temperature)

        print(f"[Sensor A] 거리: {dist_A if dist_A else '-'} cm")
        print(f"[Sensor B] 거리: {dist_B if dist_B else '-'} cm")

        now = time.time()

        if dist_A is not None and MIN_DETECTION_DISTANCE <= dist_A <= MAX_DETECTION_DISTANCE:
            if self.start_time_A is None:
                self.start_time_A = now
        else:
            self.start_time_A = None

        if dist_B is not None and MIN_DETECTION_DISTANCE <= dist_B <= MAX_DETECTION_DISTANCE:
            if self.start_time_B is None:
                self.start_time_B = now
        else:
            self.start_time_B = None


        if (self.start_time_A and self.start_time_B and
            now - self.start_time_A >= DETECTION_DURATION and
            now - self.start_time_B >= DETECTION_DURATION and
            not self.detected):

            msg = TaskState()
            msg.robot_state = ""
            msg.qr_info = ""
            msg.state = "detected"
            self.publisher.publish(msg)
            self.get_logger().info("Sensor A & B 모두 3초 이상 감지 → state='detected' 메시지 전송됨")
            self.detected = True

        if not (self.start_time_A and self.start_time_B):
            self.detected = False

    def destroy_node(self):
        super().destroy_node()
        gpio.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = ProximityDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
