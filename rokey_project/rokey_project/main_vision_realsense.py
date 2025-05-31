import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rokey_project.realsense import ImgNode
from rokey_interfaces.msg import TaskState
import cv2
import time


class VisionNode(Node):
    '''노드 생성 및 초기화'''
    def __init__(self):
        super().__init__('vision_node')

        # RealSense 이미지 노드 초기화
        self.img_node = ImgNode()

        # 로봇 상태 메시지 subscriber
        self.subscription = self.create_subscription(TaskState, '/robot_state', self.robot_state_callback, 10)
        
        # QR 코드 정보 publisher
        self.qr_info_publisher = self.create_publisher(TaskState, '/qr_info', 10)

        self.robot_state = ''     # 현재 로봇 상태 저장 변수
        self.qr_detected = False  # QR 코드가 최초로 인식되었는지 여부

        # 첫 프레임 받을 때까지 잠시 spin
        self.get_logger().info("[INFO] RealSense 초기화 중...")
        rclpy.spin_once(self.img_node)
        self.get_logger().info("[INFO] RealSense 초기화 완료!")
        time.sleep(0.5)


    '''로봇 상태 메시지 수신 시 호출되는 콜백 함수'''
    def robot_state_callback(self, msg):
        # robot state 갱신
        self.robot_state = msg.robot_state
        self.get_logger().info(f'📥 [Robot State 수신] "{msg.robot_state}"')

        if msg.robot_state == 'check_qr':
            self.get_logger().info("[INFO] 카메라 QR 코드 인식 시작...")
        elif msg.robot_state == 'check_text':
            self.get_logger().info("[INFO] 카메라 서랍 text 인식 시작...")
        elif msg.robot_state == 'check_pill':
            self.get_logger().info("[INFO] 카메라 알약 인식 시작...")
            

    '''QR 코드를 탐지하고 시각화하는 함수'''
    def detect_qr(self, frame):
         # QR 코드 디코딩
        detector = cv2.QRCodeDetector()
        data, points, _ = detector.detectAndDecode(frame)

        # QR 코드가 인식되었을 때
        if points is not None and data:
            # 인식된 QR 코드 영역에 사각형 박스 그리기
            points = points[0].astype(int)
            for i in range(len(points)):
                pt1 = tuple(points[i])
                pt2 = tuple(points[(i + 1) % len(points)])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(frame, data, (points[0][0], points[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # QR 코드가 처음 인식된 경우에만 퍼블리시
            if not self.qr_detected:
                self.get_logger().info(f"✅ QR 코드 인식됨: {data}")
                self.qr_detected = True

                qr_msg = TaskState()
                qr_msg.qr_info = data
                self.qr_info_publisher.publish(qr_msg)
                self.get_logger().info(f"📤 QR info publish: {data}")

        return frame


    '''카메라 프레임을 주기적으로 처리하는 루프 함수'''
    def camera_loop(self):
        # RealSense 프레임 수신
        rclpy.spin_once(self.img_node, timeout_sec=0.01)
        frame = self.img_node.get_color_frame()
        if frame is None:
            self.get_logger().warn("⚠️  RealSense 프레임 없음")
            return None
        
        # robot_state가 'check_qr'일 때만 QR 코드 인식
        if self.robot_state == 'check_qr':
            frame = self.detect_qr(frame)
        else:
            self.qr_detected = False  # 상태 바뀌면 다시 QR 탐지 대기

        return frame


def main(args=None):
    rclpy.init(args=args)

    vision_node = VisionNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(vision_node)
    executor.add_node(vision_node.img_node)

    try:
        while rclpy.ok():
            rclpy.spin_once(vision_node, timeout_sec=0.01)
            frame = vision_node.camera_loop()
            if frame is not None:
                cv2.imshow("RealSense View", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            time.sleep(0.001)  # 너무 빠른 루프 방지

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    vision_node.destroy_node()
    vision_node.img_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
