import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
import cv2
import numpy as np
import time
import threading

# 카메라 번호 설정
CAMERA_NUM = 6

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(TaskState, '/task_state', 10)
        self.subscription = self.create_subscription(TaskState, "/robot_state", self.robot_state_callback, 10)

        self.qr_detector = cv2.QRCodeDetector()
        self.cap = cv2.VideoCapture(CAMERA_NUM)

        self.msg = TaskState()

        self.last_qr_data = ''

        if not self.cap.isOpened():
            self.get_logger().error("❌ 웹캠을 열 수 없습니다.")
            return
    

    '''robot state를 수신하는 subscriber callback 함수'''
    def robot_state_callback(self, msg):
        self.msg.robot_state = msg.robot_state
        self.get_logger().info(f'로봇 상태 수신: "{msg.robot_state}"')

        if msg.robot_state == 'qr_mode':
            self.get_logger().info("[INFO] 웹캠 QR 코드 인식 시작...")


    '''QR code를 인식하고 정보를 publish 하는 함수'''
    def check_qr(self, frame):
        data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        if bbox is not None and data:
            # QR 바운딩 박스 그리기
            bbox = np.int32(bbox).reshape(-1, 2)
            for i in range(len(bbox)):
                pt1 = tuple(bbox[i])
                pt2 = tuple(bbox[(i + 1) % len(bbox)])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            cv2.putText(frame, data, (bbox[0][0], bbox[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # QR 정보 publish (QR이 인식된 최초 한번만 publish)
            if data != self.last_qr_data:
                msg = TaskState()
                msg.qr_info = data
                self.publisher_.publish(msg)
                self.get_logger().info(f"[QR 인식] Publish: '{msg.qr_info}'")
                self.last_qr_data = data
        
        # else:
        #     self.last_qr_data = ''


    '''카메라를 가동하는 메인 함수'''
    def run_camera(self):
        try:
            self.get_logger().info("[INFO] 웹캠 시작... 'q' 키를 눌러 종료")

            # while not msg.state:
            while rclpy.ok():
                ret, frame = self.cap.read()

                if not ret:
                    self.get_logger().error("❌ 프레임을 읽을 수 없습니다.")
                    break
                
                # robot_state가 qr_mode일 때 QR code를 인식
                if self.msg.robot_state == 'qr_mode':
                    self.check_qr(frame)

                # 영상 출력
                cv2.imshow('Webcam', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info("[INFO] vision_node 종료")



def main(args=None):

    rclpy.init(args=args)
    vision_node = VisionNode()

    # ROS 콜백을 위한 스레드 실행
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(vision_node,), daemon=True)
    ros_spin_thread.start()

    # QR 인식 함수 실행 (OpenCV 메인 루프)
    vision_node.run_camera()

    # 종료 처리
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()