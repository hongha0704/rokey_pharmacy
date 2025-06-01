import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(TaskState, '/task_state', 10)

        # 카메라 번호 설정 (기본 웹캠: 0)
        self.CAMERA_NUM = 0
        self.qr_detector = cv2.QRCodeDetector()
        self.cap = cv2.VideoCapture(self.CAMERA_NUM)

        if not self.cap.isOpened():
            self.get_logger().error("❌ 웹캠을 열 수 없습니다.")
            return

        self.get_logger().info("[INFO] 웹캠 QR 코드 인식 시작... 'q' 키를 눌러 종료")

        self.run_qr_loop()

    def run_qr_loop(self):
        try:
            while rclpy.ok():
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("❌ 프레임을 읽을 수 없습니다.")
                    break

                data, bbox, _ = self.qr_detector.detectAndDecode(frame)

                if bbox is not None and data:
                    # 바운딩 박스 그리기
                    bbox = np.int32(bbox).reshape(-1, 2)
                    for i in range(len(bbox)):
                        pt1 = tuple(bbox[i])
                        pt2 = tuple(bbox[(i + 1) % len(bbox)])
                        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                    cv2.putText(frame, data, (bbox[0][0], bbox[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                    # ROS2 퍼블리시
                    msg = TaskState()
                    msg.state = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"[QR 인식] Publish: '{data}'")

                # 영상 출력
                cv2.imshow('Webcam QR Scanner', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cap.release()
            cv2.destroyAllWindows()
            self.get_logger().info("[INFO] vision_node 종료")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()