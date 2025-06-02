import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rokey_project.realsense import ImgNode
from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import RobotState
from rokey_interfaces.msg import QRInfo
from rokey_interfaces.msg import PillLoc
import cv2
import time

import random
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class VisionNode(Node):
    '''노드 생성 및 초기화'''
    def __init__(self):
        super().__init__('vision_node')

        # RealSense 이미지 노드 초기화
        self.img_node = ImgNode()

        # 로봇 상태 메시지 subscriber
        self.subscription = self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)
        
        # QR 코드 정보 publisher
        self.qr_info_publisher = self.create_publisher(QRInfo, '/qr_info', 10)

        # 약 위치, 각도 publisher
        self.pill_loc_publisher = self.create_publisher(PillLoc, '/pill_loc', 10)

        # YOLO 가중치 파일 이름, 신뢰도 설정
        self.diarrhea_yolo_weights = 'diarrhea.pt'
        self.dyspepsia_yolo_weights = 'dyspepsia.pt'
        self.dermatitis_yolo_weights = 'dermatitis_2.pt'
        self.cold_yolo_weights = 'cold.pt'
        self.CONFIDENCE = 0.50

        # 현재 로봇 상태 저장 변수
        self.robot_state = ''

        # QR 코드가 최초로 인식되었는지 여부
        self.qr_detected = False

        # YOLO 모델 관련 변수 초기화
        self.yolo_model = None
        self.yolo_start_time = None
        self.yolo_running = False

        # 약의 위치 및 각도를 저장하는 리스트 (x, y, theta)
        self.pill_loc = [0, 0, 0]

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

        elif msg.robot_state == 'detect_pill':
            self.get_logger().info("[INFO] 카메라 알약 인식 시작...")

            # self.disease = 'dermatitis'  ############ 테스트용 ############

            if self.disease == 'diarrhea':
                self.yolo_weights = self.diarrhea_yolo_weights
            elif self.disease == 'dyspepsia':
                self.yolo_weights = self.dyspepsia_yolo_weights
            elif self.disease == 'dermatitis':
                self.yolo_weights = self.dermatitis_yolo_weights
            elif self.disease == 'cold':
                self.yolo_weights = self.cold_yolo_weights

            self.load_yolo_model()
            

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
                self.disease = data.split()[0]
                self.pill_list = data.split()[1:]
                self.get_logger().info(f"✅ QR 코드 인식됨 {data}")
                self.get_logger().info(f"💊 병: {self.disease}, 약: {self.pill_list}")
                self.qr_detected = True

                qr_msg = QRInfo()
                qr_msg.disease = self.disease
                qr_msg.pill = self.pill_list
                self.qr_info_publisher.publish(qr_msg)
                self.get_logger().info(f"📤 QR info publish: {data}")

        return frame
    

    '''YOLO 모델을 로드하는 함수'''
    def load_yolo_model(self):
        if self.yolo_model is None:
            self.get_logger().info("[INFO] YOLO 세그멘테이션 모델 로드 중...")
            package_share_directory = get_package_share_directory('rokey_project')
            weights = os.path.join(package_share_directory, 'weights', self.yolo_weights)
            self.yolo_model = YOLO(weights)

            # 클래스별 고유 색상 생성
            class_names = self.yolo_model.names
            self.class_colors = {cls_id: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for cls_id in class_names}

            self.yolo_start_time = time.time()
            self.yolo_running = True
            self.get_logger().info("[INFO] YOLO 모델 로드 완료! Segmentation 시작")


    '''YOLO 세그멘테이션으로 알약 탐지 및 마스크를 표시하는 함수'''
    def detect_pill_with_yolo(self, frame):
        if not self.yolo_running or self.yolo_model is None:
            # 모델이 준비되지 않았으면 원본 프레임 반환
            return frame

        results = self.yolo_model(frame, verbose=False)

        annotated_frame = frame.copy()

        if results and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()  # (num_masks, H, W)
            boxes = results[0].boxes

            for i, box in enumerate(boxes):
                conf = box.conf.item()
                if conf < self.CONFIDENCE:
                    continue

                cls = int(box.cls[0])
                class_name = self.yolo_model.names[cls]
                color = self.class_colors.get(cls, (0, 255, 0))

                mask = masks[i]
                mask_bool = mask > 0.5

                colored_mask = np.zeros_like(annotated_frame, dtype=np.uint8)
                colored_mask[:, :, 0] = color[0]
                colored_mask[:, :, 1] = color[1]
                colored_mask[:, :, 2] = color[2]

                alpha = 0.5
                annotated_frame[mask_bool] = cv2.addWeighted(colored_mask, alpha, annotated_frame, 1 - alpha, 0)[mask_bool]

                # 마스크로부터 타원 찾기 및 중심점, 회전각 시각화
                mask_uint8 = (mask_bool.astype(np.uint8)) * 255
                contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours and len(contours[0]) >= 5:
                    ellipse = cv2.fitEllipse(contours[0])
                    (center, axes, angle) = ellipse

                    # 타원 그리기
                    cv2.ellipse(annotated_frame, ellipse, color, 2)
                    # 회전 각도 텍스트 출력
                    angle_text = f"{angle:.1f} deg"
                    cv2.putText(annotated_frame, angle_text, (int(center[0]) + 35, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    # 중심점 좌표 텍스트 출력
                    center_text = f"({int(center[0])}, {int(center[1])})"
                    cv2.putText(annotated_frame, center_text, (int(center[0]) + 35, int(center[1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    # 약 위치 저장
                    self.pill_loc = [int(center[0]), int(center[1]), int(angle)]
                    print(f"self.pill_loc = {self.pill_loc}")

                # 마스크 내 픽셀 좌표 기반 클래스 이름 텍스트 출력
                ys, xs = np.where(mask_bool)
                if len(xs) > 0 and len(ys) > 0:
                    x1, y1 = np.min(xs), np.min(ys)
                    cv2.putText(annotated_frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


        # 일정 시간 경과 후 YOLO 모델 종료 처리
        elapsed = time.time() - self.yolo_start_time
        second = 20.0
        if elapsed > second:
            self.get_logger().info(f"[INFO] YOLO 모델 {second}초 경과, 메모리 해제 중...")
            self.yolo_model = None
            self.yolo_running = False
            self.get_logger().info("[INFO] YOLO 모델 메모리 해제 완료!")

            pill_loc_msg = PillLoc()
            pill_loc_msg.x = self.pill_loc[0]
            pill_loc_msg.y = self.pill_loc[1]
            pill_loc_msg.theta = self.pill_loc[2]
            self.pill_loc_publisher.publish(pill_loc_msg)
            self.get_logger().info(f"📤 Pill location publish: {pill_loc_msg}")
            self.get_logger().info(f"📤 Pill location (x = {pill_loc_msg.x}, y = {pill_loc_msg.y}, z = {pill_loc_msg.theta})")

        return annotated_frame


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
        elif self.robot_state == 'detect_pill':
            frame = self.detect_pill_with_yolo(frame)
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
