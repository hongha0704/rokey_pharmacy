import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rokey_project.realsense import ImgNode
from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import RobotState
from rokey_interfaces.msg import QRInfo
from rokey_interfaces.msg import PillLoc
from rokey_interfaces.msg import TextLoc
from collections import defaultdict
import cv2
import time
from PIL import Image
import torch
from torchvision import transforms, models

import random
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
from scipy.spatial.transform import Rotation


class VisionNode(Node):
    '''노드 생성 및 초기화'''
    def __init__(self):
        super().__init__('vision_node')

        # RealSense 이미지 노드 초기화
        self.img_node = ImgNode()

        # 첫 프레임 받을 때까지 잠시 spin
        self.get_logger().info("[INFO] RealSense 초기화 중...")
        rclpy.spin_once(self.img_node)
        self.get_logger().info("[INFO] RealSense 초기화 완료!")
        time.sleep(0.5)

        # 카메라 캘리브레이션 설정
        self.intrinsics = self.img_node.get_camera_intrinsic()
        while self.intrinsics is None:
            self.get_logger().error("[ERROR] 카메라 intrinsic 정보를 불러오지 못했습니다.")
            time.sleep(1)
        current_dir = os.path.dirname(__file__)
        file_path = os.path.join(current_dir, "T_gripper2camera.npy")
        self.gripper2cam = np.load(file_path)

        # 로봇 상태 메시지 subscriber
        self.robot_state_subscription = self.create_subscription(RobotState, '/robot_state', self.robot_state_callback, 10)
        
        # 로봇 current_posx 메시지 subscriber
        self.robot_current_posx_subscription = self.create_subscription(RobotState, '/robot_current_posx', self.robot_current_posx_callback, 10)

        # QR 코드 정보 publisher
        self.qr_info_publisher = self.create_publisher(QRInfo, '/qr_info', 10)

        # 약 위치, 각도 publisher
        self.pill_loc_publisher = self.create_publisher(PillLoc, '/pill_loc', 10)

        # 서랍 text 위치 publisher
        self.text_loc_publisher = self.create_publisher(TextLoc, "/text_loc", 10)

        # YOLO 가중치 파일 이름, 신뢰도 설정
        self.diarrhea_yolo_weights = 'diarrhea.pt'
        self.dyspepsia_yolo_weights = 'dyspepsia.pt'
        self.dermatitis_yolo_weights = 'dermatitis_2.pt'
        self.cold_yolo_weights = 'cold.pt'
        self.CONFIDENCE = 0.50

        # 현재 로봇 상태 저장 변수
        self.robot_state = ''
        self.robot_current_posx = []

        # QR 코드가 최초로 인식되었는지 여부
        self.qr_detected = False
        self.detected_diseases = []

        '''추가'''
        # 집어야 하는 약의 리스트 (예: ['monodoxy_cap', 'monodoxy_cap', 'monodoxy_cap', 'ganakhan_tab', 'ganakhan_tab'])
        self.pill_list = []
        self.pill_list_index = 0
        
        '''추가'''
        # 약의 형태에 따라 원 또는 타원으로 추정하기 위한 리스트
        self.ellipse_pill_list = ['amoxicle_tab', 'sudafed_tab','monodoxy_cap', 'nexilen_tab', 'medilacsenteric_tab', 'otillen_tab']
        self.circle_pill_list = ['panstar_tab', 'ganakhan_tab', 'magmil_tab', 'samsung_octylonium_tab', 'famodine']
        
        # text_loc가 최초로 인식되었는지 여부
        self.text_loc_detected = False

        # YOLO 모델 관련 변수 초기화
        self.yolo_model = None
        self.yolo_start_time = None
        self.yolo_running = False

        # 약의 위치 및 각도를 저장하는 리스트 (x, y, theta)
        self.pill_loc = [0, 0, 0]


    '''로봇 current_posx 메시지 수신 시 호출되는 콜백 함수'''
    def robot_current_posx_callback(self, msg):
        # robot current_posx 갱신
        self.robot_current_posx = msg.current_posx
        self.get_logger().info(f'📥 Robot current_posx 수신')


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

            self.disease = 'dermatitis'  ############ 테스트용 ############

            if self.disease == 'diarrhea':
                self.yolo_weights = self.diarrhea_yolo_weights
            elif self.disease == 'dyspepsia':
                self.yolo_weights = self.dyspepsia_yolo_weights
            elif self.disease == 'dermatitis':
                self.yolo_weights = self.dermatitis_yolo_weights
            elif self.disease == 'cold':
                self.yolo_weights = self.cold_yolo_weights

            self.load_yolo_model()

        elif msg.robot_state == 'pick_pill':
            self.get_logger().info("[INFO] 로봇 pick pill 시작...")
            

    '''QR 코드를 탐지하고 시각화하는 함수'''
    def detect_qr(self, frame):
        # QR 코드 디코딩
        detector = cv2.QRCodeDetector()
        try:
            data, points, _ = detector.detectAndDecode(frame)
        except:
            data, points = None, None

        # 약코드 → 약이름
        code_to_drug = {
            "A02X1": "nexilen_tab",
            "A02AA04": "magmil_tab",
            "A07FA01": "medilacsenteric_tab",
            "A03AB06": "samsung_octylonium_tab",
            "A02BA03": "famodine",
            "A02X2": "otillen_tab",
            "M01AE14": "panstar_tab",
            "J01CR02": "amoxicle_tab",
            "R01BA02": "sudafed_tab",
            "J01AA02": "monodoxy_cap",
            "A03FA07": "ganakan_tab"
        }

        # 약이름 → 증상군
        drug_to_symptom = {
            "nexilen_tab": "dermatitis",
            "magmil_tab": "dermatitis",
            "monodoxy_cap": "dermatitis",
            "ganakan_tab": "dermatitis",
            "medilacsenteric_tab": "dyspepsia",
            "samsung_octylonium_tab": "diarrhea",
            "famodine": "diarrhea",
            "otillen_tab": "diarrhea",
            "panstar_tab": "cold",
            "amoxicle_tab": "cold",
            "sudafed_tab": "cold",
        }

        # QR 코드가 인식되었을 때
        if points is not None and data:
            # 인식된 QR 코드 영역에 사각형 박스 그리기
            points = points[0].astype(int)
            for i in range(len(points)):
                pt1 = tuple(points[i])
                pt2 = tuple(points[(i + 1) % len(points)])
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(frame, data, (points[0][0], points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # QR 코드가 처음 인식된 경우에만 퍼블리시
            if not self.qr_detected:
                lines = data.strip().split("\n")
                name_id = lines[0]
                prescriptions = lines[1:]

                # 증상군 → 약 이름 리스트 매핑
                self.required_pills = []
                self.required_counts = {}
                self.detected_pill_counts = defaultdict(int)
                self.detected_pill_locs = defaultdict(list)
                symptom_to_pills = defaultdict(list)
                symptom_to_dosages = defaultdict(list)

                self.qr_detected = True

                self.get_logger().info(f"✅ QR 코드 인식됨\n{data}")
                self.get_logger().info(f"🧾 환자: {name_id}")

                for line in prescriptions:
                    parts = line.strip().split()
                    if not parts:
                        continue

                    code = parts[0]
                    drug = code_to_drug.get(code, "unknown")
                    symptom = drug_to_symptom.get(drug, "unknown")

                    # 약과 복용 정보 저장
                    symptom_to_pills[symptom].append(drug)
                    dosage_info = " ".join(parts[1:])  # 예: "1 3 4"
                    symptom_to_dosages[symptom].append(dosage_info)

                for symptom, pills in symptom_to_pills.items():
                    dosages = symptom_to_dosages[symptom]
                    self.get_logger().info(f"💊 병: {symptom}, 약: {pills}, 복용: {dosages}")

                    self.detected_diseases.append(symptom)
                    calculated_dosages = []
                    for dosage in dosages:
                        try:
                            parts = dosage.strip().split()
                            if len(parts) >= 3:
                                times_per_day = int(parts[1])
                                total_days = int(parts[2])
                                total_count = times_per_day * total_days
                                calculated_dosages.append(str(total_count))
                            else:
                                calculated_dosages.append("0")
                        except ValueError:
                            calculated_dosages.append("0")

                    for pill, dosage in zip(pills, calculated_dosages):
                        if pill not in self.required_pills:
                            self.required_pills.append(pill)
                        self.required_counts[pill] = int(dosage) if dosage.isdigit() else 0
                    self.get_logger().info(f"📦 필요한 약 목록: {self.required_pills}")
                    self.get_logger().info(f"📦 약별 필요한 개수: {self.required_counts}")

                    '''추가'''
                    # 집어야 하는 약의 리스트 생성 (예: ['monodoxy_cap', 'monodoxy_cap', 'monodoxy_cap', 'ganakhan_tab', 'ganakhan_tab'])
                    for pill_name in self.required_pills:
                        for _ in range(self.required_counts[pill_name]):
                            self.pill_list.append(pill_name)
                    print(f'self.pill_list = {self.pill_list}')

                    # 메시지에 담아 publish
                    qr_msg = QRInfo()
                    qr_msg.disease = symptom
                    qr_msg.pill = pills
                    qr_msg.dosages = calculated_dosages
                    qr_msg.total_pills_count = len(self.pill_list)
                    self.qr_info_publisher.publish(qr_msg)
                    self.get_logger().info(f"📤 QR info publish: 병{symptom}, 약={pills}, 복용={calculated_dosages}")
                    self.get_logger().info(f"📤 QR info publish: 총 처방할 약의 개수 = {qr_msg.total_pills_count}")

        return frame


    '''서랍의 text를 classification하는 함수'''
    def load_text_model(self, frame):
        # 📌 설정
        package_share_directory = get_package_share_directory('rokey_project')

        CLASSIFIER_PATH = os.path.join(package_share_directory, 'weights', 'text_classifier.pth')
        CLASSIFICATION_SIZE = (64, 128)
        CONFIDENCE = 0.75

        # 🧠 Classification 모델 로드
        checkpoint = torch.load(CLASSIFIER_PATH)
        model_state = checkpoint["model_state_dict"]
        classification_classes = checkpoint["class_names"]

        classifier = models.resnet18(weights="IMAGENET1K_V1")
        classifier.fc = torch.nn.Linear(classifier.fc.in_features, len(classification_classes))
        classifier.load_state_dict(model_state)
        classifier.eval()
        classifier = classifier.cuda() if torch.cuda.is_available() else classifier.cpu()

        # 🔄 분류용 전처리 정의
        transform = transforms.Compose([
            transforms.Resize(CLASSIFICATION_SIZE),
            transforms.ToTensor(),
            transforms.Normalize((0.5,), (0.5,))
        ])

        # YOLO 로드
        weights = os.path.join(package_share_directory, 'weights', 'text.pt')
        yolo_model = YOLO(weights)

        PRESET_COLORS = [
            (255, 0, 0),     # 빨강
            (0, 255, 0),     # 초록
            (0, 0, 255),     # 파랑
            (255, 255, 0),   # 노랑
        ]

        # 색상 매핑 (클래스 개수만큼만 잘라서 매핑)
        class_colors = {
            class_name: PRESET_COLORS[i % len(PRESET_COLORS)]
            for i, class_name in enumerate(classification_classes)
        }

        # YOLO 감지
        results = yolo_model(frame, verbose=False)
        boxes = [box for box in results[0].boxes if box.conf.item() >= CONFIDENCE]

        annotated_frame = frame.copy()

        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            crop = frame[y1:y2, x1:x2]

            #  분류기 입력 준비
            image = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
            image = transform(image).unsqueeze(0)
            image = image.cuda() if torch.cuda.is_available() else image.cpu()

            with torch.no_grad():
                output = classifier(image)
                probabilities = torch.softmax(output, dim=1)
                conf, predicted = torch.max(probabilities, 1)
                class_name = classification_classes[predicted.item()]
                confidence = conf.item()

            #  병 이름과 일치하면 좌표 출력
            if class_name in self.detected_diseases:
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                height, width, _ = frame.shape
                # 구역 판별
                if center_x < width // 2 and center_y < height // 2:
                    loc = 3 # 좌상
                elif center_x >= width // 2 and center_y < height // 2:
                    loc= 4  # 우상
                elif center_x < width // 2 and center_y >= height // 2:
                    loc = 1  # 좌하
                else:
                    loc = 2  # 우하
                
                # 🎨 classifier 클래스 기준 색상
                color = class_colors.get(class_name, (0, 255, 0))
                label = f"{class_name} ({confidence:.2f})"

                # 바운딩 박스 및 라벨 시각화
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255,0,255) , 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # text_loc이 처음 인식된 경우에만 퍼블리시
                if not self.text_loc_detected:
                    self.text_loc_detected = True
                    self.get_logger().info(f"✅ QR 코드 병명 '{class_name}' 텍스트 인식됨!")
                    self.get_logger().info(f"📍 위치 좌표: x = {center_x}, y = {center_y}, 구역 = {loc}")

                    msg = TextLoc()
                    msg.text_loc = loc
                    self.text_loc_publisher.publish(msg)

        return annotated_frame


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
    def detect_pill_yolo(self, frame):
        if not self.yolo_running or self.yolo_model is None:
            # 모델이 준비되지 않았으면 원본 프레임 반환
            return frame

        results = self.yolo_model(frame, verbose=False)

        annotated_frame = frame.copy()

        # ROI 사각형 그리기
        if self.disease == 'diarrhea':
            roi_start = (298, 168)
            roi_end = (488, 258)
        elif self.disease == 'dyspepsia':
            roi_start = (323, 176)
            roi_end = (508, 256)
        elif self.disease == 'dermatitis':
            roi_start = (285, 170)
            roi_end = (463, 258)
        elif self.disease == 'cold':
            roi_start = (287, 185)
            roi_end = (477, 280)
        cv2.rectangle(annotated_frame, roi_start, roi_end, (255, 255, 255), 1)

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

                # 마스크 내 픽셀 좌표 기반 클래스 이름 텍스트 출력
                ys, xs = np.where(mask_bool)
                if len(xs) > 0 and len(ys) > 0:
                    x1, y1 = np.min(xs), np.min(ys)
                    cv2.putText(annotated_frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # 세그멘테이션 마스크의 외곽선을 사용하여 원 또는 타원 추정
                if contours and len(contours[0]) >= 5:
                    # 약 모양이 타원형일 때 타원 모양 추정
                    if class_name in self.ellipse_pill_list:
                        ellipse = cv2.fitEllipse(contours[0])
                        (center, axes, angle) = ellipse

                        # 타원 그리기
                        if ellipse[1][0] > 0 and ellipse[1][1] > 0:
                            cv2.ellipse(annotated_frame, ellipse, color, 2)
                        else:
                            print(f"[경고] 유효하지 않은 ellipse: {ellipse}")
                            
                        # 회전 각도, 중심점 좌표 텍스트 출력
                        angle_text = f"{angle:.1f} deg"
                        center_text = f"({int(center[0])}, {int(center[1])})"
                        cv2.putText(annotated_frame, angle_text, (int(center[0]) + 35, int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.putText(annotated_frame, center_text, (int(center[0]) + 35, int(center[1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    # 약 모양이 원형일 때 원 모양 추정
                    elif class_name in self.circle_pill_list:
                        (x, y), radius = cv2.minEnclosingCircle(contours[0])
                        center = (int(x), int(y))
                        radius = int(radius)
                        angle = 0

                        center_text = f"({int(center[0])}, {int(center[1])})"
                        cv2.circle(annotated_frame, center, radius, color, 2)
                        cv2.putText(annotated_frame, center_text, (int(center[0]) + 35, int(center[1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                    '''추가'''
                    # 집어야하는 약 순서대로 좌표 저장 (예: ['monodoxy_cap', 'monodoxy_cap', 'monodoxy_cap', 'ganakhan_tab', 'ganakhan_tab'])
                    # ROI 안에 있는 약만 저장
                    if (class_name == self.pill_list[self.pill_list_index]
                        and roi_start[0] <= int(center[0]) <= roi_end[0]
                        and roi_start[1] <= int(center[1]) <= roi_end[1]
                    ):
                        # 약 위치 저장
                        self.pill_loc = [int(center[0]), int(center[1]), int(angle)]

        # 일정 시간 경과 후 YOLO 모델 종료 처리
        elapsed = time.time() - self.yolo_start_time
        second = 2.0
        if elapsed > second:
            self.get_logger().info(f"[INFO] YOLO 모델 {second}초 경과, 메모리 해제 중...")
            self.yolo_model = None
            self.yolo_running = False
            self.get_logger().info("[INFO] YOLO 모델 메모리 해제 완료!")

            '''추가'''
            # self.pill_list_index가 처방해야 할 약의 총 개수보다 높으면 루프 종료
            if len(self.pill_list) <= self.pill_list_index:
                self.get_logger().info("[INFO] 약 모두 처방 완료!")
            else:
                # 약의 img 좌표를 robot base 좌표로 변환
                x_base, y_base, z_base = self.coordinate_transformation(self.pill_loc[0], self.pill_loc[1])

                pill_name = self.pill_list[self.pill_list_index]
                total = self.pill_list.count(pill_name)
                index = self.pill_list[:self.pill_list_index].count(pill_name)

                pill_loc_msg = PillLoc()
                pill_loc_msg.x = int(x_base)
                pill_loc_msg.y = int(y_base)
                pill_loc_msg.theta = self.pill_loc[2]
                pill_loc_msg.pill_name = pill_name
                pill_loc_msg.index = index
                pill_loc_msg.total = total

                self.pill_loc_publisher.publish(pill_loc_msg)
                self.get_logger().info(f"📤 Pill publish: {pill_name} ({index+1}/{total}) → (x : {pill_loc_msg.x}, y : {pill_loc_msg.y}, theta : {pill_loc_msg.theta})")

                self.pill_list_index += 1

                # self.get_logger().info(f"📤 Pill location publish: {pill_loc_msg}")
                # self.get_logger().info(f"📤 Pill location (x_base = {pill_loc_msg.x}, y_base = {pill_loc_msg.y}, theta = {pill_loc_msg.theta})")

        return annotated_frame
    

    '''img 좌표에서 robot base 좌표로 변환하는 함수'''
    def coordinate_transformation(self, x, y):
        depth_frame = self.img_node.get_depth_frame()
        while depth_frame is None or np.all(depth_frame == 0):
            self.get_logger().info("retry get depth img")
            rclpy.spin_once(self.img_node)
            depth_frame = self.img_node.get_depth_frame()

        # print(f"img cordinate: ({x}, {y})")
        z = self.get_depth_value(x, y, depth_frame)
        camera_center_pos = self.get_camera_pos(x, y, z, self.intrinsics)
        # print(f"camera cordinate: ({camera_center_pos})")

        gripper_coordinate = self.transform_to_base(camera_center_pos)
        # print(f"gripper cordinate: ({gripper_coordinate})")

        return gripper_coordinate

    def get_depth_value(self, center_x, center_y, depth_frame):
        height, width = depth_frame.shape
        if 0 <= center_x < width and 0 <= center_y < height:
            depth_value = depth_frame[center_y, center_x]
            return depth_value
        self.get_logger().warn(f"out of image range: {center_x}, {center_y}")
        return None
    
    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        camera_z = center_z

        return (camera_x, camera_y, camera_z)
    
    def transform_to_base(self, camera_coords):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        # gripper2cam = np.load(self.gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        base2gripper = self.get_robot_pose_matrix(*self.robot_current_posx)
        timer = time.time()

        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]
    
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T



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
            frame = self.detect_pill_yolo(frame)
        elif self.robot_state == 'check_text':
            frame = self.load_text_model(frame)
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
