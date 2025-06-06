'''QR코드 인식을 테스트하는 파일'''

import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import QRInfo
import cv2
import numpy as np
from collections import defaultdict


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.qr_info_publisher = self.create_publisher(QRInfo, '/qr_info', 10)

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
                    "medilacsenteric_tab": "dyspepsia",
                    "samsung_octylonium_tab": "diarrhea",
                    "famodine": "diarrhea",
                    "otillen_tab": "diarrhea",
                    "panstar_tab": "cold",
                    "amoxicle_tab": "cold",
                    "sudafed_tab": "cold",
                    "monodoxy_cap": "dermatitis",
                    "ganakan_tab": "dermatitis"
                }

                if bbox is not None and data:
                    # 바운딩 박스 그리기
                    bbox = np.int32(bbox).reshape(-1, 2)
                    for i in range(len(bbox)):
                        pt1 = tuple(bbox[i])
                        pt2 = tuple(bbox[(i + 1) % len(bbox)])
                        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                    cv2.putText(frame, data, (bbox[0][0], bbox[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # if not self.qr_detected:
                lines = data.strip().split("\n")
                name_id = lines[0]
                prescriptions = lines[1:]

                # 증상군 → 약 이름 리스트 매핑
                symptom_to_pills = defaultdict(list)

                for line in prescriptions:
                    parts = line.strip().split()
                    if not parts:
                        continue
                    code = parts[0]
                    drug = code_to_drug.get(code, "unknown")
                    symptom = drug_to_symptom.get(drug, "unknown")
                    symptom_to_pills[symptom].append(drug)

                # self.qr_detected = True
                self.get_logger().info(f"✅ QR 코드 인식됨\n{data}")
                self.get_logger().info(f"🧾 환자: {name_id}")

                for symptom, pills in symptom_to_pills.items():
                    self.get_logger().info(f"💊 병: {symptom}, 약: {pills}")

                    # 메시지에 담아 publish
                    qr_msg = QRInfo()
                    qr_msg.disease = symptom
                    qr_msg.pill = pills
                    self.qr_info_publisher.publish(qr_msg)
                    self.get_logger().info(f"📤 QR info publish: 병={symptom}, 약={pills}")

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
    