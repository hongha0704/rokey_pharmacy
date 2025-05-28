import cv2
import random
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# 기본 카메라: 0, realsense gray: 4, realsense: 6
CAMERA_NUM = 6

# 신뢰도
CONFIDENCE = 0.40

# YOLO 모델 로드
package_share_directory = get_package_share_directory('rokey_project')
weights = os.path.join(package_share_directory, 'weights', 'diarrhea_segmentation_best.pt')

model = YOLO(weights)

# 클래스별 고유 색상 생성
class_names = model.names  # 딕셔너리 형태: {0: 'class0', 1: 'class1', ...}
class_colors = {cls_id: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                for cls_id in class_names}

# 웹캠 열기
cap = cv2.VideoCapture(CAMERA_NUM)

if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 읽을 수 없습니다.")
        break

    # YOLO 추론
    results = model(frame)

    annotated_frame = frame.copy()

    if results and results[0].masks is not None:
        masks = results[0].masks.data.cpu().numpy()  # (num_masks, H, W)
        boxes = results[0].boxes

        for i, box in enumerate(boxes):
            conf = box.conf.item()
            if conf < CONFIDENCE:
                continue

            cls = int(box.cls[0])
            class_name = class_names[cls]
            color = class_colors.get(cls, (0, 255, 0))

            mask = masks[i]
            colored_mask = np.zeros_like(annotated_frame, dtype=np.uint8)
            colored_mask[:, :, 0] = color[0]
            colored_mask[:, :, 1] = color[1]
            colored_mask[:, :, 2] = color[2]

            mask_bool = mask > 0.5

            alpha = 0.5
            annotated_frame[mask_bool] = cv2.addWeighted(colored_mask, alpha, annotated_frame, 1 - alpha, 0)[mask_bool]

            # 마스크 영역의 경계 박스 계산해서 텍스트 위치 지정
            ys, xs = np.where(mask_bool)
            if len(xs) > 0 and len(ys) > 0:
                x1, y1 = np.min(xs), np.min(ys)
                # 텍스트 그리기 (클래스 이름)
                cv2.putText(annotated_frame, class_name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 화면 출력
    cv2.imshow(f"YOLOv11 Webcam Segmentation (Conf >= {CONFIDENCE})", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
