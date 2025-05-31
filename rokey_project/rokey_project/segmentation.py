'''카메라를 열고 객체의 segmentation 마스크를 그려주는 파일'''

import cv2
import random
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# 기본 카메라: 0, realsense gray: 4, realsense: 6
CAMERA_NUM = 4

# 신뢰도
CONFIDENCE = 0.40

# YOLO 모델 로드
package_share_directory = get_package_share_directory('rokey_project')
weights = os.path.join(package_share_directory, 'weights', 'dermatitis_2.pt')
model = YOLO(weights)

# 클래스별 고유 색상 생성 (랜덤 색상 생성)
class_names = model.names  # 딕셔너리 형태: {0: 'class0', 1: 'class1', ...}
class_colors = {cls_id: (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)) for cls_id in class_names}

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

    # YOLO 추론 수행 (단일 프레임에 대해 객체 검출 및 분할 실행)
    results = model(frame)

    # 출력용 프레임 복사 (마스크, 텍스트 등을 이 위에 시각화함)
    annotated_frame = frame.copy()

    # 마스크 결과가 존재할 경우 처리
    if results and results[0].masks is not None:
        masks = results[0].masks.data.cpu().numpy()  # 세그멘테이션 마스크 (num_masks, H, W)
        boxes = results[0].boxes    # 해당 마스크에 대한 바운딩 박스 정보

        # 모든 탐지된 객체에 대해 반복
        for i, box in enumerate(boxes):
            # 일정 신뢰도 이상인 박스만 필터링
            conf = box.conf.item()
            if conf < CONFIDENCE:
                continue

            cls = int(box.cls[0])   # 클래스 ID
            class_name = class_names[cls]   # 클래스 이름
            color = class_colors.get(cls, (0, 255, 0))  # 클래스 색상

            mask = masks[i]  # 현재 객체의 마스크 (H, W)
            mask_bool = mask > 0.5 # 마스크 이진화 (True/False로 표현)

            # 클래스 색상으로 채워진 마스크 생성 (컬러 마스크)
            colored_mask = np.zeros_like(annotated_frame, dtype=np.uint8)
            colored_mask[:, :, 0] = color[0]
            colored_mask[:, :, 1] = color[1]
            colored_mask[:, :, 2] = color[2]

            # 투명도 적용하여 원본 프레임에 컬러 마스크 오버레이
            alpha = 0.5
            annotated_frame[mask_bool] = cv2.addWeighted(colored_mask, alpha, annotated_frame, 1 - alpha, 0)[mask_bool]

            # 마스크의 픽셀 좌표 추출 (텍스트 위치 계산용)
            ys, xs = np.where(mask_bool)
            if len(xs) > 0 and len(ys) > 0:
                x1, y1 = np.min(xs), np.min(ys)
                # 텍스트 그리기 (클래스 이름)
                # cv2.putText(annotated_frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # 화면 출력
    cv2.imshow(f"YOLOv11 Webcam Segmentation (Conf >= {CONFIDENCE})", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
