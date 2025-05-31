"""웹캠에 보이는 사진을 찍고 저장하는 파일"""

import cv2
import os
from datetime import datetime

# 카메라 번호(웹캠:0, realsense(gray):4, realsense(rgb):6)
CAMERA_NUM = 4

# 저장 경로 설정(경로 설정 주의)
save_dir = '/home/hongha/rokey_pharmacy_ws/src/rokey_project/image/object_detection'
os.makedirs(save_dir, exist_ok=True)  # 디렉토리가 없으면 생성

# 웹캠 열기
cap = cv2.VideoCapture(CAMERA_NUM)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

print("스페이스바를 눌러 사진을 찍고, ESC 키를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 프레임 화면에 출력
    cv2.imshow('Webcam', frame)

    key = cv2.waitKey(1)

    if key == 27:  # ESC 키
        break
    elif key == 32:  # 스페이스바
        # 현재 시간을 문자열로 포맷
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(save_dir, f'captured_{timestamp}.jpg')
        cv2.imwrite(filename, frame)
        print(f"{filename} 파일로 저장했습니다.")

# 자원 해제
cap.release()
cv2.destroyAllWindows()
