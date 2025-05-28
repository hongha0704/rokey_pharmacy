import cv2
import numpy as np

# 웹캠 열기 (기본 카메라는 0번, 다르면 1, 2 등으로 바꾸세요)
cap = cv2.VideoCapture(6)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 1. 흑백 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 2. 이진화
    _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

    # 3. 윤곽선 추출
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    for cnt in contours:
        if len(cnt) >= 5:
            try:
                ellipse = cv2.fitEllipse(cnt)
                (x, y), (MA, ma), angle = ellipse
                print(f"Angle: {angle:.1f}")
            except Exception as e:
                print(f"Failed to fit ellipse: {e}")

            # 타원 그리기
            cv2.ellipse(frame, ellipse, (0, 255, 0), 2)

            # 중심점 표시
            cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)

            # 회전각 텍스트 출력
            text = f"Angle: {angle:.1f}"
            cv2.putText(frame, text, (int(x) + 10, int(y) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            print(text)

    # 결과 프레임 출력
    try:
        cv2.imshow("Binary", binary)
        cv2.waitKey(1)
    except cv2.error as e:
        print("imshow error:", e)

    # 'q' 키 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
