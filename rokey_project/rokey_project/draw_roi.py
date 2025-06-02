import cv2

# 전역 변수
drawing = False        # 마우스 클릭 여부
ix, iy = -1, -1        # 시작 좌표
roi = None             # ROI 좌표 저장

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, roi, frame_copy

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        frame_copy = frame.copy()

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            frame[:] = frame_copy.copy()
            cv2.rectangle(frame, (ix, iy), (x, y), (0, 255, 0), 2)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        roi = (min(ix, x), min(iy, y), abs(ix - x), abs(iy - y))
        print(f"ROI selected: x={roi[0]}, y={roi[1]}, w={roi[2]}, h={roi[3]}")
        cv2.rectangle(frame, (roi[0], roi[1]), (roi[0]+roi[2], roi[1]+roi[3]), (0, 255, 0), 2)

# 비디오 캡처 시작 (웹캠)
cap = cv2.VideoCapture(0)

cv2.namedWindow("Select ROI")
cv2.setMouseCallback("Select ROI", draw_rectangle)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if not drawing:
        frame_copy = frame.copy()

    cv2.imshow("Select ROI", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):  # 종료
        break
    elif key == ord('s') and roi is not None:  # ROI 잘라서 저장
        x, y, w, h = roi
        roi_img = frame[y:y+h, x:x+w]
        cv2.imshow("Selected ROI", roi_img)
        cv2.imwrite("roi_selected.png", roi_img)
        print("ROI 이미지 저장됨: roi_selected.png")

cap.release()
cv2.destroyAllWindows()
