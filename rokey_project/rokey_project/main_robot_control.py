import rclpy
import DR_init
import time
from rokey_project.onrobot import RG
from rclpy.node import Node

from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import RobotState
from rokey_interfaces.msg import QRInfo
from rokey_interfaces.msg import PillLoc
from rokey_interfaces.msg import TextLoc
from rokey_interfaces.msg import Medicine
from rokey_interfaces.msg import MedicineArray

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# gripper
GRIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

DR_init.__dsr__node = node

try:
    from DSR_ROBOT2 import (
        set_tool,
        set_tcp,
        movej,
        movel,
        movesj,
        get_current_posx,
        task_compliance_ctrl,
        release_compliance_ctrl,
        check_force_condition,
        set_desired_force,
        movec,
        release_force,
        move_periodic,
        DR_MV_MOD_REL, DR_FC_MOD_REL,
        DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_TOOL,
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")


## vel, acc, pos setting
VELOCITY, ACC = 60, 60

# home pos
JReady = posj(0, 0, 90, 0, 90, 0)

'''처방 pos'''
# qr코드 확인하는 pos
Jcheck_qr_waypoint = posj(11.65, -2.53, 104.26, 38.19, 15.43, 47.83)
Jcheck_qr = posj(36.46, 15.43, 103.02, 105.37, -124.10, 30.93)

# text 확인/분류하는 pos
Jcheck_text_waypoint = posj(41.83, -11.13, 118.59, 88.00, -59.42, 30.94)
Jcheck_text = posj(18.57, 25.38, 133.04, 27.09, -53.22, 78.63)
Jdrawer_common_waypoint = posj(33.86, 10.54, 133.05, 59.08, -53.21, -19.66)

# 서랍장 1 여는 pos
Jdrawer_1_waypoint = posj(35.66, 21.27, 123.61, 66.64, -51.36, -22.32)
Jdrawer_1_before = posj(41.24, 44.64, 114.95, 64.66, -78.87, -16.58)
Jdrawer_1 = posj(38.72, 46.16, 110.99, 62.74, -76.97, -18.58)
Jdrawer_1_campose_waypoint_1 = posj(53.62, -2.45, 132.33, 90.67, -49.28, -8.03)
Jdrawer_1_campose_waypoint_2 = posj(10.43, -1.98, 95.34, -0.27, 66.35, 19.62)
Jdrawer_1_campose = posj(8.42, 37.14, 56.34, -0.04, 86.69, 8.17)  # 서랍과 약 40mm 떨어져 있음

# 서랍장 2 여는 pos
Jdrawer_2_waypoint = posj(16.08, 0.17, 126.43, 37.27, -6.41, -16.43)
Jdrawer_2_before = posj(34.09, 40.49, 115.27, 70.75, -73.73, -21.57)
Jdrawer_2 = posj(33.85, 48.76, 105.56, 73.53, -81.06, -23.81)
Jdrawer_2_campose_waypoint_1 = posj(54.40, 14.92, 133.18, 92.12, -77.52, -12.70)
Jdrawer_2_campose_waypoint_2 = posj(-5.98, -6.89, 117.01, 16.59, 25.99, 2.08)
Jdrawer_2_campose = posj(-1.23, 33.86, 60.91, -0.15, 85.20, -0.97)  # 서랍과 약 40mm 떨어져 있음

# 서랍장 3 여는 pos
Jdrawer_3_waypoint = posj(26.96, 0.46, 123.41, 55.28, -20.74, -1.47)
Jdrawer_3_before = posj(38.99, 24.64, 122.28, 60.31, -65.51, -23.13)
Jdrawer_3 = posj(40.34, 37.54, 106.66, 71.04, -76.01, -36.03)
Jdrawer_3_campose_waypoint_1 = posj(53.75, -8.13, 128.98, 98.52, -57.10, -38.47)
Jdrawer_3_campose_waypoint_2 = posj(3.61, -9.53, 109.52, 17.47, 29.35, -3.82)
Jdrawer_3_campose = posj(8.49, 39.57, 37.93, 0.01, 102.25, 8.01)  # 서랍과 약 40mm 떨어져 있음

# 서랍장 4 여는 pos
Jdrawer_4_before = posj(33.33, 28.64, 114.48, 72.34, -69.72, -39.56)
Jdrawer_4 = posj(34.58, 37.26, 104.93, 77.86, -78.05, -39.56)
Jdrawer_4_campose_waypoint_1 = posj(44.97, -4.13, 124.37, 103.19, -66.70, -68.25)
Jdrawer_4_campose_waypoint_2 = posj(-1.34, 1.23, 87.45, 10.53, 55.46, -15.12)
Jdrawer_4_campose = posj(-0.74, 37.13, 41.93, 0.00, 100.28, -1.19)  # 서랍과 약 40mm 떨어져 있음

# 약 봉투 위치(아침, 점심, 저녁)
Jpill_pouch_morning = posj(-57.74, -3.45, 112.01, -0.24, 71.29, -57.01)
Jpill_pouch_afternoon = posj(-52.54, -9.25, 117.53, 0.27, 71.57, -51.60)
Jpill_pouch_evening = posj(-45.24, -15.29, 122.61, -0.20, 72.47, -44.57)

# 서랍장 1 닫는 pos
Jclose_drawer_1_before = posj(16.31, 27.14, 70.41, -0.25, 83.05, 16.21)
Jclose_drawer_1 = posj(16.47, 28.58, 74.62, -0.25, 77.93, 16.22)
Jclose_drawer_1_waypoint_1 = posj(16.19, 30.73, 65.18, -0.26, 85.55, 15.70)
Jclose_drawer_1_waypoint_2 = posj(16.41, 33.27, 74.73, -0.26, 73.70, 15.70)
Xclose_drawer_1_finish = [622.86, 158.94, 20.69, 107.82, -179.98, 107.89]

# 서랍장 2 닫는 pos
Jclose_drawer_2_before = posj(5.32, 23.49, 75.73, -0.14, 80.71, 5.50)
Jclose_drawer_2 = posj(5.47, 24.73, 80.54, -0.14, 74.38, 5.50)
Jclose_drawer_2_waypoint_1 = posj(5.07, 27.96, 69.19, -0.12, 82.79, 4.88)
Jclose_drawer_2_waypoint_2 = posj(5.09, 30.05, 79.43, -0.12, 70.94, 4.88)
Xclose_drawer_2_finish = [617.19, 53.66, 14.59, 17.79, -179.33, 17.57]

# 서랍장 3 닫는 pos
Jclose_drawer_3_before = posj(16.28, 27.04, 57.88, 0.01, 95.40, 15.43)
Jclose_drawer_3 = posj(16.35, 27.34, 61.80, 0.01, 91.45, 15.43)
Jclose_drawer_3_waypoint_1 = posj(14.26, 33.54, 48.20, -0.21, 99.02, 12.43)
Jclose_drawer_3_waypoint_2 = posj(14.64, 33.18, 61.54, -0.22, 86.20, 12.68)
Xclose_drawer_3_finish = [622.89, 152.01, 94.94, 26.84, -178.81, 24.90]

# 서랍장 4 닫는 pos
Jclose_drawer_4_before = posj(5.02, 22.61, 62.98, -0.09, 94.10, 3.59)
Jclose_drawer_4 = posj(5.35, 22.61, 69.38, -0.09, 87.79, 3.59)
Jclose_drawer_4_waypoint_1 = posj(4.57, 30.89, 50.01, -0.08, 99.06, 2.70)
Jclose_drawer_4_waypoint_2 = posj(4.57, 29.28, 67.09, -0.08, 83.98, 2.70)
Xclose_drawer_4_finish = [620.25, 50.17, 99.27, 15.96, -179.43, 14.11]

# 약봉지를 종이 봉투에 넣어주는 pos
Jput_pill_in_bag_waypoint = posj(83.86, -4.41, 97.21, -0.70, 87.08, 0.00)
Xput_pill_in_bag = [39.54, 348.45, 30.77, 6.09, 179.24, -77.56]

'''비처방 pos'''
#서랍 보는 위치
Jpos1 = [-29.40, -7.43, 124.59, 78.41, 25.69, -2.70]

Jpos_A = [-19.94, 38.18, 97.09, 125.47, 23.01, -49.61]  # 1층 오른쪽
Jpos_B = [-9.91, 35.84, 101.08, 152.81, 21.22, -70.26]  # 1층 왼쪽
Jpos_C = [-19.75, 29.09, 79.78, 59.51, 25.19, 24.11]    # 2층 오른쪽
Jpos_D = [-9.66, 24.64, 86.72, 57.60, 11.25, 27.05]     # 2층 왼쪽

# 1층 movesx list - 왼쪽
floor_1_L_1 = posx(-15.35, 15.98, 120.60, 133.16, 19.83, -54.41)

# 2층 movec list 
floor_2_1 = posx(-30, 0, 40, 0, 0, 0)
floor_2_2 = posx(-60, 0, -40, 0, 0, 0)

# 1층 물품 빼낼 때 경유지점
Jpos_r = posj(-25.83, 24.10, 113.35, 118.33, 29.32, -47.92)         # 1층 오른쪽
Jpos_l = posj(-15.92, 20.66, 115.66, 125.60, 20.21, -47.92)         # 1층 왼쪽
Jpos_between = posj(-15.35, -9.32, 125.77, 153.57, -7.82, -67.06)   # 좌측 우측 공통 경유 지점
Put_place = posj(-35.25, 12.47, 90.79, 0.00, 76.73, -35.22)         # 임시 계산대
Jpos_R = [Jpos_r, Jpos_between, Put_place]                          # movesj list
Jpos_L = [Jpos_l, Jpos_between, Put_place]                          # movesj list

# 각 비처방 약들의 그립 크기
grip_dict = {"tylenol": 150, "bandage": 300, "sore_patch": 300, "codaewon_syrup": 80}


# 플래그 및 수신 데이터
qr_data_received = False
voice_received = False
text_loc_data_received = False
detected_flag = False
medicine_loc_received = False

# QR 정보 초기화
qr_disease = None
qr_pill_list = None
qr_total_pills_count = 0

# 약 위치 초기화
x_base, y_base, theta = 0, 0, 0
pill_name, index, total = None, None, None

# text 위치 초기화
text_loc = None

# 비처방약 위치, 이름 초기화
medicine_loc = 0
medicines_name = []

# robot_state publisher 생성
robot_state_publisher = node.create_publisher(RobotState, "/robot_state", 10)

# robot_current_posx publisher 생성
robot_current_posx_publisher = node.create_publisher(RobotState, "/robot_current_posx", 10)

# 비처방약 publisher 생성
medicine_publisher = node.create_publisher(Medicine, "/medicine_name", 10)

# 마무리 상태와 약 이름 publisher 생성
finish_publisher = node.create_publisher(TaskState, '/task_state', 10)


'''사람 감지 정보(초음파 센서)를 수신하는 콜백 함수'''
def task_state_callback(msg):
    global detected_flag
    if msg.state == "detected":
        detected_flag = True
        node.get_logger().info("🔔 state='detected' 메시지 수신!")
        

'''QR 코드 인식 위치로 이동하고 정보를 수신하는 함수'''
def move_check_qr():
    VELOCITY, ACC = 100, 100
    global detected_flag, qr_data_received
    node.get_logger().info("========== 🏁 move_check_qr() 시작! ==========")

    # 홈위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 초음파 subscription 대기 -> state == "detected" break, 구독 시작
    ultra_subscription = node.create_subscription(TaskState, '/task_state', task_state_callback, 10)

    # 감지 대기 루프
    node.get_logger().info("⏳ state='detected' 메시지 대기 중...")
    while not detected_flag:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info("✅사용자 감지됨")
    time.sleep(1)

    # qr code 체크하는 위치로 이동
    movesj([Jcheck_qr_waypoint, Jcheck_qr], vel=VELOCITY, acc=ACC)

    # QR code 정보 또는 Voice를 수신하는 subscriber 생성
    qr_info_subscription = node.create_subscription(QRInfo, "/qr_info", qr_callback, 10)
    voice_subscription = node.create_subscription(MedicineArray, "/medicine", voice_callback, 10)

    # 'check_qr' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'check_qr' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "check_qr"
    robot_state_publisher.publish(robot_state_msg)

    # QR 정보가 들어올 때까지 대기
    node.get_logger().info("🕐 QR 정보 또는 Voice 대기 중...")
    while rclpy.ok() and not qr_data_received and not voice_received:
        rclpy.spin_once(node, timeout_sec=0.1)  # 100ms 간격으로 체크

    time.sleep(1)

    # 더 이상 필요 없는 subscriber 제거
    node.destroy_subscription(qr_info_subscription)
    node.destroy_subscription(voice_subscription)
    node.destroy_subscription(ultra_subscription)
    node.get_logger().info("========== 🏁 move_check_qr() 종료 ==========")


'''QR 정보를 수신하는 콜백 함수'''
def qr_callback(msg):
    global qr_data_received, qr_disease, qr_pill_list, qr_total_pills_count
    if msg.disease != "":
        qr_disease = msg.disease
        qr_pill_list = msg.pill
        qr_total_pills_count = msg.total_pills_count
        qr_data_received = True
        node.get_logger().info(f"========== ✅ QR 정보 수신 ==========")
        node.get_logger().info(f"💊 병: {qr_disease}, 약: {qr_pill_list}")
        node.get_logger().info(f"💊 처방할 약의 총 개수: {qr_total_pills_count}")
        node.get_logger().info("========== QR 정보 수신 완료 ==========")


'''voice 수신을 수신하는 콜백 함수'''
def voice_callback(msg):
    global voice_received, medicines, medicines_name

    medicines = msg.medicines  # medicines (리스트)
    medicines_name = [medicines[i] for i in range(len(medicines)) if not i % 2]
    print(f"medicines_name = {medicines_name}")
    voice_received = True
    node.get_logger().info(f"========== ✅ Voice 수신 ==========")
    node.get_logger().info(f"💊 비처방약: {medicines}")
    node.get_logger().info("========== Voice 수신 완료 ==========")
    

'''서랍 텍스트 인식 위치로 이동하고 상태를 퍼블리시하는 함수'''
def move_check_text():
    VELOCITY, ACC = 100, 100

    node.get_logger().info("✅ QR 정보 수신 완료, 다음 동작으로 진행")
    time.sleep(1)

    node.get_logger().info("========== 🏁 move_check_text() 시작! ==========")

    # 텍스트 인식 위치로 이동
    movesj([Jcheck_text_waypoint, Jcheck_text], vel=VELOCITY, acc=ACC)

    # text_loc 정보 subscriber
    node.get_logger().info(f"서랍 text 인식중...")
    text_loc_subscription = node.create_subscription(TextLoc, "/text_loc", text_loc_callback, 10)

    # 'check_text' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'check_text' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "check_text"
    robot_state_publisher.publish(robot_state_msg)
    time.sleep(0.5)

    # text_loc 정보가 들어올 때까지 대기
    node.get_logger().info("🕐 text_loc 정보 대기 중...")
    while rclpy.ok() and not text_loc_data_received:
        rclpy.spin_once(node, timeout_sec=0.1)  # 100ms 간격으로 체크

    node.get_logger().info("✅ text_loc 정보 수신 완료, 서랍 여는 동작 진행")
    time.sleep(3)

    # 'open_drawer' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'open_drawer' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "open_drawer"
    robot_state_publisher.publish(robot_state_msg)
    time.sleep(0.5)

    node.get_logger().info("========== 🏁 move_check_text() 종료 ==========")


'''text_loc 정보를 수신하는 콜백 함수'''
def text_loc_callback(msg):
    global text_loc, text_loc_data_received
    text_loc_data_received = True
    text_loc = msg.text_loc
    node.get_logger().info(f"========== ✅ text location 수신 ==========")
    node.get_logger().info(f"📥 text_loc 수신됨: 서랍 번호 [{text_loc}]")
    node.get_logger().info("========== text location 수신 완료 ==========")


'''서랍장1 open motion'''
def open_drawer_1():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 open_drawer_1() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_common_waypoint, Jdrawer_1_before, Jdrawer_1], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 16mm)
    gripper.move_gripper(160)
    time.sleep(0.5)

    # 서랍장 열기 (x축 -140mm)
    movel([-140, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 조금 빠지기 (x축 -30mm)
    movel([-40, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    # 서랍장 cam pose로 이동
    movesj([Jdrawer_1_campose_waypoint_1, Jdrawer_1_campose_waypoint_2, Jdrawer_1_campose], vel=VELOCITY, acc=ACC)
    
    node.get_logger().info("========== 🏁 open_drawer_1() 종료 ==========")


'''서랍장2 open motion'''
def open_drawer_2():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 open_drawer_2() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_common_waypoint, Jdrawer_2_before, Jdrawer_2], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 16mm)
    gripper.move_gripper(160)
    time.sleep(0.5)

    # 서랍장 열기 (x축 -140mm)
    movel([-140, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 조금 빠지기 (x축 -30mm)
    movel([-40, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    # 서랍장 cam pose로 이동
    movesj([Jdrawer_2_campose_waypoint_1, Jdrawer_2_campose_waypoint_2, Jdrawer_2_campose], vel=VELOCITY, acc=ACC)
    
    node.get_logger().info("========== 🏁 open_drawer_2() 종료 ==========")


'''서랍장3 open motion'''
def open_drawer_3():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 open_drawer_3() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_common_waypoint, Jdrawer_3_before, Jdrawer_3], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 16mm)
    gripper.move_gripper(160)
    time.sleep(0.5)

    # 서랍장 열기 (x축 -140mm)
    movel([-140, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 조금 빠지기 (x축 -30mm)
    movel([-40, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    # 서랍장 cam pose로 이동
    movesj([Jdrawer_3_campose_waypoint_1, Jdrawer_3_campose_waypoint_2, Jdrawer_3_campose], vel=VELOCITY, acc=ACC)
    
    node.get_logger().info("========== 🏁 open_drawer_3() 종료 ==========")


'''서랍장4 open motion'''
def open_drawer_4():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 open_drawer_4() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_common_waypoint, Jdrawer_4_before, Jdrawer_4], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 16mm)
    gripper.move_gripper(160)
    time.sleep(0.5)

    # 서랍장 열기 (x축 -140mm)
    movel([-140, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 조금 빠지기 (x축 -30mm)
    movel([-40, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    # 서랍장 cam pose로 이동
    movesj([Jdrawer_4_campose_waypoint_1, Jdrawer_4_campose_waypoint_2, Jdrawer_4_campose], vel=VELOCITY, acc=ACC)
    
    node.get_logger().info("========== 🏁 open_drawer_4() 종료 ==========")


'''서랍 4개 중 하나를 선택하고 여는 함수'''
def select_and_open_drawer():
    node.get_logger().info("========== 🏁 select_and_open_drawer() 시작! ==========")

    global text_loc
    node.get_logger().info(f"💊 병: {qr_disease}, 약: {qr_pill_list}")

    if text_loc == 1:
        node.get_logger().info(f"🗄️  1번 서랍을 엽니다!")
        open_drawer_1()

    elif text_loc == 2:
        node.get_logger().info(f"🗄️  2번 서랍을 엽니다!")
        open_drawer_2()

    elif text_loc == 3:
        node.get_logger().info(f"🗄️  3번 서랍을 엽니다!")
        open_drawer_3()

    elif text_loc == 4:
        node.get_logger().info(f"🗄️  4번 서랍을 엽니다!")
        open_drawer_4()


'''서랍장 위 약을 인식하는 자세로 이동하는 함수'''
def move_drawer_campose():
    global text_loc

    if text_loc == 1:
        movej(Jdrawer_1_campose, vel=VELOCITY, acc=ACC)

    elif text_loc == 2:
        movej(Jdrawer_2_campose, vel=VELOCITY, acc=ACC)

    elif text_loc == 3:
        movej(Jdrawer_3_campose, vel=VELOCITY, acc=ACC)

    elif text_loc == 4:
        movej(Jdrawer_4_campose, vel=VELOCITY, acc=ACC)


'''약 탐지 상태와 로봇의 current_posx를 퍼블리시하는 함수'''
def publish_check_pill_state():
    node.get_logger().info("========== 🏁 publish_check_pill_state() 시작! ==========")

    # 'detect_pill' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'detect_pill' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "detect_pill"
    robot_state_publisher.publish(robot_state_msg)

    # 로봇의 current_posx를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'current_posx' 퍼블리시 중...")
    robot_current_posx_msg = RobotState()
    robot_current_posx_msg.current_posx = get_current_posx()[0]
    robot_current_posx_publisher.publish(robot_current_posx_msg)

    node.get_logger().info("========== 🏁 publish_check_pill_state() 종료 ==========")


'''약 위치와 자세 메시지를 subscribe하고, 약을 집는 함수'''
def pick_pill():
    VELOCITY, ACC = 100, 100

    global pill_name
    node.get_logger().info("========== 🏁 pick_pill() 시작! ==========")
    
    global x_base, y_base, theta, qr_disease, text_loc
    x_base, y_base, theta = 0, 0, 0  # 초기화

    # 약의 위치와 자세 정보를 수신하는 subscriber 생성
    pill_loc_subscription = node.create_subscription(PillLoc, "/pill_loc", pill_loc_callback, 10)

    # 약의 위치와 자세 정보를 수신할 때까지 spin
    while not x_base:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(pill_loc_subscription)

    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "pick_pill"
    robot_state_publisher.publish(robot_state_msg)
    node.get_logger().info(f"📤 'pick_pill' 상태 퍼블리시 중...")
    time.sleep(1)

    # 서랍의 위치 별 z값 설정
    if text_loc == 1:
        z = 23.00
    elif text_loc == 2:
        z = 24.12
    elif text_loc == 3:
        z = 111.00
        # y축 오차로 인한 보정
        y_base -= 5
    elif text_loc == 4:
        z = 111.00
        # x, y축 오차로 인한 보정
        x_base -= 3
        y_base -= 5
    node.get_logger().info(f"💊 x = {x_base}, y = {y_base}, z = {z}")

    # 약 있는 위치의 x, y 좌표로 가고, 6축을 theta만큼 회전하기
    current_pos = get_current_posx()[0]
    pick_pos = posx([x_base, y_base, current_pos[2], current_pos[3], current_pos[4], current_pos[5]])
    movel(pick_pos, vel=VELOCITY, acc=ACC)
    movej([0, 0, 0, 0, 0, theta], vel=VELOCITY, acc=ACC, mod=1)

    # 약에 따라서 그리퍼 너비 조정
    if pill_name == 'amoxicle_tab' or pill_name == 'panstar_tab' or pill_name == 'magmil_tab':
        gripper.move_gripper(170)   # 그리퍼 17mm 만큼 열기
    else:
        gripper.move_gripper(150)   # 그리퍼 15mm 만큼 열기
    time.sleep(1)

    # 약 있는 위치로 내리기
    current_pos = get_current_posx()[0]
    pick_pos_down = posx([current_pos[0], current_pos[1], z, current_pos[3], current_pos[4], current_pos[5]])
    movel(pick_pos_down, vel=VELOCITY, acc=ACC)

    # 순응제어 on
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(0.5)

    # 약에 따라서 그리퍼 너비 조정
    if pill_name == 'amoxicle_tab' or pill_name == 'panstar_tab' or pill_name == 'magmil_tab':
        gripper.move_gripper(100)   # 그리퍼 10mm 만큼 열기
    else:
        gripper.move_gripper(80)    # 그리퍼 8mm로 닫기
    time.sleep(0.5)

    # 순응제어 off
    release_compliance_ctrl()
    time.sleep(0.5)

    # 집고 z축으로 200mm 올리기
    movel([0, 0, 80, 0, 0, 0], vel=VELOCITY, acc=VELOCITY, mod=1)

    node.get_logger().info("========== 🏁 pick_pill() 종료 ==========")


'''약 위치 정보를 수신하는 콜백 함수'''
def pill_loc_callback(msg):
    global x_base, y_base, theta, pill_name, index, total
    x_base = msg.x
    y_base = msg.y
    theta = msg.theta

    pill_name = msg.pill_name
    index = msg.index
    total = msg.total

    # theta가 90도 이상이면, 반대방향으로 회전(회전하는 각도 최소화 하기 위해)
    if theta > 90:
        theta -= 180

    node.get_logger().info(f"========== ✅ 약 위치, 자세 정보 수신 ==========")
    node.get_logger().info(f"💊 x = {x_base}, y = {y_base}, theta = {theta}")
    node.get_logger().info(f"📥 {pill_name} 위치 수신: index={index+1}/{total}")

    node.get_logger().info("========== 약 위치, 자세 정보 수신 완료 ==========")


'''약을 약 봉투 위치에 place하는 함수'''
def place_pill():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 place_pill() 시작! ==========")

    global pill_name, index, total

    if total == 1:
        target = Jpill_pouch_afternoon
    elif total == 2:
        target = [Jpill_pouch_morning, Jpill_pouch_evening][index % 2]
    elif total >= 3:
        target = [Jpill_pouch_morning, Jpill_pouch_afternoon, Jpill_pouch_evening][index % 3]
    else:
        target = JReady

    node.get_logger().info(f"📦 {pill_name} {index+1}/{total} → {target}")
    movesj([JReady, target], vel=VELOCITY, acc=ACC)

    # 그리퍼 15mm 만큼 열기
    gripper.move_gripper(150)
    time.sleep(0.2)

    # 한번 털기
    move_periodic(amp =[0,0,10,0,0,0], period=0.7, atime=0.2, repeat=1, ref=DR_TOOL)
    time.sleep(0.2)

    node.get_logger().info("========== 🏁 place_pill() 종료 ==========")


'''서랍 4개 중 하나를 선택하고 닫는 함수'''
def select_and_close_drawer():
    node.get_logger().info("========== 🏁 select_and_close_drawer() 시작! ==========")

    global text_loc

    if text_loc == 1:
        node.get_logger().info(f"🗄️  1번 서랍을 닫습니다!")
        close_drawer_1()

    elif text_loc == 2:
        node.get_logger().info(f"🗄️  2번 서랍을 닫습니다!")
        close_drawer_2()

    elif text_loc == 3:
        node.get_logger().info(f"🗄️  3번 서랍을 닫습니다!")
        close_drawer_3()

    elif text_loc == 4:
        node.get_logger().info(f"🗄️  4번 서랍을 닫습니다!")
        close_drawer_4()


'''서랍장1 close motion'''
def close_drawer_1():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 close_drawer_1() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 닫는 위치로 이동
    movesj([JReady, Jclose_drawer_1_before, Jclose_drawer_1], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 17mm)
    gripper.move_gripper(170)
    time.sleep(1)

    # 서랍장 살짝 들기 (z축 6mm, x축 -6mm)
    movel([-6.0, 0, 6.0, 0, 0, 0], vel=50, acc=50, mod=1)
    time.sleep(0.5)

    # 서랍장 넣기 (x축 80mm)
    movel([80.0, 0, 0, 0, 0, 0], vel=30, acc=30, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 위로 빠지기 (z축 40mm)
    movel([0, 0, 40.0, 0, 0, 0], vel=50, acc=50, mod=1)

    # 서랍장 닫기 마무리
    movesj([Jclose_drawer_1_waypoint_1, Jclose_drawer_1_waypoint_2], vel=VELOCITY, acc=ACC)
    movel(Xclose_drawer_1_finish, vel=30, acc=30)

    movej(JReady, vel=VELOCITY, acc=ACC)

    node.get_logger().info("========== 🏁 close_drawer_1() 종료 ==========")


'''서랍장2 close motion'''
def close_drawer_2():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 close_drawer_2() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 닫는 위치로 이동
    movesj([JReady, Jclose_drawer_2_before, Jclose_drawer_2], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 17mm)
    gripper.move_gripper(170)
    time.sleep(1)

    # 서랍장 살짝 들기 (z축 6mm, x축 -6mm)
    movel([-6.0, 0, 6.0, 0, 0, 0], vel=50, acc=50, mod=1)
    time.sleep(0.5)

    # 서랍장 넣기 (x축 80mm)
    movel([80.0, 0, 0, 0, 0, 0], vel=30, acc=30, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 위로 빠지기 (z축 40mm)
    movel([0, 0, 40.0, 0, 0, 0], vel=50, acc=50, mod=1)

    # 서랍장 닫기 마무리
    movesj([Jclose_drawer_2_waypoint_1, Jclose_drawer_2_waypoint_2], vel=VELOCITY, acc=ACC)
    movel(Xclose_drawer_2_finish, vel=30, acc=30)

    movej(JReady, vel=VELOCITY, acc=ACC)

    node.get_logger().info("========== 🏁 close_drawer_2() 종료 ==========")


'''서랍장3 close motion'''
def close_drawer_3():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 close_drawer_3() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 닫는 위치로 이동
    movesj([JReady, Jclose_drawer_3_before, Jclose_drawer_3], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 17mm)
    gripper.move_gripper(170)
    time.sleep(1)

    # 서랍장 살짝 들기 (z축 6mm, x축 -6mm)
    movel([-6.0, 0, 6.0, 0, 0, 0], vel=50, acc=50, mod=1)
    time.sleep(0.5)

    # 서랍장 넣기 (x축 100mm)
    movel([100.0, 0, 0, 0, 0, 0], vel=30, acc=30, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 위로 빠지기 (z축 40mm)
    movel([0, 0, 40.0, 0, 0, 0], vel=50, acc=50, mod=1)

    # 서랍장 닫기 마무리
    movesj([Jclose_drawer_3_waypoint_1, Jclose_drawer_3_waypoint_2], vel=VELOCITY, acc=ACC)
    movel(Xclose_drawer_3_finish, vel=30, acc=30)

    movej(JReady, vel=VELOCITY, acc=ACC)

    node.get_logger().info("========== 🏁 close_drawer_3() 종료 ==========")


'''서랍장4 close motion'''
def close_drawer_4():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 close_drawer_4() 시작! ==========")

    # 그리퍼 열기(30mm)
    gripper.move_gripper(300)

    # 서랍장 닫는 위치로 이동
    movesj([JReady, Jclose_drawer_4_before, Jclose_drawer_4], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 17mm)
    gripper.move_gripper(170)
    time.sleep(1)

    # 서랍장 살짝 들기 (z축 6mm, x축 -6mm)
    movel([-6.0, 0, 6.0, 0, 0, 0], vel=50, acc=50, mod=1)
    time.sleep(0.5)

    # 서랍장 넣기 (x축 100mm)
    movel([100.0, 0, 0, 0, 0, 0], vel=30, acc=30, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 위로 빠지기 (z축 40mm)
    movel([0, 0, 40.0, 0, 0, 0], vel=50, acc=50, mod=1)

    # 서랍장 닫기 마무리
    movesj([Jclose_drawer_4_waypoint_1, Jclose_drawer_4_waypoint_2], vel=VELOCITY, acc=ACC)
    movel(Xclose_drawer_4_finish, vel=30, acc=30)

    movej(JReady, vel=VELOCITY, acc=ACC)

    node.get_logger().info("========== 🏁 close_drawer_4() 종료 ==========")


'''처방한 약봉지를 종이 봉투에 넣어주는 함수'''
def put_pill_in_bag():
    VELOCITY, ACC = 100, 100
    node.get_logger().info("========== 🏁 put_pill_in_bag() 시작! ==========")

    # 홈위치에서 대기
    movej([0, 0, 90, -30, 90, 0], vel=VELOCITY, acc=ACC)

    # 그리퍼 열기 (10mm)
    gripper.move_gripper(100)

    # x 방향에 외력이 가해질 때까지 대기
    print("약을 그리퍼에 넣고 로봇을 쳐주세요! (x방향)")
    val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

    while True:
        val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

        # x 방향으로 쳤을 때 약 봉투 포장 시작
        if val_x == 0:
            print(f"check_force_condition X: {val_x}")
            print("약 포장을 시작합니다.")
            break
        
    # 그리퍼 닫기 (3mm)
    time.sleep(0.5)
    gripper.move_gripper(30)
    time.sleep(1)

    # 약 종이 봉투에 담기
    movej(Jput_pill_in_bag_waypoint, vel=50, acc=50)
    movel(Xput_pill_in_bag, vel=VELOCITY, acc=ACC)

    # 그리퍼 열기 (20mm)
    gripper.move_gripper(200)
    time.sleep(1)

    # 홈으로
    movej(Jput_pill_in_bag_waypoint, vel=VELOCITY, acc=ACC)
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)


''' 비처방 약 서랍 물품 집는 함수'''
def handle_BTC(medicine, pick_pos, place_pos, is_floor2=False):
    VELOCITY, ACC = 100, 100

    global qr_pill_list

    qr_pill_list = [medicine.name]
    grip_size = grip_dict.get(medicine.name)

    # 2층이라면 임시로 그리퍼 좁히기
    if is_floor2:
        gripper.move_gripper(550)

    # 선택된 픽업 위치로 이동
    movej(pick_pos, vel=VELOCITY, acc=ACC)
    time.sleep(0.5)

    # 실제 물체 크기대로 그리퍼 좁힘
    gripper.move_gripper(grip_size)
    time.sleep(0.5)

    # 2층이면 턱 넘기
    if is_floor2:
        movec(floor_2_1, floor_2_2, time=2, mod=DR_MV_MOD_REL)
        # 놓을 위치로 이동
        movej(place_pos, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
    else:
        movesj(place_pos, vel=VELOCITY, acc=ACC)

    # x 방향에 외력이 가해질 때까지 대기
    print("약을 가져가 주세요! (x방향 외력)")
    val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

    while True:
        val_x = check_force_condition(DR_AXIS_X, 10, 30, DR_TOOL)

        # x 방향으로 쳤을 때 약 봉투 포장 시작
        if val_x == 0:
            print(f"check_force_condition X: {val_x}")
            print("약 포장을 시작합니다.")
            break
        
    # 외력이 가해지면 그리퍼 열기 (70mm)
    time.sleep(0.5)
    gripper.move_gripper(700)
    time.sleep(1)

    # 9. 시작 위치로 복귀
    movej(JReady, vel=VELOCITY, acc=ACC)


def BTC_1(medicine):
    handle_BTC(medicine, pick_pos=Jpos_B, place_pos=Jpos_L, is_floor2=False)

def BTC_2(medicine):
    handle_BTC(medicine, pick_pos=Jpos_A, place_pos=Jpos_R, is_floor2=False)

def BTC_3(medicine):
    handle_BTC(medicine, pick_pos=Jpos_D, place_pos=Put_place, is_floor2=True)

def BTC_4(medicine):
    handle_BTC(medicine, pick_pos=Jpos_C, place_pos=Put_place, is_floor2=True)


'''비처방약 물품을 vision_node에 퍼블리시하는 함수'''
def publish_medicine(medicine):
    # medicine을 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 medicine 퍼블리시 중...")
    medicine_publisher.publish(medicine)


'''선반을 바라보는 자세로 이동 후 'shelf_state'를 vision에 퍼블리시하는 함수'''
def move_shelf_state():
    VELOCITY, ACC = 100, 100
    movej(Jpos1, vel=VELOCITY, acc=ACC)

    # 'shelf_state' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'shelf_state' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "shelf_state"
    robot_state_publisher.publish(robot_state_msg)


'''medicine의 위치를 subscribtion하고, 물건을 집을 위치를 선택하는 함수'''
def choice_BTC(medicine):

    global medicine_loc, medicine_loc_received

    # medicine의 위치를 수신하는 subscriber 생성
    medicine_loc_subscription = node.create_subscription(TextLoc, "/medicine_loc", medicine_loc_callback, 10)

    # medicine의 위치를 subscription할 때까지 대기
    node.get_logger().info("🕐 medicine_loc 대기 중...")
    while rclpy.ok() and not medicine_loc_received:
        rclpy.spin_once(node, timeout_sec=0.1)  # 100ms 간격으로 체크

    time.sleep(1)

    # 더 이상 필요 없는 subscriber 제거
    node.destroy_subscription(medicine_loc_subscription)

    # 'pick_medicine' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'pick_medicine' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "pick_medicine"
    robot_state_publisher.publish(robot_state_msg)

    gripper.move_gripper(600)

    if medicine_loc == 1:
        BTC_1(medicine)
    elif medicine_loc == 2:
        BTC_2(medicine)
    elif medicine_loc == 3:
        BTC_3(medicine)
    elif medicine_loc == 4:
        BTC_4(medicine)


'''medicine_loc 정보를 수신하는 콜백 함수'''
def medicine_loc_callback(msg):
    global medicine_loc, medicine_loc_received
    medicine_loc_received = True
    medicine_loc = msg.text_loc
    node.get_logger().info(f"========== ✅ medicine location 수신 ==========")
    node.get_logger().info(f"📥 medicine_loc 수신됨: [{medicine_loc}]번 위치")
    node.get_logger().info("========== medicine location 수신 완료 ==========")


'''약 탐지 상태와 로봇의 current_posx를 퍼블리시하는 함수'''
def finish_publish():
    global qr_disease, qr_pill_list
    node.get_logger().info("========== 🏁 finish_publish() 시작! ==========")

    qr_pill_string = f"'{ ' '.join(qr_pill_list) }'"

    # 'explain_medicine' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'explain_medicine' 상태 퍼블리시 중...")
    finish_msg = TaskState()
    finish_msg.state = "explain_medicine"
    finish_msg.qr_info = qr_pill_string
    finish_publisher.publish(finish_msg)
    node.get_logger().info(f"📤  퍼블리시 완료! State : {finish_msg.state}, pill : {finish_msg.qr_info}")

    node.get_logger().info("========== 🏁 finish_publish() 종료 ==========")



def main(args=None):
    global qr_data_received, voice_received, qr_total_pills_count

    move_check_qr()
    # QR 인식 시 처방약
    if qr_data_received:
        move_check_text()
        select_and_open_drawer()
        for _ in range(qr_total_pills_count):
            move_drawer_campose()
            publish_check_pill_state()
            pick_pill()
            place_pill()
        select_and_close_drawer()
        put_pill_in_bag()
        finish_publish()

    # Voice 인식 시 비처방약
    elif voice_received:
        for medicine in medicines_name:
            print(f"medicine = {medicine}")
            publish_medicine(medicine)
            move_shelf_state()
            choice_BTC(medicine)
            finish_publish()

    movej(JReady, vel=VELOCITY, acc=ACC)
    time.sleep(3)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
