# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time
from rokey_project.onrobot import RG

from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import RobotState
from rokey_interfaces.msg import QRInfo
from rokey_interfaces.msg import PillLoc
from rokey_interfaces.msg import TextLoc

from rclpy.node import Node

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


# 플래그 및 수신 데이터
qr_data_received = False
qr_disease = None
qr_pill_list = None

# 약 위치 초기화
x_base, y_base, theta = 0, 0, 0

# text 위치 초기화
text_loc = None
text_loc_data_received = False

# robot_state publisher 생성
robot_state_publisher = node.create_publisher(RobotState, "/robot_state", 10)

# robot_current_posx publisher 생성
robot_current_posx_publisher = node.create_publisher(RobotState, "/robot_current_posx", 10)


'''QR 코드 인식 위치로 이동하고 정보를 수신하는 함수'''
def move_check_qr():
    VELOCITY, ACC = 100, 100
    global qr_data_received

    # 홈위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # qr code 체크하는 위치로 이동
    movesj([Jcheck_qr_waypoint, Jcheck_qr], vel=VELOCITY, acc=ACC)

    # QR code 정보를 수신하는 subscriber 생성
    qr_info_subscription = node.create_subscription(QRInfo, "/qr_info", qr_callback, 10)

    # 'check_qr' 상태를 VisionNode에 퍼블리시
    node.get_logger().info(f"📤 'check_qr' 상태 퍼블리시 중...")
    robot_state_msg = RobotState()
    robot_state_msg.robot_state = "check_qr"
    robot_state_publisher.publish(robot_state_msg)

    # QR 정보가 들어올 때까지 대기
    node.get_logger().info("🕐 QR 정보 대기 중...")
    while rclpy.ok() and not qr_data_received:
        rclpy.spin_once(node, timeout_sec=0.1)  # 100ms 간격으로 체크

    node.get_logger().info("✅ QR 정보 수신 완료, 다음 동작으로 진행")
    time.sleep(2)

    # 더 이상 필요 없는 subscriber 제거
    node.destroy_subscription(qr_info_subscription)


'''QR 정보를 수신하는 콜백 함수'''
def qr_callback(msg):
    global qr_data_received, qr_disease, qr_pill_list
    if msg.disease != "":
        qr_disease = msg.disease
        qr_pill_list = msg.pill
        qr_data_received = True
        node.get_logger().info(f"✅ QR 정보 수신")
        node.get_logger().info(f"💊 병: {qr_disease}, 약: {qr_pill_list}")


'''서랍 텍스트 인식 위치로 이동하고 상태를 퍼블리시하는 함수'''
def move_check_text():
    VELOCITY, ACC = 100, 100

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


'''text_loc 정보를 수신하는 콜백 함수'''
def text_loc_callback(msg):
    global text_loc, text_loc_data_received
    text_loc_data_received = True
    text_loc = msg.text_loc
    node.get_logger().info(f"📥 text_loc 수신됨: 서랍 번호 [{text_loc}]")


'''서랍장1 open motion'''
def open_drawer_1():
    VELOCITY, ACC = 100, 100

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


'''서랍장2 open motion'''
def open_drawer_2():
    VELOCITY, ACC = 100, 100

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


'''서랍장3 open motion'''
def open_drawer_3():
    VELOCITY, ACC = 100, 100

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


'''서랍장4 open motion'''
def open_drawer_4():
    VELOCITY, ACC = 100, 100
    
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


'''서랍 4개 중 하나를 선택하고 여는 함수'''
def select_and_open_drawer():
    global qr_disease
    global text_loc
    node.get_logger().info(f"💊 병: {qr_disease}, 약: {qr_pill_list}")

    # if qr_disease == 'diarrhea':
    if text_loc == 1:
        node.get_logger().info(f"🗄️  1번 서랍을 엽니다!")
        open_drawer_1()
        publish_check_pill_state()

    # elif qr_disease == 'dyspepsia':
    elif text_loc == 2:
        node.get_logger().info(f"🗄️  2번 서랍을 엽니다!")
        open_drawer_2()
        publish_check_pill_state()

    # elif qr_disease == 'dermatitis':
    elif text_loc == 3:
        node.get_logger().info(f"🗄️  3번 서랍을 엽니다!")
        open_drawer_3()
        publish_check_pill_state()

    # elif qr_disease == 'cold':
    elif text_loc == 4:
        node.get_logger().info(f"🗄️  4번 서랍을 엽니다!")
        open_drawer_4()
        publish_check_pill_state()


'''약 탐지 상태와 로봇의 current_posx를 퍼블리시하는 함수'''
def publish_check_pill_state():
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


'''약 위치와 자세 메시지를 subscribe하고, 약을 집는 함수'''
def pick_pill():
    VELOCITY, ACC = 100, 100
    global x_base, y_base, theta, qr_disease

    # 약의 위치와 자세 정보를 수신하는 subscriber 생성
    pill_loc_subscription = node.create_subscription(PillLoc, "/pill_loc", pill_loc_callback, 10)

    # 약의 위치와 자세 정보를 수신할 때까지 spin
    while not x_base:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(pill_loc_subscription)
    node.get_logger().info(f"📤 'detect_pill' 상태 퍼블리시 중...")
    time.sleep(1)

    # 서랍의 위치 별 z값 설정
    qr_disease = 'dermatitis'   ## 테스트용 변수설정
    if qr_disease == 'diarrhea':
        z = 24.09
    if qr_disease == 'dyspepsia':
        z = 24.12
    elif qr_disease == 'dermatitis':
        z = 111.63
    elif qr_disease == 'cold':
        z = 111.23
    node.get_logger().info(f"💊 x = {x_base}, y = {y_base}, z ={z}")

    # 약 있는 위치의 x, y 좌표로 가고, 6축을 theta만큼 회전하기
    current_pos = get_current_posx()[0]
    pick_pos = posx([x_base, y_base, current_pos[2], current_pos[3], current_pos[4], current_pos[5]])
    movel(pick_pos, vel=VELOCITY, acc=ACC)
    movej([0, 0, 0, 0, 0, theta], vel=VELOCITY, acc=ACC, mod=1)

    # 그리퍼 15mm 만큼 열기
    gripper.move_gripper(150)
    time.sleep(1)

    # 약 있는 위치로 내리기
    current_pos = get_current_posx()[0]
    pick_pos_down = posx([current_pos[0], current_pos[1], z, current_pos[3], current_pos[4], current_pos[5]])
    movel(pick_pos_down, vel=VELOCITY, acc=ACC)

    # 순응제어 on
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(0.5)

    # 그리퍼 8mm로 닫기
    gripper.move_gripper(80)
    time.sleep(0.5)

    # 순응제어 off
    release_compliance_ctrl()
    time.sleep(0.5)

    # 집고 z축으로 200mm 올리기
    movel([0, 0, 80, 0, 0, 0], vel=VELOCITY, acc=VELOCITY, mod=1)


'''약 위치 정보를 수신하는 콜백 함수'''
def pill_loc_callback(msg):
    global x_base, y_base, theta
    x_base = msg.x
    y_base = msg.y
    theta = msg.theta

    # theta가 90도 이상이면, 반대방향으로 회전(회전하는 각도 최소화 하기 위해)
    if theta > 90:
        theta -= 180

    node.get_logger().info(f"✅ 약 위치, 자세 정보 수신 완료")
    node.get_logger().info(f"💊 x = {x_base}, y = {y_base}, theta = {theta}")


'''약을 약 봉투 위치에 place하는 함수'''
def place_pill():
    VELOCITY, ACC = 50, 50
    movesj([JReady, Jpill_pouch_morning], vel=VELOCITY, acc=ACC)
    movej(Jpill_pouch_morning, vel=VELOCITY, acc=ACC)
    # movej(Jpill_pouch_afternoon, vel=VELOCITY, acc=ACC)
    # movej(Jpill_pouch_evening, vel=VELOCITY, acc=ACC)
    time.sleep(0.5)

    # 그리퍼 15mm 만큼 열기
    gripper.move_gripper(150)
    time.sleep(1)
    movej(JReady, vel=VELOCITY, acc=ACC)



def main(args=None):

    move_check_qr()
    move_check_text()
    select_and_open_drawer()
    pick_pill()
    place_pill()
    

    ############ 선반3 테스트용  ############
    # movej(Jdrawer_3_campose, vel=VELOCITY, acc=ACC)

    # publish_check_pill_state()
    # pick_pill()
    # place_pill()
    
    # movej(Jdrawer_3_campose, vel=VELOCITY, acc=ACC)

    ############ 선반3 테스트용 ############

    rclpy.shutdown()


if __name__ == "__main__":
    main()
