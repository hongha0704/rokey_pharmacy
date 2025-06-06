# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time
from rokey_project.onrobot import RG

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
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")


## vel, acc, pos setting
VELOCITY, ACC = 60, 60

JReady = [0, 0, 90, 0, 90, 0]

# qr코드 확인하는 pos
Jcheck_qr_waypoint = posj(11.65, -2.53, 104.26, 38.19, 15.43, 47.83)
Jcheck_qr = posj(36.46, 15.43, 103.02, 105.37, -124.10, 30.93)

# text 확인/분류하는 pos
Jcheck_text_waypoint = posj(41.83, -11.13, 118.59, 88.00, -59.42, 30.94)
# Jcheck_text = posj(20.28, 2.42, 130.21, 72.87, -20.18, 31.70) ## text 체크 위치 수정
Jcheck_text = posj(18.57, 25.38, 133.04, 27.09, -53.22, 78.63)  ## text 체크 위치 수정
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


'''qr code check 모션'''
def move_check_qr():
    VELOCITY, ACC = 100, 100
    # 홈위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # qr code 체크하는 위치로 이동
    movesj([Jcheck_qr_waypoint, Jcheck_qr], vel=VELOCITY, acc=ACC)

    # qr code 체크중 ...
    time.sleep(2)


'''text check 모션'''
def move_check_text():
    VELOCITY, ACC = 100, 100
    # qr code 체크하는 위치로 이동
    movesj([Jcheck_text_waypoint, Jcheck_text], vel=VELOCITY, acc=ACC)

    # text 체크중 ...
    time.sleep(2)


'''서랍장1 open motion'''
def open_drawer_1():
    VELOCITY, ACC = 100, 100
    # 홈위치
    # movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    # movesj([Jdrawer_1_waypoint, Jdrawer_1_before, Jdrawer_1], vel=VELOCITY, acc=ACC)
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
    # 홈위치
    # movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    # movesj([Jdrawer_2_waypoint, Jdrawer_2_before, Jdrawer_2], vel=VELOCITY, acc=ACC)
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
    # 홈위치
    # movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    # movesj([Jdrawer_3_waypoint, Jdrawer_3_before, Jdrawer_3], vel=VELOCITY, acc=ACC)
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
    # 홈위치
    # movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    # movesj([Jdrawer_4_waypoint, Jdrawer_4_before, Jdrawer_4], vel=VELOCITY, acc=ACC)
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


def main(args=None):

    movej(JReady, vel=VELOCITY, acc=ACC)
    
    # move_check_qr()
    move_check_text()

    open_drawer_1()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
