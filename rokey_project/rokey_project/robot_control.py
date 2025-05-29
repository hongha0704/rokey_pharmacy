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
Jdrawer_3_waypoint = posj(16.08, 0.17, 126.43, 37.27, -6.41, -16.43)
Jdrawer_3_before = posj(43.43, 32.43, 103.79, 83.22, -71.32, -39.30)
Jdrawer_3 = posj(41.59, 38.98, 102.98, 75.35, -77.94, -39.30)


'''서랍장1 open motion'''
def open_drawer_1():
    VELOCITY, ACC = 100, 100
    # 홈위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_1_waypoint, Jdrawer_1_before, Jdrawer_1], vel=VELOCITY, acc=ACC)

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
    movej(JReady, vel=VELOCITY, acc=ACC)
    gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_2_waypoint, Jdrawer_2_before, Jdrawer_2], vel=VELOCITY, acc=ACC)

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
    VELOCITY, ACC = 30, 30
    # 홈위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    # gripper.move_gripper(300)

    # 서랍장 집는 위치로 이동
    movesj([Jdrawer_3_waypoint, Jdrawer_3_before, Jdrawer_3], vel=VELOCITY, acc=ACC)

    # 서랍장 집기 (그리퍼 16mm)
    # gripper.move_gripper(160)
    time.sleep(0.5)

    # 서랍장 열기 (x축 -140mm)
    movel([-140, 0, 0, 0, 0, 0], vel=60, acc=60, mod=1)

    # 서랍장 놓기 (그리퍼 30mm)
    # gripper.move_gripper(300)
    time.sleep(0.5)

    # 서랍장 놓고 뒤로 조금 빠지기 (x축 -30mm)
    movel([-40, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)  ###

    # # 서랍장 cam pose로 이동
    # movesj([Jdrawer_2_campose_waypoint_1, Jdrawer_2_campose_waypoint_2, Jdrawer_2_campose], vel=VELOCITY, acc=ACC)



def main(args=None):

    open_drawer_3()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
