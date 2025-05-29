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
        task_compliance_ctrl,
        release_compliance_ctrl,
        get_current_posx,
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

## vel, acc, pos setting
VELOCITY, ACC = 60, 60
JReady = [0, 0, 90, 0, 90, 0]


def main(args=None):
    
    '''약을 집는 동작 테스트 시작'''
    # 홈으로
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 그리퍼 15mm 만큼 열기
    gripper.move_gripper(150)
    time.sleep(3)

    # 약 있는 위치로 내리기
    movel([0, 0, -83.48, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    # 순응제어 on
    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
    time.sleep(0.5)

    # 그리퍼 8mm로 닫기
    # gripper.move_gripper(80)
    gripper.move_gripper(40)
    time.sleep(0.5)

    # 순응제어 off
    release_compliance_ctrl()
    time.sleep(0.5)

    # 집고 z축으로 100mm 올리기
    movel([0, 0, 100, 0, 0, 0], vel=VELOCITY, acc=VELOCITY, mod=1)

    # 집고 x축으로 150mm 앞으로 가기
    movel([150, 0, 0, 0, 0, 0], vel=VELOCITY, acc=VELOCITY, mod=1)

    # 그리퍼 15mm 만큼 열기
    gripper.move_gripper(150)
    time.sleep(1)

    # 홈으로
    movej(JReady, vel=VELOCITY, acc=ACC)

    '''약을 집는 동작 테스트 끝'''

    rclpy.shutdown()


if __name__ == "__main__":
    main()
