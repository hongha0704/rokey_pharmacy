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
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

VELOCITY, ACC = 60, 60

def main(args=None):

    JReady = [0, 0, 90, 0, 90, 0]
    Jpos1 = [4.87, -9.07, 120.09, 8.26, 23.19, -10.18]
    Jpos2 = [8.91, 10.16, 127.43, 17.12, -22.55, -10.29]
    Jpos3 = [10.84, 43.07, 126.15, 12.56, -78.80, -2.87]
    Jpos4 = [10.23, 44.97, 121.51, 12.97, -77.06, -1.37]

    ## 서랍장 open
    gripper.move_gripper(300)

    movej(JReady, vel=VELOCITY, acc=ACC)
    movej(Jpos1, vel=VELOCITY, acc=ACC)
    movej(Jpos2, vel=VELOCITY, acc=ACC)
    movej(Jpos3, vel=VELOCITY, acc=ACC)
    movej(Jpos4, vel=VELOCITY, acc=ACC)

    gripper.move_gripper(150)
    movel([-70, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)
    gripper.move_gripper(300)
    movel([-15, 0, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
