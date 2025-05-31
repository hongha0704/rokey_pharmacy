import cv2
import rclpy
from rclpy.node import Node
from realsense import ImgNode
from scipy.spatial.transform import Rotation
from onrobot import RG

import time
import numpy as np

import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# ROS2 노드 초기화
rclpy.init()
node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)

DR_init.__dsr__node = node

try:
    from DSR_ROBOT2 import (
        get_current_posx,
        movej,
        movel,
        wait,
        task_compliance_ctrl,
        release_compliance_ctrl,
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit(True)


class TestNode(Node):
    def __init__(self):
        # 노드 이름 'test_node'로 ROS2 노드 초기화
        super().__init__("test_node")

         # RealSense 이미지 구독 노드 생성
        self.img_node = ImgNode()

        # 콜백 한 번만 수행하여 초기 이미지와 카메라 정보 수신
        rclpy.spin_once(self.img_node)
        time.sleep(1)

        # 카메라 내부 파라미터 (초점 거리, 주점) 받아오기
        self.intrinsics = self.img_node.get_camera_intrinsic()

        # gripper 좌표계에서 카메라 좌표계로의 변환 행렬 불러오기 (.npy 파일)
        self.gripper2cam = np.load("T_gripper2camera.npy")

        # OnRobot RG2 그리퍼 객체 생성 및 초기화 (통신 설정: IP, PORT)
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        # pos setting
        self.JReady = posj([0, 0, 90, 0, 90, -90])

    def mouse_callback(self, event, x, y, flags, param):
        # 마우스 왼쪽 버튼 클릭 이벤트 발생 시 실행
        if event == cv2.EVENT_LBUTTONDOWN:
            # 현재 뎁스 프레임을 가져옴
            depth_frame = self.img_node.get_depth_frame()

            # 유효한 뎁스 이미지가 올 때까지 반복 (None이거나 모든 픽셀이 0인 경우 재시도)
            while depth_frame is None or np.all(depth_frame == 0):
                self.get_logger().info("retry get depth img")
                rclpy.spin_once(self.img_node)
                depth_frame = self.img_node.get_depth_frame()

            # 클릭한 위치 (x, y)의 뎁스 값(z) 얻기
            z = self.get_depth_value(x, y, depth_frame)
            print(f"img cordinate: ({x}, {y}, {z})")

            # 픽셀 좌표 (x, y, z)를 카메라 좌표계로 변환
            camera_center_pos = self.get_camera_pos(x, y, z, self.intrinsics)
            print(f"camera cordinate: ({camera_center_pos})")

            # 카메라 좌표계를 그리퍼(혹은 베이스) 좌표계로 변환
            gripper_coordinate = self.transform_to_base(camera_center_pos)
            print(f"gripper cordinate: ({gripper_coordinate})")

            # 로봇에게 해당 좌표로 이동하여 픽업 및 드롭 동작 수행
            self.pick_and_drop(*gripper_coordinate)
            print("=" * 100)

    '''픽셀 좌표와 깊이, 카메라 내재 파라미터를 이용해 카메라 좌표계 상의 3D 위치를 계산하는 함수'''
    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        camera_z = center_z

        # 카메라 좌표계의 3D 위치 반환
        return (camera_x, camera_y, camera_z)
    
    '''위치 와 오일러 각을 받아 로봇의 4x4 포즈 행렬을 생성'''
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        # ZYZ 오일러 각을 이용해 회전 행렬 생성
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()

        # 단위 행렬 생성 후, 상단 3x3 영역에 회전 행렬 할당
        T = np.eye(4)
        T[:3, :3] = R

        # 위치 벡터 할당
        T[:3, 3] = [x, y, z]

        # 4x4 로봇 포즈 행렬 반환
        return T
       

    def transform_to_base(self, camera_coords):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        # gripper2cam = np.load(self.gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        timer = time.time()

        base2cam = base2gripper @ self.gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def open_img_node(self):
        rclpy.spin_once(self.img_node)
        img = self.img_node.get_color_frame()

        cv2.setMouseCallback("Webcam", self.mouse_callback, img)
        cv2.imshow("Webcam", img)

        # if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
        #     break

    def get_depth_value(self, center_x, center_y, depth_frame):
        height, width = depth_frame.shape
        if 0 <= center_x < width and 0 <= center_y < height:
            depth_value = depth_frame[center_y, center_x]
            return depth_value
        self.get_logger().warn(f"out of image range: {center_x}, {center_y}")
        return None
    

    def pick_and_drop(self, x, y, z):
        current_pos = get_current_posx()[0]
        pick_pos = posx([x, y, current_pos[2], current_pos[3], current_pos[4], current_pos[5]])
        # TODO: Write pick and drop function
        print(f"x, y, z = {x, y, z}")
        movel(pick_pos, vel=VELOCITY, acc=ACC)

        # 그리퍼 15mm 만큼 열기
        self.gripper.move_gripper(150)
        time.sleep(1)

        # 약 있는 위치로 내리기
        current_pos = get_current_posx()[0]
        current_pos_down = posx([current_pos[0], current_pos[1], 10.27, current_pos[3], current_pos[4], current_pos[5]])
        result = movel(current_pos_down, vel=VELOCITY, acc=ACC)
        print(f"result = {result}")

        # 순응제어 on
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        # 그리퍼 8mm로 닫기
        self.gripper.move_gripper(80)
        # self.gripper.move_gripper(40)
        time.sleep(0.5)

        # 순응제어 off
        release_compliance_ctrl()
        time.sleep(0.5)

        # 집고 z축으로 20mm 올리기
        movel([0, 0, 20, 0, 0, 0], vel=VELOCITY, acc=VELOCITY, mod=1)

        # 그리퍼 15mm 만큼 열기
        self.gripper.move_gripper(150)


def main():

    cv2.namedWindow("Webcam")

    test_node = TestNode()

    while True:
        test_node.open_img_node()

        if cv2.waitKey(1) & 0xFF == 27:  # ESC 키로 종료
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
