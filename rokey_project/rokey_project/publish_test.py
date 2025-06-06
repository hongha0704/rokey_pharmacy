'''사용자 입력을 받아 publish 하여 통신을 테스트하는 파일'''

import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import RobotState  # 메시지 타입 import

class TaskStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.publisher_ = self.create_publisher(RobotState, '/robot_state', 10)
        self.get_logger().info("RobotState 퍼블리셔 노드 시작됨.")

        # 1초마다 콜백 함수 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        user_input = input("robot_state 입력 (예: MOVING, IDLE, DONE 등): ")
        msg = RobotState()
        msg.robot_state = user_input
        self.publisher_.publish(msg)
        self.get_logger().info(f'퍼블리시함: robot_state="{msg.robot_state}"')

def main(args=None):
    rclpy.init(args=args)
    node = TaskStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
