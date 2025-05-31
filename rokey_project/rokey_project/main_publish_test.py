import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState  # 메시지 타입 import

class TaskStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.publisher_ = self.create_publisher(TaskState, '/robot_state', 10)
        self.get_logger().info("TaskState 퍼블리셔 노드 시작됨.")

        # 1초마다 콜백 함수 실행
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        user_input = input("robot_state 입력 (예: MOVING, IDLE, DONE 등): ")
        msg = TaskState()
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
