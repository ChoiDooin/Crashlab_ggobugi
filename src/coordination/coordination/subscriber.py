import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
class TaskNavigator(Node):
    def __init__(self):
        super().__init__('task_navigate')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.publisher_ = self.create_publisher(String, '/arrived', 10)
        self.get_logger().info("Publisher created for topic")
        # 좌표 설정
        self.coordinates = {
            '1': {'x': 3.5, 'y': 0.028, 'z': 0.0},
            '2': {'x': 3.5, 'y': 1.0, 'z': 0.0},
            '3': {'x': 3.5, 'y': -1.0, 'z': 0.0},
            '4': {'x': 3.5, 'y': 0.5, 'z': 0.0}
        }
        self.is_navigating = False  # 현재 네비게이션 상태
        self.subscription = self.create_subscription(
            String,
            '/hospital_location',
            self.receive_goal,
            10
        )
        self.get_logger().info("TaskNavigator initialized and ready to receive goals.")
    def receive_goal(self, msg):
        if self.is_navigating:
            self.get_logger().warn("Robot is currently navigating. Please wait until it finishes.")
            return
#응급실 화장실 편의점 접수/수납
        try:
            msg = String()
            if msg.data == '응급실':
                  task_number = 1
            elif msg.data == '화장실':
                  task_number = 2
            elif msg.data == '편의점':
                  task_number = 3
            elif msg.data == '접수/수납':
                  task_number = 4
            # task_number = str(msg.data)
            self.get_logger().info(f"Received location: '{task_number}'")
            self.send_goal(task_number)
        except Exception as e:
            self.get_logger().error(f"Error in receiving goal: {e}")
    def send_goal(self, task_number):
        if task_number not in self.coordinates:
            self.get_logger().error(f"Invalid task number: {task_number}")
            return
        target = self.coordinates[task_number]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = target['z']
        goal_msg.pose.pose.orientation.w = 1.0
        self.action_client.wait_for_server()
        self.get_logger().info(f"Sending robot to task {task_number} (x={target['x']}, y={target['y']})")
        self.is_navigating = True  # 네비게이션 상태 활성화
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.is_navigating = False  # 네비게이션 상태 비활성화
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    def get_result_callback(self, future):
        result = future.result()
        self.is_navigating = False  # 네비게이션 상태 초기화
        if result.status == 4:  # 4: SUCCEEDED
            self.get_logger().info("Goal succeeded! Waiting for next location.")
            msg = String()
            msg.data = '1'
            self.publisher_.publish(msg)
        else:
            self.get_logger().error(f"Goal failed with status: {result.status}")
def main(args=None):
    rclpy.init(args=args)
    navigator = TaskNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        if rclpy.ok():  # Check if rclpy is still running
            navigator.destroy_node()
            rclpy.shutdown()
if __name__ == '__main__':
    main()