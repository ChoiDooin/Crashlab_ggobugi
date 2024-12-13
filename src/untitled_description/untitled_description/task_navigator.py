import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class TaskNavigator(Node):
    def __init__(self):
        super().__init__('task_navigator')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 좌표 설정
        self.coordinates = {
            '1': {'x': -3.365413427352e9053, 'y': -2.4908666610717773, 'z': 0.004314422607421875},
            '2': {'x': 3.8217453956604004, 'y': 0.6492373943328857, 'z': 0.002655029296875},
            '3': {'x': 8.854761123657227, 'y': -3.640552282333374, 'z': 0.0036067962646484375},
            '4': {'x': 2.0, 'y': 6.0, 'z': 0.0}
        }

    def send_goal(self, task_number):
        if task_number not in self.coordinates:
            self.get_logger().error(f"Invalid task number: {task_number}")
            return

        target = self.coordinates[task_number]

        # 목표 좌표 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        goal_msg.pose.pose.position.z = target['z']
        goal_msg.pose.pose.orientation.w = 1.0

        # 액션 서버가 활성화될 때까지 대기
        self.action_client.wait_for_server()
        self.get_logger().info(f"Sending robot to task {task_number} (x={target['x']}, y={target['y']})")
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = TaskNavigator()

    try:
        while rclpy.ok():
            # 사용자 입력 대기
            task_number = input("Enter task number (1-4): ").strip()
            navigator.send_goal(task_number)
    except KeyboardInterrupt:
        print("\nShutting down task navigator.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
