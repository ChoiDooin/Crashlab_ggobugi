import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class ZoneSubscriber(Node):
    def __init__(self):
        super().__init__('zone_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/zone_command',
            self.zone_callback,
            10
        )
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Define coordinates for each zone
        self.zones = {
            1: [2.0, 2.0, 0.0, 0.0, 1.0],  # x, y, z, orientation_z, orientation_w
            2: [4.0, 2.0, 0.0, 0.0, 1.0],
            3: [2.0, 4.0, 0.0, 0.0, 1.0],
            4: [4.0, 4.0, 0.0, 0.0, 1.0],
        }
        self.get_logger().info("Zone Subscriber Node Initialized. Waiting for commands...")

    def zone_callback(self, msg):
        zone_id = msg.data
        if zone_id not in self.zones:
            self.get_logger().error(f"Unknown zone ID: {zone_id}")
            return

        self.get_logger().info(f"Navigating to zone {zone_id}...")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Set goal coordinates
        zone_coords = self.zones[zone_id]
        goal_pose.pose.position.x = zone_coords[0]
        goal_pose.pose.position.y = zone_coords[1]
        goal_pose.pose.position.z = zone_coords[2]
        goal_pose.pose.orientation.z = zone_coords[3]
        goal_pose.pose.orientation.w = zone_coords[4]

        # Navigate to the goal
        self.navigator.goToPose(goal_pose)

        # Wait until the task is complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and feedback.estimated_time_remaining:
                self.get_logger().info(f"ETA: {feedback.estimated_time_remaining.sec} seconds")

        # Check result
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info(f"Arrived at zone {zone_id} successfully!")
        elif result == BasicNavigator.TaskResult.FAILED:
            self.get_logger().error(f"Failed to reach zone {zone_id}.")
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().warn(f"Navigation to zone {zone_id} was canceled.")

def main(args=None):
    rclpy.init(args=args)
    node = ZoneSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

