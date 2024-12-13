import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ZonePublisher(Node):
    def __init__(self):
        super().__init__('zone_publisher')
        self.publisher = self.create_publisher(Int32, '/zone_command', 10)
        self.get_logger().info("Zone Publisher Node Initialized. Enter numbers (1-4):")

    def publish_zone(self, zone_id):
        msg = Int32()
        msg.data = zone_id
        self.publisher.publish(msg)
        self.get_logger().info(f"Published zone ID: {zone_id}")

def main(args=None):
    rclpy.init(args=args)
    node = ZonePublisher()

    try:
        while rclpy.ok():
            user_input = input("Enter zone number (1-4) or 'q' to quit: ").strip()
            if user_input.lower() == 'q':
                print("Exiting...")
                break
            if user_input.isdigit() and int(user_input) in range(1, 5):
                node.publish_zone(int(user_input))
            else:
                print("Invalid input. Please enter a number between 1 and 4.")
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

