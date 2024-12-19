import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 메시지 타입 추가


class LocationPublisherNode(Node):
    def __init__(self):
        super().__init__('Location_publisher_node')
        self.cnt = 0
        # Publisher 설정 (std_msgs/String 사용)
        self.publisher_ = self.create_publisher(String, '/hospital_location', 10)
        self.get_logger().info("Publisher created for topic")

        # Timer 설정: 0.5초마다 JSON 파일 읽기 및 발행
        self.timer = self.create_timer(0.5, self.publish_location)
        
    def publish_location(self):
        msg= String()
        msg.data='응급실'
        if self.cnt == 0 :
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published location data: {msg.data}")
            msg.data = '응급실'
        #     self.cnt = self.cnt + 1
        # if self.cnt == 1 :
        #     self.publisher_.publish(msg)
        #     self.get_logger().info(f"Published location data: {msg.data}")
        #     self.cnt = self.cnt + 1



def main(args=None):

    rclpy.init(args=args)
    node = LocationPublisherNode()
    try:
        rclpy.spin(node)  # 노드를 계속 실행시켜서 타이머가 작동하도록 유지
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

