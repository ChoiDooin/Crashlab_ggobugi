import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DynamicTransformPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_transform_publisher')

        # Transform Broadcaster 초기화
        self.tf_broadcaster = TransformBroadcaster(self)

        # 주기적으로 TF를 송출하는 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.publish_transforms)  # 10Hz

        self.get_logger().info("Dynamic Transform Publisher Node Started")

    def publish_transforms(self):
        # 현재 ROS2 시간 가져오기
        current_time = (self.get_clock().now() + rclpy.time.Duration(seconds=0.05)).to_msg()  # 오프셋 50ms

        # Transform 목록 정의
        transforms = [
            ('odom', 'base_footprint', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ('base_footprint', 'base_link', (0.0, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
            ('base_link', 'laser_frame', (0.2, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0))
        ]

        # 각 Transform을 브로드캐스트
        for parent_frame, child_frame, translation, rotation in transforms:
            t = TransformStamped()
            t.header.stamp = current_time  # 타임스탬프 설정
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = rotation

            # Transform을 전송
            self.tf_broadcaster.sendTransform(t)
            self.get_logger().debug(f"Publishing Transform: {parent_frame} -> {child_frame}")

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # 노드 생성
    node = DynamicTransformPublisher()

    try:
        # 노드 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node Stopped by User")
    finally:
        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

