import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point  # 메시지 타입 추가
from std_msgs.msg import String
import json
from pathlib import Path

do = False

class JsonPublisherNode(Node):
    def __init__(self):
        super().__init__('json_publisher_node')

        # JSON 파일 경로 설정
        package_share_directory = get_package_share_directory('json_data')
        self.json_file_path = Path(package_share_directory) / 'receive_data.json'
        self.get_logger().info(f"JSON file path set to: {self.json_file_path}")

        # Publisher 설정 (std_msgs/String 사용)
        self.publisher_ = self.create_publisher(Point, 'json_data', 10)
        self.User_detect_ = self.create_publisher(String, 'user_detect', 10)
        self.get_logger().info("Publisher created for topic 'json_data'.")

        # Timer 설정: 0.5초마다 JSON 파일 읽기 및 발행
        self.timer = self.create_timer(0.5, self.publish_json_data)

    def publish_json_data(self):
        try: 
            with open(self.json_file_path, 'r') as file:
                data = json.load(file)
                # JSON 데이터의 마지막 항목 추출
                if data:
                  x = data[-1]["x"]
                  y = data[-1]["y"]
                  size = data[-1]["size"]

                  point=Point()
                  point.x=x
                  point.y=y
                  point.z=size
                
                else:
                  global do
                  point=Point()
                  point.x=0.0
                  point.y=0.0
                  point.z=0.0
                  user=String()
                  if not do:
                    user.data='1'
                    self.User_detect_.publish(user)
                    do = True
                  
                self.publisher_.publish(point)
                self.get_logger().info(f"Published: x={point.x}, y={point.y}, size={point.z}")

        except FileNotFoundError:
            self.get_logger().error(f"File not found: {self.json_file_path}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON format in file: {self.json_file_path}")
        except IndexError:
            self.get_logger().error(f"No valid data found in JSON file: {self.json_file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = JsonPublisherNode()
    try:
        rclpy.spin(node)  # 노드를 계속 실행시켜서 타이머가 작동하도록 유지
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()