from launch_ros.actions import Node
from launch import LaunchDescription
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('untitled_description')

    # URDF 모델을 Xacro에서 로드합니다.
    xacro_file = os.path.join(share_dir, 'urdf', 'untitled.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # robot_state_publisher 노드 설정
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )
<<<<<<< HEAD

=======
>>>>>>> 27c8e20 (Maybe Final)
    # joint_state_publisher 노드 설정
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
<<<<<<< HEAD
    
    tf2_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
     )
    tf2_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    tf2_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
=======
>>>>>>> 27c8e20 (Maybe Final)


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
<<<<<<< HEAD
        tf2_map_to_odom,
        tf2_odom_to_base_footprint,
        tf2_base_footprint_to_base_link,
=======
>>>>>>> 27c8e20 (Maybe Final)
    ])

