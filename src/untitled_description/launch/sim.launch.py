from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 경로 설정
    share_dir = get_package_share_directory('untitled_description')
    gazebo_world_path = os.path.join(share_dir, 'world', 'house.world')
    rviz_config_path = os.path.join(share_dir, 'config', 'rjrj.rviz')

    # Gazebo 실행
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('untitled_description'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': gazebo_world_path}.items()
    )

    # RViz 실행
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    # Map Server 실행 및 초기화
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': 'me_map.yaml'},
            {'use_sim_time': True}
        ]
    )
    map_server_lifecycle = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='map_server_lifecycle',
        arguments=['map_server']
    )

    # AMCL 실행 및 초기화
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    amcl_lifecycle = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='amcl_lifecycle',
        arguments=['amcl']
    )

    # Navigation2 실행
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # LaunchDescription에 추가
    return LaunchDescription([
        gazebo_launch,
        rviz_node,
        map_server_node,
        map_server_lifecycle,
        amcl_node,
        amcl_lifecycle,
        navigation_launch
    ])
