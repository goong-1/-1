import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 가제보 월드 파일 경로
    world_path = os.path.join(pkg_share, 'world', 'tracking_room.world')
    
    # 2. URDF 파일 경로
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 3. 가제보 실행 (월드 포함)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 4. 로봇 상태 퍼블리셔 (URDF 뼈대 정보 방송)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. 가제보에 로봇 소환!
    # 5-1. 0번 로봇 소환 (리더 - 키보드 조종)
    spawn_robot_0 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'robot_0',          # 가제보 내 이름
            '-x', '0.5', '-y', '0.0',    # 초기 위치 (중앙에서 약간 앞)
            '-z', '0.1'
        ],
        output='screen'
    )

    # 5-2. 1번 로봇 소환 (팔로워 - 아루코 추적)
    spawn_robot_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'robot_1',          # 가제보 내 이름
            '-x', '-0.5', '-y', '0.0',   # 초기 위치 (0번보다 1m 뒤)
            '-z', '0.1'
        ],
        output='screen'
    )

    # 6. 통신 브릿지 (ROS2 <-> Gazebo)
    # 각 로봇의 조종 토픽을 /model/robot_0/cmd_vel 형태로 분리합니다.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. 로봇 제어: ROS에서 명령을 내리면 가제보 로봇이 움직임 (ROS -> GZ)
            '/model/robot_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # 2. 카메라 영상: 가제보에서 찍은 화면을 ROS 파이썬 노드로 보냄 (GZ -> ROS)
            # 🌟 [수정] 기호를 ] 에서 [ 로 변경했습니다.
            '/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            
            # 3. 오도메트리: 가제보의 로봇 위치를 ROS로 가져옴 (GZ -> ROS)
            '/model/robot_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/robot_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot_0,
        spawn_robot_1,
        bridge
    ])