import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the address of the yaml file 
    config = os.path.join(
        get_package_share_directory('ulite6_move'),
        'config',
        'params.yaml')

    # Xarm driver launch
    xarm_driver_launch = os.path.join(
        get_package_share_directory('xarm_api'),
        'launch',
        'lite6_driver.launch.py'
    )

    xarm_driver_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xarm_driver_launch),
        launch_arguments={'robot_ip': '192.168.1.31'}.items()
    )

    # ROS-TCP-endpoint node
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )
    # xyzTrack servo node
    xyzTrack_servo = Node(
        package='ulite6_move',
        executable='xyzrTrack_servo',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        xarm_driver_launch_description,
        ros_tcp_endpoint,
        TimerAction(period=5.0, actions=[xyzTrack_servo]) # wait for 5 seconds to start the node
    ])