import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    description_package = LaunchConfiguration("description_package", default = 'isaac_cam_conveyor')

    rviz_config_filename = 'rviz/isaac_cam_conveyor-config.rviz'
    # Specify the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('isaac_cam_conveyor'),
        rviz_config_filename
    )

    urdf_file_name = 'urdf/kicker/side_kicker.urdf'
    urdf = os.path.join(
        get_package_share_directory('isaac_cam_conveyor'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='kicker',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'frame_prefix': 'kicker/'}],
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_1',
            output='screen',
            namespace='kicker_01',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': 'World_Kickers_side_kicker_01_/',
                'publish_frequency': 10.0,}],
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_2',
            output='screen',
            namespace='kicker_02',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'frame_prefix': 'World_Kickers_side_kicker_02_/',}],
            ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='kicker',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='kicker_01',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='kicker_02',
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]),
    ])