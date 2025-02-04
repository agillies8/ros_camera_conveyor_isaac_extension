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
            # Robot state publishers below for each kicker. Added namespace and prefix so tf tree and joint_subscriber get the right kicker
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='kicker',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc,
                         'frame_prefix':'kicker/'}],
            arguments=[urdf]),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_1',
            output='screen',
            namespace='kicker_01',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix':'kicker_01/',
                'publish_frequency': 10.0,}],
            ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_2',
            output='screen',
            namespace='kicker_02',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc,
                         'frame_prefix':'kicker_02/',}],
            ),
            #Static publishers below correct the tf naming coming from isaac sim so they can all be on the same tree
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'kicker_base', 'kicker/kicker_base']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'World_Kickers_side_kicker_01_kicker_base', 'kicker_01/kicker_base']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'World_Kickers_side_kicker_02_kicker_base', 'kicker_02/kicker_base']
        ),
        #the 3 yolo detection actor nodes below take in the labeled data from the yolo nodes and create commands for the kickers. the actual yolo nodes are in a different container, see docker compose for those
        Node(
            package='isaac_cam_conveyor',
            executable='yolo_detection_actor',
            name='yolo_detection_actor',
            namespace='kicker',
            parameters=[
                {'target_classes': ['red-box']},
                {'x_min': 300.0},
                {'x_max': 450.0},
                {'y_min': 250.0},
                {'y_max': 450.0},
                {'joint_command_topic': 'joint_command'},
                {'joint_name': 'kicker_joint'},
                {'image_sub_topic' : 'detections'}
            ]
        ),
        Node(
            package='isaac_cam_conveyor',
            executable='yolo_detection_actor',
            name='yolo_detection_actor',
            namespace='kicker_01',
            parameters=[
                {'target_classes': ['green-box']},
                {'x_min': 300.0},
                {'x_max': 450.0},
                {'y_min': 250.0},
                {'y_max': 450.0},
                {'joint_command_topic': 'joint_command'},
                {'joint_name': 'kicker_joint'},
                {'image_sub_topic' : 'detections'}
            ]
        ),
        Node(
            package='isaac_cam_conveyor',
            executable='yolo_detection_actor',
            name='yolo_detection_actor',
            namespace='kicker_02',
            parameters=[
                {'target_classes': ['blue-box']},
                {'x_min': 300.0},
                {'x_max': 450.0},
                {'y_min': 250.0},
                {'y_max': 450.0},
                {'joint_command_topic': 'joint_command'},
                {'joint_name': 'kicker_joint'},
                {'image_sub_topic' : 'detections'}
            ]
        ),
        #this one launches rviz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]),
    ])