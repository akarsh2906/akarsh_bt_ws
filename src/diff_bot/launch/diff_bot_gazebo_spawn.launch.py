#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    diff_bot_path = get_package_share_directory('diff_bot')
    
    # Retrieve launch configuration arguments
    position_x = LaunchConfiguration("position_x", default="2.05")
    position_y = LaunchConfiguration("position_y", default="1.077")
    orientation_yaw = LaunchConfiguration("orientation_yaw", default="0.5065")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='')

    position_x = "2.05"
    position_y = "1.077"
    orientation_yaw = "0.5065"
    
    position_x = "-2.65"
    position_y = "-1.91"
    orientation_yaw = "0"

    position_x = "0"
    position_y = "0"
    orientation_yaw = "0"

    # Path to the Xacro file
    xacro_path = join(diff_bot_path, 'urdf', 'diff_bot.urdf.xacro')
    #doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot_namespace,
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot_namespace, '/joint_states"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot_namespace, '_robot"']), #default enitity name _bcr_bot
            '-z', "0.28",
            '-x', position_x,
            '-y', position_y,
            '-Y', orientation_yaw
        ]
    )

    diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', join(get_package_share_directory('diff_bot'), 'rviz', 'test.rviz')]
    )


    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper_node',
        output='screen',
        remappings=[
            ("cmd_vel_in", "cmd_vel"),
            ("cmd_vel_out", "diff_controller/cmd_vel")
        ]
    )



    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot_namespace", default_value = robot_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        robot_state_publisher,
        spawn_entity,
        # rviz,
        # twist_stamper,
        # diff_controller,
        # joint_state_broadcaster
    ])
