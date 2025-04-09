import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 'prefix' 인자 선언 (기본값은 빈 문자열)
    ld.add_action(DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for robot frames (optional)'
    ))
    
    # 만약 is_sim도 사용한다면 선언이 필요합니다.
    ld.add_action(DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description='Flag to indicate simulation mode'
    ))

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('scout_description'),
        'rviz',
        'view_robot.rviz'
        # 안해도 오류는 없는데, 저장된 설정을 가져옵니다. 이런 느낌이다.
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        on_exit=Shutdown()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'frame_prefix': LaunchConfiguration('prefix'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('scout_description'),
                        'urdf/scout_v2',
                        'scout_v2.xacro',
                    ]),
                    ' is_sim:=', LaunchConfiguration('is_sim'),
                    ' prefix:=', LaunchConfiguration('prefix'),
                ]),
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

	#rviz에서 urdf를 보고 싶으니 추가를 한다고 이해
    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)

    return ld