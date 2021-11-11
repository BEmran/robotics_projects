import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('coding_test').find('coding_test')
    urdf_file = os.path.join(pkg_share, 'simple_robot.urdf')
    rviz_file = os.path.join(pkg_share, 'config.rviz')
    print(f'Using URDF {urdf_file}')
    print(f'Using RVIZ config {rviz_file}')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[rsp_params]),

        Node(package='coding_test', executable='my_dummy_joint_controller', output='screen'),

        Node(package='coding_test', executable='high_level_controller', output='screen'),

        Node(package='coding_test', executable='path_planner', output='screen'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', rviz_file])
    ])
