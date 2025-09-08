from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='go2_cv_pipeline_tests',
        executable='go2_user_test_commands',
    ))

    ld.add_action(Node(
        package='go2_cv_pipeline_tests',
        executable='go2_cv_test',
    ))

    return ld