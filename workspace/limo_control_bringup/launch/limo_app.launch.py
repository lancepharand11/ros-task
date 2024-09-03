from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    limo_state_node = Node(
        package="limo_control",
        executable="limo_node",
        output="screen",
    )

    controller_config = os.path.join(
        get_package_share_directory('limo_control'),
        'diff_drive_controller.yaml'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    ld.add_action(limo_state_node)
    ld.add_action(controller_manager_node)
    
    return ld