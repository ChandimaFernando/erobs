from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the node obstacle_builder with a parameter file."""
    action_cmd = Node(
        package="hello_moveit",
        executable="move_to_pose",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    return ld
