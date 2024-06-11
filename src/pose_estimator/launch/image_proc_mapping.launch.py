from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='depth_image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Multithreaded container
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[
                        ('depth_to_rgb/image_rect', '/depth_to_rgb/image_raw'),
                        ('depth_to_rgb/camera_info', '/depth_to_rgb/camera_info'),
                        ('points', '/depth/points')
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    remappings=[
                        ('image', '/depth_to_rgb/image_raw'),
                        ('camera_info', '/depth_to_rgb/camera_info'),
                        ('image_rect', '/depth_to_rgb/image_rect')
                    ]
                )
            ],
            output='screen',
        ),
    ])
