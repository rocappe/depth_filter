from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='depth_sub_topic', default_value='/zed2/zed_node/depth/depth_registered',
            description='Depth image topic to filter'
        ),
        DeclareLaunchArgument(
            name='color_info_topic', default_value='/zed2/zed_node/depth/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            name='depth_pub_topic', default_value='/zed2/filtered_depth',
            description='Filtered depth image topic'
        ),
        Node(
            package='depth_filter', executable='depth_filter_node',
            remappings=[('depth_sub_topic', [LaunchConfiguration(variable_name='depth_sub_topic')]),
                        ('color_info_topic', [LaunchConfiguration(variable_name='color_info_topic')]),
                        ('depth_pub_topic', [LaunchConfiguration(variable_name='depth_pub_topic')])],
        )
    ])
