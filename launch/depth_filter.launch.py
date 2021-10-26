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
        DeclareLaunchArgument(
            name='max_height', default_value='1.5',
            description=''
        ),
        DeclareLaunchArgument(
            name='min_height', default_value='-0.1',
            description=''
        ),
        DeclareLaunchArgument(
            name='max_distance', default_value='5.0',
            description=''
        ),
        DeclareLaunchArgument(
            name='min_distance', default_value='2.0',
            description=''
        ),
        Node(
            package='depth_filter', executable='depth_filter_node',
            parameters=[{
                  "max_height": LaunchConfiguration("max_height"),
                  "min_height": LaunchConfiguration("min_height"),
                  "max_distance": LaunchConfiguration("max_distance"),
                  "min_distance": LaunchConfiguration("min_distance")
                  }],
            remappings=[('depth_sub_topic', [LaunchConfiguration('depth_sub_topic')]),
                        ('color_info_topic', [LaunchConfiguration('color_info_topic')]),
                        ('depth_pub_topic', [LaunchConfiguration('depth_pub_topic')])],
        )
    ])
