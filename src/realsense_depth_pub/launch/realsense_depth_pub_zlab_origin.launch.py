from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('serial', default_value=''),
        DeclareLaunchArgument('publish_full', default_value='true'),
        DeclareLaunchArgument('depth_w', default_value='480'),
        DeclareLaunchArgument('depth_h', default_value='270'),
        DeclareLaunchArgument('out_w', default_value='64'),
        DeclareLaunchArgument('out_h', default_value='36'),
        DeclareLaunchArgument('hfov', default_value='89.24'),
        DeclareLaunchArgument('vfov', default_value='58.06'),
        DeclareLaunchArgument('depth_fps', default_value='60'),
        DeclareLaunchArgument('frame_depth', default_value='camera_depth_optical_frame'),
        DeclareLaunchArgument('topic_depth', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('topic_depth_info', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('topic_small', default_value='/camera/depth/image_64x36'),
        DeclareLaunchArgument('topic_small_info', default_value='/camera/depth/camera_info_64x36'),
        DeclareLaunchArgument('enable_color', default_value='false'),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('enable_ir', default_value='false'),
    ]

    node = Node(
        package='realsense_depth_pub',
        executable='depth_publisher_node',
        name='depth_pub',
        output='screen',
        parameters=[{
            'serial': LaunchConfiguration('serial'),
            'depth_w': LaunchConfiguration('depth_w'),
            'depth_h': LaunchConfiguration('depth_h'),
            'depth_fps': LaunchConfiguration('depth_fps'),
            'publish_rate_hz': 60,
            'out_w': LaunchConfiguration('out_w'),
            'out_h': LaunchConfiguration('out_h'),
            'hfov': LaunchConfiguration('hfov'),
            'vfov': LaunchConfiguration('vfov'),
            'publish_full': LaunchConfiguration('publish_full'),
            'frame_depth': LaunchConfiguration('frame_depth'),
            'topic_depth': LaunchConfiguration('topic_depth'),
            'topic_depth_info': LaunchConfiguration('topic_depth_info'),
            'topic_small': LaunchConfiguration('topic_small'),
            'topic_small_info': LaunchConfiguration('topic_small_info'),
            'enable_color': LaunchConfiguration('enable_color'),
            'enable_imu': LaunchConfiguration('enable_imu'),
            'enable_ir': LaunchConfiguration('enable_ir'),
            'decimation': 1,
            'min_dist': 0.2,
            'max_dist': 2.5,
            'spat_alpha': 0.45,
            'spat_delta': 20.0,
            'spat_holes': 2,
            'temp_holes': 4,
            'temp_alpha': 0.45,
            'temp_delta': 20.0,
            'hole1': 1,
            'hole2': 2,
        }],
    )

    # 可选：自动打开 RViz（没有该文件就先注释掉）
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', PathJoinSubstitution([FindPackageShare('realsense_depth_pub'), 'rviz', 'depth_dual_view.rviz'])],
    #     output='screen',
    # )

    actions = [node]
    if 'rviz_node' in locals():
        actions.append(locals()['rviz_node'])

    return LaunchDescription(declared_arguments + actions)
