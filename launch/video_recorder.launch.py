import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, PushRosNamespace, SetParametersFromFile, SetParameter
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    ros_images_to_files_dir = get_package_share_directory('ros_images_to_files')

    # Get launch directories
    ros_images_to_files_launch_dir = os.path.join(ros_images_to_files_dir, 'launch')

    # Declare launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default="False")
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default=False)
    image_topic = LaunchConfiguration('image_topic', default='/camera/image_raw')
    image_topic_is_compressed = LaunchConfiguration('image_topic_is_compressed', default=False)
    output_file_name = LaunchConfiguration('output_file_name', default='output.avi')
    queue_size = LaunchConfiguration('queue_size', default=100)
    fps = LaunchConfiguration('fps', default=30.0)
    qos = LaunchConfiguration('qos', default="SENSOR_DATA")
    normalize_depth = LaunchConfiguration('normalize_depth', default=True)
    normalized_max = LaunchConfiguration('normalized_max', default=255)
    show_image = LaunchConfiguration('show_image', default=False)

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')

    declare_namespace_cmd = DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Top-level namespace to distinguish between each F1/10 (robot).')

    declare_use_namespace_cmd = DeclareLaunchArgument(
            'use_namespace',
            default_value=use_namespace,
            description='Use namespace if true.')

    declare_image_topic_cmd = DeclareLaunchArgument(
            'image_topic',
            default_value=image_topic,
            description='Image topic to subscribe to.')

    declare_image_topic_is_compressed_cmd = DeclareLaunchArgument(
            'image_topic_is_compressed',
            default_value=image_topic_is_compressed,
            description='Image topic is compressed.')

    declare_output_file_name_cmd = DeclareLaunchArgument(
            'output_file_name',
            default_value=output_file_name,
            description='Output file name.')

    declare_queue_size_cmd = DeclareLaunchArgument(
            'queue_size',
            default_value=queue_size,
            description='Queue size.')

    declare_fps_cmd = DeclareLaunchArgument(
            'fps',
            default_value=fps,
            description='Frames per second.')

    declare_qos_cmd = DeclareLaunchArgument(
            'qos',
            default_value=qos,
            description='QoS.')

    declare_normalize_depth_cmd = DeclareLaunchArgument(
            'normalize_depth',
            default_value=normalize_depth,
            description='Normalize depth.')

    declare_normalized_max_cmd = DeclareLaunchArgument(
            'normalized_max',
            default_value=normalized_max,
            description='Normalized max.')

    declare_show_image_cmd = DeclareLaunchArgument(
            'show_image',
            default_value=show_image,
            description='Show image.')

    launch_args = [
        declare_use_sim_time_cmd,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_image_topic_cmd,
        declare_image_topic_is_compressed_cmd,
        declare_output_file_name_cmd,
        declare_queue_size_cmd,
        declare_fps_cmd,
        declare_qos_cmd,
        declare_normalize_depth_cmd,
        declare_normalized_max_cmd,
        declare_show_image_cmd,
    ]

    # Launch nodes
    video_recorder_node = Node(
            package='ros_images_to_files',
            executable='video_recorder_node',
            name='video_recorder',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'image_topic': image_topic},
                {'image_topic_is_compressed': image_topic_is_compressed},
                {'output_file_name': output_file_name},
                {'queue_size': queue_size},
                {'fps': fps},
                {'qos': qos},
                {'normalize_depth': normalize_depth},
                {'normalized_max': normalized_max},
                {'show_image': show_image},
            ]
    )

    # add nodes to the launch description
    nodes_to_launch = GroupAction(
            actions=[
                PushRosNamespace(
                        condition=IfCondition(use_namespace),
                        namespace=namespace
                ),
                SetParameter(name='use_sim_time', value=use_sim_time),
                # nodes
                video_recorder_node,
            ]
    )  # append F1/10 namespace to all nodes
    ld = LaunchDescription(launch_args + [nodes_to_launch])

    return ld

