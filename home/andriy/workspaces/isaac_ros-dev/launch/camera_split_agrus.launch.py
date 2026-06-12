import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Нода захвата камеры (ISP -> NVMM)
    argus_node = ComposableNode(
        name='argus_camera',
        package='isaac_ros_argus_camera',
        # ИСПРАВЛЕННАЯ СТРОКА НИЖЕ:
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        parameters=[{
            'camera_id': 0,
            'module_id': 0, # В новых версиях параметр называется так
        }],
        remappings=[
            ('image_raw', '/camera/pilot/image_color'),
            ('camera_info', '/camera/pilot/camera_info')
        ]
    )

    # 2. Нода аппаратной конвертации формата в mono8 для VINS
    converter_node = ComposableNode(
        name='format_converter',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'mono8',
            'image_width': 1280,
            'image_height': 720
        }],
        remappings=[
            ('image_raw', '/camera/pilot/image_color'),
            ('image', '/camera/vins/image_mono')
        ]
    )

    # 3. Многопоточный контейнер
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[argus_node, converter_node],
        output='screen'
    )

    return launch.LaunchDescription([container])