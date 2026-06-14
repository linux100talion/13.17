import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Захват через стандартный V4L2 (сохраняет кадр в RAM, передает в ROS)
    v4l2_node = ComposableNode(
        name='v4l2_camera',
        package='v4l2_camera',
        plugin='v4l2_camera::V4L2Camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1920, 1200],
            # Важно: для AR0234 укажите правильный формат пикселей
            # Обычно это 'YUYV' или сырой байер ('RGGB', 'BGGR' и т.д.)
            # 'pixel_format': 'RGGB' 
        }],
        remappings=[
            ('image_raw', '/camera/raw_bayer')
        ]
    )

    # 2. Аппаратный дебайеринг на GPU/VIC
    # Узел принимает стандартное ROS-сообщение, переносит его в VRAM и обрабатывает аппаратно
    debayer_node = ComposableNode(
        name='debayer_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::DebayerNode',
        remappings=[
            ('image_raw', '/camera/raw_bayer'),
            ('image', '/camera/pilot/image_color')
        ]
    )

    # 3. Аппаратная конвертация на GPU (подготовка mono8 для визуальной одометрии)
    converter_node = ComposableNode(
        name='format_converter',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'mono8',
        }],
        remappings=[
            ('image_raw', '/camera/pilot/image_color'),
            ('image', '/camera/vins/image_mono')
        ]
    )

    # 4. Единый контейнер
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[v4l2_node, debayer_node, converter_node],
        output='screen'
    )

    return launch.LaunchDescription([container])