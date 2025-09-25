# launch/velodyne_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Аргументы
    device_ip    = LaunchConfiguration('device_ip')
    udp_port     = LaunchConfiguration('port')
    frame_id     = LaunchConfiguration('frame_id')
    fixed_frame  = LaunchConfiguration('fixed_frame')
    target_frame = LaunchConfiguration('target_frame')
    model        = LaunchConfiguration('model')

    decl_args = [
        DeclareLaunchArgument('device_ip',    default_value='192.168.2.201'),
        DeclareLaunchArgument('port',         default_value='2368'),
        DeclareLaunchArgument('frame_id',     default_value='velodyne'),
        DeclareLaunchArgument('fixed_frame',  default_value='base_link'),
        DeclareLaunchArgument('target_frame', default_value='velodyne'),
        DeclareLaunchArgument('model',        default_value='VLP16'),
    ]

    # Пути к конфигам
    pkg_share = get_package_share_directory('lidar_package')
    driver_params = os.path.join(pkg_share, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    calib = os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml')

    nodes = [
        # 1) Статический TF: base_link -> velodyne
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0','0','0','0','0','0', fixed_frame, target_frame],
            output='screen'
        ),

        # 2) Трансформ облака
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[{
                'model': model,
                'calibration': calib,
                'fixed_frame': fixed_frame,
                'target_frame': target_frame,
            }],
        ),

        # 3) Драйвер с твоим YAML
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[driver_params, {          # сначала YAML
                'device_ip': device_ip,          # затем возможные переопределения
                'port': udp_port,
                'frame_id': frame_id,
                'model': model,
            }],
        ),
    ]

    return LaunchDescription(decl_args + nodes)
