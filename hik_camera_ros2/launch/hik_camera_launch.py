import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    生成启动描述，用于启动海康相机ROS2节点，并配置初始参数。
    """
    
    # 定义节点的参数
    # 您可以在这里修改默认值，或者在启动时通过命令行覆盖
    # 例如: ros2 launch hik_camera_ros2 hik_camera_launch.py frame_rate:=50.0
    hik_camera_params = [
        {'camera_sn': ''},                  # 相机序列号，如果为空，则连接第一个找到的相机
        {'exposure_time': 10000.0},          # 曝光时间 (us)
        {'gain': 5.0},                      # 增益
        {'frame_rate': 30.0},               # 帧率
        {'pixel_format': 'bgr8'},           # 像素格式 ('mono8', 'bgr8', 'rgb8', etc.)
        {'camera_frame_id': 'camera_optical_frame'}, # 图像坐标系
        {'topic_name': 'image_raw'}         # 图像话题名称
    ]
    
    ld = LaunchDescription()
    
    # 定义要启动的节点
    hik_camera_node = Node(
        package='hik_camera_ros2',         # 您的功能包名称
        executable='hik_camera_node',      # 在CMakeLists.txt中定义的可执行文件名称
        name='hik_camera',                 # 节点的运行时名称
        output='screen',                   # 将节点的输出打印到屏幕
        emulate_tty=True,                  # 模拟一个终端，以确保日志颜色和格式正确
        parameters=hik_camera_params,      # 传递参数给节点
    )
    
    pkg_share_dir = get_package_share_directory('hik_camera_ros2')
    
    rviz_config_path = os.path.join(pkg_share_dir, 'config', 'camera_view.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    
    ld.add_action(hik_camera_node)
    ld.add_action(rviz_node)
    
    return ld