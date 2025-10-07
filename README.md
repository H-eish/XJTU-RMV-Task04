# XJTU-RMV-Task04 &emsp;--ROS2封装的海康相机SDK
该功能包实现了：  
1.海康相机的自动连接，具备断线重连功能  
2.稳定地从相机采集数据，将采集到的图像数据转化为sensor_msgs/msg/Image格式，并发布到可配置topic /image_raw  
3.能通过命令行读取或设置相机各参数，包括曝光时间、帧率等（参数详见hik_camera_ros2/launch/hik_camera_launch.py）  
## 如何配置、编译和运行该节点
### 使用的环境
操作系统：Ubuntu22.04  
ROS 2 Humble
### 编译和运行
将功能包/hik_camera_ros2放置到工作空间的/src下（为了方便，直接copy了海康的库，可能不是很规范）  
  
进入工作空间  
  
终端里使用  
colcon build --packages-select hik_camera_ros2  
来编译  
  
功能包使用了launch文件，故先  
source install/setup.bash(zsh)  
后  
ros2 launch hik_camera_ros2 hik_camera_launch.py  
## 参数的读取和修改
在新终端里通过ros2 param set <参数名> <数值>  
ros2 param get <参数名>  
来实现  
    
成功支持实时帧率的读取以及图像格式的实时修改（比如bgr8改mono8）