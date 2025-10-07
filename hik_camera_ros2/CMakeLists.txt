cmake_minimum_required(VERSION 3.8)
project(hik_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# 寻找海康SDK库
find_library(MVS_LIBRARY
  NAMES MvCameraControl
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

# 如果库没找到，立即报错并停止
if(NOT MVS_LIBRARY)
  message(FATAL_ERROR "Hikvision MVS library (libMvCameraControl.so) not found in ${CMAKE_CURRENT_SOURCE_DIR}/lib. Please check the path.")
endif()

# 打印找到的库路径，方便调试
message(STATUS "Found MVS library: ${MVS_LIBRARY}")


add_executable(hik_camera_node src/hik_camera_node.cpp)

target_include_directories(hik_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 步骤1：只用 ament_target_dependencies 链接 ROS 2 的包
ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  image_transport
)

# 步骤2：用一个单独的、不带关键字的 target_link_libraries 链接外部库
# 这样就和 ament_target_dependencies 内部使用的“普通格式”保持一致了
target_link_libraries(hik_camera_node
  ${MVS_LIBRARY}
)


install(TARGETS
  hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY lib/
  DESTINATION lib # 安装到 install/hik_camera_ros2/lib/ 目录下
  FILES_MATCHING PATTERN "*.so*" # 确保所有 .so 文件及其可能的符号链接都被安装
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
