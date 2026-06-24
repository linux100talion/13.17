cmake_minimum_required(VERSION 3.16)
project(camera_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc cudaimgproc cudaarithm videoio)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV CUDA support: ${OpenCV_CUDA_VERSION}")

# Боевая нода (CUDA): дебайер на cv::cuda::*, линкует полный OpenCV (вкл. CUDA).
add_executable(camera_node camera_node.cpp)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_node ${OpenCV_LIBS})

# Drop-in CPU-нода (без GPU): тот же camera_core.hpp, дебайер на cv::* (CPU).
# Линкуется ТОЛЬКО против не-CUDA модулей OpenCV — бинарь не тянет CUDA-рантайм.
add_executable(camera_node_cpu camera_node_cpu.cpp)

ament_target_dependencies(camera_node_cpu
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(camera_node_cpu PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(camera_node_cpu opencv_core opencv_imgproc opencv_videoio)

install(TARGETS camera_node camera_node_cpu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
