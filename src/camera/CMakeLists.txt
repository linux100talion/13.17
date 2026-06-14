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

add_executable(camera_node camera_node.cpp)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(camera_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_node ${OpenCV_LIBS})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
