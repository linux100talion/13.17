 
cmake_minimum_required(VERSION 3.10)
project(camera_streamer)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Ищем OpenCV в системе

set(OpenCV_DIR "/opt/opencv-4.10-cuda/lib/cmake/opencv4")
# Или Через CMAKE_PREFIX_PATH
# cmake \
#   -DCMAKE_PREFIX_PATH=/opt/opencv-4.8-cuda \
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

add_executable(camera_streamer main.cpp)

# Подключаем OpenCV и потоки
target_include_directories(camera_streamer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_streamer ${OpenCV_LIBS} pthread)