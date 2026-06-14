 
cmake_minimum_required(VERSION 3.10)
project(camera_streamer)

set(OpenCV_DIR /usr/lib/aarch64-linux-gnu/cmake/opencv4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Ищем OpenCV в системе
find_package(OpenCV REQUIRED)

add_executable(camera_streamer main.cpp)

# Подключаем OpenCV и потоки
target_include_directories(camera_streamer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_streamer ${OpenCV_LIBS} pthread)