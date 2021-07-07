cmake_minimum_required(VERSION 3.12)
project(3dDynamicsSim LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "build/")
set(CMAKE_CXX_STANDARD 17)

include_directories("include/" "../matplotlib-cpp-master/" "/usr/include/python3.8")

find_package(Eigen3 REQUIRED)

add_executable(main src/main.cpp src/utils.cpp src/RigidBody.cpp)
target_link_libraries(main Eigen3::Eigen python3.8)