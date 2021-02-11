cmake_minimum_required(VERSION 3.5)
project(SimpleController)

find_package(ur_rtde REQUIRED)
add_executable(SimpleController SimpleController.cpp)
target_link_libraries(SimpleController PRIVATE ur_rtde::rtde)