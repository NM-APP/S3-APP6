cmake_minimum_required(VERSION 3.8)
project(robotsim)

find_package(Python REQUIRED)
set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wconversion -Wno-unused -O3)
enable_testing()

include_directories(${PROJECT_SOURCE_DIR})

add_library(robot
    robot/robot.cpp
    robot/robot_field.cpp)

add_executable(robotsim src/robotsim.cpp)
target_link_libraries(robotsim robot pthread)

# Laboratoire
add_executable(ex1 lab/ex1.cpp)
target_link_libraries(ex1 pthread)
add_executable(ex2 lab/ex2.cpp)
target_link_libraries(ex2 pthread)
add_executable(ex3 lab/ex3.cpp)
add_executable(ex4 lab/ex4.cpp)