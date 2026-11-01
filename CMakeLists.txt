cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    geometry_msgs
    )

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    roslint
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    geometry_msgs
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

## 插件本体，库名等于包名，
## 必须与 hero_chassis_controller_plugins.xml 里的 lib/libhero_chassis_controller 一致。
add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

## /cmd_vel 演示发布者（录视频用的工具，不属于控制器本体）
add_executable(${PROJECT_NAME}_cmd_vel_demo
    src/cmd_vel_demo_node.cpp
    )

add_dependencies(${PROJECT_NAME}_cmd_vel_demo
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}_cmd_vel_demo
    ${catkin_LIBRARIES}
    )

##########################
## Static code analysis ##
##########################

roslint_cpp()
