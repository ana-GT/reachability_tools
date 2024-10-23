cmake_minimum_required(VERSION 3.16)
project(reachability_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(DEPS 
  rosidl_default_generators
  std_msgs
  geometry_msgs
  sensor_msgs
  vision_msgs
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()


rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ChainInfo.msg"
"msg/ReachParams.msg"
"msg/ReachData.msg"
"msg/ReachSample.msg"
"msg/ReachGraph.msg"
"msg/Param.msg"
"msg/PlaceRobotSolution.msg"
"srv/GetHandToUser.srv"
"srv/SetRobotPose.srv"
"srv/MoveRobotToTask.srv"
"srv/GenerateReachPoses.srv"
DEPENDENCIES geometry_msgs std_msgs sensor_msgs vision_msgs
)


ament_export_dependencies(${DEPS})

ament_package()
