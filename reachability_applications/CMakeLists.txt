cmake_minimum_required(VERSION 3.16)
project(reachability_applications)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)
set(DEPS 
  rclcpp
  tf2_ros
  tf2_eigen
  tf2_eigen_kdl
  tf2_kdl
  tf2_geometry_msgs
  reachability_msgs
  reachability_description
  rach_ik
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)


include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(simple_mobile_pose_node src/simple_mobile_pose_node.cpp)
ament_target_dependencies(simple_mobile_pose_node ${DEPS})

add_executable(simple_get_reach_data_node src/simple_get_reach_data_node.cpp)
ament_target_dependencies(simple_get_reach_data_node ${DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  simple_mobile_pose_node
  simple_get_reach_data_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch #config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
