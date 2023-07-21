cmake_minimum_required(VERSION 3.16)
project(reachability_description)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(DEPS 
  ament_cmake
  rclcpp
  trac_ik_lib
  urdf
  srdfdom
  robot_unit
  pcl_conversions
  tf2_eigen
  tf2_eigen_kdl
  reachability_msgs
  generate_parameter_library
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}  
)

set(REACH_PARAM_LIB reachability_description_parameters)
generate_parameter_library(${REACH_PARAM_LIB} src/reachability_description_parameters.yaml)

set(linked_fcl_libs /home/ana/local/lib/libfcl.so;/usr/lib/x86_64-linux-gnu/libccd.so;/usr/lib/x86_64-linux-gnu/libm.so;/opt/ros/galactic/lib/liboctomap.so;/opt/ros/galactic/lib/liboctomath.so)
add_library(${PROJECT_NAME} SHARED src/reachability_description.cpp src/reach_data.cpp src/reach_serialization.cpp)
target_link_libraries(${PROJECT_NAME} ${linked_fcl_libs} ${REACH_PARAM_LIB})
ament_target_dependencies( ${PROJECT_NAME} ${DEPS})

add_executable(generate_reachability_node src/generate_reachability_node.cpp)
target_link_libraries(generate_reachability_node ${PROJECT_NAME})
ament_target_dependencies(generate_reachability_node ${DEPS})

add_executable(load_reachability_node src/load_reachability_node.cpp)
target_link_libraries(load_reachability_node ${PROJECT_NAME})
ament_target_dependencies(load_reachability_node ${DEPS})

add_executable(estimate_reachability_limits_node src/estimate_reachability_limits_node.cpp)
target_link_libraries(estimate_reachability_limits_node ${PROJECT_NAME})
ament_target_dependencies(estimate_reachability_limits_node ${DEPS})


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} 
  ${REACH_PARAM_LIB}
  generate_reachability_node  
  load_reachability_node 
  estimate_reachability_limits_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_libraries(
    ${PROJECT_NAME}
)

#ament_export_targets(
#  reachability_generation_node
#)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
