cmake_minimum_required(VERSION 3.16)
project(reachability_description)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)

set(DEPS 
  rclcpp
  trac_ik_lib
  urdf
  srdfdom
  robot_unit
  pcl_conversions
  tf2_eigen
  tf2_eigen_kdl
  tf2_kdl
  reachability_msgs
  robot_sim_msgs
  generate_parameter_library
  pluginlib
  interactive_markers
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}  
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(REACH_PARAM_LIB reachability_description_parameters)
generate_parameter_library(${REACH_PARAM_LIB} src/reachability_description_parameters.yaml)


add_library(${PROJECT_NAME} SHARED 
            src/reachability_description.cpp 
            src/reach_graph.cpp 
            src/reach_serialization.cpp
            src/reach_utilities.cpp
            src/geom_s2.cpp
            src/geom_se2.cpp)
target_link_libraries(${PROJECT_NAME} ${REACH_PARAM_LIB})
ament_target_dependencies( ${PROJECT_NAME} ${DEPS})

add_library(reach_graph_aggregated src/reach_graph_aggregated.cpp)
target_link_libraries(reach_graph_aggregated ${PROJECT_NAME} ${OpenCV_LIBRARIES})


##############################################################################
# Utilities to generate, load, estimate reachability data
add_executable(generate_reachability_node tools/generate_reachability_node.cpp)
target_link_libraries(generate_reachability_node ${PROJECT_NAME})
ament_target_dependencies(generate_reachability_node ${DEPS})

add_executable(load_reachability_node tools/load_reachability_node.cpp)
target_link_libraries(load_reachability_node ${PROJECT_NAME})
ament_target_dependencies(load_reachability_node ${DEPS})

add_executable(estimate_reachability_limits_node tools/estimate_reachability_limits_node.cpp)
target_link_libraries(estimate_reachability_limits_node ${PROJECT_NAME})
ament_target_dependencies(estimate_reachability_limits_node ${DEPS})

add_executable(view_gmm_node tools/view_gmm_node.cpp)
target_link_libraries(view_gmm_node ${PROJECT_NAME})
ament_target_dependencies(view_gmm_node ${DEPS})


##########
## Apps
add_executable(app_hand_to_user apps/hand_to_user.cpp)
target_link_libraries(app_hand_to_user ${PROJECT_NAME})
ament_target_dependencies(app_hand_to_user ${DEPS})

add_executable(app_robot_to_task apps/robot_to_task.cpp)
target_link_libraries(app_robot_to_task ${PROJECT_NAME})
ament_target_dependencies(app_robot_to_task ${DEPS})

add_executable(app_robot_to_task_2 apps/robot_to_task_2.cpp)
target_link_libraries(app_robot_to_task_2 reach_graph_aggregated ${PROJECT_NAME})
ament_target_dependencies(app_robot_to_task_2 ${DEPS})

add_executable(app_generate_robot_reach_ee_poses apps/generate_robot_reach_ee_poses.cpp)
target_link_libraries(app_generate_robot_reach_ee_poses ${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(app_generate_robot_reach_ee_poses ${DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} 
  ${REACH_PARAM_LIB}
  reach_graph_aggregated
  generate_reachability_node  
  load_reachability_node 
  estimate_reachability_limits_node
  view_gmm_node
  app_hand_to_user
  app_robot_to_task
  app_robot_to_task_2
  app_generate_robot_reach_ee_poses
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
