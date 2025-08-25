cmake_minimum_required(VERSION 3.16.0)
project(rach_ik_plugins)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
set(DEPS
    rclcpp 
    urdf
    kdl_parser
    tf2_ros
    tf2_eigen
    srdfdom
    reachability_msgs
    robot_unit
    pluginlib
    rach_ik
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

# NLOpt
find_package(NLopt REQUIRED)

# plugin
pluginlib_export_plugin_description_file(rach_ik plugins.xml)

include_directories(include)

add_library(optimize_relaxed_ik SHARED
  src/optimize_relaxed_ik.cpp
  src/objectives/objective_relaxed_ik.cpp
  src/constraints/constraint_elbow_wrist.cpp
)
ament_target_dependencies(optimize_relaxed_ik ${DEPS})
target_link_libraries(optimize_relaxed_ik ${NLOPT_LIBRARIES})

add_library(optimize_low_elbow SHARED
  src/optimize_low_elbow.cpp
  src/objectives/objective_ee_diff.cpp
  src/constraints/constraint_elbow_wrist.cpp
)
ament_target_dependencies(optimize_low_elbow ${DEPS})
target_link_libraries(optimize_low_elbow ${NLOPT_LIBRARIES})

add_library(optimize_mobile_heuristic SHARED
  src/optimize_mobile_heuristic.cpp
  src/objectives/objective_mobile_ee_diff.cpp
  src/constraints/constraint_elbow_wrist.cpp
)
ament_target_dependencies(optimize_mobile_heuristic ${DEPS})
target_link_libraries(optimize_mobile_heuristic ${NLOPT_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS 
  optimize_relaxed_ik
  optimize_low_elbow
  optimize_mobile_heuristic
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install( FILES plugins.xml
         DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(optimize_relaxed_ik optimize_low_elbow optimize_mobile_heuristic)
ament_export_include_directories(include)
ament_export_dependencies(${DEPS})

ament_package()
