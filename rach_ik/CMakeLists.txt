cmake_minimum_required(VERSION 3.16.0)
project(rach_ik)

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
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

# NLOpt
find_package(NLopt REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
src/optimize.cpp 
src/objective.cpp
src/conversion_utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_link_libraries(${PROJECT_NAME} ${NLOPT_LIBRARIES})

############# Nodes ###############
add_executable(${PROJECT_NAME}_node nodes/rach_ik_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node ${DEPS})


install(TARGETS ${PROJECT_NAME} 
  ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

ament_package()
