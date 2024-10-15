cmake_minimum_required(VERSION 3.16)
project(reach_plugin_reuleaux)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(DEPS 
  ament_cmake
  rclcpp
  tf2_eigen
  tf2_eigen_kdl
  reachability_msgs
  reachability_description
  generate_parameter_library
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(
  include
)

pluginlib_export_plugin_description_file(reachability_description plugins.xml)

#set(REACH_PARAM_LIB reachability_description_parameters)
#generate_parameter_library(${REACH_PARAM_LIB} src/reachability_description_parameters.yaml)

add_library(quaternion_discretization src/quaternion_discretization.cpp)
target_link_libraries(quaternion_discretization Eigen3::Eigen)


add_library(${PROJECT_NAME} SHARED 
            src/reach_graph_reuleaux.cpp)
target_link_libraries(${PROJECT_NAME} quaternion_discretization)
ament_target_dependencies( ${PROJECT_NAME} ${DEPS})


add_executable( test_cube_discretization apps/test_cube_discretization.cpp)
target_link_libraries(test_cube_discretization quaternion_discretization)

add_executable( test_tesseract_discretization apps/test_tesseract_discretization.cpp)
target_link_libraries(test_tesseract_discretization quaternion_discretization)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} 
  quaternion_discretization
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY launch config
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_export_libraries(
    ${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
