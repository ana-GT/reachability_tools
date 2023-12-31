cmake_minimum_required(VERSION 3.16.0)
project(robot_unit)

add_compile_options(-std=c++17)

set(DEPS
    rclcpp 
    urdf
    kdl_parser
    srdfdom
    geometric_shapes
    visualization_msgs
    geometry_msgs
    octomap_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_kdl
    tf2_eigen_kdl
    reachability_msgs
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

# FCL
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.5.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")


include_directories(
  include
  ${LIBFCL_INCLUDE_DIRS}
)

add_library(treefksolverpos_recursive SHARED src/treefksolverpos_recursive.cpp)
ament_target_dependencies( treefksolverpos_recursive ${DEPS})

add_library(${PROJECT_NAME} SHARED
  src/fast_helpers.cpp
  src/fast_common.cpp
  src/fast_base_collision_object.cpp
#  src/fast_environment_collision_object.cpp
  src/robot_entity.cpp
  src/fast_robot_collision_object.cpp
  src/fast_allowed_collision_matrix.cpp
)

target_link_libraries(${PROJECT_NAME} treefksolverpos_recursive)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies( ${PROJECT_NAME} ${DEPS} LIBFCL)

############# Tests ###############
#add_executable(test_env_collision tests/test_env_collision.cpp)
#target_link_libraries(test_env_collision fast_collision_detector ${catkin_LIBRARIES} ${FCL_LIBRARIES})

#add_executable(test_robot_collision tests/test_robot_collision.cpp)
#target_link_libraries(test_robot_collision fast_collision_detector ${catkin_LIBRARIES} ${FCL_LIBRARIES})

#add_executable(test_robot_object_collision tests/test_robot_object_collision.cpp)
#target_link_libraries(test_robot_object_collision fast_collision_detector ${catkin_LIBRARIES} ${FCL_LIBRARIES})

#add_executable(test_simple_collision_2_objects tests/test_simple_collision_2_objects.cpp)
#target_link_libraries(test_simple_collision_2_objects fast_collision_detector ${catkin_LIBRARIES} ${FCL_LIBRARIES})

#add_executable(test_2_robot_trajectories tests/test_2_robot_trajectories.cpp)
#target_link_libraries(test_2_robot_trajectories fast_collision_detector ${catkin_LIBRARIES} ${FCL_LIBRARIES})



install(DIRECTORY include/
  DESTINATION include
)


install(TARGETS ${PROJECT_NAME} treefksolverpos_recursive
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_libraries(
    ${PROJECT_NAME} treefksolverpos_recursive
)

##ament_export_targets(
##  export_${PROJECT_NAME}
##)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()