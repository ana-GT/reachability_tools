cmake_minimum_required(VERSION 3.16.0)
project(rach_ik)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(fcl REQUIRED)
set(DEPS
    rclcpp 
    urdf
    kdl_parser
    srdfdom
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

# NLOpt
find_package(NLopt REQUIRED)

include_directories(
  include
#  ${LIBFCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/optimize.cpp 
src/objective.cpp
src/constraint.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${DEPS})
target_link_libraries(${PROJECT_NAME} ${NLOPT_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies( ${PROJECT_NAME} ${DEPS})

############# Nodes ###############
add_executable(${PROJECT_NAME}_node nodes/rach_ik_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node ${DEPS})


install(DIRECTORY include/
  DESTINATION include
)


install(TARGETS ${PROJECT_NAME} 
  ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_libraries(
    ${PROJECT_NAME} 
)

#ament_export_targets(
#  export_${PROJECT_NAME}
#  HAS_LIBRARY_TARGET
#)

#ament_export_dependencies(${DEPS} fcl)

ament_export_include_directories(
  include
)


ament_package()