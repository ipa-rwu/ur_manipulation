cmake_minimum_required(VERSION 2.8.3)
project(ur_manipulation)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    geometry_msgs
    tf
    tf_conversions
    ur_msgs
    std_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  doc/interactivity/include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_custom_api
   src/moveit_custom_api.cpp
 )
add_dependencies(moveit_custom_api ${catkin_EXPORTED_TARGETS})
target_link_libraries(moveit_custom_api ${catkin_LIBRARIES})
catkin_package(
  LIBRARIES
    moveit_custom_api
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

add_executable(move_group_interface_tutorial_ur src/move_group_interface_tutorial_ur.cpp)
target_link_libraries(move_group_interface_tutorial_ur ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial_ur DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(learning src/learning.cpp)
target_link_libraries(learning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS learning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(seher_demo src/seher_demo.cpp
                          src/moveit_custom_api.cpp
                          )
target_link_libraries(seher_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS moveit_custom_api
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      PATTERN ".svn" EXCLUDE)
