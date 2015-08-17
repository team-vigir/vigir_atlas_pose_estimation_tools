cmake_minimum_required(VERSION 2.8.3)

project(vigir_bdi_to_tf_utils)

find_package(catkin REQUIRED COMPONENTS atlas_msgs sensor_msgs tf)

#set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS atlas_msgs sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(HEADERS
    include/vigir_bdi_to_tf_utils/bdi_to_tf.h
)

## Declare a cpp executable
add_executable(bdi_to_tf_node src/bdi_to_tf_node.cpp ${HEADERS} )
target_link_libraries(bdi_to_tf_node ${catkin_LIBRARIES})
add_dependencies(bdi_to_tf_node ${catkin_EXPORTED_TARGETS})

#install executables
install( TARGETS
    bdi_to_tf_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
