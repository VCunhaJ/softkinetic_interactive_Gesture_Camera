cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

find_package(catkin REQUIRED COMPONENTS pcl_ros cv_bridge roscpp rospy std_msgs image_transport message_generation)


# find the DepthSense SDK
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(DepthSenseSDK)
find_package(PCL 1.2 REQUIRED)

if(NOT DepthSenseSDK_FOUND)
  message(FATAL_ERROR "+++++++++ Could not locate the DepthSense SDK +++++++++
Please install the SDK before trying to build the 'softkinetic_camera' package, see README.txt.

Cannot continue, aborting.")
  return()
endif()


#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DepthSenseSDK_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

#include_directories(/softkinetic_camera/include/main.h)

add_executable(softkinetic_bringup_node src/softkinetic_start.cpp)
add_executable(multiple_segmentation_node src/plane_seg.cpp)
add_executable(ransac_seg_node src/ransac_seg.cpp)
add_executable(outliers_display_node src/outliers_display.cpp)
add_executable(euclidean_cluster_node src/euclidean_cluster_node.cpp)
add_executable(reconstruction_triangulation_node src/reconstruction_triangulation.cpp)



target_link_libraries(softkinetic_bringup_node ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(multiple_segmentation_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${DepthSenseSDK_LIBRARIES})
target_link_libraries(ransac_seg_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${DepthSenseSDK_LIBRARIES})
target_link_libraries(outliers_display_node  ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${DepthSenseSDK_LIBRARIES})
target_link_libraries(euclidean_cluster_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${DepthSenseSDK_LIBRARIES})
target_link_libraries(reconstruction_triangulation_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${DepthSenseSDK_LIBRARIES})


link_directories(${PCL_LIBRARY_DIRS})

install(TARGETS softkinetic_bringup_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
