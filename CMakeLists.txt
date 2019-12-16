cmake_minimum_required(VERSION 2.8.3)
project(cuke_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
  tf
# NOTE : cv_bridge is causing linker issues. Refer here :https://answers.ros.org/question/312235/error-linking-to-opencv-dnn-library/ 
)

find_package(Boost REQUIRED COMPONENTS thread)

set(OpenCV_DIR "/home/kylejosling/installation/OpenCV-3.4")
find_package(OpenCV 3.4.8 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)

MESSAGE(WARNING "install path ${OpenCV_INSTALL_PATH}")
MESSAGE(WARNING "include dirs ${OpenCV_INCLUDE_DIRS} ")
MESSAGE(WARNING "library dirs ${OpenCV_LIBRARIES} ")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################


# Generate messages in the 'msg' folder
add_message_files(
  FILES
  boundingBoxMsg.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

################################################
## Find libraries
################################################
link_directories(/usr/local/lib)
find_library(pid_LIBRARY pid)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cuke_vision 
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS Boost OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ardrone_object_tracking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(cukeVisionNode src/cukeVisionNode.cpp src/detectObject.cpp)
 target_link_libraries(cukeVisionNode
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(stereoCamNode src/stereoCamNode.cpp)
 target_link_libraries(stereoCamNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${pid_LIBRARY}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(stemDetectionNode src/stemDetectionNode.cpp)
 target_link_libraries(stemDetectionNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${pid_LIBRARY}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(baseFrameBroadcasterNode src/baseFrameBroadcasterNode.cpp)
 target_link_libraries(baseFrameBroadcasterNode
   ${catkin_LIBRARIES}
)

add_executable(boundingBoxNode src/boundingBoxNode.cpp)
 target_link_libraries(boundingBoxNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${pid_LIBRARY}
   ${realsense2_LIBRARY}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ardrone_object_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
