cmake_minimum_required(VERSION 2.8.3)
project(urVision)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

# Setting OpenCV version
find_package(OpenCV 3 REQUIRED)
message("OpenCV version: ${OpenCV_VERSION}")

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)

add_message_files(
    FILES
    weedData.msg
    weedDataArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
#  DEPENDS
#    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#  ${Boost_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
  src/PlantDetector.cpp
  src/PlantFilter.cpp
)

## Declare cpp executables
add_executable(test_image_publisher
  src/test_image_publisher_node.cpp
)

# Add message generation dependency
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Add Object Tracker Core (as core library of tracker node)
add_library(tracker_core
  src/ObjectTracker.cpp
)

add_dependencies(tracker_core
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tracker_core
  ${catkin_LIBRARIES}
)

## Add the Tracker node
add_executable(tracker
    src/tracker_node.cpp
)

add_dependencies(tracker
  tracker_core
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tracker
  tracker_core
  ${catkin_LIBRARIES}
)

add_dependencies(test_image_publisher
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(test_image_publisher
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} test_image_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_urVision.cpp
  test/AlgorithmTest.cpp)
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
)

catkin_add_gtest(ObjectTracker-test
  test/ObjectTrackerTest.cpp
)
endif()

if(TARGET ObjectTracker-test)
  target_link_libraries(ObjectTracker-test ${PROJECT_NAME}_core)
endif()