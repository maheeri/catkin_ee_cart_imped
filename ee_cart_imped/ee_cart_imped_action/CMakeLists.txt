cmake_minimum_required(VERSION 2.8.3)
project(ee_cart_imped_action)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS actionlib roscpp rospy roslib rosconsole std_msgs actionlib_msgs tf)

find_package(ee_cart_imped_msgs REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS ee_cart_imped_msgs
    CATKIN_DEPENDS actionlib roscpp rospy rosconsole roslib std_msgs actionlib_msgs tf
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
    LIBRARIES # TODO
)

#common commands for building c++ executables and libraries
add_executable(ee_cart_imped_action src/ee_cart_imped_action.cpp)
add_library(ee_cart_action_lib src/ee_cart_imped_arm.cpp)
add_executable(test_ee_cart_imped_action src/test_ee_cart_imped_action.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(test_ee_cart_imped_action ee_cart_action_lib)## Generate added messages and services with any dependencies listed here

#include_directories(include ${Boost_INCLUDE_DIRS} ${ee_cart_imped_msgs_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# ensure that headers get built before their targets (might not be completely correct) 
add_dependencies(ee_cart_imped_action ${catkin_EXPORTED_TARGETS}) 
