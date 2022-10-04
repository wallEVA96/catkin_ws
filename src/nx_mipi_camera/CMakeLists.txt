cmake_minimum_required(VERSION 3.5)
project(nx_mipi_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)

link_directories(/usr/lib/aarch64-linux-gnu/tegra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	vision_msgs
	std_msgs
	image_transport
)

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES deep_learning_nodelet
	CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${CUDA_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(nx_mipi_camera src/nx_mipi_camera.cpp src/image_converter.cpp)

add_dependencies(nx_mipi_camera 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(nx_mipi_camera
	${catkin_LIBRARIES}
	jetson-inference
)
