cmake_minimum_required(VERSION 2.8.3)
project(imu_cali)

set(CMAKE_CXX_STANDARD 14)
add_definitions(-std=c++11 -DG2O_USE_VENDORED_CERES)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  serial
  std_msgs
  std_srvs
  tf
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

list(APPEND CMAKE_MODULE_PATH /home/sino/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O REQUIRED)
set(G2O_LIBRARIES ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} ${G2O_CLI_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_DENSE} ${G2O_SOLVER_PCG} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_SOLVER_STRUCTURE_ONLY} ${G2O_SOLVER_EIGEN} ${G2O_TYPES_DATA} ${G2O_TYPES_ICP} ${G2O_TYPES_SBA} ${G2O_TYPES_SCLAM2D} ${G2O_TYPES_SIM3} ${G2O_TYPES_SLAM2D} ${G2O_TYPES_SLAM3D})

message(WARNING  "------------------ ${G2O_LIBRARIES}")
message(WARNING  "------------------ ${G2O_INCLUDE_DIR}")

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
#  INCLUDE_DIRS include
#  LIBRARIES mpu6050_serial_to_imu
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs sensor_msgs serial std_msgs std_srvs tf using_markers visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${g2o_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ./src
)

## Declare a C++ executable
add_executable(acc_cali_node src/acc_cali.cpp)

target_link_libraries(acc_cali_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${G2O_LIBRARIES}	
)


#############
## Install ##
#############


## Mark executables and/or libraries for installation
# install(TARGETS yesense_imu_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install (DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install (DIRECTORY rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
