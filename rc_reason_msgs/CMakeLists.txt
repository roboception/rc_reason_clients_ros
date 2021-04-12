cmake_minimum_required(VERSION 3.1.0)

project(rc_reason_msgs)
include(cmake/project_version.cmake)

# - Configuration for code optimization -

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Build type: DEBUG or RELEASE" FORCE)
endif ()

# - Standard definitions -

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs shape_msgs rc_common_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Box.msg
  CellFillingLevel.msg
  CollisionDetection.msg
  Compartment.msg
  DetectedTag.msg
  Grasp.msg
  GridSize.msg
  Item.msg
  ItemModel.msg
  LoadCarrier.msg
  LoadCarrierWithFillingLevel.msg
  Match.msg
  RangeBox.msg
  RangeRectangle.msg
  RangeValue.msg
  Rectangle.msg
  RegionOfInterest2D.msg
  RegionOfInterest3D.msg
  SilhouetteMatchObject.msg
  SuctionGrasp.msg
  Tag.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CalibrateBasePlane.srv
  ComputeGrasps.srv
  DeleteBasePlaneCalibration.srv
  DeleteLoadCarriers.srv
  DeleteRegionsOfInterest2D.srv
  DeleteRegionsOfInterest3D.srv
  DetectFillingLevel.srv
  DetectItems.srv
  DetectLoadCarriers.srv
  DetectTags.srv
  GetBasePlaneCalibration.srv
  GetLoadCarriers.srv
  GetRegionsOfInterest2D.srv
  GetRegionsOfInterest3D.srv
  HandEyeCalibration.srv
  HandEyeCalibrationTrigger.srv
  SetHandEyeCalibration.srv
  SetHandEyeCalibrationPose.srv
  SetLoadCarrier.srv
  SetRegionOfInterest2D.srv
  SetRegionOfInterest3D.srv
  SilhouetteMatchDetectObject.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
  rc_common_msgs
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
#  LIBRARIES rc_msgs
  CATKIN_DEPENDS std_msgs geometry_msgs shape_msgs rc_common_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

######################################
## Define information for packaging ##
######################################
# optionally specify dependencies of the debian package here (comma separated!)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "ros-$ENV{ROS_DISTRO}-std-msgs, ros-$ENV{ROS_DISTRO}-geometry-msgs, ros-$ENV{ROS_DISTRO}-shape-msgs, ros-$ENV{ROS_DISTRO}-rc-common-msgs")
include(cmake/package_debian.cmake)
