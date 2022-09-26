cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(ORB_SLAM3)
rosbuild_init()


IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()
MESSAGE("Directory " ${PROJECT_SOURCE_DIR})
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure pcl_ros nodelet)


MESSAGE("Eigen 3 VERSION:")
MESSAGE(${Eigen3_VERSION})


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${Pangolin_INCLUDE_DIRS}
/usr/include/pcl-1.8/
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
${PCL_LIBRARIES}
-lboost_system
)

include_directories(
${EIGEN3_INCLUDE_DIR}
)

# generate_dynamic_reconfigure_options(
#   config/orb_slam3.cfg
#   #...
# )


# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
# rosbuild_add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )

# target_link_libraries(MonoAR
# ${LIBS}
# )

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
#Accidentally deleted RGBD original src file, removed for now

# rosbuild_add_executable(RGBD
# src/ros_rgbd.cc
# )

# target_link_libraries(RGBD
# ${LIBS}
# )

rosbuild_add_library(pcl_map_maker
src/pcl_map_maker.cc
)
target_link_libraries(pcl_map_maker
${LIBS}
)

rosbuild_add_library(ros_common
src/ros_common.cc
)
target_link_libraries(ros_common
${LIBS} pcl_map_maker
)

rosbuild_add_executable(RGBD2
src/ros_rgbd_node.cc
)
# add_dependencies(RGBD2 ${PROJECT_NAME}_gencfg)

target_link_libraries(RGBD2
${LIBS} pcl_map_maker ros_common
)

# Node for monocular-inertial camera
rosbuild_add_executable(Mono_Inertial
src/ros_mono_inertial.cc
)

target_link_libraries(Mono_Inertial
${LIBS}
)

# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial
src/ros_stereo_inertial.cc
)

target_link_libraries(Stereo_Inertial
${LIBS}
)

# rosbuild_add_executable(map_maker2
# src/occupancy_grid.cc
# )

# target_link_libraries(map_maker2
# ${LIBS}
# )

rosbuild_add_library(RGBDNodelet
src/ros_rgbd_nodelet.cc
)
target_link_libraries(RGBDNodelet
${LIBS} pcl_map_maker ros_common
)

install(
   FILES nodelet_plugins.xml
   DESTINATION ${PROJECT_SOURCE_DIR}/devel/share
)

