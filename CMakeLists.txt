cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_subdirectory( src/xsens_api )
rosbuild_add_library( XSensDriver 
				src/xsens_api/cmt1.cpp  
				src/xsens_api/cmt2.cpp  
				src/xsens_api/cmt3.cpp  
				src/xsens_api/cmtmessage.cpp  
				src/xsens_api/cmtpacket.cpp  
				src/xsens_api/cmtscan.cpp  
				src/xsens_api/xsens_exception.cpp  
				src/xsens_api/xsens_std.cpp  
				src/xsens_api/xsens_time.cpp
				src/xsens_driver/xsens_driver.cpp
				
 )
 
 
rosbuild_add_executable(xsens src/xsens_node/xsens.cpp)

target_link_libraries(xsens XSensDriver)

