cmake_minimum_required( VERSION 2.8 )
project( ICP )

set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# eigen
include_directories( "/usr/local/Cellar/eigen/3.3.7/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED ) #COMPONENT common io 
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


add_executable( ICP_PROC icp_proc.cpp )
target_link_libraries( ICP_PROC 
   ${PCL_LIBRARIES}
)
