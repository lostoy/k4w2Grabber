cmake_minimum_required(VERSION 2.8)
project(kinect2recorder)
set(BOOST_LIBRARYDIR C:/local/boost_1_55_0/lib64-msvc-11.0)
set(BOOST_INCLUDEDIR C:/local/boost_1_55_0)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_USE_MULTITHREAD )
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost COMPONENTS regex filesystem system REQUIRED)


find_package(PCL 1.2 REQUIRED COMPONENTS common io io_ply visualization)



include_directories(${Boost_INCLUDE_DIRS} ../kinect2grabber/include "C:/Program Files/Microsoft SDKs/Kinect/v2.0-DevPreview1311/inc" ../IMUReader/include)


set(
srcs src/main.cpp
)  


add_definitions(${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS} )
add_executable(kinect2recorder ${srcs} )

target_link_libraries(kinect2recorder ${Boost_LIBRARIES} ${PCL_LIBRARIES} k2grabber IMUReader)


