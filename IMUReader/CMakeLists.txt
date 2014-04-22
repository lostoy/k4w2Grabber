cmake_minimum_required(VERSION 2.8)
project(IMUReader)
set(BOOST_LIBRARYDIR C:/local/boost_1_55_0/lib64-msvc-11.0)
set(BOOST_INCLUDEDIR C:/local/boost_1_55_0)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_USE_MULTITHREAD )
set(Boost_NO_BOOST_CMAKE ON)



include_directories(${Boost_INCLUDE_DIRS} "include")

find_package(PCL 1.2 REQUIRED COMPONENTS common visualization)
find_package(Boost COMPONENTS regex filesystem system date_time  thread chrono REQUIRED)

set(
srcs 
#	src/main.cpp
	 src/IMUReader.cpp
	 src/MadgwickAHRS.cpp
)  

include_directories(${PCL_INCLUDE_DIRS} )

#add_executable(IMUReader ${srcs} )
add_library(IMUReader ${srcs} )
target_link_libraries(IMUReader ${Boost_LIBRARIES} ${PCL_LIBRARIES})


