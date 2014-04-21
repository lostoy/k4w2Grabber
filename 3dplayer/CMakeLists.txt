cmake_minimum_required(VERSION 2.8)
project(3dplayer)
set(BOOST_LIBRARYDIR C:/local/boost_1_55_0/lib64-msvc-11.0)
set(BOOST_INCLUDEDIR C:/local/boost_1_55_0)

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(Boost_USE_MULTITHREAD )
set(Boost_NO_BOOST_CMAKE ON)

find_package(Boost COMPONENTS filesystem system REQUIRED)


find_package(PCL 1.2 REQUIRED COMPONENTS common io io_ply visualization)




set(
srcs src/3dplayer.cpp
)  


add_definitions(${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS} )
add_executable(3dplayer ${srcs} )

target_link_libraries(3dplayer ${Boost_LIBRARIES} ${PCL_LIBRARIES} )


