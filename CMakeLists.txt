cmake_minimum_required( VERSION 2.8)

project( ledmapping)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ftemplate-depth=512")


add_subdirectory( src)
file( COPY data/ DESTINATION data/)


