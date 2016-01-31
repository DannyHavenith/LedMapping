cmake_minimum_required( VERSION 2.8)

project( ledmapping)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ftemplate-depth=512")

#include_directories( ${disseqt_SOURCE_DIR}/disseqt)

add_subdirectory( src)
file( COPY data/ DESTINATION data/)


