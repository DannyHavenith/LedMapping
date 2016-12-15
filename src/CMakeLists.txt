find_package( OpenCV REQUIRED )
set( CXX_STANDARD 11) 
add_executable( LedMapping LedMapping.cpp )
target_link_libraries( LedMapping ${OpenCV_LIBS} )
