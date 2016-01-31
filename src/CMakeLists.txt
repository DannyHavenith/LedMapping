find_package( OpenCV REQUIRED )
add_executable( LedMapping LedMapping.cpp )
target_link_libraries( LedMapping ${OpenCV_LIBS} )
