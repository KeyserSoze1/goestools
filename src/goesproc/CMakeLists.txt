file(GLOB GOESPROC_SRCS *.cc)
add_executable(goesproc ${GOESPROC_SRCS})
add_sanitizers(goesproc)
install(TARGETS goesproc COMPONENT goestools RUNTIME DESTINATION bin)
target_link_libraries(goesproc lrit util assembler packet_reader dir)
target_link_libraries(goesproc opencv_core opencv_highgui opencv_imgproc)
target_link_libraries(goesproc z)

find_package(PkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)
if("${OPENCV_VERSION}" VERSION_GREATER 3.0)
  target_link_libraries(goesproc opencv_imgcodecs)
endif()
