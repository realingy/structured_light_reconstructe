cmake_minimum_required( VERSION 2.8)

project(MicroPS)

# opencv
set(OpenCV_DIR $ENV{OPENCV_DIR})
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIR}/include)
include_directories(${OpenCV_DIR}/include/opencv)
include_directories(${OpenCV_DIR}/include/opencv2)

aux_source_directory(. DIR_SRCS)
file(GLOB HEADER ./*.h)

set(EXENAME UnwrappedPhaseMicroPS)
add_executable(${EXENAME} ${DIR_SRCS} ${HEADER})
target_link_libraries(${EXENAME} ${OpenCV_LIBS})
