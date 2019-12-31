cmake_minimum_required( VERSION 2.8)

project(3DReconstruct)

# opencv
set(OpenCV_DIR $ENV{OPENCV_DIR})
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIR}/include)
include_directories(${OpenCV_DIR}/include/opencv)
include_directories(${OpenCV_DIR}/include/opencv2)

# 查找源文件
aux_source_directory(. DIR_SRCS)
FILE(GLOB HEADER ./*.h)

add_executable(GrayCodePhaseShift ${DIR_SRCS} ${HEADER})

target_link_libraries(GrayCodePhaseShift ${OpenCV_LIBS})

# install(TARGETS GrayCodePhaseShift RUNTIME DESTINATION bin)
