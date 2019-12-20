cmake_minimum_required( VERSION 2.8)

project(3DReconstruct)

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# pcl
# find_package(PCL 1.9 REQUIRED COMPONENTS common io)
find_package(PCL 1.9 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 库目录
# link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_ROOT}/3rdParty/OpenNI2/Lib)
link_directories(${PCL_ROOT}/3rdParty/VTK/lib)
link_directories(${PCL_ROOT}/3rdParty/Qhull/lib)
link_directories(${PCL_ROOT}/3rdParty/FLANN/lib)
link_directories(${PCL_ROOT}/3rdParty/Boost/lib)
link_directories(${PCL_ROOT}/lib)

# 包含目录
include_directories(${PCL_ROOT}/3rdParty/OpenNI2/Include)
include_directories(${PCL_ROOT}/3rdParty/VTK/include/vtk-8.1)
include_directories(${PCL_ROOT}/3rdParty/Qhull/include)
include_directories(${PCL_ROOT}/3rdParty/FLANN/include)
include_directories(${PCL_ROOT}/3rdParty/Eigen/eigen3)
include_directories(${PCL_ROOT}/3rdParty/Boost/include/boost-1_68)
include_directories(${PCL_ROOT}/include/pcl-1.9)

# opencv
set(OpenCV_DIR $ENV{OPENCV_DIR})
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIR}/include)
include_directories(${OpenCV_DIR}/include/opencv)
include_directories(${OpenCV_DIR}/include/opencv2)

add_executable(Phase_Shift main.cpp CalPhase.h CameraCalib.h)
add_library(Camera_Calib_LIB CameraCalib.cpp CameraCalib.h)
add_library(CalPhase_LIB CalPhase.cpp CalPhase.h)
add_library(ProPoint_LIB Propointcloud.cpp Propointcloud.h)

target_link_libraries(Phase_Shift ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(Phase_Shift Camera_Calib_LIB)
target_link_libraries(Phase_Shift CalPhase_LIB)
target_link_libraries(Phase_Shift ProPoint_LIB)


install(TARGETS Phase_Shift RUNTIME DESTINATION bin)
