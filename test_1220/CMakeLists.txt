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

file(GLOB SRC ./*.cpp)
file(GLOB HEADER ./*.h)

add_executable(Gray_Code_Phase_Shift ${SRC} ${HEADER})

target_link_libraries(Gray_Code_Phase_Shift ${OpenCV_LIBS} ${PCL_LIBRARIES})

install(TARGETS Gray_Code_Phase_Shift RUNTIME DESTINATION bin)
