cmake_minimum_required(VERSION 2.6)
project(CameraCalibration)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 头文件的搜索路径
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})

# 设置可执行文件在那个目录下生成
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

find_package( OpenCV 3.4 REQUIRED )
include_directories( 
    ${OpenCV_INCLUDE_DIRS}
)


add_subdirectory(src)
