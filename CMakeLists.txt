cmake_minimum_required(VERSION 3.10)
project(sophus_viewer)

set(CMAKE_BUILD_TYPE Debug)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

################# dependencies #################
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# Thread
find_package (Threads)

set(THIRD_PARTY_LIBS
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

################### source #####################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(examples)
