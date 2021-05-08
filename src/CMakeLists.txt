cmake_minimum_required(VERSION 3.16)
project(get_photos)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_LIBRARIES_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/extern/libSGM/include)

add_executable(get_photos get_photos.cpp)
target_link_libraries(get_photos ${OpenCV_LIBS})

add_executable(rectification rectification.cpp)
target_link_libraries(rectification ${OpenCV_LIBS})

cuda_add_executable(depth depth.cpp)
target_link_libraries(depth sgm ${CUDA_LIBRARIES} ${OpenCV_LIBS})

