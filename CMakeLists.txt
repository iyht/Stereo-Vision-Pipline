cmake_minimum_required(VERSION 3.16)
project(StereoCalibration)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_subdirectory(src)

# add SGM to build
add_subdirectory(extern/libSGM)
