project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(include_files
    src/FusionEKF.h
    src/ground_truth_package.h
    src/kalman_filter.h
    src/measurement_package.h
    src/tools.h
)

set(source_files
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/main.cpp
    src/tools.cpp)

SOURCE_GROUP(include FILES ${include_files})
SOURCE_GROUP(src FILES ${source_files})

set(all_files
  ${include_files}
    ${source_files})

add_executable(ExtendedKF ${all_files})
