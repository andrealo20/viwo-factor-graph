cmake_minimum_required(VERSION 3.16)
project(viwo_factor_graph VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)

add_executable(viwo_benchmark src/main_benchmark.cpp)
target_link_libraries(viwo_benchmark PRIVATE gtsam)
