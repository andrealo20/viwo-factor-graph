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

# Jacobian verification. This is the test that matters: a wrong analytical
# derivative does not raise an error, it produces an optimiser that converges
# to the wrong answer, so it has to be checked against numerical
# differentiation under GTSAM's own retraction.
enable_testing()

add_executable(test_jacobians tests/test_jacobians.cpp)
target_link_libraries(test_jacobians PRIVATE gtsam)

add_test(NAME jacobians COMMAND test_jacobians)
