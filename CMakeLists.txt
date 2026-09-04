cmake_minimum_required(VERSION 3.16)
project(viwo_factor_graph VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED)

# The wheel factor's evaluateError takes boost::optional<Matrix&> and the class
# exposes a boost::shared_ptr typedef, which is GTSAM's interface from 4.0 up to
# and including 4.2. GTSAM 4.3 moved the same interface to std::optional and
# std::shared_ptr, so the upper bound is real rather than untested caution.
find_package(GTSAM REQUIRED)
if(GTSAM_VERSION VERSION_LESS 4.0 OR GTSAM_VERSION VERSION_GREATER_EQUAL 4.3)
    message(FATAL_ERROR
        "GTSAM ${GTSAM_VERSION} found. This project targets 4.0 to 4.2: 4.3 "
        "replaced boost::optional with std::optional in the NoiseModelFactor "
        "interface, which wheel_odometry_factor.hpp overrides.")
endif()
message(STATUS "Building against GTSAM ${GTSAM_VERSION}")

# Header-only library carrying the include path, so the targets below and any
# consumer pick it up without a global include_directories().
add_library(viwo INTERFACE)
target_include_directories(viwo INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(viwo INTERFACE gtsam)
if(TARGET Eigen3::Eigen)
    target_link_libraries(viwo INTERFACE Eigen3::Eigen)
else()
    target_include_directories(viwo INTERFACE ${EIGEN3_INCLUDE_DIR})
endif()

add_executable(viwo_benchmark src/main_benchmark.cpp)
target_link_libraries(viwo_benchmark PRIVATE viwo)

# Jacobian verification. This is the test that matters: a wrong analytical
# derivative does not raise an error, it produces an optimiser that converges
# to the wrong answer, so it has to be checked against numerical
# differentiation under GTSAM's own retraction.
enable_testing()

add_executable(test_jacobians tests/test_jacobians.cpp)
target_link_libraries(test_jacobians PRIVATE viwo)

add_test(NAME jacobians COMMAND test_jacobians)

# The same derivatives checked again from scratch in numpy, with no GTSAM
# involved, so a shared misunderstanding of GTSAM's conventions cannot hide in
# both. Skipped rather than failed when Python or numpy is absent.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME jacobians_independent
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_jacobians.py)
    set_tests_properties(jacobians_independent PROPERTIES SKIP_RETURN_CODE 77)
endif()
