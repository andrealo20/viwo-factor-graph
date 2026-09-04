cmake_minimum_required(VERSION 3.16)
project(viwo_factor_graph VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# No version floor on Eigen: find_package with a version fails outright when
# the installed config ships no version file, and the previous CMakeLists,
# which is the one that is known to have worked, asked for none.
find_package(Eigen3 REQUIRED)

# The wheel factor's evaluateError takes boost::optional<Matrix&> and the class
# exposes a boost::shared_ptr typedef, which is GTSAM's interface from 4.0 up
# to and including 4.2. GTSAM 4.3 moved the same interface to std::optional and
# std::shared_ptr, so the range is real rather than untested caution.
#
# It is a warning and not an error, and it runs only when a version is actually
# reported. Not every GTSAM package sets the variable, and an unset variable
# compares as less than 4.0, which would refuse to configure against a perfectly
# good install. Refusing to build over a version this cannot even read would be
# worse than letting the compiler complain about the real thing.
find_package(GTSAM REQUIRED)
set(VIWO_GTSAM_VERSION "")
foreach(_v GTSAM_VERSION GTSAM_VERSION_STRING gtsam_VERSION GTSAM_PACKAGE_VERSION)
    if(DEFINED ${_v} AND NOT "${${_v}}" STREQUAL "")
        set(VIWO_GTSAM_VERSION "${${_v}}")
        break()
    endif()
endforeach()
if(VIWO_GTSAM_VERSION)
    if(VIWO_GTSAM_VERSION VERSION_LESS 4.0 OR VIWO_GTSAM_VERSION VERSION_GREATER_EQUAL 4.3)
        message(WARNING
            "GTSAM ${VIWO_GTSAM_VERSION} found. This project targets 4.0 to 4.2: "
            "4.3 replaced boost::optional with std::optional in the "
            "NoiseModelFactor interface that wheel_odometry_factor.hpp overrides.")
    endif()
else()
    set(VIWO_GTSAM_VERSION "unknown")
endif()
message(STATUS "Building against GTSAM ${VIWO_GTSAM_VERSION}")

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
