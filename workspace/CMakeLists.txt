cmake_minimum_required(VERSION 3.14)
project(cpproboplan LANGUAGES CXX)
# option(BUILD_DOC "Build doxygen documentation" OFF)
option(BUILD_EXAMPLES "Build Examples" ON)
option(BUILD_UNIT_TESTS "Build Unit tests" ON)


# Path to helper functions
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(ProjectUtils)

# Add modules
add_subdirectory(core)
add_subdirectory(planner)
add_subdirectory(inverseKinematics)
add_subdirectory(visualizer)
add_subdirectory(models)
add_subdirectory(examples)


