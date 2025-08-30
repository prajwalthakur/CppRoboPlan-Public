cmake_minimum_required(VERSION 3.14)
project_add_library(PRECOMPILE_HEADERS )

find_package(pinocchio REQUIRED)
find_package(drake REQUIRED)
# Optional: Eigen is already a dependency of pinocchio target, but finding it explicitly is harmless
find_package(Eigen3 QUIET)

add_executable(run_urdf main.cpp)

project_target_link_libraries(pinocchio::pinocchio)
project_target_link_libraries(drake::drake)
project_add_subdirectories()