cmake_minimum_required(VERSION 3.14)
project_add_example()

find_package(pinocchio REQUIRED)
find_package(drake REQUIRED)
# Optional: Eigen is already a dependency of pinocchio target, but finding it explicitly is harmless
find_package(Eigen3 QUIET)
project_target_link_libraries(pinocchio::pinocchio)
project_target_link_libraries(drake::drake)
project_target_link_libraries(core)
project_target_link_libraries(planner)
project_target_link_libraries(visualizer)
