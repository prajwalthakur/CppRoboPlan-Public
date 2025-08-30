project_add_library()
find_package(GTest)
project_target_link_libraries(GTest::gtest)
project_target_link_libraries(core)
