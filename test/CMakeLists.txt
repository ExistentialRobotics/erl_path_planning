cmake_minimum_required(VERSION 3.24)

link_libraries(${PROJECT_NAME}::${PROJECT_NAME} GTest::gtest_main)

add_executable(test_astar_2d gtest/test_astar_2d.cpp)
gtest_discover_tests(test_astar_2d)

add_executable(test_astar_se2 test_astar_se2.cpp)
target_compile_definitions(test_astar_se2
        PRIVATE
        -DDDC_MOTION_PRIMITIVES_FILE="${CMAKE_CURRENT_LIST_DIR}/ddc_motion_primitives.yaml"
        PRIVATE
        -DGRID_MAP_DATA_FILE="${CMAKE_CURRENT_LIST_DIR}/circles_map_1001x1001.txt")
gtest_discover_tests(test_astar_se2)

add_test(NAME test_astar_2d_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/test_astar_2d.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set_tests_properties(test_astar_2d_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

add_test(NAME test_astar_se2_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/test_astar_se2.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set_tests_properties(test_astar_se2_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")

add_test(NAME test_astar_grid_se2_py
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/test_astar_grid_se2.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
set_tests_properties(test_astar_grid_se2_py PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/../python")
