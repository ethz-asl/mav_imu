cmake_minimum_required(VERSION 3.0.2)
project(mav_imu)

add_compile_options(-std=c++17)

find_package(catkin_simple)

catkin_simple()

cs_add_executable(${PROJECT_NAME}_node
        src/main.cpp
        src/spi_driver.cpp
        src/imu_node.cpp
        src/imu/adis16448.cpp
        )

target_compile_definitions(${PROJECT_NAME}_node PRIVATE MODE_ROSLOG)


catkin_add_gtest(${PROJECT_NAME}_test
        test/test_entry_point.cc
        test/test_vec3.cc
        )

target_compile_definitions(${PROJECT_NAME}_test PRIVATE MODE_ROSLOG)

cs_install()
cs_export()

