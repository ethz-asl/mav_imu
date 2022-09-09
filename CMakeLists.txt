cmake_minimum_required(VERSION 3.0.2)
project(mav_imu)

add_compile_options(-std=c++17)

find_package(catkin_simple)

add_compile_definitions(MODE_ROSLOG) # Log++

catkin_simple()

cs_add_library(${PROJECT_NAME}
        src/spi_driver.cpp
        src/imu_node.cpp
        src/imu/adis16448.cpp
        )

cs_add_executable(${PROJECT_NAME}_node
        src/main.cpp
        )

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


catkin_add_gtest(${PROJECT_NAME}_test
        test/test_entry_point.cc
        test/test_vec3.cc
        test/test_adis16448_conversions.cc
        src/imu/adis16448.cpp
        )

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()

