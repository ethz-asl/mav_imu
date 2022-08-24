cmake_minimum_required(VERSION 3.0.2)
project(mav_imu)

add_compile_options(-std=c++17)

find_package(catkin_simple)

catkin_simple()

cs_add_executable(${PROJECT_NAME}_node
        src/main.cpp
        src/spi_driver.cpp
        src/imu_node.cpp
        )

cs_install()
cs_export()

