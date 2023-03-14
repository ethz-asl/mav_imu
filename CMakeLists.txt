cmake_minimum_required(VERSION 3.0.2)
project(mav_imu)

add_compile_options(-std=c++17 -w)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )
find_package(roscpp)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS roscpp
)

include_directories(
        include
        lib/lpp/include
        lib/BMI08x-Sensor-API
        ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/spi_driver.cpp
        src/imu_node.cpp
        src/imu/adis16448.cpp
        src/imu/bmi088.cpp
        src/imu/ImuFactory.cpp)

# Bosch API
add_library(${PROJECT_NAME}_BMI08x
        lib/BMI08x-Sensor-API/bmi08a.c
        lib/BMI08x-Sensor-API/bmi08g.c
        lib/BMI08x-Sensor-API/bmi08xa.c)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_BMI08x)

add_subdirectory(lib/lpp)
target_link_directories(${PROJECT_NAME} PUBLIC lib/lpp)
target_link_libraries(${PROJECT_NAME} Log++ ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
        src/main.cpp
        )

target_compile_definitions(${PROJECT_NAME} PUBLIC MODE_ROSLOG) # Log++

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


catkin_add_gtest(${PROJECT_NAME}_test
        test/test_entry_point.cc
        test/test_vec3.cc
        test/test_adis16448.cc
        )

target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

