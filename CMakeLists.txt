cmake_minimum_required(VERSION 3.0.2)
project(mav_imu)

add_compile_options(-std=c++17 -w)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        lpp
        )
find_package(roscpp)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        roscpp
        lpp
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/BMI08x-Sensor-API)
    message(STATUS "BMI08x-Sensor-API found")
    SET(MAV_IMU_BUILD_BOSCH ON)
    include_directories(lib/BMI08x-Sensor-API)
else()
    message(WARNING "BMI08x-Sensor-API not found. Bosch BMI088 will not be supported.")
endif()

include_directories(
        include
        ${catkin_INCLUDE_DIRS})

if (MAV_IMU_BUILD_BOSCH)
    set(MAV_IMU_BOSCH_SRC_FILE src/imu/bmi088.cpp)
endif ()

add_library(${PROJECT_NAME}
        src/spi_driver.cpp
        src/imu_node.cpp
        src/imu/adis16448.cpp
        ${MAV_IMU_BOSCH_SRC_FILE}
        src/imu/ImuFactory.cpp)

if (MAV_IMU_BUILD_BOSCH)
    target_compile_definitions(${PROJECT_NAME} PUBLIC MAV_IMU_BUILD_BOSCH)
# Bosch API
add_library(${PROJECT_NAME}_BMI08x
        lib/BMI08x-Sensor-API/bmi08a.c
        lib/BMI08x-Sensor-API/bmi08g.c
        lib/BMI08x-Sensor-API/bmi08xa.c)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_BMI08x)
endif()

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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

