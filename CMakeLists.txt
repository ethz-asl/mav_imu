cmake_minimum_required(VERSION 3.5)
project(mav_imu)

add_compile_options(-std=c++17 -w)

# Valid modes are: MODE_LPP MODE_GLOG MODE_ROSLOG MODE_DEFAULT
# Use add_definitions(-DMODE_LPP) if cmake version <= 3.11
add_compile_definitions(MODE_LPP)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
        include
        lib/BMI08x-Sensor-API)

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

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

add_executable(${PROJECT_NAME}_node
        src/main.cpp
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE MODE_LPP) # Log++

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()