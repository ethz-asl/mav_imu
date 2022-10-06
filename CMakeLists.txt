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
        ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/spi_driver.cpp
        src/imu_node.cpp
        src/imu/adis16448.cpp
        src/imu/ImuFactory.cpp)

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

