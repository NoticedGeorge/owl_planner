cmake_minimum_required(VERSION 3.16)
project(owl_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT DEFINED PCL_DIR)
    set(PCL_DIR "/usr/local/share/pcl-1.15")
endif()

find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_executable(owl_planner
    owl_planner_main.cpp
    src/Config.cpp
    src/Controller.cpp
    src/Perception.cpp
    src/Pipeline.cpp
    src/Visualizer.cpp
    src/World.cpp
    src/Bypass.cpp
    src/ObstacleProcessor.cpp
    src/Planner.cpp
    src/PolynomialTrajectory.cpp
    src/safeField.cpp
)

target_include_directories(owl_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(owl_planner PRIVATE ${PCL_DEFINITIONS})
target_compile_options(owl_planner PRIVATE -Wall -Wextra)

target_link_libraries(owl_planner PRIVATE
    Eigen3::Eigen
    Ceres::ceres
    yaml-cpp
    ${PCL_LIBRARIES}
    Threads::Threads
)
