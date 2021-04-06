cmake_minimum_required(VERSION 2.8.3)
project(cube_convert)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
    image_transport
        darknet_ros_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${PROJECT_SOURCE_DIR}/include)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        LIBRARIES
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        cv_bridge
        image_transport
        darknet_ros_msgs
)


add_executable(cube_convert_node src/main.cpp)
target_link_libraries(cube_convert_node ${catkin_LIBRARIES} ${OpenCV_LIBS})


