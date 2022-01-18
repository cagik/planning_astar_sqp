cmake_minimum_required(VERSION 3.0.2)
project(astar_osqp)

add_definitions(-std=c++11 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

set(catkin_deps
        roscpp
        grid_map_ros
        ros_viz_tools
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(gflags REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS OpenCV
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/main.cpp
        src/tool.cpp
        src/astar.cpp
        src/smoothosqpproblem.cpp
        src/interpolation1d.cpp
        src/KDTree.cpp
        src/corridor.cpp
        )


target_link_libraries(${PROJECT_NAME} glog gflags ${catkin_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp)


add_executable(${PROJECT_NAME}_demo src/main.cpp)
target_link_libraries(${PROJECT_NAME}_demo  ${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_demo /usr/local/lib/libosqp.so)
