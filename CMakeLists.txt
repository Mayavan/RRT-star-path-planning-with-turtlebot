cmake_minimum_required(VERSION 2.8.3)
project(rrt_star_path_planning_turtlebot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package( OpenCV REQUIRED )
find_package(PythonLibs 2.7)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(rrt_demo src/main.cpp src/Planner.cpp src/Map_manager.cpp src/controller.cpp)
add_executable(showImage src/imager.cpp src/Map_manager.cpp)

target_include_directories(rrt_demo PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(rrt_demo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
target_link_libraries(showImage ${catkin_LIBRARIES} ${OpenCV_LIBS})
