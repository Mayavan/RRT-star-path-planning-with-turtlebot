cmake_minimum_required(VERSION 2.8.3)
project(project5)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package( OpenCV REQUIRED )
find_package(PythonLibs 2.7)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project5
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable(Main src/Main.cpp src/Planner.cpp src/Map_manager.cpp)
add_executable(showImage src/imager.cpp src/Map_manager.cpp)

#target_include_directories(Main PRIVATE ${PYTHON_INCLUDE_DIRS})

#target_link_libraries(Main ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
target_link_libraries(showImage ${catkin_LIBRARIES} ${OpenCV_LIBS})
