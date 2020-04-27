cmake_minimum_required(VERSION 2.8.3)
project(dnn_system_identification)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common_srv
  flight_controller
  genmsg
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

FIND_PACKAGE(PythonInterp)
FIND_PACKAGE(PythonLibs)

# add_message_files(
#   FILES

# )

# add_service_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime common_srv tensorflow
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

FILE(GLOB SOURCE_FILES *.hpp *.cpp *.h
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_executable(${PROJECT_NAME} src/main.cpp ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
