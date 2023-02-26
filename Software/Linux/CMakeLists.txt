cmake_minimum_required(VERSION 2.8.3)
project(robot_dog)
SET(CMAKE_CXX_COMPILER "/usr/bin/arm-linux-gnueabihf-g++")
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -std=c++11 -pipe -DDEBUG")

include_directories(
  include/
  include/robot/
  include/cubic_interpolation/
  include/debug/
  include/serial/
  include/filter/
  include/queue/
  include/remote/
  include/singleton/
  include/Tools/
  include/IntertialUnit/
)

add_executable(robot_dog_node 
  app/node.cpp
  src/debug/my_debug.cpp
  src/debug/socket_paint.cpp
  src/filter/filter.cpp
  src/remote/remote.cpp
  src/robot/DataProgress.cpp
  src/robot/Robot.cpp
  src/robot/RobotDog.cpp
  src/robot/RobotArm.cpp
  src/cubic_interpolation/CubicInterpolation.cpp
  src/Tools/sem.cpp
  src/IntertialUnit/intertialController.cpp
  src/IntertialUnit/virtrualExecuter.cpp
)
target_link_libraries(robot_dog_node -lrt -lpthread -static-libstdc++)
