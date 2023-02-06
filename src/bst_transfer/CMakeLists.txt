cmake_minimum_required(VERSION 3.5)
project(bst_transfer)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#add_definitions(-stdlib=libstdc++)

find_package(ament_cmake  REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(std_img_talker src/pub_image.cpp)
ament_target_dependencies(std_img_talker rclcpp std_msgs sensor_msgs cv_bridge)
#target_link_libraries(std_msg_talker -lstdc++)


add_executable(std_img_listener src/sub_image.cpp)
ament_target_dependencies(std_img_listener rclcpp std_msgs)
#target_link_libraries(std_msg_listener -lstdc++)

install(TARGETS
std_img_talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
std_img_listener
  DESTINATION lib/${PROJECT_NAME})


ament_package()

