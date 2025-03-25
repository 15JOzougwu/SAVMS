cmake_minimum_required(VERSION 3.5)
project(my_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(PahoMqttCpp REQUIRED)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Add flags for Gazebo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED src/model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} rclcpp::rclcpp)

add_library(velodyne_plugin SHARED src/velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})

add_executable(mqtt_publisher src/publisher_member_function.cpp)
ament_target_dependencies(mqtt_publisher rclcpp std_msgs)
target_link_libraries(mqtt_publisher
        PahoMqttCpp::paho-mqttpp3  # C++ library
)

install(TARGETS
  mqtt_publisher
  model_push
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
