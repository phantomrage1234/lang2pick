cmake_minimum_required(VERSION 3.8)
project(so101_control)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)

include_directories(include)

# --- 1. Build scservo static library ---
add_library(scservo STATIC
  driver/SCS.cpp
  driver/SCSCL.cpp
  driver/SCSerial.cpp
  driver/SMS_STS.cpp
  driver/SMSCL.cpp

)

target_include_directories(scservo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/driver>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/driver>
)

# --- 2. Hardware Plugin library (Shared) ---
add_library(${PROJECT_NAME} SHARED
  src/so101_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  scservo
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  hardware_interface
  controller_interface
)

# --- 3. Export plugin description to pluginlib ---
pluginlib_export_plugin_description_file(hardware_interface so101_hardware_interface.xml)

# --- 4. Install targets & headers ---
install(TARGETS ${PROJECT_NAME} scservo
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install public headers
install(DIRECTORY include/
  DESTINATION include
)

# Install driver headers separately to keep structure clean
install(DIRECTORY driver/
  DESTINATION include/${PROJECT_NAME}/driver
  FILES_MATCHING PATTERN "*.h"
)

# --- 5. Export package info for downstream packages ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp pluginlib hardware_interface controller_interface)
ament_export_include_directories(include)
ament_export_libraries(scservo ${PROJECT_NAME})

ament_package()
