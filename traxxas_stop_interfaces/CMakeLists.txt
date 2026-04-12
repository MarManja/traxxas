cmake_minimum_required(VERSION 3.8)
project(traxxas_stop_interfaces)
 
# Compiladores con warnings estándar
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# Dependencias obligatorias para generar interfaces
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
 
# Declara la action que debe compilarse
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/StopRoutine.action"
  DEPENDENCIES action_msgs
)
 
ament_package()