cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

foreach(directory config dae launch meshes urdf xacro worlds)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

install(PROGRAMS
  scripts/activate_controllers_while_stepping.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
