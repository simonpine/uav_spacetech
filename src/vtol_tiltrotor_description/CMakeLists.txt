cmake_minimum_required(VERSION 3.8)
project(vtol_tiltrotor_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config rviz worlds
        DESTINATION share/${PROJECT_NAME}
        OPTIONAL)

ament_package()
