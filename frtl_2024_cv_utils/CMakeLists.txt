cmake_minimum_required(VERSION 3.5)
project(frtl_2024_cv_utils)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
