cmake_minimum_required(VERSION 3.5)

project(simulator)

find_package(ament_cmake REQUIRED)

# Collect all source files
set(src)
add_subdirectory(description)


install(
  DIRECTORY
    description/
  DESTINATION share/${PROJECT_NAME}/description
)

install(
  DIRECTORY
    bridge/
  DESTINATION share/${PROJECT_NAME}/bridge
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()