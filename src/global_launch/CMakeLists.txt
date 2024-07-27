cmake_minimum_required(VERSION 3.5)

project(global_launch)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim)

find_package(sdfformat_vendor REQUIRED)
find_package(sdfformat)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport)

add_executable(my_executable src/exe.cpp)
target_link_libraries(my_executable gz-math::core)


install(
    DIRECTORY global_launch
    DESTINATION share/${PROJECT_NAME}
)