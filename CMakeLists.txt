cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)
set(CMAKE_COMPILER_IS_GNUCXX 1)

project(vdbApp)

# add vdbWrapper
file(GLOB_RECURSE VDB_WRAPPER_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h     
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.cpp
)

add_executable(${PROJECT_NAME} main.cpp ${VDB_WRAPPER_SRC})
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")


find_package(OpenVDB REQUIRED)
find_package(PCL REQUIRED)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} OpenVDB::openvdb)
