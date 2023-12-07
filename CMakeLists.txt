cmake_minimum_required(VERSION 3.5)
project(computer_graphics_hw04)

set(CMAKE_CXX_STANDARD 17)

set(EXE_NAME hw04)
set(CXX_SOURCES main.cpp)

# add_subdirectory(third-party/glm)

add_executable(${EXE_NAME} ${CXX_SOURCES})
target_include_directories(${EXE_NAME} PRIVATE third-party/glm)
