cmake_minimum_required(VERSION 3.10)

# Project Name
project(UAV_Simulation)

# Use C++17 (Standard for modern threading features)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

include_directories(
  ../external/AntTweakBar-1.16/include/ ../external/glfw-3.1.2/include/
  ../external/glm-0.9.7.1/ ../external/glew-1.13.0/include/
  ../external/assimp-3.0.1270/include/ ../external/bullet-2.81-rev2613/src/)

set(ALL_LIBS ${OPENGL_LIBRARY} glfw GLEW_1130)

# Find the system's threading library (Required for std::thread on Linux/Mac)
find_package(Threads REQUIRED)

# Define the executable Replace 'main.cpp' with 'test_main.cpp' if you are
# running the test script
add_executable(UAV_Sim main.cpp ECE_UAV.cpp ECE_UAV.h 	../common/shader.cpp
	../common/shader.hpp
	../common/controls.cpp
	../common/controls.hpp
	../common/texture.cpp
	../common/texture.hpp
	../common/objloader.cpp
	../common/objloader.hpp
	../common/vboindexer.cpp
	../common/vboindexer.hpp
)

# Link the threading library to the executable
target_link_libraries(UAV_Sim PRIVATE Threads::Threads)
