# Root CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(PSXEmulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable testing
enable_testing()

# Add Notes subdirectories
add_subdirectory(Notes001)
add_subdirectory(Notes002)
add_subdirectory(Notes003) 