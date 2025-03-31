# Root CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(ps1emu LANGUAGES CXX)  # Explicitly specify C++ language

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing to help track progress
enable_testing()

# Function to add a specific notes project
function(add_notes_project PROJECT_NAME)
    # Create a new subdirectory project
    add_subdirectory(${PROJECT_NAME})
endfunction()

# Dynamically find and add all Notes directories
file(GLOB NOTES_DIRS "Notes*")
foreach(NOTES_DIR ${NOTES_DIRS})
    if(IS_DIRECTORY ${NOTES_DIR})
        # Extract the directory name
        get_filename_component(NOTES_NAME ${NOTES_DIR} NAME)
        
        # Add the notes project
        add_notes_project(${NOTES_NAME})
    endif()
endforeach()