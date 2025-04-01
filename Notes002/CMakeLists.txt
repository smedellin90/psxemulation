# Specify source files
set(SOURCES
    cpu_core.cpp
    cpu_core_tests.cpp
)

# Specify header files 
set(HEADERS
    cpu_core.h
)

# Create executable
add_executable(psx_emulation_notes002 ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(psx_emulation_notes002 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set compiler flags
if(MSVC)
    target_compile_options(psx_emulation_notes002 PRIVATE /W4)
else()
    target_compile_options(psx_emulation_notes002 PRIVATE -Wall -Wextra -pedantic)
endif()

# Output binary to bin directory
set_target_properties(psx_emulation_notes002
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)