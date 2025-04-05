# Specify common source files
set(COMMON_SOURCES
    cpu_core.cpp
    interrupt_controller.cpp
)

# Specify header files 
set(HEADERS
    cpu_core.h
    interrupt_controller.h
)

# Create CPU tests executable
add_executable(psx_emulation_cpu_tests 
    ${COMMON_SOURCES} 
    cpu_core_tests.cpp
    ${HEADERS}
)

# Create interrupt controller tests executable
add_executable(psx_emulation_interrupt_tests 
    ${COMMON_SOURCES} 
    interrupt_controller_tests.cpp
    ${HEADERS}
)

# Include directories (for both executables)
target_include_directories(psx_emulation_cpu_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psx_emulation_interrupt_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set compiler flags (for both executables)
if(MSVC)
    target_compile_options(psx_emulation_cpu_tests PRIVATE /W4)
    target_compile_options(psx_emulation_interrupt_tests PRIVATE /W4)
else()
    target_compile_options(psx_emulation_cpu_tests PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(psx_emulation_interrupt_tests PRIVATE -Wall -Wextra -pedantic)
endif()

# Output binaries to bin directory
set_target_properties(psx_emulation_cpu_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(psx_emulation_interrupt_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)