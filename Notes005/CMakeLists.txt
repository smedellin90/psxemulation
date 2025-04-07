# Specify common source files
set(COMMON_SOURCES
    cpu_core.cpp
    interrupt_controller.cpp
    memory_controller.cpp
)

# Specify header files 
set(HEADERS
    cpu_core.h
    interrupt_controller.h
    memory_controller.h
)

# Create CPU tests executable
add_executable(psx_emulation_cpu_tests_005
    ${COMMON_SOURCES} 
    cpu_core_tests.cpp
    ${HEADERS}
)

# Create interrupt controller tests executable
add_executable(psx_emulation_interrupt_tests_005
    ${COMMON_SOURCES} 
    interrupt_controller_tests.cpp
    ${HEADERS}
)

# Create memory controller tests executable
add_executable(psx_emulation_memory_tests_005
    ${COMMON_SOURCES} 
    memory_controller_tests.cpp
    ${HEADERS}
)

# Create CPU-Memory Controller integration tests executable
add_executable(psx_emulation_cpu_memory_cache_tests_005
    ${COMMON_SOURCES} 
    cpu_memory_controller_and_cache_tests.cpp
    ${HEADERS}
)

# Create CPU-Interrupt Controller integration tests executable
add_executable(psx_emulation_cpu_interrupt_tests_005
    ${COMMON_SOURCES} 
    cpu_interrupt_controller_tests.cpp
    ${HEADERS}
)

# Include directories (for all executables)
target_include_directories(psx_emulation_cpu_tests_005 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psx_emulation_interrupt_tests_005 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psx_emulation_memory_tests_005 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psx_emulation_cpu_memory_cache_tests_005 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psx_emulation_cpu_interrupt_tests_005 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Set compiler flags (for all executables)
if(MSVC)
    target_compile_options(psx_emulation_cpu_tests_005 PRIVATE /W4)
    target_compile_options(psx_emulation_interrupt_tests_005 PRIVATE /W4)
    target_compile_options(psx_emulation_memory_tests_005 PRIVATE /W4)
    target_compile_options(psx_emulation_cpu_memory_cache_tests_005 PRIVATE /W4)
    target_compile_options(psx_emulation_cpu_interrupt_tests_005 PRIVATE /W4)
else()
    target_compile_options(psx_emulation_cpu_tests_005 PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(psx_emulation_interrupt_tests_005 PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(psx_emulation_memory_tests_005 PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(psx_emulation_cpu_memory_cache_tests_005 PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(psx_emulation_cpu_interrupt_tests_005 PRIVATE -Wall -Wextra -pedantic)
endif()

# Output binaries to bin directory
set_target_properties(psx_emulation_cpu_tests_005
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(psx_emulation_interrupt_tests_005
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(psx_emulation_memory_tests_005
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(psx_emulation_cpu_memory_cache_tests_005
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

set_target_properties(psx_emulation_cpu_interrupt_tests_005
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)