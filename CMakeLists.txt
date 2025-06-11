cmake_minimum_required(VERSION 3.22)

#
# User is free to modify the file as much as necessary
#

list(APPEND CMAKE_MODULE_PATH "{{sr:cmake_path}}")
message("Build CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
include("cmake/gcc-arm-none-eabi.cmake")

# Core project settings
project(flexptp-demo)
enable_language(C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Core MCU flags, CPU type, instruction set and FPU setup
set(cpu_PARAMS 
    # Other parameters
    # -mthumb
    # -mcpu, -mfloat, -mfloat-abi, ...
)

# Set MCU type
set(MCU_TYPE STM32F439xx)

# Sources
set(sources_SRCS)

# Include directories for all compilers
set(include_DIRS 
    ${CMAKE_SOURCE_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Middlewares/Inc
    ${CMAKE_SOURCE_DIR}/Src
    ${CMAKE_SOURCE_DIR})

# Include directories for each compiler
set(include_c_DIRS)
set(include_cxx_DIRS)
set(include_asm_DIRS)

# Symbols definition for all compilers
set(symbols_SYMB)

# Symbols definition for each compiler
set(symbols_c_SYMB)
set(symbols_cxx_SYMB)
set(symbols_asm_SYMB)

# Link directories and names of libraries
set(link_DIRS)
set(link_LIBS freertos_kernel)

# Linker script
set(linker_script_SRC)

# Compiler options
set(compiler_OPTS)

# Linker options
set(linker_OPTS) 

# Now call generated cmake
# This will add script generated
# information to the project
include("cmake/vscode_generated.cmake")

# Link directories setup
# Must be before executable is added
link_directories(${CMAKE_PROJECT_NAME} ${link_DIRS})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${sources_SRCS})

# Add definitions

set(ETH_STACK "LWIP") # select "LWIP" or "ETHERLIB"

set(compile_DEFS ${MCU_TYPE} USE_HAL_DRIVER DEBUG CMSIS_OS2 ETH_STACK="${ETH_STACK}")
add_compile_definitions(${compile_DEFS})

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    ${include_DIRS}
    $<$<COMPILE_LANGUAGE:C>: ${include_c_DIRS}>
    $<$<COMPILE_LANGUAGE:CXX>: ${include_cxx_DIRS}>
    $<$<COMPILE_LANGUAGE:ASM>: ${include_asm_DIRS}>
)

# Add Middlewares
add_subdirectory(Middlewares)

add_compile_definitions(CMSIS_device_header="${device_header}") # exported from Middlewares
if (ETH_STACK STREQUAL "ETHERLIB")
    add_compile_definitions("ETH_ETHERLIB")
elseif(ETH_STACK STREQUAL "LWIP")
    add_compile_definitions("ETH_LWIP")
endif()

# Add Drivers (HAL and CMSIS)
add_subdirectory(Drivers)

# Add custom Ethernet driver
#add_subdirectory(Drivers/EthDrv)

# Add Modules
add_subdirectory(Modules)

# Add generic sources
add_subdirectory(Src)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${symbols_SYMB}
    $<$<COMPILE_LANGUAGE:C>: ${symbols_c_SYMB}>
    $<$<COMPILE_LANGUAGE:CXX>: ${symbols_cxx_SYMB}>
    $<$<COMPILE_LANGUAGE:ASM>: ${symbols_asm_SYMB}>

    # Configuration specific
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Release>: >
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME} ${link_LIBS})

# Compiler options
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    ${cpu_PARAMS}
    ${compiler_OPTS}
    -Wall
    -Wextra
    # -Wpedantic
    -Wno-unused-parameter
    $<$<COMPILE_LANGUAGE:C>: >
    $<$<COMPILE_LANGUAGE:CXX>:

    # -Wno-volatile
    # -Wold-style-cast
    # -Wuseless-cast
    # -Wsuggest-override
    >
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp -MMD -MP>
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>
)

# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${cpu_PARAMS}
    ${linker_OPTS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -u _printf_float # STDIO float formatting support (remove if not used)
    --specs=nosys.specs
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,-z,max-page-size=8 # Allow good software remapping across address space (with proper GCC section making)
    -Wl,--print-memory-usage
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)