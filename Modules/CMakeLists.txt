target_include_directories(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

get_property(include_dirs TARGET ${CMAKE_PROJECT_NAME} PROPERTY INCLUDE_DIRECTORIES)
# foreach(dir ${include_dirs})
#   message(STATUS "dir='${dir}'")
# endforeach()

set(EMBFMT_CPU_PARAMS ${cpu_PARAMS})
add_subdirectory(embfmt)
target_link_libraries(${CMAKE_PROJECT_NAME} embfmt)

add_subdirectory(blocking_io)

if (ETH_STACK STREQUAL "ETHERLIB")
    set(ETH_STACK_LIB etherlib)
    set(ETHERLIB_INCLUDES ${include_dirs})
    set(ETHERLIB_CPU_PARAMS ${cpu_PARAMS})
    add_subdirectory(etherlib)

    target_link_libraries(${CMAKE_PROJECT_NAME} etherlib)
elseif(ETH_STACK STREQUAL "LWIP")
    set(ETH_STACK_LIB lwipcore)
    set(LWIP_DIR ${CMAKE_CURRENT_LIST_DIR}/lwip)
    set(LWIP_PORT_DIR ${CMAKE_CURRENT_LIST_DIR}/lwip_port)
    #set(LWIP_CONTRIB_DIR ${LWIP_DIR}/contrib)
    set(LWIP_INCLUDE_DIRS
        ${LWIP_DIR}/src/include
        ${LWIP_DIR}/contrib
        ${LWIP_PORT_DIR}/arch
        ${LWIP_PORT_DIR}
        ${include_dirs}
    )
    message(${LWIP_PORT_DIR})
    set(LWIP_COMPILER_FLAGS ${cpu_PARAMS})
    include(${LWIP_DIR}/src/Filelists.cmake)
    #include(${LWIP_DIR}/contrib/Filelists.cmake)
    
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${LWIP_DIR}/src/include ${LWIP_PORT_DIR})
    target_include_directories(lwipcore PUBLIC ${LWIP_DIR}/src/include ${LWIP_PORT_DIR})
    target_sources(lwipcore PUBLIC ${LWIP_PORT_DIR}/OS/sys_arch.c)
    #target_sources(${CMAKE_PROJECT_NAME} PUBLIC ${LWIP_PORT_DIR}/OS/sys_arch.c)

    target_link_libraries(${CMAKE_PROJECT_NAME} lwipcore)
endif()

# set(FLEXPTP_INCLUDES ${LWIP_INCLUDE_DIRS} ${include_dirs})
# set(FLEXPTP_CPU_PARAMS ${cpu_PARAMS})
# add_subdirectory(flexptp)

if (ETH_STACK STREQUAL "ETHERLIB")
    set(FLEXPTP_HWPORT "F439_ETHERLIB")
elseif(ETH_STACK STREQUAL "LWIP")
    set(FLEXPTP_HWPORT "F439_LWIP")
endif()

set(FLEXPTP_NSD ${ETH_STACK})
set(FLEXPTP_SERVO "PID")
set(FLEXPTP_INCLUDES ${include_dirs} ${LWIP_DIR}/src/include ${LWIP_PORT_DIR}/include)
set(FLEXPTP_CPU_PARAMS ${cpu_PARAMS})
set(FLEXPTP_COMPILE_DEFS ${symbols_c_SYMB})
add_subdirectory(flexptp)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${FLEXPTP_INCLUDE_EXPORT}) 
target_link_libraries(${CMAKE_PROJECT_NAME} flexptp)
