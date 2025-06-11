target_include_directories(
    ${CMAKE_PROJECT_NAME} PUBLIC
    Device/ST/STM32F4xx/Include/
    Include
    CMSIS_RTOS_V2)

target_sources(
    ${CMAKE_PROJECT_NAME}
    PUBLIC
    CMSIS_RTOS_V2/cmsis_os2.c
)
