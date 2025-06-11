set(F4HAL_ROOT ${CMAKE_CURRENT_LIST_DIR}/stm32f4xx-hal-driver)

set(F4HAL_SRC
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_adc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_adc_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_can.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_cec.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_cortex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_crc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_cryp.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_cryp_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dac.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dac_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dcmi.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dcmi_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dfsdm.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dma2d.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dma.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dma_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_dsi.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_eth.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_exti.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_flash.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_flash_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_flash_ramfunc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_fmpi2c.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_fmpi2c_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_fmpsmbus.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_fmpsmbus_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_gpio.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_hash.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_hash_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_hcd.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_i2c.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_i2c_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_i2s.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_i2s_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_irda.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_iwdg.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_lptim.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_ltdc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_ltdc_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_mmc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_nand.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_nor.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_pccard.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_pcd.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_pcd_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_pwr.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_pwr_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_qspi.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_rcc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_rcc_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_rng.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_rtc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_rtc_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_sai.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_sai_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_sd.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_sdram.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_smartcard.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_smbus.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_spdifrx.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_spi.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_sram.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_tim.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_tim_ex.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_uart.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_usart.c
    ${F4HAL_ROOT}/Src/stm32f4xx_hal_wwdg.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_adc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_crc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_dac.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_dma2d.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_dma.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_exti.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_fmc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_fmpi2c.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_fsmc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_gpio.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_i2c.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_lptim.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_pwr.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_rcc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_rng.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_rtc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_sdmmc.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_spi.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_tim.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_usart.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_usb.c
    ${F4HAL_ROOT}/Src/stm32f4xx_ll_utils.c    
)


add_library(F4HAL STATIC ${F4HAL_SRC})

target_compile_options(F4HAL PUBLIC ${cpu_PARAMS})
target_compile_definitions(F4HAL PUBLIC ${compile_DEFS})
target_include_directories(F4HAL PUBLIC 
    ${F4HAL_ROOT}/Inc
    ${CMAKE_CURRENT_LIST_DIR}/../Inc
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_LIST_DIR}/CMSIS/Include
)

target_include_directories(
    ${CMAKE_PROJECT_NAME} PUBLIC
    ${F4HAL_ROOT}/Inc
)

target_link_libraries(${CMAKE_PROJECT_NAME} F4HAL)

add_subdirectory(CMSIS)

add_subdirectory(EthDrv)
