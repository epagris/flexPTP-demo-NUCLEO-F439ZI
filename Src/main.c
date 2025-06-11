// STDLIB includes
#include <memory.h>
#include <stdint.h>
#include <stdlib.h>

// HAL includes
#include <stm32f4xx_hal.h>
#include <stm32f4xx_ll_rng.h>
#include <string.h>
#include <system_stm32f4xx.h>

// OS-related includes
#include <FreeRTOS.h>
#include <cmsis_os2.h>

// our includes
#include "cliutils/cli.h"
#include "cmds.h"

#include "flexptp/event.h"
#include "flexptp/logging.h"
#include "flexptp/profiles.h"
#include "flexptp/ptp_profile_presets.h"
#include "flexptp/settings_interface.h"
#include "standard_output/serial_io.h"
#include "standard_output/standard_output.h"
#include "standard_output/term_colors.h"

#ifdef ETH_ETHERLIB
#include "ethernet/ethernet_etherlib.h"
#elif defined(ETH_LWIP)
#include "ethernet/ethernet_lwip.h"
#endif

#define FLEXPTP_INITIAL_PROFILE ("gPTP")

// ------------------------

void Error_Handler(void);

// ------------------------

#define TARGET_SYSCLK_MHZ (configCPU_CLOCK_HZ / 1000000)

void init_osc_and_clk() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; // TODO: turn HSE bypass OFF if using X3 instead of the board controller's clock output
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; 
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = TARGET_SYSCLK_MHZ;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }

    // compute SYSCLK values (HSE_VALUE must be correct when invoking this)
    SystemCoreClockUpdate();

    // set tick frequency
    HAL_SetTickFreq(HAL_TICK_FREQ_1KHZ);

    // configure internal voltage regulator
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // configure GPIOs for high-speed operation
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_EnableCompensationCell();

    // apply silicon bug workaround
    if (HAL_GetREVID() == 0x1001) {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

void print_welcome_message() {
    MSGraw("\033[2J\033[H");
    MSG(ANSI_COLOR_BGREEN "Hi!" ANSI_COLOR_BYELLOW " This is a flexPTP demo for the STMicroelectronics NUCLEO-F439ZI (STM32F439) board.\n\n"
                          "The application is built on FreeRTOS, flexPTP is currenty compiled against %s and uses the supplied example %s Network Stack Driver. "
                          "In this demo, the underlying Ethernet stack can be either lwip or EtherLib, the 'ETH_STACK' CMake variable (in the main CMakeLists.txt file) determines which one will be used. "
                          "The STM32F4xx PTP hardware module driver is also picked from the bundled ones. This flexPTP instance features a full CLI control interface, the help can be listed by typing '?' once the flexPTP has loaded. "
                          "The initial PTP preset that loads upon flexPTP initialization is the 'gPTP' (802.1AS) profile. It's a nowadays common profile, but we encourage "
                          "you to also try out the 'default' (plain IEEE 1588) profile and fiddle around with other options as well. The application will try to acquire an IP-address with DHCP. "
                          "Once the IP-address is secured, you might start the flexPTP module by typing 'flexptp'. 'Have a great time! :)'\n\n" ANSI_COLOR_RESET,
        ETH_STACK, ETH_STACK);

    MSG(ANSI_COLOR_BRED "By default, the MCU clock is sourced by the onboard (STLink) board controller on this devboard. According to our observations, this clock signal is loaded "
                        "with heavy noise rendering the clock synchronization unable to settle precisely. We highly recommend to solder a 8 MHz oscillator onto "
                        " the designated X3 pads to achieve the best results!\n\n" ANSI_COLOR_RESET);

    // MSG("Freq: %u\n", SystemCoreClock);
}

void task_startup(void *arg) {
    // initialize serial IO
    serial_io_init();

    // print welcome message
    print_welcome_message();

    // initialize CLI
    cli_init();

    // initialize CLI commands
    cmd_init();

    // initialize Ethernet
#ifdef ETH_ETHERLIB
    init_ethernet();
#elif defined(ETH_LWIP)
    init_ethernet();
#endif

    for (;;) {
        osDelay(1000);
    }
}

void flexptp_user_event_cb(PtpUserEventCode uev) {
    switch (uev) {
    case PTP_UEV_INIT_DONE:
        ptp_load_profile(ptp_profile_preset_get(FLEXPTP_INITIAL_PROFILE));
        ptp_print_profile();

        ptp_log_enable(PTP_LOG_DEF, true);
        ptp_log_enable(PTP_LOG_BMCA, true);
        break;
    default:
        break;
    }
}

void init_randomizer() {
    __HAL_RCC_RNG_CLK_ENABLE();

    LL_RNG_Enable(RNG);

    while (!LL_RNG_IsActiveFlag_DRDY(RNG)) {
    }

    srand(LL_RNG_ReadRandData32(RNG));
}

int main(void) {
    // initialize FPU and several system blocks
    SystemInit();

    // initialize oscillator and clocking
    init_osc_and_clk();

    // initialize rand()
    init_randomizer();

    // initialize HAL
    HAL_Init();

    // initialize the FreeRTOS kernel
    osKernelInitialize();

    // create startup thread
    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(attr));
    attr.stack_size = 2048;
    attr.name = "init";
    osThreadNew(task_startup, NULL, &attr);

    // start the FreeRTOS!
    osKernelStart();

    /* Loop forever */
    for (;;)
        ;
}

// ------------------------

uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".FreeRTOSHeapSection")));

// ------------------------

void vApplicationTickHook(void) {
    HAL_IncTick();
}

void vApplicationIdleHook(void) {
    return;
}

void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

void NMI_Handler(void) {
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}
