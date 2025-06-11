#include "serial_io.h"
#include "blocking_io/blocking_fifo.h"
#include "cmsis_os2.h"

#include <stdint.h>
#include <stm32f4xx_hal.h>
#include <string.h>

static void serial_io_tx_cplt(UART_HandleTypeDef *huart);
static void serial_io_rx_cplt(UART_HandleTypeDef *huart);
static void serial_io_thread(void * arg);

#define SERIAL_IO_FIFO_LEN (16384)

static UART_HandleTypeDef uart;
static DMA_HandleTypeDef dma;
static osEventFlagsId_t flags;
static osThreadId_t th;
static BFifo fifo;
static uint8_t buf[SERIAL_IO_FIFO_LEN];
static char c;

#define SERIAL_IO_TX_CPLT_FLAG (0x01)

void serial_io_init() {
    // initialize FIFO
    bfifo_create(&fifo, buf, SERIAL_IO_FIFO_LEN);

    // initialize flags and thread
    flags = osEventFlagsNew(NULL);

    osThreadAttr_t attr;
    memset(&attr, 0, sizeof(osThreadAttr_t));
    attr.name = "serialio";
    attr.stack_size = 2048;
    th = osThreadNew(serial_io_thread, NULL, &attr);

    // Enable peripheral clocks
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    // Initialize pins as USART3 Tx and Rx
    GPIO_InitTypeDef gpio;

    // TX
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Alternate = GPIO_AF7_USART3;
    gpio.Speed = GPIO_SPEED_HIGH;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &gpio);

    // RX
    gpio.Pin = GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOD, &gpio);

    // Initialize USART3
    uart.Instance = USART3;
    uart.Init.BaudRate = 115200;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    // huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&uart);

    HAL_UART_RegisterCallback(&uart, HAL_UART_TX_COMPLETE_CB_ID, serial_io_tx_cplt);
    HAL_UART_RegisterCallback(&uart, HAL_UART_RX_COMPLETE_CB_ID, serial_io_rx_cplt);

    HAL_UART_Receive_IT(&uart, (uint8_t *)&c, 1);

    HAL_NVIC_SetPriority(USART3_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    // configure DMA
    dma.Instance = DMA1_Stream3;
    dma.Init.Channel = DMA_CHANNEL_4;
    dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma.Init.PeriphInc = DMA_PINC_DISABLE;
    dma.Init.MemInc = DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma.Init.Mode = DMA_NORMAL;
    dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma.Init.FIFOMode = DMA_FIFO_THRESHOLD_FULL;
    dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    dma.Init.MemBurst = DMA_MBURST_SINGLE;
    dma.Init.PeriphBurst = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&dma);

    __HAL_LINKDMA(&uart, hdmatx, dma);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void DMA1_Stream3_IRQHandler() {
    HAL_DMA_IRQHandler(&dma);
}

void USART3_IRQHandler() {
    HAL_UART_IRQHandler(&uart);
}

void serial_io_write(const char *str, uint32_t len) {
    bfifo_push(&fifo, (const uint8_t *)str, len);
}

//  ----------

#define SERIAL_IO_LINEBUF_LEN (256)
static uint8_t lineBuf[SERIAL_IO_LINEBUF_LEN];

static void serial_io_thread(void * arg) {
    while (true) {
        bfifo_wait_avail(&fifo);
        uint32_t size = bfifo_read(&fifo, lineBuf, SERIAL_IO_LINEBUF_LEN);
        bfifo_pop(&fifo, size, 0);
        HAL_UART_Transmit_DMA(&uart, lineBuf, size);
        osEventFlagsWait(flags, SERIAL_IO_TX_CPLT_FLAG, 0, osWaitForever);
    }
}

//  ----------

static void serial_io_tx_cplt(UART_HandleTypeDef *huart) {
    osEventFlagsSet(flags, SERIAL_IO_TX_CPLT_FLAG);
}

__weak void serial_io_rx_callback(const uint8_t *data, uint32_t size) {
    (void)data;
    (void)size;
}

static void serial_io_rx_cplt(UART_HandleTypeDef *huart) {
    char k = c;
    HAL_UART_Receive_IT(&uart, (uint8_t *)&c, 1);
    serial_io_rx_callback((const uint8_t *)&k, 1);
}