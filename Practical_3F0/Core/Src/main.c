/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (STM32F4 – Practical 3, Task 1)
  * @brief          : Mandelbrot timing + checksum (no LCD) with UART prints
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Choose the image sizes you used in Practical 1B:
static const int kSizes[][2] = {
    {64, 64},
    {128, 128},
    {256, 256}
    // Add/remove to match exactly what you used previously
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t start_time;
uint32_t end_time;
uint32_t execution_time;
uint64_t checksum;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_fixed_point(int width, int height, int max_iterations);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Retarget printf to UART2
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();      // Set this to 120 MHz in CubeMX
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  // Indicate start
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  printf("\r\n--- Practical 3: Task 1 (STM32F4, %lu Hz) ---\r\n",
         (unsigned long)HAL_RCC_GetHCLKFreq());
  printf("MAX_ITER=%d\r\n", MAX_ITER);

  // Loop over the same image sizes you used in Practical 1B
  for (size_t i = 0; i < (sizeof(kSizes)/sizeof(kSizes[0])); ++i) {
    int width  = kSizes[i][0];
    int height = kSizes[i][1];

    // Time the Mandelbrot (pick one: double or fixed-point)
    start_time = HAL_GetTick();
    // checksum = calculate_mandelbrot_double(width, height, MAX_ITER);
    checksum = calculate_mandelbrot_fixed_point(width, height, MAX_ITER);
    end_time = HAL_GetTick();
    execution_time = end_time - start_time;

    printf("SIZE=%dx%d  ITER=%d  TIME=%lu ms  CHECKSUM=%llu\r\n",
           width, height, MAX_ITER,
           (unsigned long)execution_time,
           (unsigned long long)checksum);
  }

  // Indicate end
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  while (1) {
    // Idle
  }
}

/* ===== Mandelbrot (double) ===== */
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
  uint64_t sum = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double x0 = (x / (double)width)  * 3.5 - 2.5;
      double y0 = (y / (double)height) * 2.0 - 1.0;
      double xi = 0.0;
      double yi = 0.0;
      int iteration = 0;

      while (iteration < max_iterations && (xi*xi + yi*yi) <= 4.0) {
        double tmp = xi*xi - yi*yi + x0;
        yi = 2.0*xi*yi + y0;
        xi = tmp;
        iteration++;
      }
      sum += (uint64_t)iteration;
    }
  }
  return sum;
}

/* ===== Mandelbrot (fixed-point Q16.16) ===== */
#define FIXED_SHIFT 16
#define FIXED_SCALE (1 << FIXED_SHIFT)
#define TO_FIXED(x) ((int32_t)((x) * FIXED_SCALE))

uint64_t calculate_mandelbrot_fixed_point(int width, int height, int max_iterations) {
  uint64_t sum = 0;

  const int32_t scale_3_5 = TO_FIXED(3.5);
  const int32_t scale_2_5 = TO_FIXED(2.5);
  const int32_t scale_2_0 = TO_FIXED(2.0);
  const int32_t scale_1_0 = TO_FIXED(1.0);
  const int32_t scale_4_0 = TO_FIXED(4.0);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int32_t x0 = ((x * scale_3_5) / width)  - scale_2_5;
      int32_t y0 = ((y * scale_2_0) / height) - scale_1_0;
      int32_t xi = 0;
      int32_t yi = 0;
      int iteration = 0;

      while (iteration < max_iterations) {
        int64_t xi_sq = ((int64_t)xi * xi) >> FIXED_SHIFT;
        int64_t yi_sq = ((int64_t)yi * yi) >> FIXED_SHIFT;

        if ((xi_sq + yi_sq) > scale_4_0) break;

        int32_t tmp = (int32_t)(xi_sq - yi_sq) + x0;
        yi = (int32_t)((((int64_t)2 * xi * yi) >> FIXED_SHIFT) + y0);
        xi = tmp;
        iteration++;
      }
      sum += (uint64_t)iteration;
    }
  }
  return sum;
}

/**
  * @brief System Clock Configuration
  * @note  Generate this with CubeMX for 120 MHz HCLK.
  */
void SystemClock_Config(void)
{
  // Use CubeMX to generate 120 MHz config.
  // Keep SysTick at 1kHz so HAL_GetTick() is 1 ms resolution.
}

/**
  * @brief GPIO Initialization Function
  *        PB0, PB1 as LEDs (adjust if your board uses different pins)
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief USART2 Initialization Function (115200 8N1)
  *        Adjust pins/instance if your board routes a different UART to ST-Link VCP.
  */
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
