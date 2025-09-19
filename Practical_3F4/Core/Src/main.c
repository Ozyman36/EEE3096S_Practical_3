/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mandelbrot benchmark (no LCD) — STM32F4
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>   // not required for prints here, but harmless
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER   100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Choose your image size (same as Practical 1B baseline)
#define IMG_W  128
#define IMG_H  128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t start_time = 0;
uint32_t end_time   = 0;
uint32_t execution_time = 0;     // ms
uint64_t checksum = 0;           // sum of escape iterations

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
uint64_t mandelbrot_double(int width, int height, int max_iterations);
uint64_t mandelbrot_fixed_q16(int width, int height, int max_iterations);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  // LED on PB0 = START, PB1 = END (change pins if your board differs)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  start_time = HAL_GetTick();

  // ---- Choose ONE version to run for Task 1 ----
  //checksum = mandelbrot_double(IMG_W, IMG_H, MAX_ITER);
  checksum = mandelbrot_fixed_q16(IMG_W, IMG_H, MAX_ITER);

  end_time = HAL_GetTick();
  execution_time = end_time - start_time;  // milliseconds

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // finished

  // From here, just read: execution_time, checksum in the debugger.
  // Keep the MCU alive so you can inspect variables.
  while (1) {
    // Optional: blink PB0 every ~500ms to show it's alive
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    HAL_Delay(500);
  }
  /* USER CODE END 2 */
}

/* ================= Mandelbrot (double) ================= */
uint64_t mandelbrot_double(int width, int height, int max_iterations) {
  uint64_t sum = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double x0 = (x / (double)width)  * 3.5 - 2.5;
      double y0 = (y / (double)height) * 2.0 - 1.0;
      double xi = 0.0;
      double yi = 0.0;
      int iter = 0;

      while (iter < max_iterations && (xi*xi + yi*yi) <= 4.0) {
        double tmp = xi*xi - yi*yi + x0;
        yi = 2.0*xi*yi + y0;
        xi = tmp;
        iter++;
      }
      sum += (uint64_t)iter;
    }
  }
  return sum;
}

/* ================= Mandelbrot (fixed-point Q16.16) ================= */
#define FIX_SHIFT 16
#define FIX_SCALE (1 << FIX_SHIFT)
#define TO_FIX(x) ((int32_t)((x) * (double)FIX_SCALE))

uint64_t mandelbrot_fixed_q16(int width, int height, int max_iterations) {
  uint64_t sum = 0;

  const int32_t k3_5 = TO_FIX(3.5);
  const int32_t k2_5 = TO_FIX(2.5);
  const int32_t k2_0 = TO_FIX(2.0);
  const int32_t k1_0 = TO_FIX(1.0);
  const int32_t k4_0 = TO_FIX(4.0);

  for (int y = 0; y < height; y++) {
    // Scale y to [-1.0, +1.0]
    int32_t y0 = ((int64_t)y * k2_0) / height - k1_0;

    for (int x = 0; x < width; x++) {
      // Scale x to [-2.5, +1.0]
      int32_t x0 = ((int64_t)x * k3_5) / width - k2_5;

      int32_t xi = 0;
      int32_t yi = 0;
      int iter = 0;

      while (iter < max_iterations) {
        int64_t xi_sq = ((int64_t)xi * xi) >> FIX_SHIFT;
        int64_t yi_sq = ((int64_t)yi * yi) >> FIX_SHIFT;

        if ((xi_sq + yi_sq) > k4_0) break;

        int32_t tmp = (int32_t)(xi_sq - yi_sq) + x0;
        yi = (int32_t)((((int64_t)2 * xi * yi) >> FIX_SHIFT) + y0);
        xi = tmp;

        iter++;
      }
      sum += (uint64_t)iter;
    }
  }
  return sum;
}

/**
  * @brief System Clock Configuration for 48 MHz SYSCLK
  *        - HSI (16 MHz) → PLL → SYSCLK = 48 MHz
  *        - AHB = 48 MHz, APB1 = 48 MHz, APB2 = 48 MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Configure PLL: HSI (16 MHz) * 6 / 2 = 48 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;      // VCO input = 16 MHz / 16 = 1 MHz
  RCC_OscInitStruct.PLL.PLLN = 192;     // VCO output = 1 MHz * 192 = 192 MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // SYSCLK = 192 / 4 = 48 MHz
  RCC_OscInitStruct.PLL.PLLQ = 4;       // 48 MHz USB/SDIO/Random number
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Select PLL as SYSCLK source and configure buses */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}


/**
  * @brief GPIO Initialization Function
  * PB0, PB1 as outputs for LEDs (adjust if your board differs)
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
