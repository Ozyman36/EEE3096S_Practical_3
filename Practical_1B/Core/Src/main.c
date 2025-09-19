/* Task 7: Fixed-Point Scaling Study — STM32F0 @ 48 MHz
 * - Decimal fixed-point with scale = 10^S, S in {3,4,6}
 * - Sizes: 128,160,192,224,256
 * - MAX_ITER = 100
 * - Measures: time(ms), "cycles" via TIM2 ticks (48 MHz), px/s, cycles/px, checksum
 * - Tracks: overflow_count, max|xi|, max|yi|
 */

#include "stm32f0xx_hal.h"
#include <stdint.h>

/* ---------------- Config ---------------- */
#define MAX_ITER   100
#define USE_LED    1

static const int SIZES[][2] = {
  {128,128}, {160,160}, {192,192}, {224,224}, {256,256}
};
#define NSIZES (sizeof(SIZES)/sizeof(SIZES[0]))

static const int SCALE_POWS[] = {3,4,6};
#define NPOW (sizeof(SCALE_POWS)/sizeof(SCALE_POWS[0]))

/* --------------- TIM2 @ 48 MHz --------------- */
static void TIM2_Init_48MHz_FreeRun(void)
{
  /* Enable clock */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  __DSB();

  /* Reset TIM2 */
  RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
  RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

  /* 48 MHz tick: PSC=0, 32-bit free-run */
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->EGR = TIM_EGR_UG;  // latch PSC/ARR
  TIM2->CNT = 0;
  TIM2->CR1 = TIM_CR1_CEN;
}

static inline uint32_t tim2_read(void)
{
  return TIM2->CNT;
}
/* --------------- Result structs --------------- */
typedef struct {
  int scale_pow;
  int w,h;
  uint64_t checksum;
  uint32_t time_ms;
  uint32_t cycles;           // TIM2 ticks @ 48 MHz
  double   px_per_sec;
  double   cycles_per_px;
  uint32_t overflow_count;
  int32_t  max_abs_xi;
  int32_t  max_abs_yi;
} bench_row_t;

typedef struct {
  int scale_pow;
  bench_row_t rows[NSIZES];
} bench_block_t;

volatile bench_block_t g_results[NPOW];
volatile uint32_t g_done_blocks = 0;

/* --------- GPIO & Clock (48 MHz from HSI PLL) ---------- */
static void MX_GPIO_Init(void){
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g={0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
  g.Pin=GPIO_PIN_0|GPIO_PIN_1; g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
}

static void SystemClock_Config(void){
  RCC_OscInitTypeDef o={0}; RCC_ClkInitTypeDef c={0};
  o.OscillatorType=RCC_OSCILLATORTYPE_HSI;
  o.HSIState=RCC_HSI_ON; o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
  o.PLL.PLLState=RCC_PLL_ON; o.PLL.PLLSource=RCC_PLLSOURCE_HSI;
  o.PLL.PREDIV = RCC_PREDIV_DIV1;      // 8 MHz
  o.PLL.PLLMUL = RCC_PLL_MUL12;        // 8 * 12 = 96 MHz /2? (F0 SYSCLK=48 with div)
  HAL_RCC_OscConfig(&o);

  c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;  // 48 MHz
  c.AHBCLKDivider=RCC_SYSCLK_DIV1;
  c.APB1CLKDivider=RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&c, FLASH_LATENCY_1);

  SystemCoreClockUpdate();
}

/* -------- Fixed-point Mandelbrot (decimal scale = 10^S) -------- */
__attribute__((noinline))
static uint64_t mandelbrot_fixed_decimal(
  int W, int H, int maxit, int scale_pow,
  uint32_t *overflow_count, int32_t *max_abs_xi, int32_t *max_abs_yi)
{
  int32_t S=1; for(int i=0;i<scale_pow;i++){ if (S>INT32_MAX/10) break; S*=10; }

  const int32_t k3_5 = (int32_t)(3.5 * S + 0.5);
  const int32_t k2_5 = (int32_t)(2.5 * S + 0.5);
  const int32_t k2_0 = (int32_t)(2.0 * S + 0.5);
  const int32_t k1_0 = (int32_t)(1.0 * S + 0.5);
  const int32_t k4_0 = (int32_t)(4.0 * S + 0.5);

  uint64_t sum = 0;
  uint32_t sat = 0;
  int32_t  max_x = 0, max_y = 0;
  const int32_t SAT = INT32_MAX/2;

  for (int y=0; y<H; ++y){
    int32_t y0 = (int32_t)(((int64_t)y * k2_0)/H) - k1_0;
    for (int x=0; x<W; ++x){
      int32_t x0 = (int32_t)(((int64_t)x * k3_5)/W) - k2_5;
      int32_t xi=0, yi=0; int it=0;

      while (it<maxit){
        int64_t xi_sq = ((int64_t)xi * (int64_t)xi) / S;
        int64_t yi_sq = ((int64_t)yi * (int64_t)yi) / S;
        if ((xi_sq + yi_sq) > k4_0) break;

        int32_t tmp = (int32_t)(xi_sq - yi_sq) + x0;
        int64_t prod = (int64_t)xi * (int64_t)yi;
        yi = (int32_t)(((int64_t)2 * prod)/S) + y0;
        xi = tmp;

        if (xi > SAT){ xi = SAT; sat++; }
        if (xi < -SAT){ xi = -SAT; sat++; }
        if (yi > SAT){ yi = SAT; sat++; }
        if (yi < -SAT){ yi = -SAT; sat++; }

        int32_t ax = xi>=0?xi:-xi; if (ax>max_x) max_x=ax;
        int32_t ay = yi>=0?yi:-yi; if (ay>max_y) max_y=ay;

        ++it;
      }
      sum += (uint64_t)it;
    }
  }

  *overflow_count = sat; *max_abs_xi=max_x; *max_abs_yi=max_y;
  return sum;
}

/* --------------- One run --------------- */
static void run_one(int W, int H, int pow10, bench_row_t* out)
{
  const uint32_t pixels = (uint32_t)W * (uint32_t)H;

#if USE_LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
#endif

  uint32_t t0 = tim2_read();
  uint32_t ms0 = HAL_GetTick();

  uint32_t of=0; int32_t mx=0,my=0;
  uint64_t sum = mandelbrot_fixed_decimal(W, H, MAX_ITER, pow10, &of, &mx, &my);

  uint32_t t1 = tim2_read();
  uint32_t ms1 = HAL_GetTick();

#if USE_LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
#endif

  uint32_t ticks = (uint32_t)(t1 - t0);           // wrap-safe
  uint32_t ms    = (uint32_t)(ms1 - ms0);
  uint32_t cycles = ticks;                        // TIM2 @ 48 MHz

  double t = (double)cycles / 48e6;
  double pxps = t>0 ? (pixels / t) : 0.0;
  double cpi  = pixels? ((double)cycles / (double)pixels) : 0.0;

  out->scale_pow = pow10; out->w=W; out->h=H;
  out->checksum=sum; out->time_ms=ms; out->cycles=cycles;
  out->px_per_sec=pxps; out->cycles_per_px=cpi;
  out->overflow_count=of; out->max_abs_xi=mx; out->max_abs_yi=my;

#if USE_LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
#endif
}

/* ---------------- Main ---------------- */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  TIM2_Init_48MHz_FreeRun();

  for (uint32_t b=0; b<NPOW; ++b){
    g_results[b].scale_pow = SCALE_POWS[b];
    for (uint32_t i=0; i<NSIZES; ++i){
      run_one(SIZES[i][0], SIZES[i][1], SCALE_POWS[b], (bench_row_t*)&g_results[b].rows[i]);
      HAL_Delay(50);
    }
    g_done_blocks = b+1;
  }

  while (1){
#if USE_LED
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
#endif
    HAL_Delay(400);
  }
}
