#include "stm32f4xx_hal.h"

#define GPIO_PORT       GPIOA
#define LED_PIN         GPIO_PIN_1
#define LDR_PIN         GPIO_PIN_0
#define LED_ACTIVE_HIGH 1

#define TH_DARK  2100 /* filtered value below this  -> dark, LED ON  */
#define TH_LIGHT 2700 /* filtered value above this  -> light, LED OFF */

#define AVG_WINDOW_SIZE 4

/* ---------- globals (shared with ISR context) ---------- */

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

volatile bool     g_dark = false;
volatile uint32_t readings[AVG_WINDOW_SIZE];
volatile uint8_t  idx = 0;
volatile uint32_t sum = 0;

/* ---------- helpers ---------- */

void filter_init(uint32_t initData) {
    for (uint8_t i = 0; i < AVG_WINDOW_SIZE; i++) {
        readings[i] = initData;
        sum += initData;
    }
}

uint32_t moving_avg(uint32_t rawData) {
    sum -= readings[idx];
    sum += rawData;
    readings[idx] = rawData;
    idx++;

    if (idx >= AVG_WINDOW_SIZE) {
        idx = 0;
    }

    return sum / AVG_WINDOW_SIZE;
}

static void led_set(int on) {
    GPIO_PinState s = (on == LED_ACTIVE_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIO_PORT, LED_PIN, s);
}

static uint32_t read_adc_value() {
    HAL_ADC_Start(&hadc1);

    HAL_ADC_PollForConversion(&hadc1, 10);

    uint32_t value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}

void Error_Handler(void);

static void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    g.Pin   = LED_PIN; /* LED output */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_PORT, &g);
    led_set(0);

    g.Pin       = GPIO_PIN_2;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIO_PORT, &g);

    g.Pin   = LDR_PIN; /* LDR input */
    g.Mode  = GPIO_MODE_ANALOG;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_PORT, &g);
}

static void ADC_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel                = ADC_CHANNEL_0;
    ch.Rank                   = 1;
    ch.SamplingTime           = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

static void TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 16000 - 1; /* 16 MHz / 16000 = 1 kHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 20 - 1; /* 1 kHz / 20 = 50 Hz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
}

int main(void) {
    HAL_Init();
    GPIO_Init();
    TIM2_Init();
    ADC_Init();

    HAL_ADC_Start_IT(&hadc1);

    filter_init(read_adc_value());

    while (1) {}
}

#ifdef __cplusplus
extern "C" {
#endif
/* ---------------- Interrupt handlers ---------------- */

void SysTick_Handler(void) {
    HAL_IncTick();
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

/* ---------------- Callbacks ---------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (TIM2 == htim->Instance) {
        uint32_t raw_value      = read_adc_value();
        uint32_t filtered_value = moving_avg(raw_value);

        if (!g_dark && filtered_value < TH_DARK) {
            g_dark = true;
            led_set(1);
        } else if (g_dark && filtered_value > TH_LIGHT) {
            g_dark = false;
            led_set(0);
        }
    }
}

#ifdef __cplusplus
}
#endif

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}