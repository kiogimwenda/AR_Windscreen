// System clock for the pod board's 8 MHz crystal. See Config.h ("Clock") and docs/BUILD_GUIDE.md
// Part 4.8.2.
//
// Overrides the STM32duino generic F405 variant's weak SystemClock_Config, which runs from the
// internal RC oscillator (HSI). USB full speed needs 48 MHz within +-0.25 %, the HSI is only good
// to about 1 %, and the F405 has no crystal-less USB clock recovery: on the HSI the host link may
// enumerate on one day and fail on a hot one.
//
//   HSE 8 MHz / M 8 = 1 MHz;  x N 336 = 336 MHz VCO;  / P 2 = 168 MHz core;  / Q 7 = 48 MHz USB
//   APB1 = 168 / 4 = 42 MHz (bxCAN, Config.h's bit timing);  APB2 = 84 MHz
//
// If the crystal does not start (a bad solder joint), the clock falls back to the HSI at the same
// frequencies instead of halting: the safety tasks keep running and keep everything released, and
// the unreliable USB link simply never arms the hub (Part 3.4). Bring-up check: Part 4.7.
#include <Arduino.h>

#include "hub/Config.h"

static_assert(HSE_VALUE == hub_config::kHseHz, "build with -DHSE_VALUE matching the pod crystal");

extern "C" void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {};
    RCC_ClkInitTypeDef clk = {};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = hub_config::kHseHz / 1000000;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        osc = {};
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 16;  // HSI is 16 MHz
        osc.PLL.PLLN = 336;
        osc.PLL.PLLP = RCC_PLLP_DIV2;
        osc.PLL.PLLQ = 7;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    }
    clk.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}
