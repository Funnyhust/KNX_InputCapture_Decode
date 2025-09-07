#include "knx_tx.h"
#include "config.h" // For DEBUG_SERIAL

#if KNX_TX_MODE 

extern "C" {
    #include "stm32f1xx_hal.h"
    #include "stm32f1xx_hal_rcc.h" // Để kiểm tra clock
}

// Forward declaration for custom error handler
void my_Error_Handler(void);

// Define handles here (single definition)
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch3;

// Forward declarations
extern "C" void DMA1_Channel6_IRQHandler(void);

extern "C" void MX_TIM1_Init(void) {
    // Enable clocks
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // === GPIO PA10 as AF Push-Pull (TIM1_CH3) ===
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // === TIM1 basic init ===
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 7499;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        my_Error_Handler();
    }

    // === TIM1 OC config for CH3 (PWM2) ===
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        my_Error_Handler();
    }

    // === DMA init for TIM1_CH3 ===
    hdma_tim1_ch3.Instance = DMA1_Channel6;
    hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK) {
        my_Error_Handler();
    }

    // Link DMA handle to TIM handle (CC3)
    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC3], hdma_tim1_ch3);

    // NVIC for DMA channel6
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    // Debug clock
   // DEBUG_SERIAL.printf("System Clock: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
}

// Wrapper public
void knx_tx_init(void) {
    MX_TIM1_Init();
}

// DMA IRQ handler (must be C linkage)
extern "C" void DMA1_Channel6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch3);
}

// Custom error handler to avoid conflict with macro
void my_Error_Handler(void) {
    while (1) {
        // Placeholder for error indication
    }
}
#else

#endif