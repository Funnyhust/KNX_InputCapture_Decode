

#include "knx_tx.h"
#include "config.h"
#include <string.h> // For memset

extern "C" {
  #include "stm32f1xx_hal.h"
}

#if KNX_TX_MODE
// handles được init trong knx_hal_conf.cpp
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch3;

// buffer DMA (halfword)
static uint16_t dma_buf[269];
static int dma_len = 0;

// ===== Thông số timing (72 MHz) =====
#define BIT_PERIOD   104   // ~104µs
#define T0_HIGH      35  // ~69µs (xung cao cho bit 0, theo ý bạn)
#define T0_TOL       700    // ~10µs dung sai (không dùng)

// ===== Encode bit =====
static void encode_bit(uint8_t bit) {
    if (dma_len >= (int)(sizeof(dma_buf) / sizeof(dma_buf[0]))) return;
    if (bit) {
        dma_buf[dma_len++] = 0;  // bit 1 => luôn Low (~104µs, đảo ngược)
    } else {
        dma_buf[dma_len++] = T0_HIGH;  // bit 0 => xung High ~69µs
    }
}

// ===== Encode byte (Start + 8 data + parity + Stop) =====
static void encode_byte(uint8_t b) {
    uint8_t parity_count = 0;

    encode_bit(0); // start = 0

    for (int i = 0; i < 8; i++) {
        uint8_t bit = (b >> i) & 0x01;
        encode_bit(bit);
        if (bit) parity_count++;
    } 

    // parity even
    encode_bit(parity_count & 1);
  //  encode_bit(1);    // stop = 1
    encode_bit(1);
    encode_bit(1);
    encode_bit(1);
}
// ===== Prepare frame =====
static void prepare_frame(uint8_t *data, int len) {
    memset(dma_buf, 0, sizeof(dma_buf));
    dma_len = 0;
    for (int i = 0; i < len; i++) {
        encode_byte(data[i]);
    }
 //    DEBUG_SERIAL.printf("Prepared frame, dma_len: %d\r\n", dma_len);
}

// ===== Public send function =====
void knx_send_frame(uint8_t *data, int len) {
    prepare_frame(data, len);
    // for (int i = 0; i < dma_len; i++) {
    //    DEBUG_SERIAL.printf("dma_buf[%d] = %d\r\n", i, dma_buf[i]);
    // }

    delay(3); // Đợi một chút để chắc chắn Timer đã dừng
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dma_buf, dma_len);
}

// ===== Callback khi DMA hoàn tất =====
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
        //DEBUG_SERIAL.printf("PWM Finished, DMA State: %d\r\n", hdma_tim1_ch3.State);
    }
}

#else

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch3;
extern UART_HandleTypeDef huart1;
// ===== KNX timing (72 MHz) =====
#define BIT_PERIOD   7499  // ~104µs per bit
#define T0_PULSE     2520  // ~35µs pulse for bit 0

// ===== DMA buffer =====
static uint16_t dma_buf[512];
static int dma_len = 0;

// ===== Encode 1 bit =====
static void encode_bit(uint8_t bit, uint32_t &t) {
    if (dma_len >= (int)(sizeof(dma_buf)/sizeof(dma_buf[0]))) return;

    if (bit) {
        t += BIT_PERIOD;
        dma_buf[dma_len++] = t;
    } else {
        t += T0_PULSE;
        dma_buf[dma_len++] = t;
        t += (BIT_PERIOD - T0_PULSE);
        dma_buf[dma_len++] = t;
    }
}

// ===== Encode 1 byte (LSB first + parity even) =====
static void encode_byte(uint8_t b, uint32_t &t) {
    uint8_t parity = 0;

    encode_bit(0, t); // start bit

    for (int i = 0; i < 8; i++) {
        uint8_t bit = (b >> i) & 0x01;
        encode_bit(bit, t);
        if (bit) parity++;
    }

    encode_bit(parity & 1, t); // parity even
    encode_bit(1, t);          // stop bits
    encode_bit(1, t);
    encode_bit(1, t);
}

// ===== Prepare frame =====
static void prepare_frame(uint8_t *data, int len) {
    dma_len = 0;
    uint32_t t = 0;
    for (int i = 0; i < len; i++) {
        encode_byte(data[i], t);
    }
    DEBUG_SERIAL.printf("DMA_Len: %d ", dma_len);
}

// ===== KNX DMA complete handler =====
static void knx_dma_complete(void) {
    HAL_TIM_OC_Stop_DMA(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop(&htim1);
    __HAL_DMA_DISABLE(&hdma_tim1_ch3);
    __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch3, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3 | DMA_FLAG_GL3);

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    htim1.State = HAL_TIM_STATE_READY;
    hdma_tim1_ch3.State = HAL_DMA_STATE_READY;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    DEBUG_SERIAL.printf("DMA_Complete ");
}

// ===== Public KNX send frame =====
void knx_send_frame(uint8_t *data, int len) {
    prepare_frame(data, len);

    if (dma_len == 0) {
        DEBUG_SERIAL.printf("Empty_Buffer ");
        return;
    }

    // Kiểm tra trạng thái
    if (htim1.State == HAL_TIM_STATE_BUSY) {
        DEBUG_SERIAL.printf("TIM_Busy ");
        return;
    }
    if (hdma_tim1_ch3.State == HAL_DMA_STATE_BUSY) {
        DEBUG_SERIAL.printf("DMA_Busy ");
        return;
    }
    if (TIM1->CR1 & TIM_CR1_CEN) {
        DEBUG_SERIAL.printf("TIM1_Running ");
        HAL_TIM_Base_Stop(&htim1); // Tắt TIM1 nếu đang chạy
        htim1.State = HAL_TIM_STATE_READY;
    }
    if (DMA1_Channel3->CCR & DMA_CCR_EN) {
        DEBUG_SERIAL.printf("DMA1_Channel3_Enabled ");
        __HAL_DMA_DISABLE(&hdma_tim1_ch3); // Tắt DMA nếu đang bật
        __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch3, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3 | DMA_FLAG_GL3);
        hdma_tim1_ch3.State = HAL_DMA_STATE_READY;
    }

    // In trạng thái DMA1->ISR
    uint32_t isr = DMA1->ISR;
    if (isr & DMA_ISR_GIF3) DEBUG_SERIAL.printf("GIF3_Set ");
    if (isr & DMA_ISR_TCIF3) DEBUG_SERIAL.printf("TCIF3_Set ");
    if (isr & DMA_ISR_HTIF3) DEBUG_SERIAL.printf("HTIF3_Set ");
    if (isr & DMA_ISR_TEIF3) DEBUG_SERIAL.printf("TEIF3_Set ");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // Dừng và đặt lại trạng thái
    HAL_TIM_OC_Stop_DMA(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Base_Stop(&htim1);
    __HAL_DMA_DISABLE(&hdma_tim1_ch3);
    __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch3, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3 | DMA_FLAG_GL3);

    HAL_StatusTypeDef status = HAL_TIM_OC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dma_buf, dma_len);
    if (status != HAL_OK) {
        DEBUG_SERIAL.printf("DMA_Start_Error: %d ", status);
    } else {
        DEBUG_SERIAL.printf("DMA_Started ");
    }
}

// ===== KNX TX init =====
void knx_tx_init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_TIM1_ENABLE();

    if (!(__HAL_RCC_TIM1_IS_CLK_ENABLED())) {
        DEBUG_SERIAL.printf("TIM1_Clock_Disabled ");
    }
    if (!(__HAL_RCC_DMA1_IS_CLK_ENABLED())) {
        DEBUG_SERIAL.printf("DMA1_Clock_Disabled ");
    }

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
        DEBUG_SERIAL.printf("TIM_Init_Error ");
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        DEBUG_SERIAL.printf("TIM_OC_Config_Error ");
    }
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);

    hdma_tim1_ch3.Instance = DMA1_Channel3;
    hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_HIGH;
    __HAL_DMA_ENABLE_IT(&hdma_tim1_ch3, DMA_IT_TC | DMA_IT_TE); // Bật cả interrupt lỗi
    if (HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK) {
        DEBUG_SERIAL.printf("DMA_Init_Error ");
    }

    __HAL_LINKDMA(&htim1, hdma[TIM_DMA_ID_CC3], hdma_tim1_ch3);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    // Xóa cờ DMA ban đầu
    __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch3, DMA_FLAG_TC3 | DMA_FLAG_HT3 | DMA_FLAG_TE3 | DMA_FLAG_GL3);
}

// ===== DMA IRQ handler =====
extern "C" void DMA1_Channel3_IRQHandler(void) {
    uint32_t isr = DMA1->ISR;
    if (isr & DMA_ISR_GIF3) DEBUG_SERIAL.printf("GIF3 ");
    if (isr & DMA_ISR_TCIF3) DEBUG_SERIAL.printf("TCIF3 ");
    if (isr & DMA_ISR_HTIF3) DEBUG_SERIAL.printf("HTIF3 ");
    if (isr & DMA_ISR_TEIF3) DEBUG_SERIAL.printf("TEIF3 ");

    HAL_DMA_IRQHandler(&hdma_tim1_ch3);
    DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3;

    DEBUG_SERIAL.printf("DMA_IRQHandler_Called ");
}

// ===== HAL callback khi DMA hoàn tất =====
void HAL_DMA_IRQHandlerCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma == &hdma_tim1_ch3) {
        DEBUG_SERIAL.printf("DMA_Callback ");
        knx_dma_complete();
    }
}

// ===== Debug DMA status =====
void debug_dma_status(void) {
    uint32_t isr = DMA1->ISR;
    if (isr & DMA_ISR_TCIF3) {
        DEBUG_SERIAL.printf("TCIF3_Set ");
    }
    if (isr & DMA_ISR_HTIF3) {
        DEBUG_SERIAL.printf("HTIF3_Set ");
    }
    if (isr & DMA_ISR_TEIF3) {
        DEBUG_SERIAL.printf("TEIF3_Set ");
    }
    if (TIM1->DIER & TIM_DIER_CC3DE) {
        DEBUG_SERIAL.printf("CC3DE_Enabled ");
    }
    if (TIM1->CR1 & TIM_CR1_CEN) {
        DEBUG_SERIAL.printf("Timer_Running ");
    }
}
#endif