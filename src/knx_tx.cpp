

#include "knx_tx.h"
#include "config.h"
#include <string.h> // For memset

extern "C" {
  #include "stm32f1xx_hal.h"
}
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
    // Ban đầu có delay(3); dễ lỗi
    //delay(3); // Đợi một chút để chắc chắn Timer đã dừng
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)dma_buf, dma_len);
}

// ===== Callback khi DMA hoàn tất =====
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3);
        //DEBUG_SERIAL.printf("PWM Finished, DMA State: %d\r\n", hdma_tim1_ch3.State);
    }
}


