#include <Arduino.h>
#include "ic_knx.h"
#include "config.h"
#include "knx_tx.h"

// TIM_HandleTypeDef htim1;
// DMA_HandleTypeDef hdma_tim1_ch3;
// UART debug
HardwareSerial DEBUG_SERIAL(PA3, PA2);


     uint8_t on_frame[9]  = {0xBC, 0x12, 0x07, 0x00, 0x05,0xE1,0x00, 0x81, 0x33};
     uint8_t off_frame[9] = {0xBC, 0x12, 0x07, 0x00, 0x05,0xE1,0x00, 0x80, 0x32};
     uint8_t test_frame1[1] = {0x08};
     uint8_t test_frame2[5] = {0xAA,0xAA, 0xAA,0xAA,0xAA};
    //  uint8_t test_frame3[1] = {0x55};
    //  uint8_t test_frame4[1] = {0x99};
// PA9 dùng cho Input Capture (TIM1_CH2)
  KNX_IC knx(TIM1, 2, PA9, TIM2);

void knxByteHandler(uint8_t data) {
    DEBUG_SERIAL.write(data);
    // DEBUG_SERIAL.printf("Byte: 0x%02X\n", data);
}

void setup() {
    delay(2000);
   DEBUG_SERIAL.begin(19200, SERIAL_8E1); // parity even
   knx.begin(knxByteHandler);
   knx_tx_init();
}

void loop() {
    knx_send_frame(test_frame1, 1);
    delay(1000);
    knx_send_frame(test_frame2, 5);
    delay(1000);
    // knx_send_frame(test_frame3, 1);
    // delay(1000);
    // knx_send_frame(test_frame4, 1);
    // delay(1000);

   // DEBUG_SERIAL.println(SystemCoreClock);
   // DEBUG_SERIAL.println("TEST");
    // tất cả xử lý trong ISR
    // DEBUG_SERIAL.println("Looping");
    // delay(1000);
}

