#include <Arduino.h>
#include "ic_knx.h"
#include "config.h"

// UART debug
HardwareSerial DEBUG_SERIAL(PA3, PA2);

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
}

void loop() {
    // tất cả xử lý trong ISR
    // DEBUG_SERIAL.println("Looping");
    // delay(1000);
}
