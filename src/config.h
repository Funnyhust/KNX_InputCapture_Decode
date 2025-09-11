#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Khai báo serial debug dùng chung
extern HardwareSerial DEBUG_SERIAL;
#define KNX_TX_MODE 1 // 1: PWM, 0: OC
#define KNX_RX_MODE 1 // 1: EXTI + Timer, 0: Input Capture + Timer
#define KNX_SEND_UART_MODE 1 // Gửi cả Frame, 0: Gửi từng byte

#define KNX_SEND_MODE 0   //1 Gửi cả, 0 gửi byte

// bool knx_sending=false;
// bool ack_received = false;
// void knx_ack_callback() {
//   ack_received = true;  // Gọi khi nhận được ACK từ bus
// }
#endif
