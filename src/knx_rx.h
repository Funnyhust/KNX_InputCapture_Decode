#include "config.h"
#if KNX_RX_MODE
#ifndef KNX_RX_H
#define KNX_RX_H

#include <stdint.h>
#include <stdbool.h>
#include <HardwareTimer.h>
#include <HardwareSerial.h>

#define KNX_TX_PIN PA9

#if KNX_SEND_UART_MODE
typedef void (*knx_frame_callback_t)(const uint8_t *data, uint8_t length);
#else
typedef void (*knx_frame_callback_t)(const uint8_t byte);
#endif

     
// Khởi tạo: truyền vào callback xử lý telegram
void knx_rx_init(knx_frame_callback_t cb);

void knx_exti_irq(void);
// Hàm gọi trong Timer IRQ 104µs (bit sampling)
void knx_timer_tick(void);

bool get_knx_rx_flag();


#endif // STKNX_DRIVER_H
#endif