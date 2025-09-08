#ifndef STKNX_H
#define STKNX_H

#include <stdint.h>
#include <stdbool.h>
#include <HardwareSerial.h>

#define KNX_TX_PIN PA9

typedef void (*knx_frame_callback_t)(const uint8_t byte);

     
// Khởi tạo: truyền vào callback xử lý telegram
void knx_init(knx_frame_callback_t cb);

void knx_exti_irq(void);
// Hàm gọi trong Timer IRQ 104µs (bit sampling)
void knx_timer_tick(void);


#endif // STKNX_DRIVER_H
