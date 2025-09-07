#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

typedef void (*knx_byte_callback_t)(uint8_t);

class KNX_IC {
public:
    KNX_IC(TIM_TypeDef *tim_ic, uint8_t ch_ic, uint8_t pin_ic,
           TIM_TypeDef *tim_tick);

    void begin(knx_byte_callback_t cb);

private:
    HardwareTimer timer_ic;
    HardwareTimer timer_tick;
    uint8_t channel_ic;
    uint8_t pin;

    bool sawPulse;
    uint8_t bitCount;
    uint8_t currentByte;
    knx_byte_callback_t callback;

    void onCapture(uint32_t width);
    void onTick();
    void reset();
};
