#include "config.h"
#if KNX_RX_MODE ==0

#include "ic_knx.h"

volatile bool RX_flag = false;
volatile uint8_t parity_count = 0;

KNX_IC::KNX_IC(TIM_TypeDef *tim_ic, uint8_t ch_ic, uint8_t pin_ic,
               TIM_TypeDef *tim_tick)
    : timer_ic(tim_ic), timer_tick(tim_tick) {
    channel_ic = ch_ic;
    pin = pin_ic;
    sawPulse = false;
    bitCount = 0;
    currentByte = 0;
    callback = nullptr;
}

void KNX_IC::begin(knx_byte_callback_t cb) {
  //  DEBUG_SERIAL.println("KNX_IC::begin");
    callback = cb;

    // cấu hình Input Capture ban đầu: Rising
    timer_ic.setMode(channel_ic, TIMER_INPUT_CAPTURE_RISING, pin);
    timer_ic.setPrescaleFactor((SystemCoreClock / 1000000) - 1); // tick = 1 µs

    timer_ic.attachInterrupt(channel_ic, [this](void) {
        static uint32_t lastCaptureRise = 0;
        static bool waitingFall = false;

        uint32_t now = timer_ic.getCaptureCompare(channel_ic);

        if (!waitingFall) {
            // Rising edge
            lastCaptureRise = now;
            waitingFall = true;

            // chuyển sang Falling
            switch (channel_ic) {
                case 1: timer_ic.getHandle()->Instance->CCER |= TIM_CCER_CC1P; break;
                case 2: timer_ic.getHandle()->Instance->CCER |= TIM_CCER_CC2P; break;
                case 3: timer_ic.getHandle()->Instance->CCER |= TIM_CCER_CC3P; break;
                case 4: timer_ic.getHandle()->Instance->CCER |= TIM_CCER_CC4P; break;
            }

           // DEBUG_SERIAL.printf("IC RISE: %lu\n", now);
        } else {
            // Falling edge
            uint32_t overflow = timer_ic.getOverflow() + 1;
            uint32_t width = (now >= lastCaptureRise)
                                 ? (now - lastCaptureRise)
                                 : (overflow - lastCaptureRise + now);

           // DEBUG_SERIAL.printf("IC FALL: %lu, Width=%lu\n", now, width);

            this->onCapture(width);

            waitingFall = false;

            // chuyển lại sang Rising
            switch (channel_ic) {
                case 1: timer_ic.getHandle()->Instance->CCER &= ~TIM_CCER_CC1P; break;
                case 2: timer_ic.getHandle()->Instance->CCER &= ~TIM_CCER_CC2P; break;
                case 3: timer_ic.getHandle()->Instance->CCER &= ~TIM_CCER_CC3P; break;
                case 4: timer_ic.getHandle()->Instance->CCER &= ~TIM_CCER_CC4P; break;
            }
        }
    });

    timer_ic.resume();

    // cấu hình Timer tick 104 µs (chu kỳ bit KNX)
    timer_tick.setPrescaleFactor((SystemCoreClock / 1000000) - 1); // 1 µs
    timer_tick.setOverflow(104);
    timer_tick.attachInterrupt([this]() { this->onTick(); });
    timer_tick.resume();
}

void KNX_IC::onCapture(uint32_t width) {
    if (width >= 15 && width <= 65) {
        if (!RX_flag) {
            RX_flag = true;
            timer_tick.resume();
        }
        sawPulse = true;
    }
}

void KNX_IC::onTick() {
    uint8_t bit = sawPulse ? 0 : 1;
    sawPulse = false;

    if (bitCount == 0) {
        if (bit != 0) { // Start bit phải là 0
            reset();
            return;
        }
    } else if (bitCount >= 1 && bitCount <= 8) {
        currentByte >>= 1;
        if (bit) {
            currentByte |= 0x80;
        }
    } else if (bitCount == 9) {
        // TODO: parity check nếu cần
    } else if (bitCount == 10) {
        if (bit != 1) { // Stop bit phải là 1
            reset();
            return;
        }
        if (callback) {
            callback(currentByte);
        }
        reset();
        return;
    }

    bitCount++;
}

void KNX_IC::reset() {
    bitCount = 0;
    currentByte = 0;
    sawPulse = false;
    RX_flag = false;
    timer_tick.pause();
}
#endif