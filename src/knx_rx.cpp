#include "config.h"
#if KNX_RX_MODE


#include "knx_rx.h"
#include <Arduino.h>


#define BIT0_MIN_US 25
#define BIT0_MAX_US 55
#define KNX_MAX_FRAME_LEN 23


HardwareTimer timer(TIM2);

static uint8_t bit_idx = 0, byte_idx = 0, cur_byte = 0;
static volatile bool bit0 = false;
static uint8_t pulse_start = 0;
static knx_frame_callback_t callback_fn = nullptr;
static volatile uint8_t parity_bit = 0;
static volatile uint8_t received_frame[KNX_MAX_FRAME_LEN];


static volatile bool RX_flag=false;
static uint8_t total = 0;


static uint8_t knx_checksum(uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= data[i];  // XOR từng byte
    }
    return sum;
}
void knx_rx_init(knx_frame_callback_t cb) {
  timer.setPrescaleFactor((SystemCoreClock/1000000) -1);   // CK_CNT = 8MHz / (8+1) = 1MHz
  timer.setOverflow(104);       
  timer.attachInterrupt(knx_timer_tick);
 // timer2.setPrescaleFactor(63);   
  //timer2.attachInterrupt(knx_send_tick);
 // timer2.setOverflow(1); 
  attachInterrupt(digitalPinToInterrupt(KNX_TX_PIN), knx_exti_irq, CHANGE);
  pinMode(KNX_TX_PIN, INPUT);
  callback_fn = cb;
  bit_idx = byte_idx = cur_byte = 0;
  bit0 = false;
  pulse_start = 0;
}


void knx_exti_irq(void) {
  if(!RX_flag){
    //Reset timer về 0
      Serial3.write(0x00);
      RX_flag = true;
      timer.refresh();
      timer.resume(); // Bật lại timer để bắt đầu nhận dữ liệu
  }
    //  Serial3.write(0x23);
  static uint8_t last = 0;
  // digitalRead
  //uint8_t lvl = digitalRead(KNX_TX_PIN); // dùng chân D2 làm KNX_RX

  // thanh ghi mức cao/thấp
// Đọc mức logic tại PA9
  uint8_t lvl = (GPIOA->IDR & (1 << 9)) ? 1 : 0;
  //bước 1: Nếu là sườn lên -> lưu lại time điểm này bằng bộ đếm timer
  //Bước 2: sườn xuống -> tính khoảng thời gian từ lúc sườn lên đến sườn xuống và kiểm tra khoảng time thỏa mãn ko? Nếu có thì bit 0/1
  //bước 3: 
  uint8_t now = timer.getCount();
  if (lvl && !last) pulse_start = now;
  else if (!lvl && last) {
    uint8_t w = now >= pulse_start ? now - pulse_start :104-pulse_start + now;
    if (w >= BIT0_MIN_US && w <= BIT0_MAX_US){
         bit0 = true;
        //  DEBUG_SERIAL.printf("%lu",w);
    }   
  }
  last = lvl;
}


void reset_knx_receiver() {
  memset((void*)received_frame, 0, sizeof(received_frame));
  bit_idx = 0;
  //byte_idx = 0;
  total = 0;
  cur_byte = 0;
  bit0 = false;
  RX_flag= false;
}
#if KNX_SEND_UART_MODE
void knx_timer_tick(void) {
  uint8_t bit = bit0 ? 0 : 1;
  bit0 = false;
  bit_idx++;
  if (bit_idx == 1) {
    cur_byte = 0;
    parity_bit = 0;
  } 
  else if (bit_idx >= 2 && bit_idx <= 9) {
    cur_byte >>= 1;
    if (bit) {
      cur_byte |= 0x80;
      parity_bit++;
    }
  } 
    else if (bit_idx == 10) {
    if ((parity_bit & 1) == bit) {
      return;
    }
  } 
  if (bit_idx == 11 && bit == 1) {
    received_frame[byte_idx] = cur_byte;
    //if (callback_fn) callback_fn(cur_byte);
    cur_byte = 0;
    bit_idx = 0;
    byte_idx++;
    timer.pause();
    RX_flag = false;
  }
  if (byte_idx >= 6 && total == 0) {
    total = 6 + (received_frame[5] & 0x0F) + 1 + 1;
    if (total> KNX_MAX_FRAME_LEN) {
      reset_knx_receiver();
      return;
    }
  }
      if (total > 0 && byte_idx == total) {
      detachInterrupt(digitalPinToInterrupt(KNX_TX_PIN));
      if (callback_fn) callback_fn((const uint8_t*) received_frame, total);
      reset_knx_receiver();
      return;
    }
}
#else
void knx_timer_tick(void) {
  uint8_t bit = bit0 ? 0 : 1;
  bit0 = false;
  bit_idx++;
  if (bit_idx == 1) {
    cur_byte = 0;
    parity_bit = 0;
  } 
  else if (bit_idx >= 2 && bit_idx <= 9) {
    cur_byte >>= 1;
    if (bit) {
      cur_byte |= 0x80;
      parity_bit++;
    }
  } 
    else if (bit_idx == 10) {
    if ((parity_bit & 1) == bit) {
      return;
    }
  } 
  if (bit_idx == 11 && bit == 1) {
    if (callback_fn) callback_fn(cur_byte);
    cur_byte = 0;
    bit_idx = 0;
    byte_idx++;
    RX_flag = false;
    timer.pause();
  }
}
#endif
#endif
