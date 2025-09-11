#include <Arduino.h>
//#include "ic_knx.h"
#include "config.h"
#include "knx_tx.h"
#include "knx_rx.h"
#include <IWatchdog.h>

#define KNX_RX_BUFFER_MAX_SIZE 23
static uint8_t uart_rx_buf[KNX_RX_BUFFER_MAX_SIZE];


static uint8_t knx_rx_buf[KNX_RX_BUFFER_MAX_SIZE];
static volatile bool Rx_flag=false;

static volatile uint8_t byte_idx=0;
HardwareSerial DEBUG_SERIAL(PA3, PA2);
// Received byte

static uint8_t Uart_length = 0;
static uint8_t Send_length = 0;

uint8_t send_count = 0;
bool send_done = false;

//RETRY
#define MAX_RETRY 3
#define ACK_TIMEOUT 15  // ms
static bool ack_received = false;
static bool knx_is_sending = false;

//Is
static volatile bool knx_is_receved = false;



// ================== QUEUE =====================
#define MAX_QUEUE 200  // tối đa 5 frame chờ gửi
struct Frame {
  uint8_t data[KNX_RX_BUFFER_MAX_SIZE];
  uint8_t len;
};

Frame queue[MAX_QUEUE];
volatile uint8_t q_head = 0, q_tail = 0;
volatile uint8_t q_count = 0;

// Hàm thêm frame vào queue
bool enqueue_frame(uint8_t *data, uint8_t len) {
  if (q_count >= MAX_QUEUE) return false; // đầy
  memcpy(queue[q_tail].data, data, len);
  queue[q_tail].len = len;
  q_tail = (q_tail + 1) % MAX_QUEUE;
  q_count++;
  return true;
}

// Hàm lấy frame ra khỏi queue
bool dequeue_frame(Frame *f) {
  if (q_count == 0) return false;
  memcpy(f->data, queue[q_head].data, queue[q_head].len);
  f->len = queue[q_head].len;
  q_head = (q_head + 1) % MAX_QUEUE;
  q_count--;
  return true;
}


uint8_t knx_calc_checksum(const uint8_t *data, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len - 1; i++) { // không tính byte cuối
    cs ^= data[i];
  }
  return ~cs;
}

#if KNX_SEND_MODE //Gửi cả Frame
static uint32_t last_byte_time = 0;
void handle_knx_frame(const uint8_t byte) {
  knx_is_receved = true;
  uint32_t now = millis();

  // Nếu quá 3ms không nhận byte mới => reset buffer
  if ((now - last_byte_time) > 3) {
    Send_length =0;
    memset((void*)knx_rx_buf, 0, sizeof(knx_rx_buf));
    byte_idx = 0;
  }
  last_byte_time = now;

  // Lưu byte
  knx_rx_buf[byte_idx++] = byte;

  // Tính độ dài frame khi đã có ít nhất 6 byte
  if (byte_idx >= 6) {
    Send_length = 6 + (knx_rx_buf[5] & 0x0F) + 1 + 1;
    if (byte_idx == Send_length) {
      if (knx_rx_buf[Send_length-1]==knx_calc_checksum(knx_rx_buf,Send_length)){
        if(knx_is_sending){
          ack_received=true;
          knx_is_sending = false;
        }
        // DEBUG_SERIAL.println("Checksum Valid");
      }
      // else  
      // { 
      // DEBUG_SERIAL.println("Checksum Invalid");
      // DEBUG_SERIAL.write(knx_rx_buf,Send_length);
      // }
      Rx_flag = true;
      knx_is_receved = false;

    }
  }
  }
#else // Gửi Byte
static uint8_t Rx_byte=0;
void handle_knx_frame(const uint8_t byte) {
 // DEBUG_SERIAL.write(byte);
 Rx_byte = byte;
 Rx_flag = true;
}
#endif

void setup() {
   DEBUG_SERIAL.begin(115200, SERIAL_8E1); // parity even
   IWatchdog.begin(500000); 
   knx_rx_init(handle_knx_frame);
   knx_tx_init();
}


bool read_uart_frame() {
  static uint8_t index = 0;
  static uint8_t total = 0;
  while (DEBUG_SERIAL.available()) {
    uint8_t byte = DEBUG_SERIAL.read();
    if (index < KNX_RX_BUFFER_MAX_SIZE) {
      uart_rx_buf[index++] = byte;
    } else {
      // Tràn buffer, reset lại
      index = 0;
      total = 0;
      return false;
    }
    if (index >= 6&& total==0) {
      total = 6 + (uart_rx_buf[5] & 0x0F) + 1 + 1;
      Uart_length = total;
      if (total > KNX_RX_BUFFER_MAX_SIZE) {
        index = 0;
        total = 0;
        return false;
      }
    }
    if (total > 0 && index >= total) {
      // Có thể kiểm tra checksum ở đây nếu muốn
      index = 0;
      total = 0;
      return true;
    }
  }
  return false;
}

// Hàm gửi KNX có retry
bool knx_send_with_retry(uint8_t *frame, uint8_t len) {
  for (uint8_t attempt = 0; attempt < MAX_RETRY; attempt++) {
    ack_received = false;
    knx_is_sending = true;
    knx_send_frame(frame, len); // Gửi bản tin
    //DEBUG_SERIAL.printf("Lan %d",attempt);
    // for(int i=0; i<len; i++){
    //   DEBUG_SERIAL.printf("test:%d", frame[i]);
    // }
    uint32_t start = millis();
    while (millis() - start < ACK_TIMEOUT) {
      if (ack_received) {
        return true; // Gửi thành công
      }
    }
      if(attempt == (MAX_RETRY-1)){
       knx_is_sending=false;
      }
    // Nếu tới đây là timeout
    // DEBUG_SERIAL.println("Retry knx_sending...");
  }
  return false; // Thất bại sau 3 lần
}



// Loop-----------------------------------------------------------------
void loop() {
   // knx_send_frame(on_frame, 9);   
// Nhận frame từ UART → bỏ vào queue
  if (read_uart_frame()) {
    enqueue_frame(uart_rx_buf, Uart_length);
    Uart_length = 0;
  }

  // Nếu KNX đang rảnh → lấy frame từ queue ra gửi
  if (!knx_is_sending && q_count > 0) {
    Frame f;
    if (dequeue_frame(&f)) {
      knx_send_frame(f.data, f.len);
    }
  }

#if KNX_SEND_MODE
  if(Rx_flag) {
    Rx_flag = false;
    //TEST
    //DEBUG_SERIAL.write(0xAA);
    //KNX_Received_Data
    DEBUG_SERIAL.write(knx_rx_buf, Send_length);
  }
#else
  if(Rx_flag) {
    Rx_flag = false;
    DEBUG_SERIAL.write(Rx_byte); // Gửi byte nhận được qua Serial để kiểm tra
  }
#endif
  //Reload watchdog timer
  IWatchdog.reload();
}
