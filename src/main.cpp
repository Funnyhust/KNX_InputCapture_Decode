#include "config.h"

#if CODE_STATE
#include <Arduino.h>
//#include "ic_knx.h"
#include "knx_tx.h"
#include "knx_rx.h"
#include <IWatchdog.h>

//  uint8_t testFrame[9]={0xBC, 0xAA, 0x55, 0x55,0xAA, 0xE1, 0x00, 0x80, 0x99};
//  uint8_t FB_Frame[9]={0xBC, 0x00, 0x55, 0x33,0xAA, 0xE1, 0x00, 0x80, 0xEE};

//  uint8_t onFrame[9]={0xBC, 0x11, 0x01, 0x00,0x04, 0xE1, 0x00, 0x80, 0x36};
// uint8_t offFrame[9]={0xBC, 0x11, 0x01, 0x00,0x04, 0xE1, 0x00, 0x81, 0x37};

#define KNX_RX_BUFFER_MAX_SIZE 23
static uint8_t uart_rx_buf[KNX_RX_BUFFER_MAX_SIZE];


static uint8_t knx_rx_buf[KNX_RX_BUFFER_MAX_SIZE];
static volatile bool Rx_flag=false;

static volatile uint8_t byte_idx=0;
HardwareSerial DEBUG_SERIAL(USART2);
HardwareSerial Serial3(USART3);

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
static bool knx_is_receved = false;


uint8_t knx_calc_checksum(const uint8_t *data, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len - 1; i++) { // không tính byte cuối
    cs ^= data[i];
  }
  return ~cs;
}

#if KNX_RX_MODE //Gửi cả Frame
static uint32_t last_byte_time = 0;
void handle_knx_frame(const uint8_t byte) {
  static bool knx_is_receved = true;
  uint32_t now = millis();

  // Nếu quá 3ms không nhận byte mới => reset buffer
  //----------------------------------------------------------
// Nếu dưới 3ms mà đã nhận bản tin mới thì sao, thì lỗi sai buffer gửi à??
// Không dùng retry nữa, phần retry để anh Hiếu nên có thể gửi từng byte thay vì gửi cả frame


  //-------------------------------------------------------------
  if ((now - last_byte_time) > 2) {
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
        }
       //  Serial3.println("Checksum Valid");
      }
      else  
      { 
      Serial3.println("Checksum Invalid");
      Serial3.write(0xA5);
      Serial3.write(knx_rx_buf,Send_length);
      }
      Rx_flag = true;
      static bool knx_is_receved = false;

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
   DEBUG_SERIAL.begin(19200, SERIAL_8E1); // parity even
   Serial3.begin(19200,SERIAL_8E1);
   IWatchdog.begin(500000); 
   knx_rx_init(handle_knx_frame);
   knx_tx_init();
}


bool read_uart_frame() {
  static uint8_t index = 0;
  static uint8_t total = 0;
  //Serial3.printf("Hello");
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

static uint16_t idx =0;
static uint16_t idx2=0;

// Loop-----------------------------------------------------------------


void loop() {
   // knx_send_frame(on_frame, 9);   
  if (read_uart_frame()) {
    //Co retry
    // Serial3.write(0xAA);
    // Serial3.printf("UART->KNX: ");
    // Serial3.println(idx);
    // idx++;
    knx_send_frame(uart_rx_buf,Uart_length);  // Gửi ra KNX bus
    //Khong retry
    //knx_send_frame(uart_rx_buf, Uart_length); // Gửi bản tin
    // memset((void*)uart_rx_buf, 0, sizeof(uart_rx_buf));
    Uart_length = 0;
    }

#if KNX_RX_MODE
  if(Rx_flag) {
    Rx_flag = false;
    //TEST
    //DEBUG_SERIAL.write(0xAA);
    //KNX_Received_Data
    // Serial3.printf("KNX->UART: ");
    // Serial3.println(idx2);
    // idx2++;
    //DEBUG_SERIAL.write(knx_rx_buf, Send_length);
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

// uint32_t last_send=0;
// void loop() {
//   if(!knx_is_receved){
//     uint32_t now= millis();
//    if((now-last_send)==100){
//          knx_send_frame(testFrame, 9);   
//          last_send= now;
//    }
//   }
//   IWatchdog.reload();s
// }

#else

#include <Arduino.h>
//#include "ic_knx.h"
#include "config.h"
#include "knx_tx.h"
#include "knx_rx.h"
#include <IWatchdog.h>

#define KNX_BUFFER_MAX_SIZE 23

uint8_t testFrame[9]={0xBC, 0xAA, 0x55, 0x55,0xAA, 0xE1, 0x00, 0x80, 0x22};
// uint8_t FB_Frame[9]={0xBC, 0x00, 0x55, 0x33,0xAA, 0xE1, 0x00, 0x80, 0xEE};


void MX_NVIC_Init(void)
{
    // Ưu tiên cao hơn cho Timer (để KNX đọc không bị block)
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  // Ưu tiên cao nhất cho EXTI (để KNX đọc không bị block)
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  // Ưu tiên trung bình cho USART1
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  // Ưu tiên thấp nhất cho USART2
  HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static uint32_t seed = 1;
uint8_t random_2_to_10(void) {
    seed = seed * 1664525UL + 1013904223UL;   // LCG
    return (seed % 9) + 8;  // [2..10]
}


//UART
HardwareSerial Serial3(USART3);
HardwareSerial DEBUG_SERIAL(USART2);
static uint8_t Uart_length = 0;
static uint8_t uart_rx_buf[KNX_BUFFER_MAX_SIZE];

//===============================================
bool read_uart_frame() {
  static uint8_t index = 0;
  static uint8_t total = 0;
  while (DEBUG_SERIAL.available()) {
    uint8_t byte = DEBUG_SERIAL.read();
    if (index < KNX_BUFFER_MAX_SIZE) {
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
      if (total > KNX_BUFFER_MAX_SIZE) {
        index = 0;
        total = 0;
        return false;
      }
    }
    if (total > 0 && index >= total) {
      // Có thể kiểm tra checksum ở đây nếu muốn
      //DEBUG_SERIAL.write(uart_rx_buf,Uart_length);
      index = 0;
      total = 0;
      return true;
    }
  }
  return false;
}
//===============================================

//KNX-TX
  static uint32_t backoff_time = 0;
  static bool waiting_backoff = false; 

//KNX-RX ========================================================================================================================================
static uint8_t knx_rx_buf[KNX_BUFFER_MAX_SIZE];
static volatile bool knx_rx_flag=false;
static volatile bool knx_is_receved = false;
static volatile uint8_t knx_rx_length = 0;
static volatile uint8_t byte_idx=0;

//===============CheckSum=================
uint8_t knx_calc_checksum(const uint8_t *data, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len - 1; i++) { // không tính byte cuối
    cs ^= data[i];
  }
  return ~cs;
} 
//============================================


#if KNX_RX_MODE //Gửi cả Frame
static uint32_t last_byte_time = 0;
void handle_knx_frame(const uint8_t byte) {

  knx_is_receved = true;
  uint32_t now = micros();
  //Serial3.print(now);
  // Nếu quá 1400us không nhận byte mới => reset buffer
  if ((now - last_byte_time) > 1450) {
    knx_rx_length =0;
    memset((void*)knx_rx_buf, 0, sizeof(knx_rx_buf));
    byte_idx = 0;
  }
  last_byte_time = now;

  // Lưu byte
  knx_rx_buf[byte_idx++] = byte;

  // Tính độ dài frame khi đã có ít nhất 6 byte
  if (byte_idx >= 6) {
    knx_rx_length = 6 + (knx_rx_buf[5] & 0x0F) + 1 + 1;
    if (byte_idx == knx_rx_length) {
      if (knx_rx_buf[knx_rx_length-1]==knx_calc_checksum(knx_rx_buf,knx_rx_length)){
      knx_rx_flag = true;
      knx_is_receved = false;
    }
      else  
      { 
      Serial3.println("Checksum Invalid");
      Serial3.write(0xA5);
      Serial3.write(knx_rx_buf,knx_rx_length);
      }
  }
  }
   // Serial3.println(micros());
  }
#else // Gửi Byte
static uint8_t Rx_byte=0;
static uint32_t last_byte_time = 0;
void handle_knx_frame(const uint8_t byte) {
  static uint32_t now = micros();
  knx_is_receved =true;
  if(now -last_byte_time>180){
    knx_is_receved = false;
  }
  DEBUG_SERIAL.write(byte);
  last_byte_time=now;
//  Rx_byte = byte;
//  knx_rx_flag = true;
}
#endif
//==========================================================================================================================================
// ================== QUEUE =====================
#define MAX_QUEUE 50  // tối đa 5 frame chờ gửi
struct Frame {
  uint8_t data[KNX_BUFFER_MAX_SIZE];
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

// void test_busy_bus(){
//   knx_send_frame(testFrame,9);
//   delay(100);
// }


void setup() {
   DEBUG_SERIAL.begin(19200, SERIAL_8E1); // parity even
   Serial3.begin(19200,SERIAL_8E1);
   IWatchdog.begin(500000);
   knx_rx_init(handle_knx_frame);
   knx_tx_init();
   MX_NVIC_Init();
   Serial3.printf("Start\r\n");
}

// Loop-----------------------------------------------------------------
void loop() {
  #if KNX_RX_MODE
  if(knx_rx_flag) {
    knx_rx_flag = false;
    //Serial3.printf("Serial OK after KNX receiver\r\n");
    DEBUG_SERIAL.write(knx_rx_buf, knx_rx_length);
  }
  if(micros()-last_byte_time>1500){
    knx_is_receved = false;
  }
#else
  if(knx_rx_flag) {
    knx_rx_flag = false;
    DEBUG_SERIAL.write(Rx_byte); // Gửi byte nhận được qua Serial để kiểm tra
  }
#endif
  // Nhận frame từ UART → bỏ vào queue
  if (read_uart_frame()) {
    //Serial3.printf("Serial OK after UART receiver\r\n");
    enqueue_frame(uart_rx_buf, Uart_length);
    Uart_length = 0;
  }
  // Nếu KNX đang rảnh → lấy frame từ queue ra gửi


if (q_count > 0) {
  if (!knx_is_receved) {
    if (!waiting_backoff) {
      // Bắt đầu backoff ngẫu nhiên
      backoff_time = millis() + random_2_to_10();// 2-10ms
      waiting_backoff = true; 
    }
    else if (millis() >= backoff_time) {
      // Backoff xong, gửi frame
      Frame f;
      if (dequeue_frame(&f)) {
        for(int i=0; i<f.len; i++){
          Serial3.printf("%02X ", f.data[i]);
        }
        Serial3.println(); 
        knx_send_frame(f.data, f.len);
      }
      waiting_backoff = false;
    }
  } else {
    // Nếu bus bận trong lúc chờ → reset backoff
    waiting_backoff = false;
  }
  }
    //Reload watchdog timer
  IWatchdog.reload();
}


// void loop() {
//   #if KNX_RX_MODE
//   if(knx_rx_flag) {
//     knx_rx_flag = false;
//     //Serial3.printf("Serial OK after KNX receiver\r\n");
//     DEBUG_SERIAL.write(knx_rx_buf, knx_rx_length);
//   }
//   if(micros()-last_byte_time>1500){
//     knx_is_receved = false;
//   }
// #else
//   if(knx_rx_flag) {
//     knx_rx_flag = false;
//     DEBUG_SERIAL.write(Rx_byte); // Gửi byte nhận được qua Serial để kiểm tra
//   }
// #endif
//   // Nhận frame từ UART → bỏ vào queue
//   if (read_uart_frame()) {
//     //Serial3.printf("Serial OK after UART receiver\r\n");
//     enqueue_frame(uart_rx_buf, Uart_length);
//     Uart_length = 0;
//   }
//   // Nếu KNX đang rảnh → lấy frame từ queue ra gửi
//   for(int i=0; i<10;i++){
//   if (!knx_is_receved) {
//     if (!waiting_backoff) {
//       // Bắt đầu backoff ngẫu nhiên
//       backoff_time = millis() + random_2_to_10();// 2-10ms
//       waiting_backoff = true; 
//     }
//     else if (millis() >= backoff_time) {
//       // Backoff xong, gửi frame
//         knx_send_frame(testFrame, 9);
//       waiting_backoff = false;
//     }
//   } else {
//     // Nếu bus bận trong lúc chờ → reset backoff
//     waiting_backoff = false;
//   }
//   }
//     //Reload watchdog timer
//   delay(300);
//   IWatchdog.reload();
// }

#endif





