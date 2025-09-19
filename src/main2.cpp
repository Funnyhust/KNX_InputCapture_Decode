// #include <Arduino.h>
// //#include "ic_knx.h"
// #include "config.h"
// #include "knx_tx.h"
// #include "knx_rx.h"
// #include <IWatchdog.h>

// #define KNX_BUFFER_MAX_SIZE 23

// uint8_t testFrame[9]={0xBC, 0xAA, 0x55, 0x55,0xAA, 0xE1, 0x00, 0x80, 0x22};
// uint8_t FB_Frame[9]={0xBC, 0x00, 0x55, 0x33,0xAA, 0xE1, 0x00, 0x80, 0xEE};

// //UART
// HardwareSerial DEBUG_SERIAL(PA3, PA2);
// static uint8_t Uart_length = 0;
// static uint8_t uart_rx_buf[KNX_BUFFER_MAX_SIZE];

// //===============================================
// bool read_uart_frame() {
//   static uint8_t index = 0;
//   static uint8_t total = 0;
//   while (DEBUG_SERIAL.available()) {
//     uint8_t byte = DEBUG_SERIAL.read();
//     if (index < KNX_BUFFER_MAX_SIZE) {
//       uart_rx_buf[index++] = byte;
//     } else {
//       // Tràn buffer, reset lại
//       index = 0;
//       total = 0;
//       return false;
//     }
//     if (index >= 6&& total==0) {
//       total = 6 + (uart_rx_buf[5] & 0x0F) + 1 + 1;
//       Uart_length = total;
//       if (total > KNX_BUFFER_MAX_SIZE) {
//         index = 0;
//         total = 0;
//         return false;
//       }
//     }
//     if (total > 0 && index >= total) {
//       // Có thể kiểm tra checksum ở đây nếu muốn
//       DEBUG_SERIAL.write(uart_rx_buf,Uart_length);
//       index = 0;
//       total = 0;
//       return true;
//     }
//   }
//   return false;
// }
// //===============================================




// //KNX========================================================================================================================================
// static uint8_t knx_rx_buf[KNX_BUFFER_MAX_SIZE];
// static volatile bool knx_rx_flag=false;
// static volatile bool knx_is_receved = false;
// static uint8_t knx_rx_length = 0;
// static volatile uint8_t byte_idx=0;

// //===============CheckSum=================
// uint8_t knx_calc_checksum(const uint8_t *data, uint8_t len) {
//   uint8_t cs = 0;
//   for (uint8_t i = 0; i < len - 1; i++) { // không tính byte cuối
//     cs ^= data[i];
//   }
//   return ~cs;
// }
// //============================================



// #if KNX_RX_MODE //Gửi cả Frame
// static uint32_t last_byte_time = 0;
// void handle_knx_frame(const uint8_t byte) {
//   knx_is_receved = true;
//   uint32_t now = micros();
//   // Nếu quá 3ms không nhận byte mới => reset buffer
//   if ((now - last_byte_time) > 180) {
//     knx_rx_length =0;
//     memset((void*)knx_rx_buf, 0, sizeof(knx_rx_buf));
//     byte_idx = 0;
//   }
//   last_byte_time = now;

//   // Lưu byte
//   knx_rx_buf[byte_idx++] = byte;

//   // Tính độ dài frame khi đã có ít nhất 6 byte
//   if (byte_idx >= 6) {
//     knx_rx_length = 6 + (knx_rx_buf[5] & 0x0F) + 1 + 1;
//     if (byte_idx == knx_rx_length) {
//       // if(knx_rx_buf[knx_rx_length] ==0xEE || knx_rx_buf[knx_rx_length] ==0x22){

//       // }
//       // else{
//       //   knx_rx_flag = true;
//       //   knx_is_receved = false;
//       // }
//       knx_rx_flag = true;
//       knx_is_receved = false;
//     }
//   }
//   }
// #else // Gửi Byte
// static uint8_t Rx_byte=0;
// static uint32_t last_byte_time = 0;
// void handle_knx_frame(const uint8_t byte) {
//   static uint32_t now = micros();
//   knx_is_receved =true;
//   if(now -last_byte_time>180){
//     knx_is_receved = false;
//   }
//   DEBUG_SERIAL.write(byte);
//   last_byte_time=now;
// //  Rx_byte = byte;
// //  knx_rx_flag = true;
// }
// #endif
// //==========================================================================================================================================
// // ================== QUEUE =====================
// #define MAX_QUEUE 150  // tối đa 5 frame chờ gửi
// struct Frame {
//   uint8_t data[KNX_BUFFER_MAX_SIZE];
//   uint8_t len;
// };

// Frame queue[MAX_QUEUE];
// volatile uint8_t q_head = 0, q_tail = 0;
// volatile uint8_t q_count = 0;

// // Hàm thêm frame vào queue
// bool enqueue_frame(uint8_t *data, uint8_t len) {
//   if (q_count >= MAX_QUEUE) return false; // đầy
//   memcpy(queue[q_tail].data, data, len);
//   queue[q_tail].len = len;
//   q_tail = (q_tail + 1) % MAX_QUEUE;
//   q_count++;
//   return true;
// }

// // Hàm lấy frame ra khỏi queue
// bool dequeue_frame(Frame *f) {
//   if (q_count == 0) return false;
//   memcpy(f->data, queue[q_head].data, queue[q_head].len);
//   f->len = queue[q_head].len;
//   q_head = (q_head + 1) % MAX_QUEUE;
//   q_count--;
//   return true;
// }

// void test_busy_bus(){
//   knx_send_frame(testFrame,9);
//   delay(100);
// }


// void setup() {
//    DEBUG_SERIAL.begin(115200, SERIAL_8E1); // parity even
//    IWatchdog.begin(500000); 
//    knx_rx_init(handle_knx_frame);
//    knx_tx_init();
// }


// // Loop-----------------------------------------------------------------
// void loop() {


//   // test_busy_bus();
//   //   if(knx_rx_flag) {
//   //   knx_rx_flag = false;
//   //   //TEST
//   //   //DEBUG_SERIAL.write(0xAA);
//   //   //KNX_Received_Data
//   //   DEBUG_SERIAL.write(FB_Frame, 9);
//   //   byte_idx = 0;
//   // }


//   //Nếu nhận đủ frame thì truyền UART lên bridge
//   #if KNX_RX_MODE
//   if(knx_rx_flag) {
//     knx_rx_flag = false;
//     //TEST
//     //DEBUG_SERIAL.write(0xAA);
//     //KNX_Received_Data
//     DEBUG_SERIAL.write(knx_rx_buf, knx_rx_length);
//     memset((void*)knx_rx_buf, 0, sizeof(knx_rx_buf));
//     byte_idx = 0;

//   }
// #else
//   if(knx_rx_flag) {
//     knx_rx_flag = false;
//     DEBUG_SERIAL.write(Rx_byte); // Gửi byte nhận được qua Serial để kiểm tra
//   }
// #endif
//   // Nhận frame từ UART → bỏ vào queue
//   if (read_uart_frame()) {
//     enqueue_frame(uart_rx_buf, Uart_length);
//     Uart_length = 0;
//   }

//   // Nếu KNX đang rảnh → lấy frame từ queue ra gửi
//   if (!knx_is_receved && q_count > 0) {
//     Frame f;
//       knx_send_frame(f.data, f.len);
//     }
//   //Reload watchdog timer
//   IWatchdog.reload();
// }