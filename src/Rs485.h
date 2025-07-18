#ifndef RCT_RS485_H_
#define RCT_RS485_H_

#include <Kernel.h>
#include <mbed.h>

struct Rs485 {
  Rs485(const PinName tx, const PinName rx, const int baud, const PinName de) : bus_{tx, rx, baud}, de_{de} {
    bus_.set_blocking(0);
  }
  void uart_transmit(const uint8_t *send, const int len) {
    // printf("len: %d, %02x\n", len, send[0]);
    de_ = 1;
    flush();
    bus_.write(send, len);
    wait_us(4.5);
    de_ = 0;
  }
  template<int N>
  void uart_transmit(const uint8_t (&send)[N]) {
    uart_transmit(send, sizeof(send));
  }
  bool uart_receive(void *buf, const int len, const std::chrono::milliseconds timeout) {
    auto pre = Kernel::Clock::now();
    uint8_t *p = reinterpret_cast<uint8_t *>(buf);
    const uint8_t *end = p + len;
    // printf("len: %d, p: %d, end: %d\n", len, p, end);
    do {
      if(bus_.read(p, 1) > 0) {
        
        if (++p == end) {
          // printf("yometa2\n");
          return (wait_ns(275), true);
        }
        // printf("yometa: %d\n", *p);
      }
    } while(Kernel::Clock::now() - pre < timeout);
    // printf("failed\n");
    return false;
  }
  bool uart_receive(void *buf, const int len, const std::chrono::microseconds timeout) {
    auto pre = Kernel::Clock::now();
    uint8_t *p = reinterpret_cast<uint8_t *>(buf);
    const uint8_t *end = p + len;
    // printf("len: %d, p: %d, end: %d\n", len, p, end);
    do {
      if(bus_.read(p, 1) > 0) {
        
        if (++p == end) {
          // printf("yometa2\n");
          return (wait_ns(275), true);
        }
        // printf("yometa: %d\n", *p);
      }
    } while(Kernel::Clock::now() - pre < timeout);
    // printf("failed\n");
    return false;
  }
  template<int N>
  bool uart_receive(uint8_t (&buf)[N], const std::chrono::milliseconds timeout) {
    return uart_receive(buf, sizeof(buf), timeout);
  }
 private:
  void flush() {
    uint8_t buf;
    while(bus_.read(&buf, 1) > 0) {}
  }
  BufferedSerial bus_;
  DigitalOut de_;
};

#endif  // RCT_RS485_H_
