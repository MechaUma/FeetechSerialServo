#if !defined(FEETECH_ESPSERIAL_H_)
#define FEETECH_ESPSERIAL_H_

#include <driver/uart.h>

#include "feetech_serialport.h"

class EspSerial : public feetech::ISerialPort
{
public:
    EspSerial(uart_port_t uart_num, int timeout_ms = 10)
    : uart_num_(uart_num), timeout_ms(timeout_ms) {}
    int timeout_ms;
    int read(uint8_t* buffer, int bytes, int timeout_ms);
    int read(uint8_t* buffer, int bytes) override { return read(buffer, bytes, timeout_ms); }
    int write(const uint8_t* data, int bytes) override { return uart_write_bytes(uart_num_, data, bytes); }
    void flushRx() override { uart_flush_input(uart_num_); }
    void flushTx() override { uart_wait_tx_done(uart_num_, portMAX_DELAY); }

    void begin(int rxpin, int txpin, int baudrate = -1);
private:
    uart_port_t uart_num_;
};

#endif // FEETECH_ESPSERIAL_H_
