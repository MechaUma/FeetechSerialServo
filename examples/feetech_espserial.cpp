#include "feetech_espserial.h"

#include <esp_timer.h>
#include <driver/uart.h>

int EspSerial::read(uint8_t* buffer, int bytes, int timeout_ms)
{
    int read_length = 0;
    const auto begin = esp_timer_get_time();
    while(read_length < bytes)
    {
        const auto elapsed_time_ms = (esp_timer_get_time() - begin) / 1000;
        if (elapsed_time_ms >= timeout_ms) { break; }
        int len = uart_read_bytes(uart_num_, &buffer[read_length], bytes - read_length, (timeout_ms - elapsed_time_ms) / portTICK_PERIOD_MS);
        if (len > 0) { read_length += len; }
    }
    return read_length;
}

void EspSerial::begin(int rxpin, int txpin, int baudrate)
{
    uart_config_t uart_config = {
        .baud_rate = (baudrate > 0) ? baudrate : 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 112,
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 2 * UART_FIFO_LEN, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, txpin, rxpin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}