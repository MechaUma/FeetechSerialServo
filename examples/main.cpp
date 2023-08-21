#include <M5Unified.h>

#include <chrono>

#include "feetech_espserial.h"
#include "feetech_scs_servo.h"

// stack-chan m5-pantilit board v0.2.1
class M5PanTiltSerial : public EspSerial
{
public:
    M5PanTiltSerial(uart_port_t uart_num) : EspSerial(uart_num) {}
    int write(const uint8_t* data, int length) override
    {
        const int retval = EspSerial::write(data, length);
        if (retval > 0)
        {
            std::unique_ptr<uint8_t[]> buffer(new uint8_t[retval]);
            read(buffer.get(), retval);
        }
        return retval;
    }
};

constexpr uart_port_t uart_num = UART_NUM_2;
M5PanTiltSerial serial(uart_num);
feetech::ScsServo servo(serial);
std::chrono::system_clock::time_point begin;


void setup()
{
    M5.begin();
    serial.begin(GPIO_NUM_18, GPIO_NUM_17);

    // Wait for the servo to be ready.
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    begin = std::chrono::system_clock::now();
}

void loop()
{
    auto elapsed = std::chrono::system_clock::now() - begin;
    auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    int64_t prevMs = 0;
    uint8_t id_list[] = {1, 2};
    if (elapsedMs > 4000)
    {
        servo.rotateV(2, 160, 100);
        // uint16_t positions[] = {600, 545};
        // uint16_t times_ms[] = {0, 0};
        // uint16_t velocities_pps[] = {300, 300};
        // servo.syncWritePosition(id_list, sizeof(id_list), positions, times_ms, velocities_pps);
        begin = std::chrono::system_clock::now();
    }
    else if (elapsedMs > 3000)
    {
        servo.moveT(2, 512, 800);
    }
    else if (elapsedMs > 2000)
    {
        servo.moveV(1, 600, 300);
        // uint16_t positions[] = {512, 512};
        // uint16_t times_ms[] = {800, 800};
        // uint16_t velocities_pps[] = {0, 0};
        // servo.syncWritePosition(id_list, sizeof(id_list), positions, times_ms, velocities_pps);
    }
    else if (elapsedMs > 1000)
    {
        servo.rotateT(1, 150, 800);
    }

    if (prevMs != elapsedMs)
    {
        prevMs = elapsedMs;
        feetech::scs::FeedbackData feedback1;
        servo.feedback(1, &feedback1);
        printf(">position:%d\n", feedback1.position);
        printf(">speed:%d\n", feedback1.speed);
        printf(">load:%d\n", feedback1.load);
        printf(">voltage:%d\n", feedback1.voltage);
        printf(">temperature:%d\n", feedback1.temperature);
        printf(">moving:%d\n", (feedback1.is_moving) ? 1 : 0);
        printf(">current:%d\n", feedback1.current);
    }

}