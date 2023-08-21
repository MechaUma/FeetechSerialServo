#if !defined(FEETECH_SCS_SERVO_H_)
#define FEETECH_SCS_SERVO_H_

#include <cmath>
#include <endian.h>

#include "feetech_protocol.h"
#include "feetech_serialport.h"
#include "feetech_servo.h"

namespace feetech
{

namespace scs
{

// based on FD1.9.8-SCS系列内存表解析_1902(1).xlsx
enum MemTable: uint8_t
{
    // EPROM (Read only)
    FIRMWARE_MAJOR_VERSION = 0x01,
    FIRMWARE_MINOR_VERSION = 0x02,
    SERVO_MAJOR_VERSION = 0x03,
    SERVO_MINOR_VERSION = 0x04,

    // EPROM (Read / Write)
    ID = 0x05,                      // 1, ID 254 (OxFE) is the broadcast ID, which does not respond to response packets.
    BAUDRATE = 0x06,                // 0, 0:1000000, 1:500000, 2:250000, 3:128000, 4:115200, 5:76800, 6:57600, 7: 38400
    RETURN_DELAY = 0x07,            // 0[2us], 0-254, Minimum unit is 2us, maximum return delay can be set to 508us (254 * 2)
    RETURN_LEVEL = 0x08,            // 1, 0: All commands, except read and PING commands, do not return response packets. 1: Return response packets for all commands
    MIN_ANGLE = 0x09,               // [WORD] 20[steps], Set the minimum operating angle limit, the value is 0 in motor mode
    MAX_ANGLE = 0x0B,               // [WORD] 1003[steps], Set the maximum operating angle limit, the value is 0 in motor mode
    MAX_TEMPERATURE = 0x0D,         // 80[℃], Maximum operating temperature limit. Set precision to 1°C
    MAX_INPUT_VOLTAGE = 0x0E,       // 90[0.1V], Maximum input voltage. Set to 90 for a maximum working voltage of 9.0V, precision is 0.1V
    MIN_INPUT_VOLTAGE = 0x0F,       // 45[0.1V], Minimum input voltage. Set to 50 for a minimum working voltage of 5.0V, precision is 0.1V
    MAX_TORQUE = 0x10,              // [WORD] 1000[0.1%], Set the maximum output torque limit for the servo. 1000 = 100% * stall torque
    PHASE = 0x12,                   // 0, Special function byte, do not modify without special requirements
    PROTECTION_BITS = 0x13,         // 36, Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 correspondingly set to 1 to enable the corresponding protection: voltage, none, temperature, none, none, overload. Set the corresponding bit to 0 to disable the corresponding protection
    LED_ALARM_BITS = 0x14,          // 37, Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 correspondingly set to 1 to enable flashing alarm: voltage, none, temperature, none, none, overload. Set the corresponding bit to 0 to disable flashing alarm
    P_GAIN = 0x15,                  // 15, Control motor proportional coefficient
    D_GAIN = 0x16,                  // 15, Control motor differential coefficient
    I_GAIN = 0x17,                  // 0, Control motor integral coefficient
    MIN_STARTUP_TORQUE = 0x18,      // [WORD] 30[0.1%], Set the minimum output startup torque for the servo. 1000 = 100% * stall torque
    CW_DEAD_ZONE = 0x1A,            // 1[steps], Minimum unit is one minimum resolution angle
    CCW_DEAD_ZONE = 0x1B,           // 1[steps], Minimum unit is one minimum resolution angle
    HYSTERESIS = 0x1C,              // 1[steps], Minimum unit is one minimum resolution angle
    PROTECTION_TORQUE = 0x25,       // 20[%], Output torque after entering overload protection. Set to 20 for 20% of the maximum torque
    PROTECTION_TIME = 0x26,         // 100[40ms], Duration for which the current load output exceeds the overload torque and remains. Set to 100 for 4 seconds, maximum can be set to 10 seconds
    OVERLOAD_TORQUE = 0x27,         // 80[%], Maximum torque threshold for starting overload protection time. Set to 80 for 80% of the maximum torque
    
    // SRAM (Read / Write)
    TORQUE_SWITCH = 0x28,           // 0, Write 0: Close torque output/damping state; Write 1: Open torque output; Write 2: Free state write
    TARGET_POSITION = 0x2A,         // [WORD] 0[steps], Each step is one minimum resolution angle, absolute position control mode, maximum corresponds to the maximum valid angle
    OPERATION_TIME = 0x2C,          // [WORD] 0[ms], Time from the current position to the target position during motion. This parameter takes effect when the running speed is 0
    OPERATION_SPEED = 0x2E,         // [WORD] 0[steps/s], Number of steps moved in one second.
    LOCK_EPROM = 0x30,              // 0, Write 0: Close write lock, the value written to the EPROM address will be saved even after power-off; Write 1: Open write lock, the value written to the EPROM address will not be saved after power-off

    // SRAM (Read only)
    FEEDBACK_POSITION = 0x38,       // [WORD] 0[steps], Feedback the current position in steps, each step is one minimum resolution angle; absolute position control mode, maximum value corresponds to the maximum valid angle
    FEEDBACK_SPEED = 0x3A,          // [WORD] 0[steps/s], Feedback the current motor speed, number of steps moved per second
    FEEDBACK_LOAD = 0x3C,           // [WORD] 0[0.1%], Current control output driving motor voltage duty cycle
    FEEDBACK_VOLTAGE = 0x3E,        // 0[0.1V], Current servo operating voltage
    FEEDBACK_TEMPERATURE = 0x3F,    // 0[℃], Current servo internal working circuit temperature
    ASYNC_WRITE_STATUS = 0x40,      // 0, Flag when using asynchronous write command
    SERVO_STATUS = 0x41,            // 0, Bit0 Bit1 Bit2 Bit3 Bit4 Bit5 correspondingly set to 1 indicates the corresponding error occurred: voltage, none, temperature, none, none, overload. The corresponding bit is 0 when there is no corresponding error
    MOVING_STATUS = 0x42,           // 0, Flag is 1 when the servo is moving, and 0 when the servo is stopped
    FEEDBACK_CURRENT = 0x45,        // [WORD]
};

struct FeedbackData
{
    int position;
    int speed;
    int load;
    int voltage;
    int temperature;
    bool is_moving;
    int current;
};

} // namespace scs

#define SCSSERVO_DEFAULT_COFIG() { .max_angle = 300, .resolution = 1024 }
struct ScsServoConfig
{
    int max_angle;
    int resolution;
};

class ScsServo : public ISerialServo
{
public:
    ScsServo(ISerialPort& serial, const ScsServoConfig& config = SCSSERVO_DEFAULT_COFIG())
    : serial_(serial), config_(config){ }
    ISerialPort& Serial() { return serial_; }
    int degreeToPulse(float degree) { return std::roundf(degree * config_.resolution / config_.max_angle); }
    int pulseToDegree(int pulse) { return (float)pulse * config_.max_angle / config_.resolution; }

    retval_t writePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t velocity_pps);
    retval_t moveT(uint8_t id, uint16_t position, uint16_t time_ms) { return writePosition(id, position, time_ms, 0); }
    retval_t moveV(uint8_t id, uint16_t position, uint16_t velocity_pps) { return writePosition(id, position, 0, velocity_pps); }
    retval_t rotateT(uint8_t id, float degree, uint16_t time_ms) { return moveT(id, degreeToPulse(degree), time_ms); }
    retval_t rotateV(uint8_t id, float degree, uint16_t velocity_dps) { return moveV(id, degreeToPulse(degree), degreeToPulse(velocity_dps)); }
    retval_t regWritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t velocity_pps);
    retval_t syncWritePosition(const uint8_t* id_list, uint8_t id_num, const uint16_t* positions, const uint16_t* times_ms, const uint16_t* velocites_pps);
    retval_t torque(uint8_t id, uint8_t mode);
    retval_t lockEprom(uint8_t id, bool lock = true);
    retval_t unlockEprom(uint8_t id) { return lockEprom(id, false); }
    retval_t feedback(uint8_t id, scs::FeedbackData* data);

private:
    ISerialPort& serial_;
    ScsServoConfig config_;
    uint16_t hton16(uint16_t value) { return htobe16(value); }
    uint16_t ntoh16(uint16_t value) { return be16toh(value); }
};

} // namespace feetech


#endif // FEETECH_SCS_SERVO_H_
