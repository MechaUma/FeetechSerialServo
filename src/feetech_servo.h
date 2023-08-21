#if !defined(FEETECH_SERVO_H_)
#define FEETECH_SERVO_H_

#include <stdint.h>

#include "feetech_serialport.h"

namespace feetech
{

class ISerialServo
{
public:
    virtual ~ISerialServo() {}
    uint8_t servo_status = 0;
    uint8_t resp_level = 1;
    virtual ISerialPort& Serial() = 0;
};

} // namespace feetech


#endif // FEETECH_SERVO_H_
