#if !defined(FEETECH_SERIALPORT_H_)
#define FEETECH_SERIALPORT_H_

#include <stdint.h>

namespace feetech
{

class ISerialPort
{
public:
    uint32_t timeout_ms;
    virtual ~ISerialPort() {}
    virtual int read(uint8_t* buffer, int bytes) = 0;
    virtual int write(uint8_t data) { return write(&data, sizeof(data)); }
    virtual int write(const uint8_t* data, int bytes) = 0;
    virtual void flushRx() = 0;
    virtual void flushTx() = 0;
};

} // namespace feetech


#endif // FEETECH_SERIALPORT_H_
