#ifndef FEETECH_PROTOCOL_H_
#define FEETECH_PROTOCOL_H_

#include <stdint.h>

#include "feetech_servo.h"

namespace feetech
{

const uint8_t PACKAGE_HEADER[] = { 0xFF, 0xFF };
constexpr uint8_t BROADCAST_ID = 0xFE;

enum class Instruction: uint8_t
{
    PING = 0x01,        ///< Query Working Status
    READ = 0x02,        ///< Query the character in the control table
    WRITE = 0x03,       ///< Write the character into the control table
    REG_WRITE = 0x04,   ///< Similar to WRITE DATA, but the control character does not act immediately after writing until ACTION.
    ACTION = 0x05,      ///< Triggering the action of REG WRITE operation
    RESET = 0x06,       ///< Reset the control table to the factory value.
    SYNC_READ = 0x82,   ///< Query multiple servos at the same time.
    SYNC_WRITE = 0x83,  ///< Controlling multiple servos at the same time
};

enum retval_t
{
    FEETECH_OK = 0,
    FEETECH_NG = 1,     ///< General error
    FEETECH_INVALID_HEADER,
    FEETECH_INVALID_ID,
    FEETECH_INVALID_LEN,
    FEETECH_INVALID_CHECKSUM,
};

#pragma pack(push, 1)
struct PacketHeader
{
    uint8_t id;
    uint8_t length;
    uint8_t inst_or_status;
};
#pragma pack(pop)

retval_t ping(ISerialServo* servo, uint8_t id, uint8_t* recv_id);
retval_t read(ISerialServo* servo, uint8_t id, uint8_t addr, uint8_t* buffer, uint8_t length);
retval_t write(ISerialServo* servo, uint8_t id, uint8_t addr, uint8_t* data, uint8_t length);
retval_t regWrite(ISerialServo* servo, uint8_t id, uint8_t addr, const uint8_t* data, uint8_t length);
retval_t action(ISerialServo* servo, uint8_t id);
retval_t reset(ISerialServo* servo, uint8_t id);
// TODO impl syncRead
retval_t syncWrite(ISerialServo* servo, const uint8_t* id_list, uint8_t id_num, uint8_t addr, const uint8_t* data, uint8_t length);

} // namespace feetech


#endif // FEETECH_PROTOCOL_H_