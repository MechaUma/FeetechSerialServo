#include "feetech_protocol.h"

#include <string.h>

namespace
{
using namespace feetech;

uint8_t calculateChecksum(const void* buffer, int length, uint8_t checksum)
{
    uint8_t raw_val = ~checksum;
    auto* buffer8 = static_cast<const uint8_t*>(buffer);
    for(int i = 0; i < length; i++) { raw_val += buffer8[i]; }
    return ~raw_val;
}
uint8_t calculateChecksum(const void* buffer, int length) { return calculateChecksum(buffer, length, ~0); }

void sendInstruction(feetech::ISerialServo* servo, uint8_t id, feetech::Instruction instruction, uint8_t addr, const uint8_t* data, uint8_t length)
{
    auto& serial = servo->Serial();
    uint8_t header[sizeof(PACKAGE_HEADER) + sizeof(PacketHeader) + sizeof(addr)];
    memcpy(header, PACKAGE_HEADER, sizeof(PACKAGE_HEADER));
    auto* packet_header = reinterpret_cast<PacketHeader*>(&header[sizeof(PACKAGE_HEADER)]);
    packet_header->id = id;
    packet_header->length = (data) ? length + 3 : 2;
    packet_header->inst_or_status = static_cast<uint8_t>(instruction);
    header[sizeof(PACKAGE_HEADER) + sizeof(PacketHeader)] = addr;
    const int header_length = (data) ? sizeof(header) : sizeof(header) - 1;
    serial.write(header, header_length);
    if (data) { serial.write(data, length); }
    serial.write(
        calculateChecksum(data, length,
        calculateChecksum(packet_header, header_length - sizeof(PACKAGE_HEADER))));
}
void sendInstruction(feetech::ISerialServo* servo, uint8_t id, feetech::Instruction instruction) { sendInstruction(servo, id, instruction, 0, nullptr, 0); }

bool recvHeader(ISerialServo* servo, PacketHeader *header, int retry_count = 10)
{
    auto& serial = servo->Serial();
    uint8_t package_header[sizeof(PACKAGE_HEADER)] = {0};
    for (int i = 0; i < retry_count; i++)
    {
        package_header[0] = package_header[1];
        if (!serial.read(&package_header[1], sizeof(*package_header))) { return false; }
        if (package_header[0] == PACKAGE_HEADER[0]
            && package_header[1] == PACKAGE_HEADER[1])
        {
            return serial.read(reinterpret_cast<uint8_t*>(header), sizeof(*header)) == sizeof(*header);
        }
    }
    return false;
}

constexpr bool needsRecvAck(uint8_t id, uint8_t resp_level)
{
    return (id != feetech::BROADCAST_ID) && (resp_level == 1);
}

retval_t recvAck(ISerialServo* servo, uint8_t id)
{
    auto& serial = servo->Serial();
    PacketHeader header;
    uint8_t checksum;
    if (!recvHeader(servo, &header)) { return FEETECH_INVALID_HEADER; }
    if (header.id != id) { return FEETECH_INVALID_ID; }
    if (header.length != 2) { return FEETECH_INVALID_LEN; }
    if (serial.read(&checksum, sizeof(checksum)) != sizeof(checksum)
        || checksum != calculateChecksum(&header, sizeof(header)))
    {
        return FEETECH_INVALID_CHECKSUM;
    }
    servo->servo_status = header.inst_or_status;
    return FEETECH_OK;
}

}

namespace feetech
{

retval_t ping(ISerialServo *servo, uint8_t id, uint8_t* recv_id)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::PING);
    serial.flushTx();

    PacketHeader header;
    uint8_t checksum;
    if (!recvHeader(servo, &header)) { return FEETECH_INVALID_HEADER; }
    if (header.id != id && id != BROADCAST_ID) { return FEETECH_INVALID_ID; }
    if (header.length != 2) { return FEETECH_INVALID_LEN; }
    if (serial.read(&checksum, sizeof(checksum)) != sizeof(checksum)
        || checksum != calculateChecksum(&header, sizeof(header)))
    {
        return FEETECH_INVALID_CHECKSUM;
    }
    *recv_id = header.id;
    servo->servo_status = header.inst_or_status;
    return FEETECH_OK;
}

retval_t read(ISerialServo *servo, uint8_t id, uint8_t addr, uint8_t *buffer, uint8_t length)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::READ, addr, &length, sizeof(length));
    serial.flushTx();

    PacketHeader header;
    if (!recvHeader(servo, &header)) { return FEETECH_INVALID_HEADER; }
    if (header.id != id) { return FEETECH_INVALID_ID; }
    if (header.length != length + 2) { return FEETECH_INVALID_LEN; }
    if (serial.read(buffer, length) != length) { return FEETECH_NG; }

    uint8_t checksum;
    if (serial.read(&checksum, sizeof(checksum)) != sizeof(checksum)
        || checksum !=  calculateChecksum(&header, sizeof(header),
                        calculateChecksum(buffer, length)))
    {
        return FEETECH_INVALID_CHECKSUM;
    }
    servo->servo_status = header.inst_or_status;
    return FEETECH_OK;
}

retval_t write(ISerialServo *servo, uint8_t id, uint8_t addr, uint8_t *data, uint8_t length)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::WRITE, addr, data, length);
    serial.flushTx();
    return needsRecvAck(id, servo->resp_level) ? recvAck(servo, id) : FEETECH_OK;
}

retval_t regWrite(ISerialServo *servo, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t length)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::REG_WRITE, addr, data, length);
    serial.flushTx();
    return needsRecvAck(id, servo->resp_level) ? recvAck(servo, id) : FEETECH_OK;
}

retval_t action(ISerialServo *servo, uint8_t id)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::ACTION);
    serial.flushTx();
    return needsRecvAck(id, servo->resp_level) ? recvAck(servo, id) : FEETECH_OK;
}

retval_t reset(ISerialServo *servo, uint8_t id)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    sendInstruction(servo, id, Instruction::RESET);
    serial.flushTx();
    return needsRecvAck(id, servo->resp_level) ? recvAck(servo, id) : FEETECH_OK;
}

retval_t syncWrite(ISerialServo *servo, const uint8_t *id_list, uint8_t id_num, uint8_t addr, const uint8_t *data, uint8_t length)
{
    auto& serial = servo->Serial();
    serial.flushRx();
    uint8_t header[sizeof(PACKAGE_HEADER) + sizeof(PacketHeader) + sizeof(addr) + sizeof(length)];
    memcpy(header, PACKAGE_HEADER, sizeof(PACKAGE_HEADER));
    auto* packet_header = reinterpret_cast<PacketHeader*>(&header[sizeof(PACKAGE_HEADER)]);
    packet_header->id = BROADCAST_ID;
    packet_header->length = id_num * (length + 1) + 4;
    packet_header->inst_or_status = static_cast<uint8_t>(Instruction::SYNC_WRITE);
    header[sizeof(PACKAGE_HEADER) + sizeof(PacketHeader)] = addr;
    header[sizeof(PACKAGE_HEADER) + sizeof(PacketHeader) + 1] = length;
    serial.write(header, sizeof(header));

    for(int i = 0; i < id_num; i++)
    {
        serial.write(id_list[i]);
        serial.write(&data[i * length], length);
    }
    serial.write(
        calculateChecksum(packet_header, sizeof(header) - sizeof(PACKAGE_HEADER),
        calculateChecksum(id_list, id_num,
        calculateChecksum(data, id_num * length))));
    serial.flushTx();
    return FEETECH_OK;
}

} // namespace feetech
