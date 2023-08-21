#include "feetech_scs_servo.h"

#include <string.h>
#include <memory>

#include "feetech_protocol.h"

namespace
{

constexpr int utoi(uint16_t data, int bit_num)
{
    return (data & (1 << bit_num)) ? -(data & ~(1 << bit_num)) : data;
}

feetech::retval_t writeByte(feetech::ISerialServo* servo, uint8_t id, uint8_t addr, uint8_t data)
{
    return feetech::write(servo, id, addr, &data, sizeof(data));
}

}

namespace feetech
{

retval_t ScsServo::writePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t velocity_pps)
{
    uint16_t data[] = { hton16(position), hton16(time_ms), hton16(velocity_pps) };
    return feetech::write(this, id, scs::TARGET_POSITION, (uint8_t*)data, sizeof(data));
}

retval_t ScsServo::regWritePosition(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t velocity_pps)
{
    uint16_t data[] = { hton16(position), hton16(time_ms), hton16(velocity_pps) };
    return feetech::regWrite(this, id, scs::TARGET_POSITION, (uint8_t*)data, sizeof(data));
}

retval_t ScsServo::syncWritePosition(const uint8_t* id_list, uint8_t id_num, const uint16_t* positions, const uint16_t* times_ms, const uint16_t* velocites_pps)
{
    constexpr int param_length = sizeof(*positions) + sizeof(*times_ms) + sizeof(*velocites_pps);
    std::unique_ptr<uint8_t[]> data(new uint8_t[id_num * param_length]);
    uint16_t* param = reinterpret_cast<uint16_t*>(data.get());
    for (int i = 0; i < id_num; i++)
    {
        *param = hton16(positions[i]); param++;
        *param = hton16(times_ms[i]); param++;
        *param = hton16(velocites_pps[i]); param++;
    }
    return feetech::syncWrite(this, id_list, id_num, scs::TARGET_POSITION, data.get(), param_length);
}

retval_t ScsServo::torque(uint8_t id, uint8_t mode)
{
    return writeByte(this, id, scs::MemTable::TORQUE_SWITCH, mode);
}

retval_t ScsServo::lockEprom(uint8_t id, bool lock)
{
    return writeByte(this, id, scs::MemTable::LOCK_EPROM, (lock) ? 1 : 0);
}

retval_t ScsServo::feedback(uint8_t id, scs::FeedbackData* data)
{
    constexpr int length = (scs::MemTable::FEEDBACK_CURRENT - scs::MemTable::FEEDBACK_POSITION) + 2;
    uint8_t buffer[length];
    auto ret_read = feetech::read(this, id, scs::MemTable::FEEDBACK_POSITION, buffer, length); 
    if (ret_read != retval_t::FEETECH_OK) { return ret_read; }
    auto offset = [&](scs::MemTable addr) { return &buffer[addr - scs::MemTable::FEEDBACK_POSITION];};
    data->position = ntoh16(*(uint16_t*)buffer);
    data->speed = utoi(ntoh16(*(uint16_t*)offset(scs::MemTable::FEEDBACK_SPEED)), 15);
    data->load = utoi(ntoh16(*(uint16_t*)offset(scs::MemTable::FEEDBACK_LOAD)), 10);
    data->voltage = *offset(scs::MemTable::FEEDBACK_VOLTAGE);
    data->temperature = *offset(scs::MemTable::FEEDBACK_TEMPERATURE);
    data->is_moving = (*offset(scs::MemTable::MOVING_STATUS) != 0);
    // To avoid a bus error on odd addresses, use memcpy() to copy.
    uint16_t current; 
    memcpy(&current, offset(scs::MemTable::FEEDBACK_CURRENT), sizeof(current));
    data->current = utoi(ntoh16(current), 15);
    return retval_t::FEETECH_OK;
}

} // namespace feetech
