// MESSAGE SENSOR_INIT_STATE support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief SENSOR_INIT_STATE message
 *
 * Sensor initialization status bitfield
 */
struct SENSOR_INIT_STATE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 103;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 155;
    static constexpr auto NAME = "SENSOR_INIT_STATE";


    uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    uint32_t sensor_init; /*<  Bitfield indicating initialization status of sensors */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_us: " << time_us << std::endl;
        ss << "  sensor_init: " << sensor_init << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << sensor_init;                   // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> sensor_init;                   // offset: 4
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
