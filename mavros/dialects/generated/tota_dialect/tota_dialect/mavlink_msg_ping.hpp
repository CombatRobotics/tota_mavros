// MESSAGE PING support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief PING message
 *
 * A ping message to test system connectivity and measure round-trip time
 */
struct PING : mavlink::Message {
    static constexpr msgid_t MSG_ID = 4;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 40;
    static constexpr auto NAME = "PING";


    uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    uint32_t seq; /*<  PING sequence number */
    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */


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
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  seq: " << seq << std::endl;
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << seq;                           // offset: 4
        map << target_system;                 // offset: 8
        map << target_component;              // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> seq;                           // offset: 4
        map >> target_system;                 // offset: 8
        map >> target_component;              // offset: 9
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
