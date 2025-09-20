// MESSAGE ARMING_STATUS support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief ARMING_STATUS message
 *
 * Vehicle arming status and safety state
 */
struct ARMING_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 104;
    static constexpr size_t LENGTH = 5;
    static constexpr size_t MIN_LENGTH = 5;
    static constexpr uint8_t CRC_EXTRA = 61;
    static constexpr auto NAME = "ARMING_STATUS";


    uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    uint8_t armed; /*<  Armed state (0=disarmed, 1=armed) */


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
        ss << "  armed: " << +armed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << armed;                         // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> armed;                         // offset: 4
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
