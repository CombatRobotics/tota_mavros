// MESSAGE WHEEL_ENCODERS support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief WHEEL_ENCODERS message
 *
 * Cumulative wheel ticks (CW = +, CCW = –) and the absolute 12-bit AS5600 CAM position used by the jumping mechanism.
 */
struct WHEEL_ENCODERS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 42003;
    static constexpr size_t LENGTH = 14;
    static constexpr size_t MIN_LENGTH = 14;
    static constexpr uint8_t CRC_EXTRA = 93;
    static constexpr auto NAME = "WHEEL_ENCODERS";


    uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    int32_t left_ticks; /*<  Left wheel encoder ticks */
    int32_t right_ticks; /*<  Right wheel encoder ticks */
    uint16_t cam_pos; /*<  12-bit AS5600 CAM position */


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
        ss << "  left_ticks: " << left_ticks << std::endl;
        ss << "  right_ticks: " << right_ticks << std::endl;
        ss << "  cam_pos: " << cam_pos << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << left_ticks;                    // offset: 4
        map << right_ticks;                   // offset: 8
        map << cam_pos;                       // offset: 12
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> left_ticks;                    // offset: 4
        map >> right_ticks;                   // offset: 8
        map >> cam_pos;                       // offset: 12
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
