// MESSAGE DRIVE_STATUS support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief DRIVE_STATUS message
 *
 * Drive system status including wheel RPM and jumping mechanism state
 */
struct DRIVE_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 105;
    static constexpr size_t LENGTH = 10;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 188;
    static constexpr auto NAME = "DRIVE_STATUS";


    uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    uint8_t arm_disarm; /*<  Arm/disarm state (0=disarmed, 1=armed) */
    int16_t rpm_left_wheel; /*< [rpm] Left wheel RPM */
    int16_t rpm_right_wheel; /*< [rpm] Right wheel RPM */
    uint8_t jumping; /*<  Jumping mechanism state */


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
        ss << "  arm_disarm: " << +arm_disarm << std::endl;
        ss << "  rpm_left_wheel: " << rpm_left_wheel << std::endl;
        ss << "  rpm_right_wheel: " << rpm_right_wheel << std::endl;
        ss << "  jumping: " << +jumping << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << rpm_left_wheel;                // offset: 4
        map << rpm_right_wheel;               // offset: 6
        map << arm_disarm;                    // offset: 8
        map << jumping;                       // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> rpm_left_wheel;                // offset: 4
        map >> rpm_right_wheel;               // offset: 6
        map >> arm_disarm;                    // offset: 8
        map >> jumping;                       // offset: 9
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
