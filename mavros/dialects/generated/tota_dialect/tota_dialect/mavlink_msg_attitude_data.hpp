// MESSAGE ATTITUDE_DATA support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief ATTITUDE_DATA message
 *
 * Vehicle attitude and angular velocity data
 */
struct ATTITUDE_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 100;
    static constexpr size_t LENGTH = 28;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 225;
    static constexpr auto NAME = "ATTITUDE_DATA";


    uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    float roll; /*< [rad] Roll angle */
    float pitch; /*< [rad] Pitch angle */
    float yaw; /*< [rad] Yaw angle */
    float rollspeed; /*< [rad/s] Roll angular speed */
    float pitchspeed; /*< [rad/s] Pitch angular speed */
    float yawspeed; /*< [rad/s] Yaw angular speed */


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
        ss << "  roll: " << roll << std::endl;
        ss << "  pitch: " << pitch << std::endl;
        ss << "  yaw: " << yaw << std::endl;
        ss << "  rollspeed: " << rollspeed << std::endl;
        ss << "  pitchspeed: " << pitchspeed << std::endl;
        ss << "  yawspeed: " << yawspeed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << roll;                          // offset: 4
        map << pitch;                         // offset: 8
        map << yaw;                           // offset: 12
        map << rollspeed;                     // offset: 16
        map << pitchspeed;                    // offset: 20
        map << yawspeed;                      // offset: 24
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> roll;                          // offset: 4
        map >> pitch;                         // offset: 8
        map >> yaw;                           // offset: 12
        map >> rollspeed;                     // offset: 16
        map >> pitchspeed;                    // offset: 20
        map >> yawspeed;                      // offset: 24
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
