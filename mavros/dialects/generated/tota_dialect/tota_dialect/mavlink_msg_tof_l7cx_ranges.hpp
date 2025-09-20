// MESSAGE TOF_L7CX_RANGES support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief TOF_L7CX_RANGES message
 *
 * 8×8 zone distances from VL53L7CX in millimetres.
 */
struct TOF_L7CX_RANGES : mavlink::Message {
    static constexpr msgid_t MSG_ID = 42001;
    static constexpr size_t LENGTH = 132;
    static constexpr size_t MIN_LENGTH = 132;
    static constexpr uint8_t CRC_EXTRA = 173;
    static constexpr auto NAME = "TOF_L7CX_RANGES";


    uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    std::array<uint16_t, 64> range_mm; /*< [mm] 8x8 zone distances array */


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
        ss << "  range_mm: [" << to_string(range_mm) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_us;                       // offset: 0
        map << range_mm;                      // offset: 4
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_us;                       // offset: 0
        map >> range_mm;                      // offset: 4
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
