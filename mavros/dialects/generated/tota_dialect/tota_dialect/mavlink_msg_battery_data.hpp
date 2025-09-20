// MESSAGE BATTERY_DATA support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief BATTERY_DATA message
 *
 * Battery status and monitoring data
 */
struct BATTERY_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 102;
    static constexpr size_t LENGTH = 41;
    static constexpr size_t MIN_LENGTH = 41;
    static constexpr uint8_t CRC_EXTRA = 2;
    static constexpr auto NAME = "BATTERY_DATA";


    uint8_t id; /*<  Battery ID */
    uint8_t battery_function; /*<  Function of the battery */
    uint8_t type; /*<  Type of battery */
    int16_t temperature; /*< [cdegC] Temperature of battery in centi-degrees Celsius */
    std::array<uint16_t, 10> voltages; /*< [mV] Individual cell voltages in millivolts */
    int16_t current_battery; /*< [cA] Battery current in centiamperes, -1: not available */
    int32_t current_consumed; /*< [mAh] Consumed charge in milliampere hours, -1: not available */
    int32_t energy_consumed; /*< [hJ] Consumed energy in hectojoules, -1: not available */
    int8_t battery_remaining; /*< [%] Remaining battery percentage, -1: not available */
    int32_t time_remaining; /*< [s] Remaining battery time in seconds, 0: not available */
    uint8_t charge_state; /*<  Battery charge state */


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
        ss << "  id: " << +id << std::endl;
        ss << "  battery_function: " << +battery_function << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  temperature: " << temperature << std::endl;
        ss << "  voltages: [" << to_string(voltages) << "]" << std::endl;
        ss << "  current_battery: " << current_battery << std::endl;
        ss << "  current_consumed: " << current_consumed << std::endl;
        ss << "  energy_consumed: " << energy_consumed << std::endl;
        ss << "  battery_remaining: " << +battery_remaining << std::endl;
        ss << "  time_remaining: " << time_remaining << std::endl;
        ss << "  charge_state: " << +charge_state << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << current_consumed;              // offset: 0
        map << energy_consumed;               // offset: 4
        map << time_remaining;                // offset: 8
        map << temperature;                   // offset: 12
        map << voltages;                      // offset: 14
        map << current_battery;               // offset: 34
        map << id;                            // offset: 36
        map << battery_function;              // offset: 37
        map << type;                          // offset: 38
        map << battery_remaining;             // offset: 39
        map << charge_state;                  // offset: 40
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> current_consumed;              // offset: 0
        map >> energy_consumed;               // offset: 4
        map >> time_remaining;                // offset: 8
        map >> temperature;                   // offset: 12
        map >> voltages;                      // offset: 14
        map >> current_battery;               // offset: 34
        map >> id;                            // offset: 36
        map >> battery_function;              // offset: 37
        map >> type;                          // offset: 38
        map >> battery_remaining;             // offset: 39
        map >> charge_state;                  // offset: 40
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
