/** @file
 *	@brief MAVLink comm protocol generated from tota_dialect.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace tota_dialect {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 10> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {4, 40, 10, 10, 3, 8, 9}, {100, 225, 28, 28, 0, 0, 0}, {101, 28, 46, 46, 0, 0, 0}, {102, 2, 41, 41, 0, 0, 0}, {103, 155, 8, 8, 0, 0, 0}, {104, 61, 5, 5, 0, 0, 0}, {105, 188, 10, 10, 0, 0, 0}, {42001, 173, 132, 132, 0, 0, 0}, {42003, 93, 14, 14, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief MAVLINK component type reported in HEARTBEAT message */
enum class MAV_TYPE : uint8_t
{
    GENERIC=0, /* Generic micro air vehicle | */
    FIXED_WING=1, /* Fixed wing aircraft | */
    QUADROTOR=2, /* Quadrotor | */
    GROUND_ROVER=10, /* Ground rover | */
    HEXAROTOR=12, /* Hexarotor | */
    OCTOROTOR=13, /* Octorotor | */
    COAXIAL=18, /* Coaxial helicopter | */
};

//! MAV_TYPE ENUM_END
constexpr auto MAV_TYPE_ENUM_END = 19;

/** @brief Micro air vehicle / autopilot class */
enum class MAV_AUTOPILOT : uint8_t
{
    GENERIC=0, /* Generic autopilot, full support for everything | */
    APM=3, /* APM autopilot | */
    PX4=12, /* PX4 autopilot | */
};

//! MAV_AUTOPILOT ENUM_END
constexpr auto MAV_AUTOPILOT_ENUM_END = 13;

/** @brief These flags encode the MAV mode */
enum class MAV_MODE_FLAG : uint8_t
{
    CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use | */
    TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled | */
    AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled | */
    GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled | */
    STABILIZE_ENABLED=16, /* 0b00010000 stabilize mode enabled | */
    HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation | */
    MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled | */
    SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed | */
};

//! MAV_MODE_FLAG ENUM_END
constexpr auto MAV_MODE_FLAG_ENUM_END = 129;

/** @brief System status flag */
enum class MAV_STATE : uint8_t
{
    UNINIT=0, /* Uninitialized system, state is unknown | */
    BOOT=1, /* System is booting up | */
    CALIBRATING=2, /* System is calibrating and not flight-ready | */
    STANDBY=3, /* System is grounded and on standby | */
    ACTIVE=4, /* System is active and might be already airborne | */
    CRITICAL=5, /* System is in a non-normal flight mode | */
    EMERGENCY=6, /* System is in an emergency state | */
    POWEROFF=7, /* System is about to shut-down | */
};

//! MAV_STATE ENUM_END
constexpr auto MAV_STATE_ENUM_END = 8;

/** @brief Enumeration of battery functions */
enum class MAV_BATTERY_FUNCTION : uint8_t
{
    UNKNOWN=0, /* Battery function is unknown | */
    ALL=1, /* Battery provides power to all systems | */
    PROPULSION=2, /* Battery for the propulsion system | */
    AVIONICS=3, /* Avionics battery | */
};

//! MAV_BATTERY_FUNCTION ENUM_END
constexpr auto MAV_BATTERY_FUNCTION_ENUM_END = 4;

/** @brief Enumeration of battery types */
enum class MAV_BATTERY_TYPE : uint8_t
{
    UNKNOWN=0, /* Not specified | */
    LIPO=1, /* Lithium polymer battery | */
    LIFE=2, /* Lithium-iron-phosphate battery | */
    LION=3, /* Lithium-ION battery | */
    NIMH=4, /* Nickel metal hydride battery | */
};

//! MAV_BATTERY_TYPE ENUM_END
constexpr auto MAV_BATTERY_TYPE_ENUM_END = 5;

/** @brief Enumeration for battery charge states */
enum class MAV_BATTERY_CHARGE_STATE : uint8_t
{
    UNDEFINED=0, /* Low battery state is not provided | */
    OK=1, /* Battery is not in low state | */
    LOW=2, /* Battery state is low | */
    CRITICAL=3, /* Battery state is critical | */
    EMERGENCY=4, /* Battery state is emergency | */
    FAILED=5, /* Battery is not present | */
    UNHEALTHY=6, /* Battery is unhealthy | */
    CHARGING=7, /* Battery is charging | */
};

//! MAV_BATTERY_CHARGE_STATE ENUM_END
constexpr auto MAV_BATTERY_CHARGE_STATE_ENUM_END = 8;


} // namespace tota_dialect
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.hpp"
#include "./mavlink_msg_attitude_data.hpp"
#include "./mavlink_msg_gps_data.hpp"
#include "./mavlink_msg_battery_data.hpp"
#include "./mavlink_msg_ping.hpp"
#include "./mavlink_msg_wheel_encoders.hpp"
#include "./mavlink_msg_tof_l7cx_ranges.hpp"
#include "./mavlink_msg_sensor_init_state.hpp"
#include "./mavlink_msg_arming_status.hpp"
#include "./mavlink_msg_drive_status.hpp"

// base include

