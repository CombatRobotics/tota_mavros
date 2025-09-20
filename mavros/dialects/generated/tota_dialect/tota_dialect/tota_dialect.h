/** @file
 *  @brief MAVLink comm protocol generated from tota_dialect.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_TOTA_DIALECT_H
#define MAVLINK_TOTA_DIALECT_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_TOTA_DIALECT.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_TOTA_DIALECT_XML_HASH 4685251723866594382

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {4, 40, 10, 10, 3, 8, 9}, {100, 225, 28, 28, 0, 0, 0}, {101, 28, 46, 46, 0, 0, 0}, {102, 2, 41, 41, 0, 0, 0}, {103, 155, 8, 8, 0, 0, 0}, {104, 61, 5, 5, 0, 0, 0}, {105, 188, 10, 10, 0, 0, 0}, {42001, 173, 132, 132, 0, 0, 0}, {42003, 93, 14, 14, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_TOTA_DIALECT

// ENUM DEFINITIONS


/** @brief MAVLINK component type reported in HEARTBEAT message */
#ifndef HAVE_ENUM_MAV_TYPE
#define HAVE_ENUM_MAV_TYPE
typedef enum MAV_TYPE
{
   MAV_TYPE_GENERIC=0, /* Generic micro air vehicle | */
   MAV_TYPE_FIXED_WING=1, /* Fixed wing aircraft | */
   MAV_TYPE_QUADROTOR=2, /* Quadrotor | */
   MAV_TYPE_GROUND_ROVER=10, /* Ground rover | */
   MAV_TYPE_HEXAROTOR=12, /* Hexarotor | */
   MAV_TYPE_OCTOROTOR=13, /* Octorotor | */
   MAV_TYPE_COAXIAL=18, /* Coaxial helicopter | */
   MAV_TYPE_ENUM_END=19, /*  | */
} MAV_TYPE;
#endif

/** @brief Micro air vehicle / autopilot class */
#ifndef HAVE_ENUM_MAV_AUTOPILOT
#define HAVE_ENUM_MAV_AUTOPILOT
typedef enum MAV_AUTOPILOT
{
   MAV_AUTOPILOT_GENERIC=0, /* Generic autopilot, full support for everything | */
   MAV_AUTOPILOT_APM=3, /* APM autopilot | */
   MAV_AUTOPILOT_PX4=12, /* PX4 autopilot | */
   MAV_AUTOPILOT_ENUM_END=13, /*  | */
} MAV_AUTOPILOT;
#endif

/** @brief These flags encode the MAV mode */
#ifndef HAVE_ENUM_MAV_MODE_FLAG
#define HAVE_ENUM_MAV_MODE_FLAG
typedef enum MAV_MODE_FLAG
{
   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, /* 0b00000001 Reserved for future use | */
   MAV_MODE_FLAG_TEST_ENABLED=2, /* 0b00000010 system has a test mode enabled | */
   MAV_MODE_FLAG_AUTO_ENABLED=4, /* 0b00000100 autonomous mode enabled | */
   MAV_MODE_FLAG_GUIDED_ENABLED=8, /* 0b00001000 guided mode enabled | */
   MAV_MODE_FLAG_STABILIZE_ENABLED=16, /* 0b00010000 stabilize mode enabled | */
   MAV_MODE_FLAG_HIL_ENABLED=32, /* 0b00100000 hardware in the loop simulation | */
   MAV_MODE_FLAG_MANUAL_INPUT_ENABLED=64, /* 0b01000000 remote control input is enabled | */
   MAV_MODE_FLAG_SAFETY_ARMED=128, /* 0b10000000 MAV safety set to armed | */
   MAV_MODE_FLAG_ENUM_END=129, /*  | */
} MAV_MODE_FLAG;
#endif

/** @brief System status flag */
#ifndef HAVE_ENUM_MAV_STATE
#define HAVE_ENUM_MAV_STATE
typedef enum MAV_STATE
{
   MAV_STATE_UNINIT=0, /* Uninitialized system, state is unknown | */
   MAV_STATE_BOOT=1, /* System is booting up | */
   MAV_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready | */
   MAV_STATE_STANDBY=3, /* System is grounded and on standby | */
   MAV_STATE_ACTIVE=4, /* System is active and might be already airborne | */
   MAV_STATE_CRITICAL=5, /* System is in a non-normal flight mode | */
   MAV_STATE_EMERGENCY=6, /* System is in an emergency state | */
   MAV_STATE_POWEROFF=7, /* System is about to shut-down | */
   MAV_STATE_ENUM_END=8, /*  | */
} MAV_STATE;
#endif

/** @brief Enumeration of battery functions */
#ifndef HAVE_ENUM_MAV_BATTERY_FUNCTION
#define HAVE_ENUM_MAV_BATTERY_FUNCTION
typedef enum MAV_BATTERY_FUNCTION
{
   MAV_BATTERY_FUNCTION_UNKNOWN=0, /* Battery function is unknown | */
   MAV_BATTERY_FUNCTION_ALL=1, /* Battery provides power to all systems | */
   MAV_BATTERY_FUNCTION_PROPULSION=2, /* Battery for the propulsion system | */
   MAV_BATTERY_FUNCTION_AVIONICS=3, /* Avionics battery | */
   MAV_BATTERY_FUNCTION_ENUM_END=4, /*  | */
} MAV_BATTERY_FUNCTION;
#endif

/** @brief Enumeration of battery types */
#ifndef HAVE_ENUM_MAV_BATTERY_TYPE
#define HAVE_ENUM_MAV_BATTERY_TYPE
typedef enum MAV_BATTERY_TYPE
{
   MAV_BATTERY_TYPE_UNKNOWN=0, /* Not specified | */
   MAV_BATTERY_TYPE_LIPO=1, /* Lithium polymer battery | */
   MAV_BATTERY_TYPE_LIFE=2, /* Lithium-iron-phosphate battery | */
   MAV_BATTERY_TYPE_LION=3, /* Lithium-ION battery | */
   MAV_BATTERY_TYPE_NIMH=4, /* Nickel metal hydride battery | */
   MAV_BATTERY_TYPE_ENUM_END=5, /*  | */
} MAV_BATTERY_TYPE;
#endif

/** @brief Enumeration for battery charge states */
#ifndef HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
#define HAVE_ENUM_MAV_BATTERY_CHARGE_STATE
typedef enum MAV_BATTERY_CHARGE_STATE
{
   MAV_BATTERY_CHARGE_STATE_UNDEFINED=0, /* Low battery state is not provided | */
   MAV_BATTERY_CHARGE_STATE_OK=1, /* Battery is not in low state | */
   MAV_BATTERY_CHARGE_STATE_LOW=2, /* Battery state is low | */
   MAV_BATTERY_CHARGE_STATE_CRITICAL=3, /* Battery state is critical | */
   MAV_BATTERY_CHARGE_STATE_EMERGENCY=4, /* Battery state is emergency | */
   MAV_BATTERY_CHARGE_STATE_FAILED=5, /* Battery is not present | */
   MAV_BATTERY_CHARGE_STATE_UNHEALTHY=6, /* Battery is unhealthy | */
   MAV_BATTERY_CHARGE_STATE_CHARGING=7, /* Battery is charging | */
   MAV_BATTERY_CHARGE_STATE_ENUM_END=8, /*  | */
} MAV_BATTERY_CHARGE_STATE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_attitude_data.h"
#include "./mavlink_msg_gps_data.h"
#include "./mavlink_msg_battery_data.h"
#include "./mavlink_msg_ping.h"
#include "./mavlink_msg_wheel_encoders.h"
#include "./mavlink_msg_tof_l7cx_ranges.h"
#include "./mavlink_msg_sensor_init_state.h"
#include "./mavlink_msg_arming_status.h"
#include "./mavlink_msg_drive_status.h"

// base include



#if MAVLINK_TOTA_DIALECT_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_ATTITUDE_DATA, MAVLINK_MESSAGE_INFO_GPS_DATA, MAVLINK_MESSAGE_INFO_BATTERY_DATA, MAVLINK_MESSAGE_INFO_SENSOR_INIT_STATE, MAVLINK_MESSAGE_INFO_ARMING_STATUS, MAVLINK_MESSAGE_INFO_DRIVE_STATUS, MAVLINK_MESSAGE_INFO_TOF_L7CX_RANGES, MAVLINK_MESSAGE_INFO_WHEEL_ENCODERS}
# define MAVLINK_MESSAGE_NAMES {{ "ARMING_STATUS", 104 }, { "ATTITUDE_DATA", 100 }, { "BATTERY_DATA", 102 }, { "DRIVE_STATUS", 105 }, { "GPS_DATA", 101 }, { "HEARTBEAT", 0 }, { "PING", 4 }, { "SENSOR_INIT_STATE", 103 }, { "TOF_L7CX_RANGES", 42001 }, { "WHEEL_ENCODERS", 42003 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_TOTA_DIALECT_H
