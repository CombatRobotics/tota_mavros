#pragma once
// MESSAGE BATTERY_DATA PACKING

#define MAVLINK_MSG_ID_BATTERY_DATA 102


typedef struct __mavlink_battery_data_t {
 int32_t current_consumed; /*< [mAh] Consumed charge in milliampere hours, -1: not available*/
 int32_t energy_consumed; /*< [hJ] Consumed energy in hectojoules, -1: not available*/
 int32_t time_remaining; /*< [s] Remaining battery time in seconds, 0: not available*/
 int16_t temperature; /*< [cdegC] Temperature of battery in centi-degrees Celsius*/
 uint16_t voltages[10]; /*< [mV] Individual cell voltages in millivolts*/
 int16_t current_battery; /*< [cA] Battery current in centiamperes, -1: not available*/
 uint8_t id; /*<  Battery ID*/
 uint8_t battery_function; /*<  Function of the battery*/
 uint8_t type; /*<  Type of battery*/
 int8_t battery_remaining; /*< [%] Remaining battery percentage, -1: not available*/
 uint8_t charge_state; /*<  Battery charge state*/
} mavlink_battery_data_t;

#define MAVLINK_MSG_ID_BATTERY_DATA_LEN 41
#define MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN 41
#define MAVLINK_MSG_ID_102_LEN 41
#define MAVLINK_MSG_ID_102_MIN_LEN 41

#define MAVLINK_MSG_ID_BATTERY_DATA_CRC 2
#define MAVLINK_MSG_ID_102_CRC 2

#define MAVLINK_MSG_BATTERY_DATA_FIELD_VOLTAGES_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_DATA { \
    102, \
    "BATTERY_DATA", \
    11, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_battery_data_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_battery_data_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_battery_data_t, type) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_battery_data_t, temperature) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT16_T, 10, 14, offsetof(mavlink_battery_data_t, voltages) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_battery_data_t, current_battery) }, \
         { "current_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_battery_data_t, current_consumed) }, \
         { "energy_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_battery_data_t, energy_consumed) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_battery_data_t, battery_remaining) }, \
         { "time_remaining", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_battery_data_t, time_remaining) }, \
         { "charge_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_battery_data_t, charge_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_DATA { \
    "BATTERY_DATA", \
    11, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_battery_data_t, id) }, \
         { "battery_function", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_battery_data_t, battery_function) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_battery_data_t, type) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_battery_data_t, temperature) }, \
         { "voltages", NULL, MAVLINK_TYPE_UINT16_T, 10, 14, offsetof(mavlink_battery_data_t, voltages) }, \
         { "current_battery", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_battery_data_t, current_battery) }, \
         { "current_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_battery_data_t, current_consumed) }, \
         { "energy_consumed", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_battery_data_t, energy_consumed) }, \
         { "battery_remaining", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_battery_data_t, battery_remaining) }, \
         { "time_remaining", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_battery_data_t, time_remaining) }, \
         { "charge_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_battery_data_t, charge_state) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type of battery
 * @param temperature [cdegC] Temperature of battery in centi-degrees Celsius
 * @param voltages [mV] Individual cell voltages in millivolts
 * @param current_battery [cA] Battery current in centiamperes, -1: not available
 * @param current_consumed [mAh] Consumed charge in milliampere hours, -1: not available
 * @param energy_consumed [hJ] Consumed energy in hectojoules, -1: not available
 * @param battery_remaining [%] Remaining battery percentage, -1: not available
 * @param time_remaining [s] Remaining battery time in seconds, 0: not available
 * @param charge_state  Battery charge state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int32_t(buf, 8, time_remaining);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 34, current_battery);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, battery_function);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_int8_t(buf, 39, battery_remaining);
    _mav_put_uint8_t(buf, 40, charge_state);
    _mav_put_uint16_t_array(buf, 14, voltages, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.time_remaining = time_remaining;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.charge_state = charge_state;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
}

/**
 * @brief Pack a battery_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type of battery
 * @param temperature [cdegC] Temperature of battery in centi-degrees Celsius
 * @param voltages [mV] Individual cell voltages in millivolts
 * @param current_battery [cA] Battery current in centiamperes, -1: not available
 * @param current_consumed [mAh] Consumed charge in milliampere hours, -1: not available
 * @param energy_consumed [hJ] Consumed energy in hectojoules, -1: not available
 * @param battery_remaining [%] Remaining battery percentage, -1: not available
 * @param time_remaining [s] Remaining battery time in seconds, 0: not available
 * @param charge_state  Battery charge state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int32_t(buf, 8, time_remaining);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 34, current_battery);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, battery_function);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_int8_t(buf, 39, battery_remaining);
    _mav_put_uint8_t(buf, 40, charge_state);
    _mav_put_uint16_t_array(buf, 14, voltages, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.time_remaining = time_remaining;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.charge_state = charge_state;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif
}

/**
 * @brief Pack a battery_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type of battery
 * @param temperature [cdegC] Temperature of battery in centi-degrees Celsius
 * @param voltages [mV] Individual cell voltages in millivolts
 * @param current_battery [cA] Battery current in centiamperes, -1: not available
 * @param current_consumed [mAh] Consumed charge in milliampere hours, -1: not available
 * @param energy_consumed [hJ] Consumed energy in hectojoules, -1: not available
 * @param battery_remaining [%] Remaining battery percentage, -1: not available
 * @param time_remaining [s] Remaining battery time in seconds, 0: not available
 * @param charge_state  Battery charge state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint8_t battery_function,uint8_t type,int16_t temperature,const uint16_t *voltages,int16_t current_battery,int32_t current_consumed,int32_t energy_consumed,int8_t battery_remaining,int32_t time_remaining,uint8_t charge_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int32_t(buf, 8, time_remaining);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 34, current_battery);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, battery_function);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_int8_t(buf, 39, battery_remaining);
    _mav_put_uint8_t(buf, 40, charge_state);
    _mav_put_uint16_t_array(buf, 14, voltages, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#else
    mavlink_battery_data_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.time_remaining = time_remaining;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.charge_state = charge_state;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
}

/**
 * @brief Encode a battery_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack(system_id, component_id, msg, battery_data->id, battery_data->battery_function, battery_data->type, battery_data->temperature, battery_data->voltages, battery_data->current_battery, battery_data->current_consumed, battery_data->energy_consumed, battery_data->battery_remaining, battery_data->time_remaining, battery_data->charge_state);
}

/**
 * @brief Encode a battery_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack_chan(system_id, component_id, chan, msg, battery_data->id, battery_data->battery_function, battery_data->type, battery_data->temperature, battery_data->voltages, battery_data->current_battery, battery_data->current_consumed, battery_data->energy_consumed, battery_data->battery_remaining, battery_data->time_remaining, battery_data->charge_state);
}

/**
 * @brief Encode a battery_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param battery_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_battery_data_t* battery_data)
{
    return mavlink_msg_battery_data_pack_status(system_id, component_id, _status, msg,  battery_data->id, battery_data->battery_function, battery_data->type, battery_data->temperature, battery_data->voltages, battery_data->current_battery, battery_data->current_consumed, battery_data->energy_consumed, battery_data->battery_remaining, battery_data->time_remaining, battery_data->charge_state);
}

/**
 * @brief Send a battery_data message
 * @param chan MAVLink channel to send the message
 *
 * @param id  Battery ID
 * @param battery_function  Function of the battery
 * @param type  Type of battery
 * @param temperature [cdegC] Temperature of battery in centi-degrees Celsius
 * @param voltages [mV] Individual cell voltages in millivolts
 * @param current_battery [cA] Battery current in centiamperes, -1: not available
 * @param current_consumed [mAh] Consumed charge in milliampere hours, -1: not available
 * @param energy_consumed [hJ] Consumed energy in hectojoules, -1: not available
 * @param battery_remaining [%] Remaining battery percentage, -1: not available
 * @param time_remaining [s] Remaining battery time in seconds, 0: not available
 * @param charge_state  Battery charge state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_data_send(mavlink_channel_t chan, uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_DATA_LEN];
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int32_t(buf, 8, time_remaining);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 34, current_battery);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, battery_function);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_int8_t(buf, 39, battery_remaining);
    _mav_put_uint8_t(buf, 40, charge_state);
    _mav_put_uint16_t_array(buf, 14, voltages, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, buf, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    mavlink_battery_data_t packet;
    packet.current_consumed = current_consumed;
    packet.energy_consumed = energy_consumed;
    packet.time_remaining = time_remaining;
    packet.temperature = temperature;
    packet.current_battery = current_battery;
    packet.id = id;
    packet.battery_function = battery_function;
    packet.type = type;
    packet.battery_remaining = battery_remaining;
    packet.charge_state = charge_state;
    mav_array_memcpy(packet.voltages, voltages, sizeof(uint16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}

/**
 * @brief Send a battery_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_data_send_struct(mavlink_channel_t chan, const mavlink_battery_data_t* battery_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_data_send(chan, battery_data->id, battery_data->battery_function, battery_data->type, battery_data->temperature, battery_data->voltages, battery_data->current_battery, battery_data->current_consumed, battery_data->energy_consumed, battery_data->battery_remaining, battery_data->time_remaining, battery_data->charge_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)battery_data, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint8_t battery_function, uint8_t type, int16_t temperature, const uint16_t *voltages, int16_t current_battery, int32_t current_consumed, int32_t energy_consumed, int8_t battery_remaining, int32_t time_remaining, uint8_t charge_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, current_consumed);
    _mav_put_int32_t(buf, 4, energy_consumed);
    _mav_put_int32_t(buf, 8, time_remaining);
    _mav_put_int16_t(buf, 12, temperature);
    _mav_put_int16_t(buf, 34, current_battery);
    _mav_put_uint8_t(buf, 36, id);
    _mav_put_uint8_t(buf, 37, battery_function);
    _mav_put_uint8_t(buf, 38, type);
    _mav_put_int8_t(buf, 39, battery_remaining);
    _mav_put_uint8_t(buf, 40, charge_state);
    _mav_put_uint16_t_array(buf, 14, voltages, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, buf, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#else
    mavlink_battery_data_t *packet = (mavlink_battery_data_t *)msgbuf;
    packet->current_consumed = current_consumed;
    packet->energy_consumed = energy_consumed;
    packet->time_remaining = time_remaining;
    packet->temperature = temperature;
    packet->current_battery = current_battery;
    packet->id = id;
    packet->battery_function = battery_function;
    packet->type = type;
    packet->battery_remaining = battery_remaining;
    packet->charge_state = charge_state;
    mav_array_memcpy(packet->voltages, voltages, sizeof(uint16_t)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_DATA, (const char *)packet, MAVLINK_MSG_ID_BATTERY_DATA_MIN_LEN, MAVLINK_MSG_ID_BATTERY_DATA_LEN, MAVLINK_MSG_ID_BATTERY_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_DATA UNPACKING


/**
 * @brief Get field id from battery_data message
 *
 * @return  Battery ID
 */
static inline uint8_t mavlink_msg_battery_data_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field battery_function from battery_data message
 *
 * @return  Function of the battery
 */
static inline uint8_t mavlink_msg_battery_data_get_battery_function(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field type from battery_data message
 *
 * @return  Type of battery
 */
static inline uint8_t mavlink_msg_battery_data_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field temperature from battery_data message
 *
 * @return [cdegC] Temperature of battery in centi-degrees Celsius
 */
static inline int16_t mavlink_msg_battery_data_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field voltages from battery_data message
 *
 * @return [mV] Individual cell voltages in millivolts
 */
static inline uint16_t mavlink_msg_battery_data_get_voltages(const mavlink_message_t* msg, uint16_t *voltages)
{
    return _MAV_RETURN_uint16_t_array(msg, voltages, 10,  14);
}

/**
 * @brief Get field current_battery from battery_data message
 *
 * @return [cA] Battery current in centiamperes, -1: not available
 */
static inline int16_t mavlink_msg_battery_data_get_current_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field current_consumed from battery_data message
 *
 * @return [mAh] Consumed charge in milliampere hours, -1: not available
 */
static inline int32_t mavlink_msg_battery_data_get_current_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field energy_consumed from battery_data message
 *
 * @return [hJ] Consumed energy in hectojoules, -1: not available
 */
static inline int32_t mavlink_msg_battery_data_get_energy_consumed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field battery_remaining from battery_data message
 *
 * @return [%] Remaining battery percentage, -1: not available
 */
static inline int8_t mavlink_msg_battery_data_get_battery_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  39);
}

/**
 * @brief Get field time_remaining from battery_data message
 *
 * @return [s] Remaining battery time in seconds, 0: not available
 */
static inline int32_t mavlink_msg_battery_data_get_time_remaining(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field charge_state from battery_data message
 *
 * @return  Battery charge state
 */
static inline uint8_t mavlink_msg_battery_data_get_charge_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Decode a battery_data message into a struct
 *
 * @param msg The message to decode
 * @param battery_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_data_decode(const mavlink_message_t* msg, mavlink_battery_data_t* battery_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_data->current_consumed = mavlink_msg_battery_data_get_current_consumed(msg);
    battery_data->energy_consumed = mavlink_msg_battery_data_get_energy_consumed(msg);
    battery_data->time_remaining = mavlink_msg_battery_data_get_time_remaining(msg);
    battery_data->temperature = mavlink_msg_battery_data_get_temperature(msg);
    mavlink_msg_battery_data_get_voltages(msg, battery_data->voltages);
    battery_data->current_battery = mavlink_msg_battery_data_get_current_battery(msg);
    battery_data->id = mavlink_msg_battery_data_get_id(msg);
    battery_data->battery_function = mavlink_msg_battery_data_get_battery_function(msg);
    battery_data->type = mavlink_msg_battery_data_get_type(msg);
    battery_data->battery_remaining = mavlink_msg_battery_data_get_battery_remaining(msg);
    battery_data->charge_state = mavlink_msg_battery_data_get_charge_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_DATA_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_DATA_LEN;
        memset(battery_data, 0, MAVLINK_MSG_ID_BATTERY_DATA_LEN);
    memcpy(battery_data, _MAV_PAYLOAD(msg), len);
#endif
}
