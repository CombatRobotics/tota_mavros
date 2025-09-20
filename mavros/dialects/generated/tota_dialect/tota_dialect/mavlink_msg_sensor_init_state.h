#pragma once
// MESSAGE SENSOR_INIT_STATE PACKING

#define MAVLINK_MSG_ID_SENSOR_INIT_STATE 103


typedef struct __mavlink_sensor_init_state_t {
 uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 uint32_t sensor_init; /*<  Bitfield indicating initialization status of sensors*/
} mavlink_sensor_init_state_t;

#define MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN 8
#define MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN 8
#define MAVLINK_MSG_ID_103_LEN 8
#define MAVLINK_MSG_ID_103_MIN_LEN 8

#define MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC 155
#define MAVLINK_MSG_ID_103_CRC 155



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENSOR_INIT_STATE { \
    103, \
    "SENSOR_INIT_STATE", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sensor_init_state_t, time_us) }, \
         { "sensor_init", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sensor_init_state_t, sensor_init) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENSOR_INIT_STATE { \
    "SENSOR_INIT_STATE", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_sensor_init_state_t, time_us) }, \
         { "sensor_init", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_sensor_init_state_t, sensor_init) }, \
         } \
}
#endif

/**
 * @brief Pack a sensor_init_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param sensor_init  Bitfield indicating initialization status of sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_init_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_us, uint32_t sensor_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint32_t(buf, 4, sensor_init);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#else
    mavlink_sensor_init_state_t packet;
    packet.time_us = time_us;
    packet.sensor_init = sensor_init;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_INIT_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
}

/**
 * @brief Pack a sensor_init_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param sensor_init  Bitfield indicating initialization status of sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_init_state_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_us, uint32_t sensor_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint32_t(buf, 4, sensor_init);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#else
    mavlink_sensor_init_state_t packet;
    packet.time_us = time_us;
    packet.sensor_init = sensor_init;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_INIT_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#endif
}

/**
 * @brief Pack a sensor_init_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param sensor_init  Bitfield indicating initialization status of sensors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_init_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_us,uint32_t sensor_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint32_t(buf, 4, sensor_init);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#else
    mavlink_sensor_init_state_t packet;
    packet.time_us = time_us;
    packet.sensor_init = sensor_init;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENSOR_INIT_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
}

/**
 * @brief Encode a sensor_init_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor_init_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_init_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_init_state_t* sensor_init_state)
{
    return mavlink_msg_sensor_init_state_pack(system_id, component_id, msg, sensor_init_state->time_us, sensor_init_state->sensor_init);
}

/**
 * @brief Encode a sensor_init_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor_init_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_init_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_init_state_t* sensor_init_state)
{
    return mavlink_msg_sensor_init_state_pack_chan(system_id, component_id, chan, msg, sensor_init_state->time_us, sensor_init_state->sensor_init);
}

/**
 * @brief Encode a sensor_init_state struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sensor_init_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_init_state_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sensor_init_state_t* sensor_init_state)
{
    return mavlink_msg_sensor_init_state_pack_status(system_id, component_id, _status, msg,  sensor_init_state->time_us, sensor_init_state->sensor_init);
}

/**
 * @brief Send a sensor_init_state message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param sensor_init  Bitfield indicating initialization status of sensors
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_init_state_send(mavlink_channel_t chan, uint32_t time_us, uint32_t sensor_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint32_t(buf, 4, sensor_init);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE, buf, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#else
    mavlink_sensor_init_state_t packet;
    packet.time_us = time_us;
    packet.sensor_init = sensor_init;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#endif
}

/**
 * @brief Send a sensor_init_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sensor_init_state_send_struct(mavlink_channel_t chan, const mavlink_sensor_init_state_t* sensor_init_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sensor_init_state_send(chan, sensor_init_state->time_us, sensor_init_state->sensor_init);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE, (const char *)sensor_init_state, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_init_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_us, uint32_t sensor_init)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint32_t(buf, 4, sensor_init);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE, buf, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#else
    mavlink_sensor_init_state_t *packet = (mavlink_sensor_init_state_t *)msgbuf;
    packet->time_us = time_us;
    packet->sensor_init = sensor_init;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR_INIT_STATE, (const char *)packet, MAVLINK_MSG_ID_SENSOR_INIT_STATE_MIN_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN, MAVLINK_MSG_ID_SENSOR_INIT_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE SENSOR_INIT_STATE UNPACKING


/**
 * @brief Get field time_us from sensor_init_state message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_sensor_init_state_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sensor_init from sensor_init_state message
 *
 * @return  Bitfield indicating initialization status of sensors
 */
static inline uint32_t mavlink_msg_sensor_init_state_get_sensor_init(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Decode a sensor_init_state message into a struct
 *
 * @param msg The message to decode
 * @param sensor_init_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_init_state_decode(const mavlink_message_t* msg, mavlink_sensor_init_state_t* sensor_init_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sensor_init_state->time_us = mavlink_msg_sensor_init_state_get_time_us(msg);
    sensor_init_state->sensor_init = mavlink_msg_sensor_init_state_get_sensor_init(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN? msg->len : MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN;
        memset(sensor_init_state, 0, MAVLINK_MSG_ID_SENSOR_INIT_STATE_LEN);
    memcpy(sensor_init_state, _MAV_PAYLOAD(msg), len);
#endif
}
