#pragma once
// MESSAGE ARMING_STATUS PACKING

#define MAVLINK_MSG_ID_ARMING_STATUS 104


typedef struct __mavlink_arming_status_t {
 uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 uint8_t armed; /*<  Armed state (0=disarmed, 1=armed)*/
} mavlink_arming_status_t;

#define MAVLINK_MSG_ID_ARMING_STATUS_LEN 5
#define MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN 5
#define MAVLINK_MSG_ID_104_LEN 5
#define MAVLINK_MSG_ID_104_MIN_LEN 5

#define MAVLINK_MSG_ID_ARMING_STATUS_CRC 61
#define MAVLINK_MSG_ID_104_CRC 61



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ARMING_STATUS { \
    104, \
    "ARMING_STATUS", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_arming_status_t, time_us) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_arming_status_t, armed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ARMING_STATUS { \
    "ARMING_STATUS", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_arming_status_t, time_us) }, \
         { "armed", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_arming_status_t, armed) }, \
         } \
}
#endif

/**
 * @brief Pack a arming_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param armed  Armed state (0=disarmed, 1=armed)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arming_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_us, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARMING_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint8_t(buf, 4, armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#else
    mavlink_arming_status_t packet;
    packet.time_us = time_us;
    packet.armed = armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARMING_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
}

/**
 * @brief Pack a arming_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param armed  Armed state (0=disarmed, 1=armed)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arming_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_us, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARMING_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint8_t(buf, 4, armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#else
    mavlink_arming_status_t packet;
    packet.time_us = time_us;
    packet.armed = armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARMING_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#endif
}

/**
 * @brief Pack a arming_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param armed  Armed state (0=disarmed, 1=armed)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arming_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_us,uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARMING_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint8_t(buf, 4, armed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#else
    mavlink_arming_status_t packet;
    packet.time_us = time_us;
    packet.armed = armed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ARMING_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
}

/**
 * @brief Encode a arming_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arming_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arming_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arming_status_t* arming_status)
{
    return mavlink_msg_arming_status_pack(system_id, component_id, msg, arming_status->time_us, arming_status->armed);
}

/**
 * @brief Encode a arming_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arming_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arming_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arming_status_t* arming_status)
{
    return mavlink_msg_arming_status_pack_chan(system_id, component_id, chan, msg, arming_status->time_us, arming_status->armed);
}

/**
 * @brief Encode a arming_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param arming_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arming_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_arming_status_t* arming_status)
{
    return mavlink_msg_arming_status_pack_status(system_id, component_id, _status, msg,  arming_status->time_us, arming_status->armed);
}

/**
 * @brief Send a arming_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param armed  Armed state (0=disarmed, 1=armed)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arming_status_send(mavlink_channel_t chan, uint32_t time_us, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ARMING_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint8_t(buf, 4, armed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_STATUS, buf, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#else
    mavlink_arming_status_t packet;
    packet.time_us = time_us;
    packet.armed = armed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#endif
}

/**
 * @brief Send a arming_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_arming_status_send_struct(mavlink_channel_t chan, const mavlink_arming_status_t* arming_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_arming_status_send(chan, arming_status->time_us, arming_status->armed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_STATUS, (const char *)arming_status, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ARMING_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arming_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_us, uint8_t armed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_uint8_t(buf, 4, armed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_STATUS, buf, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#else
    mavlink_arming_status_t *packet = (mavlink_arming_status_t *)msgbuf;
    packet->time_us = time_us;
    packet->armed = armed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_STATUS, (const char *)packet, MAVLINK_MSG_ID_ARMING_STATUS_MIN_LEN, MAVLINK_MSG_ID_ARMING_STATUS_LEN, MAVLINK_MSG_ID_ARMING_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ARMING_STATUS UNPACKING


/**
 * @brief Get field time_us from arming_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_arming_status_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field armed from arming_status message
 *
 * @return  Armed state (0=disarmed, 1=armed)
 */
static inline uint8_t mavlink_msg_arming_status_get_armed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a arming_status message into a struct
 *
 * @param msg The message to decode
 * @param arming_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_arming_status_decode(const mavlink_message_t* msg, mavlink_arming_status_t* arming_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    arming_status->time_us = mavlink_msg_arming_status_get_time_us(msg);
    arming_status->armed = mavlink_msg_arming_status_get_armed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ARMING_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ARMING_STATUS_LEN;
        memset(arming_status, 0, MAVLINK_MSG_ID_ARMING_STATUS_LEN);
    memcpy(arming_status, _MAV_PAYLOAD(msg), len);
#endif
}
