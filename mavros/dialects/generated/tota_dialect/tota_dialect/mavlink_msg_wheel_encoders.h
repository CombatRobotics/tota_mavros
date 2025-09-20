#pragma once
// MESSAGE WHEEL_ENCODERS PACKING

#define MAVLINK_MSG_ID_WHEEL_ENCODERS 42003


typedef struct __mavlink_wheel_encoders_t {
 uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 int32_t left_ticks; /*<  Left wheel encoder ticks*/
 int32_t right_ticks; /*<  Right wheel encoder ticks*/
 uint16_t cam_pos; /*<  12-bit AS5600 CAM position*/
} mavlink_wheel_encoders_t;

#define MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN 14
#define MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN 14
#define MAVLINK_MSG_ID_42003_LEN 14
#define MAVLINK_MSG_ID_42003_MIN_LEN 14

#define MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC 93
#define MAVLINK_MSG_ID_42003_CRC 93



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WHEEL_ENCODERS { \
    42003, \
    "WHEEL_ENCODERS", \
    4, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_wheel_encoders_t, time_us) }, \
         { "left_ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_wheel_encoders_t, left_ticks) }, \
         { "right_ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_wheel_encoders_t, right_ticks) }, \
         { "cam_pos", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_wheel_encoders_t, cam_pos) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WHEEL_ENCODERS { \
    "WHEEL_ENCODERS", \
    4, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_wheel_encoders_t, time_us) }, \
         { "left_ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_wheel_encoders_t, left_ticks) }, \
         { "right_ticks", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_wheel_encoders_t, right_ticks) }, \
         { "cam_pos", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_wheel_encoders_t, cam_pos) }, \
         } \
}
#endif

/**
 * @brief Pack a wheel_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param left_ticks  Left wheel encoder ticks
 * @param right_ticks  Right wheel encoder ticks
 * @param cam_pos  12-bit AS5600 CAM position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_encoders_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_us, int32_t left_ticks, int32_t right_ticks, uint16_t cam_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int32_t(buf, 4, left_ticks);
    _mav_put_int32_t(buf, 8, right_ticks);
    _mav_put_uint16_t(buf, 12, cam_pos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#else
    mavlink_wheel_encoders_t packet;
    packet.time_us = time_us;
    packet.left_ticks = left_ticks;
    packet.right_ticks = right_ticks;
    packet.cam_pos = cam_pos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_ENCODERS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
}

/**
 * @brief Pack a wheel_encoders message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param left_ticks  Left wheel encoder ticks
 * @param right_ticks  Right wheel encoder ticks
 * @param cam_pos  12-bit AS5600 CAM position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_encoders_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_us, int32_t left_ticks, int32_t right_ticks, uint16_t cam_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int32_t(buf, 4, left_ticks);
    _mav_put_int32_t(buf, 8, right_ticks);
    _mav_put_uint16_t(buf, 12, cam_pos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#else
    mavlink_wheel_encoders_t packet;
    packet.time_us = time_us;
    packet.left_ticks = left_ticks;
    packet.right_ticks = right_ticks;
    packet.cam_pos = cam_pos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_ENCODERS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#endif
}

/**
 * @brief Pack a wheel_encoders message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param left_ticks  Left wheel encoder ticks
 * @param right_ticks  Right wheel encoder ticks
 * @param cam_pos  12-bit AS5600 CAM position
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wheel_encoders_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_us,int32_t left_ticks,int32_t right_ticks,uint16_t cam_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int32_t(buf, 4, left_ticks);
    _mav_put_int32_t(buf, 8, right_ticks);
    _mav_put_uint16_t(buf, 12, cam_pos);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#else
    mavlink_wheel_encoders_t packet;
    packet.time_us = time_us;
    packet.left_ticks = left_ticks;
    packet.right_ticks = right_ticks;
    packet.cam_pos = cam_pos;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WHEEL_ENCODERS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
}

/**
 * @brief Encode a wheel_encoders struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wheel_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_encoders_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wheel_encoders_t* wheel_encoders)
{
    return mavlink_msg_wheel_encoders_pack(system_id, component_id, msg, wheel_encoders->time_us, wheel_encoders->left_ticks, wheel_encoders->right_ticks, wheel_encoders->cam_pos);
}

/**
 * @brief Encode a wheel_encoders struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wheel_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_encoders_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wheel_encoders_t* wheel_encoders)
{
    return mavlink_msg_wheel_encoders_pack_chan(system_id, component_id, chan, msg, wheel_encoders->time_us, wheel_encoders->left_ticks, wheel_encoders->right_ticks, wheel_encoders->cam_pos);
}

/**
 * @brief Encode a wheel_encoders struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param wheel_encoders C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wheel_encoders_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_wheel_encoders_t* wheel_encoders)
{
    return mavlink_msg_wheel_encoders_pack_status(system_id, component_id, _status, msg,  wheel_encoders->time_us, wheel_encoders->left_ticks, wheel_encoders->right_ticks, wheel_encoders->cam_pos);
}

/**
 * @brief Send a wheel_encoders message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param left_ticks  Left wheel encoder ticks
 * @param right_ticks  Right wheel encoder ticks
 * @param cam_pos  12-bit AS5600 CAM position
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wheel_encoders_send(mavlink_channel_t chan, uint32_t time_us, int32_t left_ticks, int32_t right_ticks, uint16_t cam_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int32_t(buf, 4, left_ticks);
    _mav_put_int32_t(buf, 8, right_ticks);
    _mav_put_uint16_t(buf, 12, cam_pos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_ENCODERS, buf, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#else
    mavlink_wheel_encoders_t packet;
    packet.time_us = time_us;
    packet.left_ticks = left_ticks;
    packet.right_ticks = right_ticks;
    packet.cam_pos = cam_pos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_ENCODERS, (const char *)&packet, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#endif
}

/**
 * @brief Send a wheel_encoders message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_wheel_encoders_send_struct(mavlink_channel_t chan, const mavlink_wheel_encoders_t* wheel_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_wheel_encoders_send(chan, wheel_encoders->time_us, wheel_encoders->left_ticks, wheel_encoders->right_ticks, wheel_encoders->cam_pos);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_ENCODERS, (const char *)wheel_encoders, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#endif
}

#if MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wheel_encoders_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_us, int32_t left_ticks, int32_t right_ticks, uint16_t cam_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int32_t(buf, 4, left_ticks);
    _mav_put_int32_t(buf, 8, right_ticks);
    _mav_put_uint16_t(buf, 12, cam_pos);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_ENCODERS, buf, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#else
    mavlink_wheel_encoders_t *packet = (mavlink_wheel_encoders_t *)msgbuf;
    packet->time_us = time_us;
    packet->left_ticks = left_ticks;
    packet->right_ticks = right_ticks;
    packet->cam_pos = cam_pos;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WHEEL_ENCODERS, (const char *)packet, MAVLINK_MSG_ID_WHEEL_ENCODERS_MIN_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN, MAVLINK_MSG_ID_WHEEL_ENCODERS_CRC);
#endif
}
#endif

#endif

// MESSAGE WHEEL_ENCODERS UNPACKING


/**
 * @brief Get field time_us from wheel_encoders message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_wheel_encoders_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field left_ticks from wheel_encoders message
 *
 * @return  Left wheel encoder ticks
 */
static inline int32_t mavlink_msg_wheel_encoders_get_left_ticks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field right_ticks from wheel_encoders message
 *
 * @return  Right wheel encoder ticks
 */
static inline int32_t mavlink_msg_wheel_encoders_get_right_ticks(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field cam_pos from wheel_encoders message
 *
 * @return  12-bit AS5600 CAM position
 */
static inline uint16_t mavlink_msg_wheel_encoders_get_cam_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Decode a wheel_encoders message into a struct
 *
 * @param msg The message to decode
 * @param wheel_encoders C-struct to decode the message contents into
 */
static inline void mavlink_msg_wheel_encoders_decode(const mavlink_message_t* msg, mavlink_wheel_encoders_t* wheel_encoders)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wheel_encoders->time_us = mavlink_msg_wheel_encoders_get_time_us(msg);
    wheel_encoders->left_ticks = mavlink_msg_wheel_encoders_get_left_ticks(msg);
    wheel_encoders->right_ticks = mavlink_msg_wheel_encoders_get_right_ticks(msg);
    wheel_encoders->cam_pos = mavlink_msg_wheel_encoders_get_cam_pos(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN? msg->len : MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN;
        memset(wheel_encoders, 0, MAVLINK_MSG_ID_WHEEL_ENCODERS_LEN);
    memcpy(wheel_encoders, _MAV_PAYLOAD(msg), len);
#endif
}
