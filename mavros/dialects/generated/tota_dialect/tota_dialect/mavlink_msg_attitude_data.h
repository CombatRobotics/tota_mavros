#pragma once
// MESSAGE ATTITUDE_DATA PACKING

#define MAVLINK_MSG_ID_ATTITUDE_DATA 100


typedef struct __mavlink_attitude_data_t {
 uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 float roll; /*< [rad] Roll angle*/
 float pitch; /*< [rad] Pitch angle*/
 float yaw; /*< [rad] Yaw angle*/
 float rollspeed; /*< [rad/s] Roll angular speed*/
 float pitchspeed; /*< [rad/s] Pitch angular speed*/
 float yawspeed; /*< [rad/s] Yaw angular speed*/
} mavlink_attitude_data_t;

#define MAVLINK_MSG_ID_ATTITUDE_DATA_LEN 28
#define MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN 28
#define MAVLINK_MSG_ID_100_LEN 28
#define MAVLINK_MSG_ID_100_MIN_LEN 28

#define MAVLINK_MSG_ID_ATTITUDE_DATA_CRC 225
#define MAVLINK_MSG_ID_100_CRC 225



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE_DATA { \
    100, \
    "ATTITUDE_DATA", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_data_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_data_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_data_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_data_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_data_t, yawspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE_DATA { \
    "ATTITUDE_DATA", \
    7, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude_data_t, time_usec) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_data_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_data_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_data_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_attitude_data_t, yawspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#else
    mavlink_attitude_data_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
}

/**
 * @brief Pack a attitude_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#else
    mavlink_attitude_data_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#endif
}

/**
 * @brief Pack a attitude_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_usec,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#else
    mavlink_attitude_data_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
}

/**
 * @brief Encode a attitude_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_data_t* attitude_data)
{
    return mavlink_msg_attitude_data_pack(system_id, component_id, msg, attitude_data->time_usec, attitude_data->roll, attitude_data->pitch, attitude_data->yaw, attitude_data->rollspeed, attitude_data->pitchspeed, attitude_data->yawspeed);
}

/**
 * @brief Encode a attitude_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_data_t* attitude_data)
{
    return mavlink_msg_attitude_data_pack_chan(system_id, component_id, chan, msg, attitude_data->time_usec, attitude_data->roll, attitude_data->pitch, attitude_data->yaw, attitude_data->rollspeed, attitude_data->pitchspeed, attitude_data->yawspeed);
}

/**
 * @brief Encode a attitude_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param attitude_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_attitude_data_t* attitude_data)
{
    return mavlink_msg_attitude_data_pack_status(system_id, component_id, _status, msg,  attitude_data->time_usec, attitude_data->roll, attitude_data->pitch, attitude_data->yaw, attitude_data->rollspeed, attitude_data->pitchspeed, attitude_data->yawspeed);
}

/**
 * @brief Send a attitude_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param roll [rad] Roll angle
 * @param pitch [rad] Pitch angle
 * @param yaw [rad] Yaw angle
 * @param rollspeed [rad/s] Roll angular speed
 * @param pitchspeed [rad/s] Pitch angular speed
 * @param yawspeed [rad/s] Yaw angular speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_data_send(mavlink_channel_t chan, uint32_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_DATA, buf, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#else
    mavlink_attitude_data_t packet;
    packet.time_usec = time_usec;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_DATA, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#endif
}

/**
 * @brief Send a attitude_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_data_send_struct(mavlink_channel_t chan, const mavlink_attitude_data_t* attitude_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_data_send(chan, attitude_data->time_usec, attitude_data->roll, attitude_data->pitch, attitude_data->yaw, attitude_data->rollspeed, attitude_data->pitchspeed, attitude_data->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_DATA, (const char *)attitude_data, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_float(buf, 4, roll);
    _mav_put_float(buf, 8, pitch);
    _mav_put_float(buf, 12, yaw);
    _mav_put_float(buf, 16, rollspeed);
    _mav_put_float(buf, 20, pitchspeed);
    _mav_put_float(buf, 24, yawspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_DATA, buf, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#else
    mavlink_attitude_data_t *packet = (mavlink_attitude_data_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->rollspeed = rollspeed;
    packet->pitchspeed = pitchspeed;
    packet->yawspeed = yawspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE_DATA, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_DATA_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN, MAVLINK_MSG_ID_ATTITUDE_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE_DATA UNPACKING


/**
 * @brief Get field time_usec from attitude_data message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_attitude_data_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from attitude_data message
 *
 * @return [rad] Roll angle
 */
static inline float mavlink_msg_attitude_data_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from attitude_data message
 *
 * @return [rad] Pitch angle
 */
static inline float mavlink_msg_attitude_data_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from attitude_data message
 *
 * @return [rad] Yaw angle
 */
static inline float mavlink_msg_attitude_data_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rollspeed from attitude_data message
 *
 * @return [rad/s] Roll angular speed
 */
static inline float mavlink_msg_attitude_data_get_rollspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitchspeed from attitude_data message
 *
 * @return [rad/s] Pitch angular speed
 */
static inline float mavlink_msg_attitude_data_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yawspeed from attitude_data message
 *
 * @return [rad/s] Yaw angular speed
 */
static inline float mavlink_msg_attitude_data_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a attitude_data message into a struct
 *
 * @param msg The message to decode
 * @param attitude_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_data_decode(const mavlink_message_t* msg, mavlink_attitude_data_t* attitude_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude_data->time_usec = mavlink_msg_attitude_data_get_time_usec(msg);
    attitude_data->roll = mavlink_msg_attitude_data_get_roll(msg);
    attitude_data->pitch = mavlink_msg_attitude_data_get_pitch(msg);
    attitude_data->yaw = mavlink_msg_attitude_data_get_yaw(msg);
    attitude_data->rollspeed = mavlink_msg_attitude_data_get_rollspeed(msg);
    attitude_data->pitchspeed = mavlink_msg_attitude_data_get_pitchspeed(msg);
    attitude_data->yawspeed = mavlink_msg_attitude_data_get_yawspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_DATA_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_DATA_LEN;
        memset(attitude_data, 0, MAVLINK_MSG_ID_ATTITUDE_DATA_LEN);
    memcpy(attitude_data, _MAV_PAYLOAD(msg), len);
#endif
}
