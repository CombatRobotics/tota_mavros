#pragma once
// MESSAGE TOF_L7CX_RANGES PACKING

#define MAVLINK_MSG_ID_TOF_L7CX_RANGES 42001


typedef struct __mavlink_tof_l7cx_ranges_t {
 uint64_t time_us; /*< [us] */
 uint16_t range_mm[64]; /*< [mm] */
} mavlink_tof_l7cx_ranges_t;

#define MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN 136
#define MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN 136
#define MAVLINK_MSG_ID_42001_LEN 136
#define MAVLINK_MSG_ID_42001_MIN_LEN 136

#define MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC 224
#define MAVLINK_MSG_ID_42001_CRC 224

#define MAVLINK_MSG_TOF_L7CX_RANGES_FIELD_RANGE_MM_LEN 64

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TOF_L7CX_RANGES { \
    42001, \
    "TOF_L7CX_RANGES", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_l7cx_ranges_t, time_us) }, \
         { "range_mm", NULL, MAVLINK_TYPE_UINT16_T, 64, 8, offsetof(mavlink_tof_l7cx_ranges_t, range_mm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TOF_L7CX_RANGES { \
    "TOF_L7CX_RANGES", \
    2, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_tof_l7cx_ranges_t, time_us) }, \
         { "range_mm", NULL, MAVLINK_TYPE_UINT16_T, 64, 8, offsetof(mavlink_tof_l7cx_ranges_t, range_mm) }, \
         } \
}
#endif

/**
 * @brief Pack a tof_l7cx_ranges message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] 
 * @param range_mm [mm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_us, const uint16_t *range_mm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, range_mm, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#else
    mavlink_tof_l7cx_ranges_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.range_mm, range_mm, sizeof(uint16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_RANGES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
}

/**
 * @brief Pack a tof_l7cx_ranges message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] 
 * @param range_mm [mm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t time_us, const uint16_t *range_mm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, range_mm, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#else
    mavlink_tof_l7cx_ranges_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.range_mm, range_mm, sizeof(uint16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_RANGES;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#endif
}

/**
 * @brief Pack a tof_l7cx_ranges message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] 
 * @param range_mm [mm] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_us,const uint16_t *range_mm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, range_mm, 64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#else
    mavlink_tof_l7cx_ranges_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.range_mm, range_mm, sizeof(uint16_t)*64);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_TOF_L7CX_RANGES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
}

/**
 * @brief Encode a tof_l7cx_ranges struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_ranges C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tof_l7cx_ranges_t* tof_l7cx_ranges)
{
    return mavlink_msg_tof_l7cx_ranges_pack(system_id, component_id, msg, tof_l7cx_ranges->time_us, tof_l7cx_ranges->range_mm);
}

/**
 * @brief Encode a tof_l7cx_ranges struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_ranges C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tof_l7cx_ranges_t* tof_l7cx_ranges)
{
    return mavlink_msg_tof_l7cx_ranges_pack_chan(system_id, component_id, chan, msg, tof_l7cx_ranges->time_us, tof_l7cx_ranges->range_mm);
}

/**
 * @brief Encode a tof_l7cx_ranges struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param tof_l7cx_ranges C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_tof_l7cx_ranges_t* tof_l7cx_ranges)
{
    return mavlink_msg_tof_l7cx_ranges_pack_status(system_id, component_id, _status, msg,  tof_l7cx_ranges->time_us, tof_l7cx_ranges->range_mm);
}

/**
 * @brief Send a tof_l7cx_ranges message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] 
 * @param range_mm [mm] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tof_l7cx_ranges_send(mavlink_channel_t chan, uint64_t time_us, const uint16_t *range_mm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN];
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, range_mm, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES, buf, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#else
    mavlink_tof_l7cx_ranges_t packet;
    packet.time_us = time_us;
    mav_array_memcpy(packet.range_mm, range_mm, sizeof(uint16_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES, (const char *)&packet, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#endif
}

/**
 * @brief Send a tof_l7cx_ranges message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tof_l7cx_ranges_send_struct(mavlink_channel_t chan, const mavlink_tof_l7cx_ranges_t* tof_l7cx_ranges)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tof_l7cx_ranges_send(chan, tof_l7cx_ranges->time_us, tof_l7cx_ranges->range_mm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES, (const char *)tof_l7cx_ranges, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#endif
}

#if MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tof_l7cx_ranges_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_us, const uint16_t *range_mm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_us);
    _mav_put_uint16_t_array(buf, 8, range_mm, 64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES, buf, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#else
    mavlink_tof_l7cx_ranges_t *packet = (mavlink_tof_l7cx_ranges_t *)msgbuf;
    packet->time_us = time_us;
    mav_array_memcpy(packet->range_mm, range_mm, sizeof(uint16_t)*64);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOF_L7CX_RANGES, (const char *)packet, MAVLINK_MSG_ID_TOF_L7CX_RANGES_MIN_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN, MAVLINK_MSG_ID_TOF_L7CX_RANGES_CRC);
#endif
}
#endif

#endif

// MESSAGE TOF_L7CX_RANGES UNPACKING


/**
 * @brief Get field time_us from tof_l7cx_ranges message
 *
 * @return [us] 
 */
static inline uint64_t mavlink_msg_tof_l7cx_ranges_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field range_mm from tof_l7cx_ranges message
 *
 * @return [mm] 
 */
static inline uint16_t mavlink_msg_tof_l7cx_ranges_get_range_mm(const mavlink_message_t* msg, uint16_t *range_mm)
{
    return _MAV_RETURN_uint16_t_array(msg, range_mm, 64,  8);
}

/**
 * @brief Decode a tof_l7cx_ranges message into a struct
 *
 * @param msg The message to decode
 * @param tof_l7cx_ranges C-struct to decode the message contents into
 */
static inline void mavlink_msg_tof_l7cx_ranges_decode(const mavlink_message_t* msg, mavlink_tof_l7cx_ranges_t* tof_l7cx_ranges)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    tof_l7cx_ranges->time_us = mavlink_msg_tof_l7cx_ranges_get_time_us(msg);
    mavlink_msg_tof_l7cx_ranges_get_range_mm(msg, tof_l7cx_ranges->range_mm);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN? msg->len : MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN;
        memset(tof_l7cx_ranges, 0, MAVLINK_MSG_ID_TOF_L7CX_RANGES_LEN);
    memcpy(tof_l7cx_ranges, _MAV_PAYLOAD(msg), len);
#endif
}
