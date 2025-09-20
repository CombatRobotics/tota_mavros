#pragma once
// MESSAGE DRIVE_STATUS PACKING

#define MAVLINK_MSG_ID_DRIVE_STATUS 105


typedef struct __mavlink_drive_status_t {
 uint32_t time_us; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 int16_t rpm_left_wheel; /*< [rpm] Left wheel RPM*/
 int16_t rpm_right_wheel; /*< [rpm] Right wheel RPM*/
 uint8_t arm_disarm; /*<  Arm/disarm state (0=disarmed, 1=armed)*/
 uint8_t jumping; /*<  Jumping mechanism state*/
} mavlink_drive_status_t;

#define MAVLINK_MSG_ID_DRIVE_STATUS_LEN 10
#define MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN 10
#define MAVLINK_MSG_ID_105_LEN 10
#define MAVLINK_MSG_ID_105_MIN_LEN 10

#define MAVLINK_MSG_ID_DRIVE_STATUS_CRC 188
#define MAVLINK_MSG_ID_105_CRC 188



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DRIVE_STATUS { \
    105, \
    "DRIVE_STATUS", \
    5, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drive_status_t, time_us) }, \
         { "arm_disarm", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_drive_status_t, arm_disarm) }, \
         { "rpm_left_wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drive_status_t, rpm_left_wheel) }, \
         { "rpm_right_wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drive_status_t, rpm_right_wheel) }, \
         { "jumping", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_drive_status_t, jumping) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DRIVE_STATUS { \
    "DRIVE_STATUS", \
    5, \
    {  { "time_us", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_drive_status_t, time_us) }, \
         { "arm_disarm", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_drive_status_t, arm_disarm) }, \
         { "rpm_left_wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_drive_status_t, rpm_left_wheel) }, \
         { "rpm_right_wheel", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_drive_status_t, rpm_right_wheel) }, \
         { "jumping", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_drive_status_t, jumping) }, \
         } \
}
#endif

/**
 * @brief Pack a drive_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param arm_disarm  Arm/disarm state (0=disarmed, 1=armed)
 * @param rpm_left_wheel [rpm] Left wheel RPM
 * @param rpm_right_wheel [rpm] Right wheel RPM
 * @param jumping  Jumping mechanism state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drive_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_us, uint8_t arm_disarm, int16_t rpm_left_wheel, int16_t rpm_right_wheel, uint8_t jumping)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRIVE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int16_t(buf, 4, rpm_left_wheel);
    _mav_put_int16_t(buf, 6, rpm_right_wheel);
    _mav_put_uint8_t(buf, 8, arm_disarm);
    _mav_put_uint8_t(buf, 9, jumping);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#else
    mavlink_drive_status_t packet;
    packet.time_us = time_us;
    packet.rpm_left_wheel = rpm_left_wheel;
    packet.rpm_right_wheel = rpm_right_wheel;
    packet.arm_disarm = arm_disarm;
    packet.jumping = jumping;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRIVE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
}

/**
 * @brief Pack a drive_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param arm_disarm  Arm/disarm state (0=disarmed, 1=armed)
 * @param rpm_left_wheel [rpm] Left wheel RPM
 * @param rpm_right_wheel [rpm] Right wheel RPM
 * @param jumping  Jumping mechanism state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drive_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_us, uint8_t arm_disarm, int16_t rpm_left_wheel, int16_t rpm_right_wheel, uint8_t jumping)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRIVE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int16_t(buf, 4, rpm_left_wheel);
    _mav_put_int16_t(buf, 6, rpm_right_wheel);
    _mav_put_uint8_t(buf, 8, arm_disarm);
    _mav_put_uint8_t(buf, 9, jumping);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#else
    mavlink_drive_status_t packet;
    packet.time_us = time_us;
    packet.rpm_left_wheel = rpm_left_wheel;
    packet.rpm_right_wheel = rpm_right_wheel;
    packet.arm_disarm = arm_disarm;
    packet.jumping = jumping;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRIVE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a drive_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param arm_disarm  Arm/disarm state (0=disarmed, 1=armed)
 * @param rpm_left_wheel [rpm] Left wheel RPM
 * @param rpm_right_wheel [rpm] Right wheel RPM
 * @param jumping  Jumping mechanism state
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_drive_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_us,uint8_t arm_disarm,int16_t rpm_left_wheel,int16_t rpm_right_wheel,uint8_t jumping)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRIVE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int16_t(buf, 4, rpm_left_wheel);
    _mav_put_int16_t(buf, 6, rpm_right_wheel);
    _mav_put_uint8_t(buf, 8, arm_disarm);
    _mav_put_uint8_t(buf, 9, jumping);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#else
    mavlink_drive_status_t packet;
    packet.time_us = time_us;
    packet.rpm_left_wheel = rpm_left_wheel;
    packet.rpm_right_wheel = rpm_right_wheel;
    packet.arm_disarm = arm_disarm;
    packet.jumping = jumping;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DRIVE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
}

/**
 * @brief Encode a drive_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param drive_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drive_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_drive_status_t* drive_status)
{
    return mavlink_msg_drive_status_pack(system_id, component_id, msg, drive_status->time_us, drive_status->arm_disarm, drive_status->rpm_left_wheel, drive_status->rpm_right_wheel, drive_status->jumping);
}

/**
 * @brief Encode a drive_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param drive_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drive_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_drive_status_t* drive_status)
{
    return mavlink_msg_drive_status_pack_chan(system_id, component_id, chan, msg, drive_status->time_us, drive_status->arm_disarm, drive_status->rpm_left_wheel, drive_status->rpm_right_wheel, drive_status->jumping);
}

/**
 * @brief Encode a drive_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param drive_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_drive_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_drive_status_t* drive_status)
{
    return mavlink_msg_drive_status_pack_status(system_id, component_id, _status, msg,  drive_status->time_us, drive_status->arm_disarm, drive_status->rpm_left_wheel, drive_status->rpm_right_wheel, drive_status->jumping);
}

/**
 * @brief Send a drive_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_us [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param arm_disarm  Arm/disarm state (0=disarmed, 1=armed)
 * @param rpm_left_wheel [rpm] Left wheel RPM
 * @param rpm_right_wheel [rpm] Right wheel RPM
 * @param jumping  Jumping mechanism state
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_drive_status_send(mavlink_channel_t chan, uint32_t time_us, uint8_t arm_disarm, int16_t rpm_left_wheel, int16_t rpm_right_wheel, uint8_t jumping)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DRIVE_STATUS_LEN];
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int16_t(buf, 4, rpm_left_wheel);
    _mav_put_int16_t(buf, 6, rpm_right_wheel);
    _mav_put_uint8_t(buf, 8, arm_disarm);
    _mav_put_uint8_t(buf, 9, jumping);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRIVE_STATUS, buf, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#else
    mavlink_drive_status_t packet;
    packet.time_us = time_us;
    packet.rpm_left_wheel = rpm_left_wheel;
    packet.rpm_right_wheel = rpm_right_wheel;
    packet.arm_disarm = arm_disarm;
    packet.jumping = jumping;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRIVE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#endif
}

/**
 * @brief Send a drive_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_drive_status_send_struct(mavlink_channel_t chan, const mavlink_drive_status_t* drive_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_drive_status_send(chan, drive_status->time_us, drive_status->arm_disarm, drive_status->rpm_left_wheel, drive_status->rpm_right_wheel, drive_status->jumping);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRIVE_STATUS, (const char *)drive_status, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_DRIVE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_drive_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_us, uint8_t arm_disarm, int16_t rpm_left_wheel, int16_t rpm_right_wheel, uint8_t jumping)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_us);
    _mav_put_int16_t(buf, 4, rpm_left_wheel);
    _mav_put_int16_t(buf, 6, rpm_right_wheel);
    _mav_put_uint8_t(buf, 8, arm_disarm);
    _mav_put_uint8_t(buf, 9, jumping);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRIVE_STATUS, buf, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#else
    mavlink_drive_status_t *packet = (mavlink_drive_status_t *)msgbuf;
    packet->time_us = time_us;
    packet->rpm_left_wheel = rpm_left_wheel;
    packet->rpm_right_wheel = rpm_right_wheel;
    packet->arm_disarm = arm_disarm;
    packet->jumping = jumping;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DRIVE_STATUS, (const char *)packet, MAVLINK_MSG_ID_DRIVE_STATUS_MIN_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_LEN, MAVLINK_MSG_ID_DRIVE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE DRIVE_STATUS UNPACKING


/**
 * @brief Get field time_us from drive_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_drive_status_get_time_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field arm_disarm from drive_status message
 *
 * @return  Arm/disarm state (0=disarmed, 1=armed)
 */
static inline uint8_t mavlink_msg_drive_status_get_arm_disarm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field rpm_left_wheel from drive_status message
 *
 * @return [rpm] Left wheel RPM
 */
static inline int16_t mavlink_msg_drive_status_get_rpm_left_wheel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field rpm_right_wheel from drive_status message
 *
 * @return [rpm] Right wheel RPM
 */
static inline int16_t mavlink_msg_drive_status_get_rpm_right_wheel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field jumping from drive_status message
 *
 * @return  Jumping mechanism state
 */
static inline uint8_t mavlink_msg_drive_status_get_jumping(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a drive_status message into a struct
 *
 * @param msg The message to decode
 * @param drive_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_drive_status_decode(const mavlink_message_t* msg, mavlink_drive_status_t* drive_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    drive_status->time_us = mavlink_msg_drive_status_get_time_us(msg);
    drive_status->rpm_left_wheel = mavlink_msg_drive_status_get_rpm_left_wheel(msg);
    drive_status->rpm_right_wheel = mavlink_msg_drive_status_get_rpm_right_wheel(msg);
    drive_status->arm_disarm = mavlink_msg_drive_status_get_arm_disarm(msg);
    drive_status->jumping = mavlink_msg_drive_status_get_jumping(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DRIVE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_DRIVE_STATUS_LEN;
        memset(drive_status, 0, MAVLINK_MSG_ID_DRIVE_STATUS_LEN);
    memcpy(drive_status, _MAV_PAYLOAD(msg), len);
#endif
}
