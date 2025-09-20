#pragma once
// MESSAGE GPS_DATA PACKING

#define MAVLINK_MSG_ID_GPS_DATA 101


typedef struct __mavlink_gps_data_t {
 uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot)*/
 int32_t lat; /*< [degE7] Latitude in degrees * 1E7*/
 int32_t lon; /*< [degE7] Longitude in degrees * 1E7*/
 int32_t alt; /*< [mm] Altitude (MSL) in millimeters*/
 int32_t alt_ellipsoid; /*< [mm] Altitude (above WGS84 ellipsoid) in millimeters*/
 uint32_t h_acc; /*< [mm] Position uncertainty in millimeters*/
 uint32_t v_acc; /*< [mm] Altitude uncertainty in millimeters*/
 uint32_t vel_acc; /*< [mm/s] Speed uncertainty in mm/s*/
 uint32_t hdg_acc; /*< [degE5] Heading accuracy in degrees * 1E5*/
 uint16_t eph; /*< [cm] GPS horizontal dilution of position (HDOP) in cm*/
 uint16_t epv; /*< [cm] GPS vertical dilution of position (VDOP) in cm*/
 uint16_t vel; /*< [cm/s] Ground speed in cm/s*/
 uint16_t cog; /*< [cdeg] Course over ground in degrees * 100*/
 uint8_t fix_type; /*<  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)*/
 uint8_t satellites_visible; /*<  Number of satellites visible*/
} mavlink_gps_data_t;

#define MAVLINK_MSG_ID_GPS_DATA_LEN 46
#define MAVLINK_MSG_ID_GPS_DATA_MIN_LEN 46
#define MAVLINK_MSG_ID_101_LEN 46
#define MAVLINK_MSG_ID_101_MIN_LEN 46

#define MAVLINK_MSG_ID_GPS_DATA_CRC 28
#define MAVLINK_MSG_ID_101_CRC 28



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_DATA { \
    101, \
    "GPS_DATA", \
    15, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gps_data_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_gps_data_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gps_data_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_data_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_data_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_gps_data_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_gps_data_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_gps_data_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_gps_data_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_gps_data_t, satellites_visible) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_data_t, alt_ellipsoid) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gps_data_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_gps_data_t, v_acc) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_gps_data_t, vel_acc) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_gps_data_t, hdg_acc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_DATA { \
    "GPS_DATA", \
    15, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gps_data_t, time_usec) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_gps_data_t, fix_type) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_gps_data_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_gps_data_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_gps_data_t, alt) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_gps_data_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_gps_data_t, epv) }, \
         { "vel", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_gps_data_t, vel) }, \
         { "cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 42, offsetof(mavlink_gps_data_t, cog) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_gps_data_t, satellites_visible) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_gps_data_t, alt_ellipsoid) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_gps_data_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_gps_data_t, v_acc) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_gps_data_t, vel_acc) }, \
         { "hdg_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_gps_data_t, hdg_acc) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param fix_type  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)
 * @param lat [degE7] Latitude in degrees * 1E7
 * @param lon [degE7] Longitude in degrees * 1E7
 * @param alt [mm] Altitude (MSL) in millimeters
 * @param eph [cm] GPS horizontal dilution of position (HDOP) in cm
 * @param epv [cm] GPS vertical dilution of position (VDOP) in cm
 * @param vel [cm/s] Ground speed in cm/s
 * @param cog [cdeg] Course over ground in degrees * 100
 * @param satellites_visible  Number of satellites visible
 * @param alt_ellipsoid [mm] Altitude (above WGS84 ellipsoid) in millimeters
 * @param h_acc [mm] Position uncertainty in millimeters
 * @param v_acc [mm] Altitude uncertainty in millimeters
 * @param vel_acc [mm/s] Speed uncertainty in mm/s
 * @param hdg_acc [degE5] Heading accuracy in degrees * 1E5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, alt_ellipsoid);
    _mav_put_uint32_t(buf, 20, h_acc);
    _mav_put_uint32_t(buf, 24, v_acc);
    _mav_put_uint32_t(buf, 28, vel_acc);
    _mav_put_uint32_t(buf, 32, hdg_acc);
    _mav_put_uint16_t(buf, 36, eph);
    _mav_put_uint16_t(buf, 38, epv);
    _mav_put_uint16_t(buf, 40, vel);
    _mav_put_uint16_t(buf, 42, cog);
    _mav_put_uint8_t(buf, 44, fix_type);
    _mav_put_uint8_t(buf, 45, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#else
    mavlink_gps_data_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
}

/**
 * @brief Pack a gps_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param fix_type  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)
 * @param lat [degE7] Latitude in degrees * 1E7
 * @param lon [degE7] Longitude in degrees * 1E7
 * @param alt [mm] Altitude (MSL) in millimeters
 * @param eph [cm] GPS horizontal dilution of position (HDOP) in cm
 * @param epv [cm] GPS vertical dilution of position (VDOP) in cm
 * @param vel [cm/s] Ground speed in cm/s
 * @param cog [cdeg] Course over ground in degrees * 100
 * @param satellites_visible  Number of satellites visible
 * @param alt_ellipsoid [mm] Altitude (above WGS84 ellipsoid) in millimeters
 * @param h_acc [mm] Position uncertainty in millimeters
 * @param v_acc [mm] Altitude uncertainty in millimeters
 * @param vel_acc [mm/s] Speed uncertainty in mm/s
 * @param hdg_acc [degE5] Heading accuracy in degrees * 1E5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_data_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, alt_ellipsoid);
    _mav_put_uint32_t(buf, 20, h_acc);
    _mav_put_uint32_t(buf, 24, v_acc);
    _mav_put_uint32_t(buf, 28, vel_acc);
    _mav_put_uint32_t(buf, 32, hdg_acc);
    _mav_put_uint16_t(buf, 36, eph);
    _mav_put_uint16_t(buf, 38, epv);
    _mav_put_uint16_t(buf, 40, vel);
    _mav_put_uint16_t(buf, 42, cog);
    _mav_put_uint8_t(buf, 44, fix_type);
    _mav_put_uint8_t(buf, 45, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#else
    mavlink_gps_data_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif
}

/**
 * @brief Pack a gps_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param fix_type  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)
 * @param lat [degE7] Latitude in degrees * 1E7
 * @param lon [degE7] Longitude in degrees * 1E7
 * @param alt [mm] Altitude (MSL) in millimeters
 * @param eph [cm] GPS horizontal dilution of position (HDOP) in cm
 * @param epv [cm] GPS vertical dilution of position (VDOP) in cm
 * @param vel [cm/s] Ground speed in cm/s
 * @param cog [cdeg] Course over ground in degrees * 100
 * @param satellites_visible  Number of satellites visible
 * @param alt_ellipsoid [mm] Altitude (above WGS84 ellipsoid) in millimeters
 * @param h_acc [mm] Position uncertainty in millimeters
 * @param v_acc [mm] Altitude uncertainty in millimeters
 * @param vel_acc [mm/s] Speed uncertainty in mm/s
 * @param hdg_acc [degE5] Heading accuracy in degrees * 1E5
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_usec,uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,uint16_t cog,uint8_t satellites_visible,int32_t alt_ellipsoid,uint32_t h_acc,uint32_t v_acc,uint32_t vel_acc,uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, alt_ellipsoid);
    _mav_put_uint32_t(buf, 20, h_acc);
    _mav_put_uint32_t(buf, 24, v_acc);
    _mav_put_uint32_t(buf, 28, vel_acc);
    _mav_put_uint32_t(buf, 32, hdg_acc);
    _mav_put_uint16_t(buf, 36, eph);
    _mav_put_uint16_t(buf, 38, epv);
    _mav_put_uint16_t(buf, 40, vel);
    _mav_put_uint16_t(buf, 42, cog);
    _mav_put_uint8_t(buf, 44, fix_type);
    _mav_put_uint8_t(buf, 45, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_DATA_LEN);
#else
    mavlink_gps_data_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GPS_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
}

/**
 * @brief Encode a gps_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_data_t* gps_data)
{
    return mavlink_msg_gps_data_pack(system_id, component_id, msg, gps_data->time_usec, gps_data->fix_type, gps_data->lat, gps_data->lon, gps_data->alt, gps_data->eph, gps_data->epv, gps_data->vel, gps_data->cog, gps_data->satellites_visible, gps_data->alt_ellipsoid, gps_data->h_acc, gps_data->v_acc, gps_data->vel_acc, gps_data->hdg_acc);
}

/**
 * @brief Encode a gps_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_data_t* gps_data)
{
    return mavlink_msg_gps_data_pack_chan(system_id, component_id, chan, msg, gps_data->time_usec, gps_data->fix_type, gps_data->lat, gps_data->lon, gps_data->alt, gps_data->eph, gps_data->epv, gps_data->vel, gps_data->cog, gps_data->satellites_visible, gps_data->alt_ellipsoid, gps_data->h_acc, gps_data->v_acc, gps_data->vel_acc, gps_data->hdg_acc);
}

/**
 * @brief Encode a gps_data struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gps_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_data_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gps_data_t* gps_data)
{
    return mavlink_msg_gps_data_pack_status(system_id, component_id, _status, msg,  gps_data->time_usec, gps_data->fix_type, gps_data->lat, gps_data->lon, gps_data->alt, gps_data->eph, gps_data->epv, gps_data->vel, gps_data->cog, gps_data->satellites_visible, gps_data->alt_ellipsoid, gps_data->h_acc, gps_data->v_acc, gps_data->vel_acc, gps_data->hdg_acc);
}

/**
 * @brief Send a gps_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot)
 * @param fix_type  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)
 * @param lat [degE7] Latitude in degrees * 1E7
 * @param lon [degE7] Longitude in degrees * 1E7
 * @param alt [mm] Altitude (MSL) in millimeters
 * @param eph [cm] GPS horizontal dilution of position (HDOP) in cm
 * @param epv [cm] GPS vertical dilution of position (VDOP) in cm
 * @param vel [cm/s] Ground speed in cm/s
 * @param cog [cdeg] Course over ground in degrees * 100
 * @param satellites_visible  Number of satellites visible
 * @param alt_ellipsoid [mm] Altitude (above WGS84 ellipsoid) in millimeters
 * @param h_acc [mm] Position uncertainty in millimeters
 * @param v_acc [mm] Altitude uncertainty in millimeters
 * @param vel_acc [mm/s] Speed uncertainty in mm/s
 * @param hdg_acc [degE5] Heading accuracy in degrees * 1E5
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_data_send(mavlink_channel_t chan, uint32_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GPS_DATA_LEN];
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, alt_ellipsoid);
    _mav_put_uint32_t(buf, 20, h_acc);
    _mav_put_uint32_t(buf, 24, v_acc);
    _mav_put_uint32_t(buf, 28, vel_acc);
    _mav_put_uint32_t(buf, 32, hdg_acc);
    _mav_put_uint16_t(buf, 36, eph);
    _mav_put_uint16_t(buf, 38, epv);
    _mav_put_uint16_t(buf, 40, vel);
    _mav_put_uint16_t(buf, 42, cog);
    _mav_put_uint8_t(buf, 44, fix_type);
    _mav_put_uint8_t(buf, 45, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    mavlink_gps_data_t packet;
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.h_acc = h_acc;
    packet.v_acc = v_acc;
    packet.vel_acc = vel_acc;
    packet.hdg_acc = hdg_acc;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)&packet, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#endif
}

/**
 * @brief Send a gps_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_data_send_struct(mavlink_channel_t chan, const mavlink_gps_data_t* gps_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_data_send(chan, gps_data->time_usec, gps_data->fix_type, gps_data->lat, gps_data->lon, gps_data->alt, gps_data->eph, gps_data->epv, gps_data->vel, gps_data->cog, gps_data->satellites_visible, gps_data->alt_ellipsoid, gps_data->h_acc, gps_data->v_acc, gps_data->vel_acc, gps_data->hdg_acc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)gps_data, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible, int32_t alt_ellipsoid, uint32_t h_acc, uint32_t v_acc, uint32_t vel_acc, uint32_t hdg_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_usec);
    _mav_put_int32_t(buf, 4, lat);
    _mav_put_int32_t(buf, 8, lon);
    _mav_put_int32_t(buf, 12, alt);
    _mav_put_int32_t(buf, 16, alt_ellipsoid);
    _mav_put_uint32_t(buf, 20, h_acc);
    _mav_put_uint32_t(buf, 24, v_acc);
    _mav_put_uint32_t(buf, 28, vel_acc);
    _mav_put_uint32_t(buf, 32, hdg_acc);
    _mav_put_uint16_t(buf, 36, eph);
    _mav_put_uint16_t(buf, 38, epv);
    _mav_put_uint16_t(buf, 40, vel);
    _mav_put_uint16_t(buf, 42, cog);
    _mav_put_uint8_t(buf, 44, fix_type);
    _mav_put_uint8_t(buf, 45, satellites_visible);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, buf, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#else
    mavlink_gps_data_t *packet = (mavlink_gps_data_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->h_acc = h_acc;
    packet->v_acc = v_acc;
    packet->vel_acc = vel_acc;
    packet->hdg_acc = hdg_acc;
    packet->eph = eph;
    packet->epv = epv;
    packet->vel = vel;
    packet->cog = cog;
    packet->fix_type = fix_type;
    packet->satellites_visible = satellites_visible;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_DATA, (const char *)packet, MAVLINK_MSG_ID_GPS_DATA_MIN_LEN, MAVLINK_MSG_ID_GPS_DATA_LEN, MAVLINK_MSG_ID_GPS_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_DATA UNPACKING


/**
 * @brief Get field time_usec from gps_data message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot)
 */
static inline uint32_t mavlink_msg_gps_data_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field fix_type from gps_data message
 *
 * @return  GPS fix type (0=no fix, 2=2D fix, 3=3D fix)
 */
static inline uint8_t mavlink_msg_gps_data_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field lat from gps_data message
 *
 * @return [degE7] Latitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_data_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lon from gps_data message
 *
 * @return [degE7] Longitude in degrees * 1E7
 */
static inline int32_t mavlink_msg_gps_data_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field alt from gps_data message
 *
 * @return [mm] Altitude (MSL) in millimeters
 */
static inline int32_t mavlink_msg_gps_data_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field eph from gps_data message
 *
 * @return [cm] GPS horizontal dilution of position (HDOP) in cm
 */
static inline uint16_t mavlink_msg_gps_data_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field epv from gps_data message
 *
 * @return [cm] GPS vertical dilution of position (VDOP) in cm
 */
static inline uint16_t mavlink_msg_gps_data_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Get field vel from gps_data message
 *
 * @return [cm/s] Ground speed in cm/s
 */
static inline uint16_t mavlink_msg_gps_data_get_vel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field cog from gps_data message
 *
 * @return [cdeg] Course over ground in degrees * 100
 */
static inline uint16_t mavlink_msg_gps_data_get_cog(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  42);
}

/**
 * @brief Get field satellites_visible from gps_data message
 *
 * @return  Number of satellites visible
 */
static inline uint8_t mavlink_msg_gps_data_get_satellites_visible(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field alt_ellipsoid from gps_data message
 *
 * @return [mm] Altitude (above WGS84 ellipsoid) in millimeters
 */
static inline int32_t mavlink_msg_gps_data_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field h_acc from gps_data message
 *
 * @return [mm] Position uncertainty in millimeters
 */
static inline uint32_t mavlink_msg_gps_data_get_h_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field v_acc from gps_data message
 *
 * @return [mm] Altitude uncertainty in millimeters
 */
static inline uint32_t mavlink_msg_gps_data_get_v_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field vel_acc from gps_data message
 *
 * @return [mm/s] Speed uncertainty in mm/s
 */
static inline uint32_t mavlink_msg_gps_data_get_vel_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field hdg_acc from gps_data message
 *
 * @return [degE5] Heading accuracy in degrees * 1E5
 */
static inline uint32_t mavlink_msg_gps_data_get_hdg_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Decode a gps_data message into a struct
 *
 * @param msg The message to decode
 * @param gps_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_data_decode(const mavlink_message_t* msg, mavlink_gps_data_t* gps_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gps_data->time_usec = mavlink_msg_gps_data_get_time_usec(msg);
    gps_data->lat = mavlink_msg_gps_data_get_lat(msg);
    gps_data->lon = mavlink_msg_gps_data_get_lon(msg);
    gps_data->alt = mavlink_msg_gps_data_get_alt(msg);
    gps_data->alt_ellipsoid = mavlink_msg_gps_data_get_alt_ellipsoid(msg);
    gps_data->h_acc = mavlink_msg_gps_data_get_h_acc(msg);
    gps_data->v_acc = mavlink_msg_gps_data_get_v_acc(msg);
    gps_data->vel_acc = mavlink_msg_gps_data_get_vel_acc(msg);
    gps_data->hdg_acc = mavlink_msg_gps_data_get_hdg_acc(msg);
    gps_data->eph = mavlink_msg_gps_data_get_eph(msg);
    gps_data->epv = mavlink_msg_gps_data_get_epv(msg);
    gps_data->vel = mavlink_msg_gps_data_get_vel(msg);
    gps_data->cog = mavlink_msg_gps_data_get_cog(msg);
    gps_data->fix_type = mavlink_msg_gps_data_get_fix_type(msg);
    gps_data->satellites_visible = mavlink_msg_gps_data_get_satellites_visible(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_DATA_LEN? msg->len : MAVLINK_MSG_ID_GPS_DATA_LEN;
        memset(gps_data, 0, MAVLINK_MSG_ID_GPS_DATA_LEN);
    memcpy(gps_data, _MAV_PAYLOAD(msg), len);
#endif
}
