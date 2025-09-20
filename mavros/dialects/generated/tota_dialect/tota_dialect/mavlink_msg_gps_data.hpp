// MESSAGE GPS_DATA support class

#pragma once

namespace mavlink {
namespace tota_dialect {
namespace msg {

/**
 * @brief GPS_DATA message
 *
 * GPS position and velocity data
 */
struct GPS_DATA : mavlink::Message {
    static constexpr msgid_t MSG_ID = 101;
    static constexpr size_t LENGTH = 46;
    static constexpr size_t MIN_LENGTH = 46;
    static constexpr uint8_t CRC_EXTRA = 28;
    static constexpr auto NAME = "GPS_DATA";


    uint32_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot) */
    uint8_t fix_type; /*<  GPS fix type (0=no fix, 2=2D fix, 3=3D fix) */
    int32_t lat; /*< [degE7] Latitude in degrees * 1E7 */
    int32_t lon; /*< [degE7] Longitude in degrees * 1E7 */
    int32_t alt; /*< [mm] Altitude (MSL) in millimeters */
    uint16_t eph; /*< [cm] GPS horizontal dilution of position (HDOP) in cm */
    uint16_t epv; /*< [cm] GPS vertical dilution of position (VDOP) in cm */
    uint16_t vel; /*< [cm/s] Ground speed in cm/s */
    uint16_t cog; /*< [cdeg] Course over ground in degrees * 100 */
    uint8_t satellites_visible; /*<  Number of satellites visible */
    int32_t alt_ellipsoid; /*< [mm] Altitude (above WGS84 ellipsoid) in millimeters */
    uint32_t h_acc; /*< [mm] Position uncertainty in millimeters */
    uint32_t v_acc; /*< [mm] Altitude uncertainty in millimeters */
    uint32_t vel_acc; /*< [mm/s] Speed uncertainty in mm/s */
    uint32_t hdg_acc; /*< [degE5] Heading accuracy in degrees * 1E5 */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  fix_type: " << +fix_type << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  eph: " << eph << std::endl;
        ss << "  epv: " << epv << std::endl;
        ss << "  vel: " << vel << std::endl;
        ss << "  cog: " << cog << std::endl;
        ss << "  satellites_visible: " << +satellites_visible << std::endl;
        ss << "  alt_ellipsoid: " << alt_ellipsoid << std::endl;
        ss << "  h_acc: " << h_acc << std::endl;
        ss << "  v_acc: " << v_acc << std::endl;
        ss << "  vel_acc: " << vel_acc << std::endl;
        ss << "  hdg_acc: " << hdg_acc << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << lat;                           // offset: 4
        map << lon;                           // offset: 8
        map << alt;                           // offset: 12
        map << alt_ellipsoid;                 // offset: 16
        map << h_acc;                         // offset: 20
        map << v_acc;                         // offset: 24
        map << vel_acc;                       // offset: 28
        map << hdg_acc;                       // offset: 32
        map << eph;                           // offset: 36
        map << epv;                           // offset: 38
        map << vel;                           // offset: 40
        map << cog;                           // offset: 42
        map << fix_type;                      // offset: 44
        map << satellites_visible;            // offset: 45
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> lat;                           // offset: 4
        map >> lon;                           // offset: 8
        map >> alt;                           // offset: 12
        map >> alt_ellipsoid;                 // offset: 16
        map >> h_acc;                         // offset: 20
        map >> v_acc;                         // offset: 24
        map >> vel_acc;                       // offset: 28
        map >> hdg_acc;                       // offset: 32
        map >> eph;                           // offset: 36
        map >> epv;                           // offset: 38
        map >> vel;                           // offset: 40
        map >> cog;                           // offset: 42
        map >> fix_type;                      // offset: 44
        map >> satellites_visible;            // offset: 45
    }
};

} // namespace msg
} // namespace tota_dialect
} // namespace mavlink
