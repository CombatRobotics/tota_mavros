/** @file
 *	@brief MAVLink comm testsuite protocol generated from tota_dialect.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "tota_dialect.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(tota_dialect, HEARTBEAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.autopilot = 84;
    packet_in.base_mode = 151;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 218;
    packet_in.mavlink_version = 29;

    mavlink::tota_dialect::msg::HEARTBEAT packet1{};
    mavlink::tota_dialect::msg::HEARTBEAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.autopilot, packet2.autopilot);
    EXPECT_EQ(packet1.base_mode, packet2.base_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.system_status, packet2.system_status);
    EXPECT_EQ(packet1.mavlink_version, packet2.mavlink_version);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, HEARTBEAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_heartbeat_t packet_c {
         963497464, 17, 84, 151, 218, 29
    };

    mavlink::tota_dialect::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.autopilot = 84;
    packet_in.base_mode = 151;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 218;
    packet_in.mavlink_version = 29;

    mavlink::tota_dialect::msg::HEARTBEAT packet2{};

    mavlink_msg_heartbeat_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.autopilot, packet2.autopilot);
    EXPECT_EQ(packet_in.base_mode, packet2.base_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.system_status, packet2.system_status);
    EXPECT_EQ(packet_in.mavlink_version, packet2.mavlink_version);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, ATTITUDE_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::ATTITUDE_DATA packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::tota_dialect::msg::ATTITUDE_DATA packet1{};
    mavlink::tota_dialect::msg::ATTITUDE_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, ATTITUDE_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_data_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::tota_dialect::msg::ATTITUDE_DATA packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::tota_dialect::msg::ATTITUDE_DATA packet2{};

    mavlink_msg_attitude_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, GPS_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::GPS_DATA packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.fix_type = 137;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.eph = 19107;
    packet_in.epv = 19211;
    packet_in.vel = 19315;
    packet_in.cog = 19419;
    packet_in.satellites_visible = 204;
    packet_in.alt_ellipsoid = 963498296;
    packet_in.h_acc = 963498504;
    packet_in.v_acc = 963498712;
    packet_in.vel_acc = 963498920;
    packet_in.hdg_acc = 963499128;

    mavlink::tota_dialect::msg::GPS_DATA packet1{};
    mavlink::tota_dialect::msg::GPS_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.fix_type, packet2.fix_type);
    EXPECT_EQ(packet1.lat, packet2.lat);
    EXPECT_EQ(packet1.lon, packet2.lon);
    EXPECT_EQ(packet1.alt, packet2.alt);
    EXPECT_EQ(packet1.eph, packet2.eph);
    EXPECT_EQ(packet1.epv, packet2.epv);
    EXPECT_EQ(packet1.vel, packet2.vel);
    EXPECT_EQ(packet1.cog, packet2.cog);
    EXPECT_EQ(packet1.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet1.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet1.h_acc, packet2.h_acc);
    EXPECT_EQ(packet1.v_acc, packet2.v_acc);
    EXPECT_EQ(packet1.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet1.hdg_acc, packet2.hdg_acc);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, GPS_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_gps_data_t packet_c {
         963497464, 963497672, 963497880, 963498088, 963498296, 963498504, 963498712, 963498920, 963499128, 19107, 19211, 19315, 19419, 137, 204
    };

    mavlink::tota_dialect::msg::GPS_DATA packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.fix_type = 137;
    packet_in.lat = 963497672;
    packet_in.lon = 963497880;
    packet_in.alt = 963498088;
    packet_in.eph = 19107;
    packet_in.epv = 19211;
    packet_in.vel = 19315;
    packet_in.cog = 19419;
    packet_in.satellites_visible = 204;
    packet_in.alt_ellipsoid = 963498296;
    packet_in.h_acc = 963498504;
    packet_in.v_acc = 963498712;
    packet_in.vel_acc = 963498920;
    packet_in.hdg_acc = 963499128;

    mavlink::tota_dialect::msg::GPS_DATA packet2{};

    mavlink_msg_gps_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.fix_type, packet2.fix_type);
    EXPECT_EQ(packet_in.lat, packet2.lat);
    EXPECT_EQ(packet_in.lon, packet2.lon);
    EXPECT_EQ(packet_in.alt, packet2.alt);
    EXPECT_EQ(packet_in.eph, packet2.eph);
    EXPECT_EQ(packet_in.epv, packet2.epv);
    EXPECT_EQ(packet_in.vel, packet2.vel);
    EXPECT_EQ(packet_in.cog, packet2.cog);
    EXPECT_EQ(packet_in.satellites_visible, packet2.satellites_visible);
    EXPECT_EQ(packet_in.alt_ellipsoid, packet2.alt_ellipsoid);
    EXPECT_EQ(packet_in.h_acc, packet2.h_acc);
    EXPECT_EQ(packet_in.v_acc, packet2.v_acc);
    EXPECT_EQ(packet_in.vel_acc, packet2.vel_acc);
    EXPECT_EQ(packet_in.hdg_acc, packet2.hdg_acc);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, BATTERY_DATA)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::BATTERY_DATA packet_in{};
    packet_in.id = 113;
    packet_in.battery_function = 180;
    packet_in.type = 247;
    packet_in.temperature = 17859;
    packet_in.voltages = {{ 17963, 17964, 17965, 17966, 17967, 17968, 17969, 17970, 17971, 17972 }};
    packet_in.current_battery = 19003;
    packet_in.current_consumed = 963497464;
    packet_in.energy_consumed = 963497672;
    packet_in.battery_remaining = 58;
    packet_in.time_remaining = 963497880;
    packet_in.charge_state = 125;

    mavlink::tota_dialect::msg::BATTERY_DATA packet1{};
    mavlink::tota_dialect::msg::BATTERY_DATA packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.id, packet2.id);
    EXPECT_EQ(packet1.battery_function, packet2.battery_function);
    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.temperature, packet2.temperature);
    EXPECT_EQ(packet1.voltages, packet2.voltages);
    EXPECT_EQ(packet1.current_battery, packet2.current_battery);
    EXPECT_EQ(packet1.current_consumed, packet2.current_consumed);
    EXPECT_EQ(packet1.energy_consumed, packet2.energy_consumed);
    EXPECT_EQ(packet1.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet1.time_remaining, packet2.time_remaining);
    EXPECT_EQ(packet1.charge_state, packet2.charge_state);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, BATTERY_DATA)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_battery_data_t packet_c {
         963497464, 963497672, 963497880, 17859, { 17963, 17964, 17965, 17966, 17967, 17968, 17969, 17970, 17971, 17972 }, 19003, 113, 180, 247, 58, 125
    };

    mavlink::tota_dialect::msg::BATTERY_DATA packet_in{};
    packet_in.id = 113;
    packet_in.battery_function = 180;
    packet_in.type = 247;
    packet_in.temperature = 17859;
    packet_in.voltages = {{ 17963, 17964, 17965, 17966, 17967, 17968, 17969, 17970, 17971, 17972 }};
    packet_in.current_battery = 19003;
    packet_in.current_consumed = 963497464;
    packet_in.energy_consumed = 963497672;
    packet_in.battery_remaining = 58;
    packet_in.time_remaining = 963497880;
    packet_in.charge_state = 125;

    mavlink::tota_dialect::msg::BATTERY_DATA packet2{};

    mavlink_msg_battery_data_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.id, packet2.id);
    EXPECT_EQ(packet_in.battery_function, packet2.battery_function);
    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.temperature, packet2.temperature);
    EXPECT_EQ(packet_in.voltages, packet2.voltages);
    EXPECT_EQ(packet_in.current_battery, packet2.current_battery);
    EXPECT_EQ(packet_in.current_consumed, packet2.current_consumed);
    EXPECT_EQ(packet_in.energy_consumed, packet2.energy_consumed);
    EXPECT_EQ(packet_in.battery_remaining, packet2.battery_remaining);
    EXPECT_EQ(packet_in.time_remaining, packet2.time_remaining);
    EXPECT_EQ(packet_in.charge_state, packet2.charge_state);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, PING)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::PING packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.seq = 963497672;
    packet_in.target_system = 29;
    packet_in.target_component = 96;

    mavlink::tota_dialect::msg::PING packet1{};
    mavlink::tota_dialect::msg::PING packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.seq, packet2.seq);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, PING)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_ping_t packet_c {
         963497464, 963497672, 29, 96
    };

    mavlink::tota_dialect::msg::PING packet_in{};
    packet_in.time_usec = 963497464;
    packet_in.seq = 963497672;
    packet_in.target_system = 29;
    packet_in.target_component = 96;

    mavlink::tota_dialect::msg::PING packet2{};

    mavlink_msg_ping_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.seq, packet2.seq);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, WHEEL_ENCODERS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.left_ticks = 963497672;
    packet_in.right_ticks = 963497880;
    packet_in.cam_pos = 17859;

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet1{};
    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.left_ticks, packet2.left_ticks);
    EXPECT_EQ(packet1.right_ticks, packet2.right_ticks);
    EXPECT_EQ(packet1.cam_pos, packet2.cam_pos);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, WHEEL_ENCODERS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_wheel_encoders_t packet_c {
         963497464, 963497672, 963497880, 17859
    };

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.left_ticks = 963497672;
    packet_in.right_ticks = 963497880;
    packet_in.cam_pos = 17859;

    mavlink::tota_dialect::msg::WHEEL_ENCODERS packet2{};

    mavlink_msg_wheel_encoders_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.left_ticks, packet2.left_ticks);
    EXPECT_EQ(packet_in.right_ticks, packet2.right_ticks);
    EXPECT_EQ(packet_in.cam_pos, packet2.cam_pos);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, TOF_L7CX_RANGES)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet_in{};
    packet_in.time_us = 963497464;
    packet_in.range_mm = {{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450, 17451, 17452, 17453, 17454, 17455, 17456, 17457, 17458, 17459, 17460, 17461, 17462, 17463, 17464, 17465, 17466, 17467, 17468, 17469, 17470, 17471, 17472, 17473, 17474, 17475, 17476, 17477, 17478, 17479, 17480, 17481, 17482, 17483, 17484, 17485, 17486, 17487, 17488, 17489, 17490, 17491, 17492, 17493, 17494, 17495, 17496, 17497, 17498, 17499, 17500, 17501, 17502, 17503, 17504, 17505, 17506 }};

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet1{};
    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.range_mm, packet2.range_mm);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, TOF_L7CX_RANGES)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_tof_l7cx_ranges_t packet_c {
         963497464, { 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450, 17451, 17452, 17453, 17454, 17455, 17456, 17457, 17458, 17459, 17460, 17461, 17462, 17463, 17464, 17465, 17466, 17467, 17468, 17469, 17470, 17471, 17472, 17473, 17474, 17475, 17476, 17477, 17478, 17479, 17480, 17481, 17482, 17483, 17484, 17485, 17486, 17487, 17488, 17489, 17490, 17491, 17492, 17493, 17494, 17495, 17496, 17497, 17498, 17499, 17500, 17501, 17502, 17503, 17504, 17505, 17506 }
    };

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet_in{};
    packet_in.time_us = 963497464;
    packet_in.range_mm = {{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450, 17451, 17452, 17453, 17454, 17455, 17456, 17457, 17458, 17459, 17460, 17461, 17462, 17463, 17464, 17465, 17466, 17467, 17468, 17469, 17470, 17471, 17472, 17473, 17474, 17475, 17476, 17477, 17478, 17479, 17480, 17481, 17482, 17483, 17484, 17485, 17486, 17487, 17488, 17489, 17490, 17491, 17492, 17493, 17494, 17495, 17496, 17497, 17498, 17499, 17500, 17501, 17502, 17503, 17504, 17505, 17506 }};

    mavlink::tota_dialect::msg::TOF_L7CX_RANGES packet2{};

    mavlink_msg_tof_l7cx_ranges_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.range_mm, packet2.range_mm);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, SENSOR_INIT_STATE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::SENSOR_INIT_STATE packet_in{};
    packet_in.time_us = 963497464;
    packet_in.sensor_init = 963497672;

    mavlink::tota_dialect::msg::SENSOR_INIT_STATE packet1{};
    mavlink::tota_dialect::msg::SENSOR_INIT_STATE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.sensor_init, packet2.sensor_init);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, SENSOR_INIT_STATE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_sensor_init_state_t packet_c {
         963497464, 963497672
    };

    mavlink::tota_dialect::msg::SENSOR_INIT_STATE packet_in{};
    packet_in.time_us = 963497464;
    packet_in.sensor_init = 963497672;

    mavlink::tota_dialect::msg::SENSOR_INIT_STATE packet2{};

    mavlink_msg_sensor_init_state_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.sensor_init, packet2.sensor_init);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, ARMING_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::ARMING_STATUS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.armed = 17;

    mavlink::tota_dialect::msg::ARMING_STATUS packet1{};
    mavlink::tota_dialect::msg::ARMING_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.armed, packet2.armed);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, ARMING_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_arming_status_t packet_c {
         963497464, 17
    };

    mavlink::tota_dialect::msg::ARMING_STATUS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.armed = 17;

    mavlink::tota_dialect::msg::ARMING_STATUS packet2{};

    mavlink_msg_arming_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.armed, packet2.armed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(tota_dialect, DRIVE_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::tota_dialect::msg::DRIVE_STATUS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.arm_disarm = 29;
    packet_in.rpm_left_wheel = 17443;
    packet_in.rpm_right_wheel = 17547;
    packet_in.jumping = 96;

    mavlink::tota_dialect::msg::DRIVE_STATUS packet1{};
    mavlink::tota_dialect::msg::DRIVE_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_us, packet2.time_us);
    EXPECT_EQ(packet1.arm_disarm, packet2.arm_disarm);
    EXPECT_EQ(packet1.rpm_left_wheel, packet2.rpm_left_wheel);
    EXPECT_EQ(packet1.rpm_right_wheel, packet2.rpm_right_wheel);
    EXPECT_EQ(packet1.jumping, packet2.jumping);
}

#ifdef TEST_INTEROP
TEST(tota_dialect_interop, DRIVE_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_drive_status_t packet_c {
         963497464, 17443, 17547, 29, 96
    };

    mavlink::tota_dialect::msg::DRIVE_STATUS packet_in{};
    packet_in.time_us = 963497464;
    packet_in.arm_disarm = 29;
    packet_in.rpm_left_wheel = 17443;
    packet_in.rpm_right_wheel = 17547;
    packet_in.jumping = 96;

    mavlink::tota_dialect::msg::DRIVE_STATUS packet2{};

    mavlink_msg_drive_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_us, packet2.time_us);
    EXPECT_EQ(packet_in.arm_disarm, packet2.arm_disarm);
    EXPECT_EQ(packet_in.rpm_left_wheel, packet2.rpm_left_wheel);
    EXPECT_EQ(packet_in.rpm_right_wheel, packet2.rpm_right_wheel);
    EXPECT_EQ(packet_in.jumping, packet2.jumping);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
