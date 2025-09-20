/*
 * VTX Control Plugin
 * Handles video transmitter control messages for MAVROS
 */

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "tota_interfaces/msg/vtx_control.hpp"

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * @brief VTX Control plugin
 * @plugin vtx_control
 *
 * This plugin subscribes to VTXControl messages and sends them as RC_CHANNELS
 * MAVLink messages at 1Hz. It also publishes the current status.
 */
class VTXControlPlugin : public plugin::Plugin
{
public:
  explicit VTXControlPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "vtx_control")
  {
    // Subscribe to VTX control toggle topic
    vtx_toggle_sub = node->create_subscription<tota_interfaces::msg::VTXControl>(
      "~/toggle", 10,
      std::bind(&VTXControlPlugin::vtx_toggle_cb, this, _1));

    // Publisher for VTX control status
    vtx_status_pub = node->create_publisher<tota_interfaces::msg::VTXControl>(
      "~/status", 10);

    // Timer for sending RC_CHANNELS at 1Hz
    timer_ = node->create_wall_timer(
      1s, std::bind(&VTXControlPlugin::timer_callback, this));

    // Initialize with default values
    initialize_default_values();

    RCLCPP_INFO(get_logger(), "VTX Control plugin initialized");
  }

  Subscriptions get_subscriptions() override
  {
    return {};
  }

private:
  // Mutex for thread safety
  std::mutex mutex_;

  // Current VTX control values (stores absolute values from ROS topic)
  tota_interfaces::msg::VTXControl current_vtx_control_;

  // Helper functions to convert between absolute values and enum values
  uint16_t resolution_abs_to_enum(uint16_t abs_val) {
    switch(abs_val) {
      case 480: return tota_interfaces::msg::VTXControl::RES_480P;
      case 720: return tota_interfaces::msg::VTXControl::RES_720P;
      case 1080: return tota_interfaces::msg::VTXControl::RES_1080P;
      default: return tota_interfaces::msg::VTXControl::RES_720P;
    }
  }

  uint16_t resolution_enum_to_abs(uint16_t enum_val) {
    switch(enum_val) {
      case tota_interfaces::msg::VTXControl::RES_480P: return 480;
      case tota_interfaces::msg::VTXControl::RES_720P: return 720;
      case tota_interfaces::msg::VTXControl::RES_1080P: return 1080;
      default: return 720;
    }
  }

  uint16_t fps_abs_to_enum(uint16_t abs_val) {
    switch(abs_val) {
      case 20: return tota_interfaces::msg::VTXControl::FPS_20;
      case 30: return tota_interfaces::msg::VTXControl::FPS_30;
      case 60: return tota_interfaces::msg::VTXControl::FPS_60;
      case 90: return tota_interfaces::msg::VTXControl::FPS_90;
      case 120: return tota_interfaces::msg::VTXControl::FPS_120;
      default: return tota_interfaces::msg::VTXControl::FPS_60;
    }
  }

  uint16_t fps_enum_to_abs(uint16_t enum_val) {
    switch(enum_val) {
      case tota_interfaces::msg::VTXControl::FPS_20: return 20;
      case tota_interfaces::msg::VTXControl::FPS_30: return 30;
      case tota_interfaces::msg::VTXControl::FPS_60: return 60;
      case tota_interfaces::msg::VTXControl::FPS_90: return 90;
      case tota_interfaces::msg::VTXControl::FPS_120: return 120;
      default: return 60;
    }
  }

  uint16_t bitrate_abs_to_enum(uint16_t abs_val) {
    switch(abs_val) {
      case 1024: return tota_interfaces::msg::VTXControl::BITRATE_1024;
      case 2048: return tota_interfaces::msg::VTXControl::BITRATE_2048;
      case 4096: return tota_interfaces::msg::VTXControl::BITRATE_4096;
      case 8196: return tota_interfaces::msg::VTXControl::BITRATE_8196;
      default: return tota_interfaces::msg::VTXControl::BITRATE_2048;
    }
  }

  uint16_t bitrate_enum_to_abs(uint16_t enum_val) {
    switch(enum_val) {
      case tota_interfaces::msg::VTXControl::BITRATE_1024: return 1024;
      case tota_interfaces::msg::VTXControl::BITRATE_2048: return 2048;
      case tota_interfaces::msg::VTXControl::BITRATE_4096: return 4096;
      case tota_interfaces::msg::VTXControl::BITRATE_8196: return 8196;
      default: return 2048;
    }
  }

  uint16_t zoom_abs_to_enum(uint16_t abs_val) {
    // Zoom values are already the same for both absolute and enum
    return abs_val;
  }

  uint16_t zoom_enum_to_abs(uint16_t enum_val) {
    // Zoom values are already the same for both absolute and enum
    return enum_val;
  }

  uint16_t mcs_abs_to_enum(uint16_t abs_val) {
    // MCS values are already the same for both absolute and enum
    return abs_val;
  }

  // ROS2 subscribers and publishers
  rclcpp::Subscription<tota_interfaces::msg::VTXControl>::SharedPtr vtx_toggle_sub;
  rclcpp::Publisher<tota_interfaces::msg::VTXControl>::SharedPtr vtx_status_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  // Constants
  static constexpr uint8_t RC_CHANNELS_COUNT = 18;
  static constexpr uint8_t RSSI_UNKNOWN = 255;

  void initialize_default_values()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Set default values using absolute values
    current_vtx_control_.resolution = 720;  // 720p
    current_vtx_control_.exposure_mask = tota_interfaces::msg::VTXControl::EXP_AUTO_OFF;
    current_vtx_control_.exposure_value = 0;
    current_vtx_control_.zoom = 1;  // 1x zoom
    current_vtx_control_.night_mode = false;
    current_vtx_control_.fps = 60;  // 60 fps
    current_vtx_control_.bitrate = 2048;  // 2048 kbps
    current_vtx_control_.adaptive_link = false;
    current_vtx_control_.mcs = 1;  // MCS 1
    current_vtx_control_.mic_toggle = true;
    current_vtx_control_.speaker_toggle = false;
    current_vtx_control_.sd_card_recording = false;
  }

  void vtx_toggle_cb(const tota_interfaces::msg::VTXControl::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Store the absolute values directly from the ROS message
    current_vtx_control_ = *msg;

    RCLCPP_DEBUG(get_logger(),
      "VTX Control updated - Res: %dp, FPS: %d, Zoom: %dx, Bitrate: %d kbps",
      current_vtx_control_.resolution,
      current_vtx_control_.fps,
      current_vtx_control_.zoom,
      current_vtx_control_.bitrate);
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Send RC_CHANNELS message
    send_rc_channels();

    // Publish current status
    vtx_status_pub->publish(current_vtx_control_);
  }

  void send_rc_channels()
  {
    // Create RC_CHANNELS message
    mavlink::common::msg::RC_CHANNELS rc_channels{};

    // Get current timestamp in milliseconds
    auto now = std::chrono::steady_clock::now();
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count();

    rc_channels.time_boot_ms = static_cast<uint32_t>(ms_since_epoch);
    rc_channels.chancount = RC_CHANNELS_COUNT;

    // Map VTX control values to RC channels - convert absolute to enum for MAVLink
    rc_channels.chan1_raw = resolution_abs_to_enum(current_vtx_control_.resolution);
    rc_channels.chan2_raw = current_vtx_control_.exposure_mask;
    rc_channels.chan3_raw = current_vtx_control_.exposure_value;
    rc_channels.chan4_raw = zoom_abs_to_enum(current_vtx_control_.zoom);
    rc_channels.chan5_raw = current_vtx_control_.night_mode ? 1 : 0;
    rc_channels.chan6_raw = fps_abs_to_enum(current_vtx_control_.fps);
    rc_channels.chan7_raw = bitrate_abs_to_enum(current_vtx_control_.bitrate);
    rc_channels.chan8_raw = current_vtx_control_.adaptive_link ? 1 : 0;
    rc_channels.chan9_raw = mcs_abs_to_enum(current_vtx_control_.mcs);
    rc_channels.chan10_raw = current_vtx_control_.mic_toggle ? 1 : 0;
    rc_channels.chan11_raw = current_vtx_control_.speaker_toggle ? 1 : 0;
    rc_channels.chan12_raw = current_vtx_control_.sd_card_recording ? 1 : 0;

    // Extra channels set to 0
    rc_channels.chan13_raw = 0;
    rc_channels.chan14_raw = 0;
    rc_channels.chan15_raw = 0;
    rc_channels.chan16_raw = 0;
    rc_channels.chan17_raw = 0;
    rc_channels.chan18_raw = 0;

    // Set RSSI to unknown
    rc_channels.rssi = RSSI_UNKNOWN;

    // Send the message
    uas->send_message(rc_channels);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
      "Sending RC_CHANNELS: Ch1=%d, Ch2=%d, Ch3=%d, Ch4=%d, Ch5=%d, Ch6=%d",
      rc_channels.chan1_raw, rc_channels.chan2_raw, rc_channels.chan3_raw,
      rc_channels.chan4_raw, rc_channels.chan5_raw, rc_channels.chan6_raw);
  }
};

} // namespace std_plugins
} // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::VTXControlPlugin)