/**
 * @brief TOF L7CX Ranges plugin
 * @file tof_ranges.cpp
 * @author Danish
 *
 * Plugin for TOF_L7CX_RANGES custom MAVLink message (ID: 42001)
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief TOF L7CX Ranges plugin
 * 
 * Handles TOF_L7CX_RANGES messages (ID: 42001) containing 8x8 zone distances
 * from VL53L7CX sensor and publishes them as ROS2 topics.
 */
class TofRangesPlugin : public plugin::Plugin {
public:
    explicit TofRangesPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "tof_ranges")
    {
        enable_node_watch_parameters();

        // Declare and watch parameters
        raw_topic = node_declare_and_watch_parameter(
            "raw_topic", "~/raw",
            [this](const rclcpp::Parameter & p) {
                raw_topic = p.as_string();
                RCLCPP_INFO(get_logger(), "TOF raw_topic updated to: %s", raw_topic.c_str());
            }
        );

        field_of_view = node_declare_and_watch_parameter(
            "field_of_view", 0.785,  // 45 degrees default
            [this](const rclcpp::Parameter & p) {
                field_of_view = p.as_double();
                RCLCPP_INFO(get_logger(), "TOF field_of_view updated to: %.3f rad", field_of_view);
            }
        );

        // Publishers for different representations
        ranges_raw_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(raw_topic, 10);
        ranges_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", 10);

        RCLCPP_INFO(get_logger(), "TOF Ranges plugin initialized - FOV: %.3f rad", field_of_view);
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&TofRangesPlugin::handle_tof_ranges)
        };
    }
    
    void connection_cb(bool connected) override
    {
        if (!connected) {
            return;
        }
        
        RCLCPP_INFO(get_logger(), "TOF Ranges: FCU connection established");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ranges_raw_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ranges_cloud_pub;

    bool first_msg_received = false;
    std::string raw_topic;
    double field_of_view;

    void handle_tof_ranges(
        const mavlink::mavlink_message_t * msg [[maybe_unused]],
        mavlink::tota_dialect::msg::TOF_L7CX_RANGES & tof_msg,
        plugin::filter::SystemAndOk filter [[maybe_unused]])
    {
        if (!first_msg_received) {
            RCLCPP_INFO(get_logger(), "Received first TOF_L7CX_RANGES message!");
            first_msg_received = true;
        }

        // Convert to ROS2 Float32MultiArray
        auto ranges_msg = std_msgs::msg::Float32MultiArray();
        ranges_msg.layout.dim.resize(2);
        ranges_msg.layout.dim[0].label = "rows";
        ranges_msg.layout.dim[0].size = 8;
        ranges_msg.layout.dim[0].stride = 64;
        ranges_msg.layout.dim[1].label = "cols";
        ranges_msg.layout.dim[1].size = 8;
        ranges_msg.layout.dim[1].stride = 8;
        
        // Convert mm to meters
        ranges_msg.data.reserve(64);
        for (int i = 0; i < 64; i++) {
            ranges_msg.data.push_back(tof_msg.range_mm[i] / 1000.0f);
        }
        
        ranges_raw_pub->publish(ranges_msg);
        
        // Also publish as point cloud
        publish_point_cloud(tof_msg);
    }

    void publish_point_cloud(const mavlink::tota_dialect::msg::TOF_L7CX_RANGES & tof_msg)
    {
        auto cloud_msg = sensor_msgs::msg::PointCloud2();
        
        // Set header
        cloud_msg.header.stamp = uas->now();
        cloud_msg.header.frame_id = "tof_sensor_frame";
        
        // Define fields
        cloud_msg.height = 8;
        cloud_msg.width = 8;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 12; // 3 floats (x, y, z)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        
        // Setup fields
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        // Fill point cloud data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        
        // Assume 45 degree field of view, centered
        const float fov = 45.0f * M_PI / 180.0f;
        const float cell_angle = fov / 8.0f;
        const float start_angle = -fov / 2.0f + cell_angle / 2.0f;
        
        for (int row = 0; row < 8; row++) {
            for (int col = 0; col < 8; col++) {
                int idx = row * 8 + col;
                float distance = tof_msg.range_mm[idx] / 1000.0f; // Convert to meters
                
                // Calculate angles for this zone
                float angle_x = start_angle + col * cell_angle;
                float angle_y = start_angle + row * cell_angle;
                
                // Convert to Cartesian coordinates
                *iter_x = distance * tan(angle_x);
                *iter_y = distance * tan(angle_y);
                *iter_z = distance;
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }
        
        ranges_cloud_pub->publish(cloud_msg);
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::TofRangesPlugin)