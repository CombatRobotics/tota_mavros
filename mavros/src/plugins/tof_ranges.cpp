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

        // Total field of view in degrees (will be converted to radians)
        fov_horizontal = node_declare_and_watch_parameter(
            "fov_horizontal", 60.0,  // 60 degrees default
            [this](const rclcpp::Parameter & p) {
                fov_horizontal = p.as_double() * M_PI / 180.0;  // Convert to radians
                RCLCPP_INFO(get_logger(), "TOF fov_horizontal updated to: %.1f degrees (%.3f rad)",
                           p.as_double(), fov_horizontal);
            }
        );

        fov_vertical = node_declare_and_watch_parameter(
            "fov_vertical", 60.0,  // 60 degrees default
            [this](const rclcpp::Parameter & p) {
                fov_vertical = p.as_double() * M_PI / 180.0;  // Convert to radians
                RCLCPP_INFO(get_logger(), "TOF fov_vertical updated to: %.1f degrees (%.3f rad)",
                           p.as_double(), fov_vertical);
            }
        );

        // Resolution parameter (4x4 or 8x8)
        resolution = node_declare_and_watch_parameter(
            "resolution", 8,  // 8x8 default
            [this](const rclcpp::Parameter & p) {
                int res = p.as_int();
                if (res != 4 && res != 8) {
                    RCLCPP_WARN(get_logger(), "Invalid resolution %d. Using default 8x8", res);
                    resolution = 8;
                } else {
                    resolution = res;
                    RCLCPP_INFO(get_logger(), "TOF resolution updated to: %dx%d", resolution, resolution);
                }
            }
        );

        // Coordinate frame orientation ("x_forward" or "z_forward")
        orientation = node_declare_and_watch_parameter(
            "orientation", "z_forward",  // Default z_forward for compatibility
            [this](const rclcpp::Parameter & p) {
                orientation = p.as_string();
                if (orientation != "x_forward" && orientation != "z_forward") {
                    RCLCPP_WARN(get_logger(), "Invalid orientation '%s'. Using default 'z_forward'", orientation.c_str());
                    orientation = "z_forward";
                } else {
                    RCLCPP_INFO(get_logger(), "TOF orientation updated to: %s", orientation.c_str());
                }
            }
        );

        // Convert degree parameters to radians for initial values
        fov_horizontal = fov_horizontal * M_PI / 180.0;
        fov_vertical = fov_vertical * M_PI / 180.0;

        // Publishers for different representations
        ranges_raw_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>(raw_topic, 10);
        ranges_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", 10);

        RCLCPP_INFO(get_logger(), "TOF Ranges plugin initialized - FOV: %.1fx%.1f deg, Resolution: %dx%d, Orientation: %s",
                   fov_horizontal * 180.0 / M_PI, fov_vertical * 180.0 / M_PI,
                   resolution, resolution, orientation.c_str());
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
    double fov_horizontal;  // Total horizontal field of view in radians
    double fov_vertical;    // Total vertical field of view in radians
    int resolution;         // Grid resolution (4 or 8)
    std::string orientation;  // "x_forward" or "z_forward"

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
        ranges_msg.layout.dim[0].size = resolution;
        ranges_msg.layout.dim[0].stride = resolution * resolution;
        ranges_msg.layout.dim[1].label = "cols";
        ranges_msg.layout.dim[1].size = resolution;
        ranges_msg.layout.dim[1].stride = resolution;

        // Convert mm to meters
        int total_cells = resolution * resolution;
        ranges_msg.data.reserve(total_cells);
        for (int i = 0; i < total_cells && i < 64; i++) {
            ranges_msg.data.push_back(tof_msg.range_mm[i] / 1000.0f);
        }
        
        ranges_raw_pub->publish(ranges_msg);
        
        // Also publish as point cloud
        publish_point_cloud(tof_msg);
    }

    void publish_point_cloud(const mavlink::tota_dialect::msg::TOF_L7CX_RANGES & tof_msg)
    {
        auto cloud_msg = sensor_msgs::msg::PointCloud2();

        // Set header with appropriate frame based on orientation
        cloud_msg.header.stamp = uas->now();
        cloud_msg.header.frame_id = (orientation == "x_forward") ? "tof_sensor_x_forward" : "tof_sensor_z_forward";

        // Define fields based on resolution
        cloud_msg.height = resolution;
        cloud_msg.width = resolution;
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

        // Use total field of view parameters
        const float angular_step_h = fov_horizontal / float(resolution);
        const float angular_step_v = fov_vertical / float(resolution);
        const float half_res = (resolution - 1) / 2.0f;

        for (int row = 0; row < resolution; row++) {
            for (int col = 0; col < resolution; col++) {
                int idx = row * resolution + col;
                if (idx >= 64) break;  // Safety check for array bounds

                float distance = tof_msg.range_mm[idx] / 1000.0f; // Convert to meters

                // Calculate angles for this zone (centered around 0)
                float azimuth = (col - half_res) * angular_step_h;   // Horizontal angle
                float elevation = (row - half_res) * angular_step_v;  // Vertical angle

                if (orientation == "x_forward") {
                    // X-forward: +X points forward (out of sensor), +Y right, +Z up
                    // Similar to the reference Python code
                    *iter_x = distance;                        // Forward
                    *iter_y = distance * tan(azimuth);        // Right
                    *iter_z = distance * tan(elevation);      // Up
                } else {  // z_forward (default)
                    // Z-forward: +Z points forward (out of sensor), +X right, +Y up
                    // Traditional sensor frame
                    *iter_z = distance;                        // Forward
                    *iter_x = distance * tan(azimuth);        // Right
                    *iter_y = -distance * tan(elevation);     // Up (negative for ROS convention)
                }

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