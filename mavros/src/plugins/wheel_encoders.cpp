/**
 * @brief Wheel Encoders plugin
 * @file wheel_encoders.cpp
 * @author Danish
 *
 * Plugin for WHEEL_ENCODERS custom MAVLink message (ID: 42003)
 */

#include <mavros/mavros_uas.hpp>
#include <mavros/plugin.hpp>
#include <mavros/plugin_filter.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace mavros {
namespace extra_plugins {

/**
 * @brief Wheel Encoders plugin
 * 
 * Handles WHEEL_ENCODERS messages (ID: 42003) containing wheel encoder ticks
 * and publishes them as ROS2 topics.
 */
class WheelEncodersPlugin : public plugin::Plugin {
public:
    explicit WheelEncodersPlugin(plugin::UASPtr uas_)
        : Plugin(uas_, "wheel_encoders")
    {
        enable_node_watch_parameters();

        // Declare and watch parameters
        joint_states_topic = node_declare_and_watch_parameter(
            "joint_states_topic", "~/joint_states",
            [this](const rclcpp::Parameter & p) {
                joint_states_topic = p.as_string();
                RCLCPP_INFO(get_logger(), "Wheel encoders joint_states_topic updated to: %s", joint_states_topic.c_str());
            }
        );

        ticks_per_revolution = node_declare_and_watch_parameter(
            "ticks_per_revolution", 1024.0,
            [this](const rclcpp::Parameter & p) {
                ticks_per_revolution = p.as_double();
                RCLCPP_INFO(get_logger(), "Wheel encoders ticks_per_revolution updated to: %.1f", ticks_per_revolution);
            }
        );

        // Publishers
        joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 10);
        raw_ticks_pub = node->create_publisher<std_msgs::msg::Int32MultiArray>("~/raw_ticks", 10);

        // Initialize last tick values for velocity calculation
        last_left_ticks = 0;
        last_right_ticks = 0;
        last_msg_time = node->now();

        RCLCPP_INFO(get_logger(), "Wheel Encoders plugin initialized - TPR: %.1f", ticks_per_revolution);
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&WheelEncodersPlugin::handle_wheel_encoders)
        };
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_ticks_pub;

    int32_t last_left_ticks;
    int32_t last_right_ticks;
    rclcpp::Time last_msg_time;
    bool first_msg_received = false;

    // Parameters
    std::string joint_states_topic;
    double ticks_per_revolution;
    const double WHEEL_RADIUS = 0.05;  // meters

    void handle_wheel_encoders(
        const mavlink::mavlink_message_t * msg [[maybe_unused]],
        mavlink::tota_dialect::msg::WHEEL_ENCODERS & enc_msg,
        plugin::filter::SystemAndOk filter [[maybe_unused]])
    {
        if (!first_msg_received) {
            RCLCPP_INFO(get_logger(), "Received first WHEEL_ENCODERS message!");
            first_msg_received = true;
            last_left_ticks = enc_msg.left_ticks;
            last_right_ticks = enc_msg.right_ticks;
            last_msg_time = node->now();
            return;
        }

        // Publish raw ticks
        auto raw_msg = std_msgs::msg::Int32MultiArray();
        raw_msg.data.resize(3);
        raw_msg.data[0] = enc_msg.left_ticks;
        raw_msg.data[1] = enc_msg.right_ticks;
        raw_msg.data[2] = enc_msg.cam_pos;  // CAM position (12-bit)
        raw_ticks_pub->publish(raw_msg);
        
        // Calculate joint states
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = node->now();
        joint_msg.header.frame_id = "base_link";
        
        // Add wheel joints
        joint_msg.name.push_back("left_wheel_joint");
        joint_msg.name.push_back("right_wheel_joint");
        joint_msg.name.push_back("cam_joint");
        
        // Calculate positions (radians)
        double left_pos = (enc_msg.left_ticks / ticks_per_revolution) * 2.0 * M_PI;
        double right_pos = (enc_msg.right_ticks / ticks_per_revolution) * 2.0 * M_PI;
        double cam_pos_rad = (enc_msg.cam_pos / 4096.0) * 2.0 * M_PI;  // 12-bit to radians
        
        joint_msg.position.push_back(left_pos);
        joint_msg.position.push_back(right_pos);
        joint_msg.position.push_back(cam_pos_rad);
        
        // Calculate velocities
        double dt = (node->now() - last_msg_time).seconds();
        if (dt > 0.0) {
            double left_delta = enc_msg.left_ticks - last_left_ticks;
            double right_delta = enc_msg.right_ticks - last_right_ticks;
            
            double left_vel = (left_delta / ticks_per_revolution) * 2.0 * M_PI / dt;
            double right_vel = (right_delta / ticks_per_revolution) * 2.0 * M_PI / dt;
            
            joint_msg.velocity.push_back(left_vel);
            joint_msg.velocity.push_back(right_vel);
            joint_msg.velocity.push_back(0.0);  // CAM velocity not calculated
        }
        
        joint_state_pub->publish(joint_msg);
        
        // Update last values
        last_left_ticks = enc_msg.left_ticks;
        last_right_ticks = enc_msg.right_ticks;
        last_msg_time = node->now();
    }
};

}  // namespace extra_plugins
}  // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::WheelEncodersPlugin)