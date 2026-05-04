#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/system_id.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" // Ensure this header is included

#include <std_msgs/msg/float64.hpp>

#include <deque>
#include <cmath>
#include "rclcpp/clock.hpp"

// // ROS2 messages
// #include "pegasus_msgs/msg/pid_statistics.hpp"
// #include "pegasus_msgs/msg/control_attitude.hpp"
// #include "pegasus_msgs/msg/control_position.hpp"

namespace autopilot {

class DragIdentification : public autopilot::Mode {

public:

    ~DragIdentification();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The waypoint service callback
    void dragID_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request, const pegasus_msgs::srv::SystemID::Response::SharedPtr response);

    // Check if the waypoint is already set
    bool settings_set_{false};

    Eigen::Vector3d compute_attitude(double t);

    // The mass of the vehicle
    double mass_;

    float prev_height_error_{0.0};

    // The target position and attitude waypoint to be at
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    double pitch_angle_{0.0}, roll_angle_{0.0}, yaw_angle_{0.0};
    float amplitude{0.0f};
    float t{0.0f};

    std::deque<double> vel_buffer_;
    std::deque<double> time_buffer_;
    size_t buffer_size_ = 100;    // tunable: window length for steady detection

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::SystemID>::SharedPtr dragID_service_{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drag_publisher_;
    // Create Vector3Stamped message
    geometry_msgs::msg::Vector3Stamped euler_msg;
};

}