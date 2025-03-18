#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/system_id.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" // Ensure this header is included
#include "rclcpp/clock.hpp"

// // ROS2 messages
// #include "pegasus_msgs/msg/pid_statistics.hpp"
// #include "pegasus_msgs/msg/control_attitude.hpp"
// #include "pegasus_msgs/msg/control_position.hpp"

namespace autopilot {

class SystemIdentification : public autopilot::Mode {

public:

    ~SystemIdentification();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

protected:

    // The waypoint service callback
    void systemID_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request, const pegasus_msgs::srv::SystemID::Response::SharedPtr response);

    // Check if the waypoint is already set
    bool settings_set_{false};

    Eigen::Vector3d compute_attitude(double t);

    // The mass of the vehicle
    double mass_;

    float prev_height_error_{0.0};

    // The target position and attitude waypoint to be at
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    float amplitude{0.0f};
    float t{0.0f};

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::SystemID>::SharedPtr systemID_service_{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_target_publisher_;
    // Create Vector3Stamped message
    geometry_msgs::msg::Vector3Stamped euler_msg;
};

}