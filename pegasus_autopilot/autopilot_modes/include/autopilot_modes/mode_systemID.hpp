#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/system_id.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/clock.hpp"

static constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;

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

    // Helper functions
    void generatePathMsg();
    void publishDroneMarker();
    void debuggingAttitudeRefs(const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk,double yaw_rad, double yaw_rate_rad);
    void debugTrajectoryRefs(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration);

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr trajectory_position_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr drone_marker_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_rate_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_real_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_rate_real_pub_;

    // Eigen::Vector3d compute_attitude(double t);
    Eigen::Vector3d compute_position(double t);
    Eigen::Vector3d compute_velocity(double t);
    Eigen::Vector3d compute_acceleration(double t);
    Eigen::Vector3d compute_jerk(double t);

    // The mass of the vehicle
    double mass_;

    float prev_height_error_{0.0};

    // The target position and attitude waypoint to be at
    Eigen::Vector3d axis{Eigen::Vector3d::Zero()};
    float amplitude{0.0f};
    float frequency{0.0f};
    float t{0.0f};

    // The waypoint service server that sets the position and attitude waypoints at a given target
    rclcpp::Service<pegasus_msgs::srv::SystemID>::SharedPtr systemID_service_{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_target_publisher_;
    // Create Vector3Stamped message
    geometry_msgs::msg::Vector3Stamped euler_msg;
    nav_msgs::msg::Path path_msg_;
};

}