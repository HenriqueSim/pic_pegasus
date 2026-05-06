#pragma once

#include <autopilot/mode.hpp>
#include "pegasus_msgs/srv/through_window.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace autopilot {

class ThroughWindowMode : public autopilot::Mode {

public:

    ~ThroughWindowMode();

    // ── Mandatory Mode interface ──────────────────────────────────────────
    void initialize() override;
    bool enter()      override;
    bool exit()       override;
    void update(double dt) override;

private:

    // ── Trajectory evaluation ─────────────────────────────────────────────

    // Evaluate the active segment polynomial and first three derivatives
    // at local segment time t_local.
    void eval_trajectory(double t_local,
                         Eigen::Vector3d& pos,
                         Eigen::Vector3d& vel,
                         Eigen::Vector3d& acc,
                         Eigen::Vector3d& jerk) const;

    // ── Control ───────────────────────────────────────────────────────────

    void set_position(const Eigen::Vector3d& pos,
                      const Eigen::Vector3d& vel,
                      const Eigen::Vector3d& acc,
                      const Eigen::Vector3d& jerk,
                      double yaw, double yaw_rate, double dt);

    double compute_output(double pos_err, double vel_err,
                          double ff_acc, unsigned int axis);

    // ── Debug / visualisation ─────────────────────────────────────────────

    void debug_attitude_refs(const Eigen::Vector3d& acc,
                             const Eigen::Vector3d& jerk,
                             const State& state);

    void generate_path_msg();
    void generate_segment_path_msgs();
    void generate_trajectory_markers(double dt_marker);
    void publish_drone_marker();

    // ── Service callback ──────────────────────────────────────────────────

    void through_window_callback(
        const pegasus_msgs::srv::ThroughWindow::Request::SharedPtr  req,
        const pegasus_msgs::srv::ThroughWindow::Response::SharedPtr res);

    // ═════════════════════════════════════════════════════════════════════
    //  Member variables
    // ═════════════════════════════════════════════════════════════════════

    // ── Trajectory segments ───────────────────────────────────────────────
    std::vector<Eigen::VectorXd> x_segments_;
    std::vector<Eigen::VectorXd> y_segments_;
    std::vector<Eigen::VectorXd> z_segments_;
    std::vector<double>          segment_durations_;

    // Cumulative start times within the entry block and within one circular lap.
    // entry_start_times_[i]  = time since t=0   for entry segment i
    // circ_start_times_[i]   = time since start of circular lap for circular segment i
    std::vector<double> entry_start_times_;
    std::vector<double> circ_start_times_;

    // Counts and durations
    int    num_entry_segments_{1};
    int    num_circular_segments_{1};
    int    num_segments_total_{2};
    int    num_laps_{1};
    double entry_duration_{0.0};        // total time of entry portion
    double circular_lap_duration_{0.0}; // duration of one circular lap
    double T_final_{0.0};               // entry_duration + num_laps * circular_lap_duration

    // Active coefficients (swapped each tick)
    Eigen::VectorXd x_coeffs_;
    Eigen::VectorXd y_coeffs_;
    Eigen::VectorXd z_coeffs_;

    // ── Clock and phase tracking ──────────────────────────────────────────
    double t_{0.0};
    bool   in_entry_phase_{true};   // true while playing entry segments
    int    current_segment_{0};     // index into x_segments_ / y_segments_ / z_segments_
    int    current_lap_{0};         // 0-indexed circular lap counter

    // ── Vehicle constants ─────────────────────────────────────────────────
    double mass_{1.0};

    // ── Controller gains ──────────────────────────────────────────────────
    Eigen::Vector3d kp_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d kd_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d ki_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d kr_{Eigen::Matrix3d::Zero()};
    Eigen::Vector3d min_output_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d max_output_{Eigen::Vector3d::Zero()};

    // ── Linear drag (inertial frame) ──────────────────────────────────────
    double rho_x_{0.6};
    double rho_y_{0.6};
    double rho_z_{0.4};

    // ── Service / trigger ─────────────────────────────────────────────────
    bool   window_point_set_{false};
    double window_position_[3]{0.0, 0.0, 0.0};
    double window_angle_{0.0};
    rclcpp::Service<pegasus_msgs::srv::ThroughWindow>::SharedPtr window_service_{nullptr};

    // ── ROS publishers ────────────────────────────────────────────────────
    // Full reference path (all segments, published once)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Per-segment reference paths: /debug/trajectory/seg{i}
    // Entry segments are dashed/grey; circular segments are coloured.
    std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> seg_path_pubs_;
    std::vector<nav_msgs::msg::Path>                               seg_path_msgs_;

    // Live tracked path: /debug/tracked_path  (reset on enter(), appended each tick)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tracked_path_pub_;
    nav_msgs::msg::Path                               tracked_path_msg_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr drone_marker_pub_;

    // Reference trajectory debug
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr traj_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr traj_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr traj_acc_pub_;

    // Real state debug
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr real_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr real_vel_pub_;

    // Attitude debug
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr att_error_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr att_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr att_rate_ref_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr att_real_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr att_rate_real_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;

    // ── Cached messages ───────────────────────────────────────────────────
    nav_msgs::msg::Path             path_msg_;
    visualization_msgs::msg::Marker marker_msg_;

    // ── Constants ─────────────────────────────────────────────────────────
    static constexpr double G       = 9.81;
    static constexpr double RAD2DEG = 180.0 / M_PI;
};

} // namespace autopilot